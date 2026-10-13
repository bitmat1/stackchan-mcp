"""Who answers the robot: Claude Code (default) or a local Ollama model.

With the ``claude`` brain, speech the robot hears is pushed to a Claude Code
session as a channel message (see :mod:`stackchan_mcp.voice_channel`) and
Claude replies through the MCP tools.

With the ``ollama`` brain the gateway answers by itself, with no cloud and
no Claude Code session: the transcript goes to Ollama's ``/api/chat`` with
a short rolling memory, and the model answers in one request as
``[face] [gesture] spoken text``. The gateway shows the face, plays the
gesture and speaks the text through the daemon's command queue (the same
path as the web panel). Small local models handle this far better than
function calling, which cost an extra round trip per gesture and made
their answers terse and confused.

Configuration (environment variables; the panel can change brain and model
at runtime, and that choice is saved to ``brain.json`` in the user config
directory, ~/Library/Application Support/stackchan-mcp on macOS):

    STACKCHAN_BRAIN         "claude" (default) or "ollama"
    OLLAMA_URL              default http://127.0.0.1:11434
    STACKCHAN_OLLAMA_MODEL  default: an installed llama/qwen model, else "llama3.1"
"""

from __future__ import annotations

import asyncio
import json
import logging
import os
import re
from collections import deque
from pathlib import Path
from typing import Any, Awaitable, Callable

import aiohttp
from platformdirs import user_config_path

logger = logging.getLogger(__name__)

BRAINS = ("claude", "ollama")
DEFAULT_OLLAMA_URL = "http://127.0.0.1:11434"
DEFAULT_OLLAMA_MODEL = "llama3.1"
#: Preferred families when no model was chosen (all support tool calls).
PREFERRED_FAMILIES = ("llama3", "qwen")
HISTORY_TURNS = 8
REQUEST_TIMEOUT_S = 120
#: Keep the model loaded between turns (loading an 8B model takes ~40 s)
#: and use a small context: a short chat needs little and it saves memory.
KEEP_ALIVE = "30m"
NUM_CTX = 4096
FALLBACK_FACES = ("idle", "happy", "thinking", "sad", "surprised", "embarrassed")
FALLBACK_GESTURES = ("nod", "shake", "tilt")

SYSTEM_PROMPT = """\
You are Stack-chan, a small, friendly desk robot. You have a face on a \
screen, a speaker, a camera and a head that can turn and tilt. A person in \
the room is talking to you; their words were transcribed from speech, so \
allow for mis-heard words.

Start every reply with your facial expression in square brackets, \
optionally followed by one head gesture in square brackets, then what you \
say. Example: [happy] [nod] Sure, I'd love to help!
Expressions: {faces}
Gestures: {gestures}

What you say is spoken aloud: one to three short, warm, natural sentences, \
no markdown, lists, emoji or URLs."""
_TAG = re.compile(r"^\s*\[([a-z_ ]+)\]", re.IGNORECASE)
_THINK = re.compile(r"<think>.*?</think>", re.DOTALL | re.IGNORECASE)

ToolRunner = Callable[[str, dict[str, Any]], Awaitable[Any]]
SchemaLoader = Callable[[], Awaitable[dict[str, dict[str, Any]]]]

_override: dict[str, str] = {}
_runner: ToolRunner | None = None
_schema_loader: SchemaLoader | None = None
_history: deque[dict[str, Any]] = deque(maxlen=HISTORY_TURNS * 2)
_lock = asyncio.Lock()


def settings_path() -> Path:
    return user_config_path("stackchan-mcp") / "brain.json"


def _saved() -> dict[str, str]:
    try:
        data = json.loads(settings_path().read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return {}
    return {k: v for k, v in data.items() if isinstance(v, str)} if isinstance(data, dict) else {}


def brain_mode() -> str:
    mode = (_override.get("brain") or _saved().get("brain") or os.getenv("STACKCHAN_BRAIN", "claude")).strip().lower()
    return mode if mode in BRAINS else "claude"


def chosen_model() -> str:
    """The model picked in the panel, saved, or set in the env ("" if none)."""
    return (
        _override.get("model") or _saved().get("model") or os.getenv("STACKCHAN_OLLAMA_MODEL", "")
    ).strip()


def ollama_model(installed: list[str] | None = None) -> str:
    """The chosen model, else the best installed one, else the default."""
    chosen = chosen_model()
    if chosen:
        return chosen
    for family in PREFERRED_FAMILIES:
        for name in installed or []:
            if name.startswith(family):
                return name
    return (installed or [DEFAULT_OLLAMA_MODEL])[0]


def ollama_url() -> str:
    return (os.getenv("OLLAMA_URL") or DEFAULT_OLLAMA_URL).rstrip("/")


def set_brain(mode: str | None = None, model: str | None = None) -> None:
    """Switch brain and/or model now and remember it across restarts."""
    if mode is not None:
        mode = mode.strip().lower()
        if mode not in BRAINS:
            raise ValueError(f"brain must be one of {BRAINS}")
        _override["brain"] = mode
    if model is not None:
        if not model.strip():
            raise ValueError("model must not be empty")
        _override["model"] = model.strip()
    data = _saved() | _override
    try:
        path = settings_path()
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")
    except OSError as exc:
        logger.warning("brain: could not save %s: %s", settings_path(), exc)
    _history.clear()


def register_tool_runner(runner: ToolRunner, schema_loader: SchemaLoader) -> None:
    """Called by the HTTP daemon: how to run tools and read their schemas.

    ``schema_loader`` returns ``{name: {"description", "inputSchema"}}``.
    """
    global _runner, _schema_loader
    _runner = runner
    _schema_loader = schema_loader


async def _choices() -> tuple[list[str], list[str]]:
    """Face and gesture names the robot accepts, from the tool schemas."""
    schemas = await _schema_loader() if _schema_loader is not None else {}

    def enum(tool: str, prop: str, fallback: tuple[str, ...]) -> list[str]:
        props = schemas.get(tool, {}).get("inputSchema", {}).get("properties", {})
        values = props.get(prop, {}).get("enum")
        return [v for v in values if v != "off"] if values else list(fallback)

    return enum("set_avatar", "face", FALLBACK_FACES), enum("gesture", "name", FALLBACK_GESTURES)


def parse_reply(raw: str, faces: list[str], gestures: list[str]) -> tuple[str | None, str | None, str]:
    """Split ``[face] [gesture] text`` into its parts; unknown tags are dropped."""
    text = _THINK.sub("", raw).strip()
    face = gesture = None
    while match := _TAG.match(text):
        tag = match.group(1).strip().lower().replace(" ", "_")
        if tag in faces and face is None:
            face = tag
        elif tag in gestures and gesture is None:
            gesture = tag
        text = text[match.end() :].lstrip()
    return face, gesture, text


async def list_ollama_models() -> tuple[bool, list[str]]:
    """(reachable, installed model names)."""
    try:
        timeout = aiohttp.ClientTimeout(total=3)
        async with aiohttp.ClientSession(timeout=timeout) as http:
            async with http.get(f"{ollama_url()}/api/tags") as resp:
                resp.raise_for_status()
                data = await resp.json()
    except (aiohttp.ClientError, asyncio.TimeoutError, ValueError):
        return False, []
    return True, sorted(m.get("name", "") for m in data.get("models", []) if m.get("name"))


async def _chat(messages: list[dict[str, Any]], model: str) -> str:
    body: dict[str, Any] = {
        "model": model,
        "messages": messages,
        "stream": False,
        "keep_alive": KEEP_ALIVE,
        "options": {"num_ctx": NUM_CTX},
    }
    timeout = aiohttp.ClientTimeout(total=REQUEST_TIMEOUT_S)
    async with aiohttp.ClientSession(timeout=timeout) as http:
        async with http.post(f"{ollama_url()}/api/chat", json=body) as resp:
            resp.raise_for_status()
            data = await resp.json()
    return (data.get("message") or {}).get("content") or ""


async def _run(name: str, args: dict[str, Any]) -> str:
    if _runner is None:
        return "tools unavailable"
    try:
        result = await _runner(name, args)
    except Exception as exc:  # a failed gesture must not stop the reply
        logger.warning("brain: tool %s failed: %s", name, exc)
        return f"error: {exc}"
    texts = [getattr(item, "text", "") for item in result] if isinstance(result, list) else [str(result)]
    return " ".join(t for t in texts if t)[:500] or "ok"


async def _say(text: str) -> None:
    if text:
        await _run("say", {"text": text})


async def respond(text: str) -> str:
    """Answer ``text`` with the local model; returns what was spoken."""
    async with _lock:  # one conversation turn at a time
        faces, gestures = await _choices()
        system = SYSTEM_PROMPT.format(faces=", ".join(faces), gestures=", ".join(gestures))
        messages = [{"role": "system", "content": system}, *_history, {"role": "user", "content": text}]
        model = chosen_model() or ollama_model((await list_ollama_models())[1])
        await _run("set_avatar", {"face": "thinking"})
        try:
            raw = await _chat(messages, model)
        except (aiohttp.ClientError, asyncio.TimeoutError) as exc:
            logger.warning("brain: Ollama request failed: %s", exc)
            from . import activity

            activity.record("error", f"Ollama ({model}) did not answer: {exc}", source="ollama")
            await _run("set_avatar", {"face": "sad"})
            await _say("Sorry, my local brain isn't answering. Is Ollama running?")
            return ""
        face, gesture, reply = parse_reply(raw, faces, gestures)
        if not reply:
            reply = "Hmm, I'm not sure what to say."
        _history.append({"role": "user", "content": text})
        # Remember the tagged form so the model keeps using the format.
        _history.append({"role": "assistant", "content": _THINK.sub("", raw).strip() or reply})
        await _run("set_avatar", {"face": face or "idle"})
        if gesture:
            await _run("gesture", {"name": gesture})
        await _say(reply)
        logger.info("brain: ollama/%s answered %r", model, reply[:80])
        return reply
