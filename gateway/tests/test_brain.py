"""Tests for the local Ollama brain, its web routes and the app launcher."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import aiohttp
import pytest
from mcp.types import TextContent

from stackchan_mcp import app, brain, voice_channel


@pytest.fixture(autouse=True)
def _isolated_brain(tmp_path, monkeypatch):
    """Fresh brain state with settings in a temp dir and a recording runner."""
    monkeypatch.setattr(brain, "settings_path", lambda: tmp_path / "brain.json")
    monkeypatch.delenv("STACKCHAN_BRAIN", raising=False)
    monkeypatch.delenv("STACKCHAN_OLLAMA_MODEL", raising=False)
    brain._override.clear()
    brain._history.clear()
    yield
    brain._override.clear()
    brain._history.clear()


async def _no_models() -> tuple[bool, list[str]]:
    return False, []


def _record_tools() -> list[tuple[str, dict[str, Any]]]:
    calls: list[tuple[str, dict[str, Any]]] = []

    async def runner(name: str, args: dict[str, Any]) -> list[TextContent]:
        calls.append((name, args))
        return [TextContent(type="text", text="ok")]

    async def schemas() -> dict[str, dict[str, Any]]:
        def enum(prop: str, values: list[str]) -> dict[str, Any]:
            return {"inputSchema": {"properties": {prop: {"enum": values}}}}

        return {
            "set_avatar": enum("face", ["idle", "happy", "sad", "thinking", "off"]),
            "gesture": enum("name", ["nod", "tilt"]),
        }

    brain.register_tool_runner(runner, schemas)
    return calls


def test_brain_defaults_to_claude_and_follows_env(monkeypatch):
    assert brain.brain_mode() == "claude"
    monkeypatch.setenv("STACKCHAN_BRAIN", "ollama")
    assert brain.brain_mode() == "ollama"
    monkeypatch.setenv("STACKCHAN_BRAIN", "nonsense")
    assert brain.brain_mode() == "claude"


def test_set_brain_is_remembered(tmp_path):
    brain.set_brain("ollama", "qwen2.5:3b")
    saved = json.loads((tmp_path / "brain.json").read_text())
    assert saved == {"brain": "ollama", "model": "qwen2.5:3b"}
    brain._override.clear()  # a restart forgets the in-memory override
    assert (brain.brain_mode(), brain.ollama_model()) == ("ollama", "qwen2.5:3b")


def test_set_brain_rejects_unknown():
    with pytest.raises(ValueError):
        brain.set_brain("gpt")


def test_ollama_brain_turns_on_voice_transcription(monkeypatch):
    monkeypatch.delenv(voice_channel.VOICE_CHANNEL_ENV_VAR, raising=False)
    assert not voice_channel.voice_channel_enabled()
    brain.set_brain("ollama")
    assert voice_channel.voice_channel_enabled()


def test_default_model_prefers_an_installed_llama_or_qwen():
    assert brain.ollama_model(["mistral:7b", "qwen3.5:9b", "llama3.1:8b"]) == "llama3.1:8b"
    assert brain.ollama_model(["mistral:7b"]) == "mistral:7b"
    assert brain.ollama_model([]) == brain.DEFAULT_OLLAMA_MODEL
    brain.set_brain(model="qwen3.5:9b")
    assert brain.ollama_model(["llama3.1:8b"]) == "qwen3.5:9b"


@pytest.mark.parametrize(
    ("raw", "expected"),
    [
        ("[happy] [nod] Sure thing!", ("happy", "nod", "Sure thing!")),
        ("[Happy] Sure.", ("happy", None, "Sure.")),
        ("[nod][sad] Oh no.", ("sad", "nod", "Oh no.")),
        ("[dancing] Hi!", (None, None, "Hi!")),
        ("<think>hmm</think>\n[happy] Hi!", ("happy", None, "Hi!")),
        ("Just text.", (None, None, "Just text.")),
    ],
)
def test_parse_reply(raw, expected):
    assert brain.parse_reply(raw, ["happy", "sad"], ["nod", "tilt"]) == expected


@pytest.mark.asyncio
async def test_respond_shows_face_and_gesture_then_speaks(monkeypatch):
    calls = _record_tools()
    seen: list[list[dict[str, Any]]] = []

    async def fake_chat(messages, model):
        seen.append(messages)
        return "[happy] [nod] Hello there!"

    monkeypatch.setattr(brain, "_chat", fake_chat)
    monkeypatch.setattr(brain, "list_ollama_models", _no_models)
    assert await brain.respond("hi") == "Hello there!"
    assert calls == [
        ("set_avatar", {"face": "thinking"}),
        ("set_avatar", {"face": "happy"}),
        ("gesture", {"name": "nod"}),
        ("say", {"text": "Hello there!"}),
    ]
    assert "Expressions:" in seen[0][0]["content"]
    assert [m["role"] for m in brain._history] == ["user", "assistant"]


@pytest.mark.asyncio
async def test_respond_without_tags_goes_back_to_idle(monkeypatch):
    calls = _record_tools()

    async def fake_chat(messages, model):
        return "Sure."

    monkeypatch.setattr(brain, "_chat", fake_chat)
    monkeypatch.setattr(brain, "list_ollama_models", _no_models)
    await brain.respond("can you hear me")
    assert calls[-2:] == [("set_avatar", {"face": "idle"}), ("say", {"text": "Sure."})]


@pytest.mark.asyncio
async def test_respond_says_so_when_ollama_is_down(monkeypatch):
    calls = _record_tools()

    async def fake_chat(messages, model):
        raise aiohttp.ClientConnectionError("refused")

    monkeypatch.setattr(brain, "_chat", fake_chat)
    monkeypatch.setattr(brain, "list_ollama_models", _no_models)
    assert await brain.respond("hello") == ""
    assert calls[-2][0] == "set_avatar" and calls[-2][1]["face"] == "sad"
    assert calls[-1][0] == "say" and "Ollama" in calls[-1][1]["text"]
    assert not brain._history


def test_panel_prefers_an_app_window(tmp_path):
    (tmp_path / "Microsoft Edge.app").mkdir()
    cmd = app.panel_window_command("http://127.0.0.1:8767/ui", apps_dir=tmp_path)
    assert cmd[:3] == ["open", "-na", "Microsoft Edge"]
    assert "--app=http://127.0.0.1:8767/ui" in cmd


def test_panel_falls_back_to_default_browser(tmp_path):
    assert app.panel_window_command("http://x/ui", apps_dir=tmp_path) == ["open", "http://x/ui"]


def test_claude_session_script_quotes_paths():
    script = app.claude_session_script("/Users/a b/claude", Path('/tmp/voice "x"'))
    assert 'do script "' in script
    assert "--dangerously-load-development-channels server:stackchan" in script
    assert '\\"x\\"' in script  # inner quotes escaped for AppleScript
