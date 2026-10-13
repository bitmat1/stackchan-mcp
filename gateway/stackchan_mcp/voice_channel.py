"""Device-driven speech → local STT → Claude Code channel message.

When the robot starts listening on its own (wake word, button press, or
LCD touch), there is no ``listen()`` tool call in flight, so nothing on
the MCP side would hear what was said. With ``STACKCHAN_VOICE_CHANNEL=1``
the gateway buffers those Opus frames (same slot the audio hook uses),
transcribes them with the local STT engine on ``listen.stop``, and pushes
the text into the connected Claude Code session as a
``notifications/claude/channel`` message. Claude then answers through the
``say`` tool, which closes the hands-free loop without any cloud service.

Configuration (environment variables):

    STACKCHAN_VOICE_CHANNEL
        "1" / "true" enables this path. Off by default.

    STACKCHAN_VOICE_STT_ENGINE
        STT engine name. Default "faster-whisper" (local).

    STACKCHAN_VOICE_STT_LANGUAGE
        ISO 639-1 code, e.g. "en". Empty = autodetect.

    STACKCHAN_VOICE_STT_MODEL
        Engine model id, e.g. "small.en" for faster-whisper.

The channel message only reaches a Claude Code session started with this
server enabled as a channel; Claude Desktop does not consume channel
notifications.
"""

from __future__ import annotations

import logging
import os
import time
from typing import Sequence

logger = logging.getLogger(__name__)

VOICE_CHANNEL_ENV_VAR = "STACKCHAN_VOICE_CHANNEL"

#: Captures shorter than this are almost always a stray button press.
MIN_FRAMES = 5  # 5 x 60 ms = 300 ms


def voice_channel_enabled() -> bool:
    """True when device-driven speech should be transcribed and answered.

    The local Ollama brain always needs the transcript, so it turns this on.
    """
    from .brain import brain_mode

    if brain_mode() == "ollama":
        return True
    return os.getenv(VOICE_CHANNEL_ENV_VAR, "").strip().lower() in {"1", "true", "yes", "on"}


async def transcribe_and_notify(frames: Sequence[bytes], *, session_id: str = "") -> str | None:
    """Transcribe a device-driven capture and push it as a channel message.

    Returns the transcript that was sent, or ``None`` when nothing was
    sent (too short, empty transcript, or an engine error). Errors are
    logged rather than raised because this runs as a fire-and-forget
    task off the WebSocket read loop.
    """
    if len(frames) < MIN_FRAMES:
        logger.info("voice-channel: capture too short (%d frames), ignoring", len(frames))
        return None

    from .stt import get_registry
    from .stt.audio_utils import decode_opus_frames

    engine_name = os.getenv("STACKCHAN_VOICE_STT_ENGINE") or "faster-whisper"
    engine = get_registry().get(engine_name)
    if engine is None:
        logger.warning(
            "voice-channel: STT engine %r not registered (available: %s)",
            engine_name,
            ", ".join(get_registry().names()) or "none",
        )
        return None

    opts: dict[str, str] = {}
    language = os.getenv("STACKCHAN_VOICE_STT_LANGUAGE", "").strip()
    if language:
        opts["language"] = language
    model = os.getenv("STACKCHAN_VOICE_STT_MODEL", "").strip()
    if model:
        opts["model"] = model

    started = time.monotonic()
    try:
        pcm = decode_opus_frames(frames)
        result = await engine.transcribe(pcm, **opts)
    except Exception:
        logger.exception("voice-channel: transcription failed (session=%s)", session_id)
        return None

    text = str(result.get("text", "")).strip()
    elapsed_ms = int((time.monotonic() - started) * 1000)
    if not text:
        logger.info("voice-channel: empty transcript after %d ms, ignoring", elapsed_ms)
        return None

    logger.info("voice-channel: heard %r (%d ms STT)", text[:80], elapsed_ms)
    from . import activity

    activity.record("heard", text, source="mic")

    from .brain import brain_mode, respond

    if brain_mode() == "ollama":
        try:
            await respond(text)
        except Exception:
            logger.exception("voice-channel: local brain failed")
        return text

    from .stdio_server import notify_stackchan_event

    # Channel meta values must all be strings (Claude Code's schema).
    await notify_stackchan_event(
        "notifications/claude/channel",
        {
            "content": text,
            "meta": {
                "event_type": "voice",
                "source": "stackchan_mic",
                "language": str(result.get("language") or language or ""),
                "duration_ms": str(len(frames) * 60),
                "ts_unix": str(time.time()),
                "session_id": session_id,
            },
        },
    )
    return text
