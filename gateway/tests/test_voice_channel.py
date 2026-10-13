"""Tests for device-driven speech -> local STT -> channel notification."""

from __future__ import annotations

from typing import Any

import pytest

from stackchan_mcp import voice_channel
from stackchan_mcp.stt import get_registry
from stackchan_mcp.stt.base import STTEngine


class _FakeSTT(STTEngine):
    name = "fake-stt"

    def __init__(self, text: str) -> None:
        self.text = text
        self.opts: dict[str, Any] = {}

    async def transcribe(self, pcm: bytes, **opts: Any) -> dict[str, Any]:
        self.opts = opts
        return {"text": self.text, "language": "en"}


@pytest.fixture
def sent(monkeypatch):
    calls: list[tuple[str, dict[str, Any]]] = []

    async def fake_notify(method: str, params: dict[str, Any]) -> None:
        calls.append((method, params))

    import stackchan_mcp.stdio_server as stdio_server
    import stackchan_mcp.stt.audio_utils as stt_audio

    monkeypatch.setattr(stdio_server, "notify_stackchan_event", fake_notify)
    monkeypatch.setattr(stt_audio, "decode_opus_frames", lambda frames: b"\x00\x00" * 160)
    monkeypatch.setenv("STACKCHAN_VOICE_STT_ENGINE", "fake-stt")
    return calls


def _register(text: str) -> _FakeSTT:
    engine = _FakeSTT(text)
    get_registry().register(engine)
    return engine


@pytest.mark.parametrize("value,expected", [("1", True), ("true", True), ("", False), ("0", False)])
def test_enabled_flag(monkeypatch, value, expected):
    monkeypatch.setenv(voice_channel.VOICE_CHANNEL_ENV_VAR, value)
    assert voice_channel.voice_channel_enabled() is expected


@pytest.mark.asyncio
async def test_transcript_sent_as_channel_message(sent, monkeypatch):
    engine = _register("  what can you see?  ")
    monkeypatch.setenv("STACKCHAN_VOICE_STT_LANGUAGE", "en")
    text = await voice_channel.transcribe_and_notify([b"x"] * 10, session_id="s1")
    assert text == "what can you see?"
    assert engine.opts == {"language": "en"}
    method, params = sent[0]
    assert method == "notifications/claude/channel"
    assert params["content"] == "what can you see?"
    assert params["meta"]["event_type"] == "voice"
    assert params["meta"]["duration_ms"] == "600"
    assert all(isinstance(v, str) for v in params["meta"].values())


@pytest.mark.asyncio
async def test_short_capture_ignored(sent):
    _register("hello")
    assert await voice_channel.transcribe_and_notify([b"x"] * 2) is None
    assert sent == []


@pytest.mark.asyncio
async def test_empty_transcript_ignored(sent):
    _register("   ")
    assert await voice_channel.transcribe_and_notify([b"x"] * 10) is None
    assert sent == []


@pytest.mark.asyncio
async def test_missing_engine_ignored(sent, monkeypatch):
    monkeypatch.setenv("STACKCHAN_VOICE_STT_ENGINE", "no-such-engine")
    assert await voice_channel.transcribe_and_notify([b"x"] * 10) is None
    assert sent == []
