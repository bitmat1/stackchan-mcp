"""Tests for the macOS ``say`` TTS engine."""

from __future__ import annotations

import sys

import pytest

from stackchan_mcp.tts import get_registry
from stackchan_mcp.tts.audio_utils import DEVICE_SAMPLE_RATE
from stackchan_mcp.tts.macos_say import DEFAULT_MACOS_SAY_VOICE, MacOSSayEngine

darwin_only = pytest.mark.skipif(sys.platform != "darwin", reason="needs macOS say")


def test_default_voice_reads_env(monkeypatch):
    monkeypatch.delenv("STACKCHAN_MACOS_SAY_VOICE", raising=False)
    assert MacOSSayEngine().default_voice == DEFAULT_MACOS_SAY_VOICE
    monkeypatch.setenv("STACKCHAN_MACOS_SAY_VOICE", "Flo (English (UK))")
    assert MacOSSayEngine().default_voice == "Flo (English (UK))"


@pytest.mark.asyncio
async def test_empty_text_rejected():
    with pytest.raises(ValueError):
        await MacOSSayEngine().synthesize("")


@darwin_only
def test_registered_on_macos():
    assert "macos-say" in get_registry().names()


@darwin_only
@pytest.mark.asyncio
async def test_synthesizes_16k_pcm():
    pcm = await MacOSSayEngine().synthesize("Hello from Stack-chan.")
    # 16-bit samples, and at least half a second of speech.
    assert len(pcm) % 2 == 0
    assert len(pcm) > DEVICE_SAMPLE_RATE * 2 // 2


@darwin_only
@pytest.mark.asyncio
async def test_unknown_voice_raises():
    with pytest.raises(RuntimeError):
        await MacOSSayEngine(default_voice="No Such Voice 123").synthesize("hi")
