"""Tests for say() playback pacing: prebuffer and Wi-Fi awake lease."""

from __future__ import annotations

import asyncio
import json
from typing import Any

import pytest

from stackchan_mcp.tts import orchestrator
from stackchan_mcp.tts.audio_utils import DEVICE_FRAME_DURATION_MS, DEVICE_SAMPLE_RATE
from stackchan_mcp import wifi_power_save

FRAME_BYTES = DEVICE_SAMPLE_RATE * DEVICE_FRAME_DURATION_MS // 1000 * 2


class _ESP32:
    device_connected = True

    def __init__(self) -> None:
        self.events: list[tuple[str, Any]] = []
        self.frame_times: list[float] = []
        self.tts_lock = asyncio.Lock()

    async def call_tool(self, name: str, arguments: dict[str, Any]):
        self.events.append(("tool", name, arguments.get("mode")))
        return {"content": [{"type": "text", "text": json.dumps({"ok": True, "previous": "min_modem", "current": arguments.get("mode")})}]}, None

    async def send_audio_frame(self, frame: bytes) -> None:
        self.frame_times.append(asyncio.get_event_loop().time())
        self.events.append(("frame",))

    async def send_tts_state(self, state: str) -> None:
        self.events.append(("tts", state))


class _Gateway:
    def __init__(self) -> None:
        self.esp32 = _ESP32()


@pytest.fixture(autouse=True)
def _fake_encode(monkeypatch):
    def fake(pcm: bytes, **kwargs):
        n = -(-len(pcm) // FRAME_BYTES)
        return [b"f%d" % i for i in range(n)]

    monkeypatch.setattr(orchestrator, "encode_opus_frames", fake)
    wifi_power_save._clear_for_tests()


@pytest.mark.asyncio
async def test_prebuffer_front_loads_frames(monkeypatch):
    monkeypatch.setenv("STACKCHAN_TTS_PREBUFFER_MS", "600")
    gw = _Gateway()
    await orchestrator.send_pcm_audio(gw, b"\x00" * FRAME_BYTES * 14)
    t = gw.esp32.frame_times
    burst = t[9] - t[0]
    # First 600 ms (10 frames) go out back-to-back, the rest are paced.
    assert burst < 0.05
    assert t[13] - t[10] > 0.15


@pytest.mark.asyncio
async def test_prebuffer_can_be_disabled(monkeypatch):
    monkeypatch.setenv("STACKCHAN_TTS_PREBUFFER_MS", "0")
    gw = _Gateway()
    await orchestrator.send_pcm_audio(gw, b"\x00" * FRAME_BYTES * 4)
    t = gw.esp32.frame_times
    assert t[3] - t[0] > 0.15


@pytest.mark.asyncio
async def test_wifi_kept_awake_while_speaking(monkeypatch):
    monkeypatch.setenv("STACKCHAN_TTS_WIFI_AWAKE", "1")
    gw = _Gateway()
    await orchestrator.send_pcm_audio(gw, b"\x00" * FRAME_BYTES * 2)
    ev = gw.esp32.events
    assert ev[0] == ("tool", "self.wifi.set_power_save", "none")
    assert ev[1] == ("tts", "start")
    assert ev[-2] == ("tts", "stop")
    assert ev[-1] == ("tool", "self.wifi.set_power_save", "min_modem")


class _TextEngine:
    name = "fake"
    supports_emoji_style = False

    async def synthesize(self, text: str, **opts: Any) -> bytes:
        return b"\x00" * FRAME_BYTES * 3


@pytest.mark.asyncio
async def test_subtitle_shown_during_speech_and_cleared(monkeypatch):
    from stackchan_mcp.tts.base import EngineRegistry

    monkeypatch.setenv("STACKCHAN_SUBTITLES", "1")
    reg = EngineRegistry()
    reg.register(_TextEngine())
    gw = _Gateway()
    calls: list[dict[str, Any]] = []

    async def call_tool(name: str, arguments: dict[str, Any]):
        if name == "self.display.set_subtitle":
            calls.append(arguments)
            gw.esp32.events.append(("subtitle", arguments["text"]))
        return {"ok": True}, None

    gw.esp32.call_tool = call_tool
    await orchestrator.synthesize_and_send({"text": "Hello there 😊", "voice": "fake"}, gateway=gw, registry=reg)
    assert calls[0]["text"] == "Hello there"
    assert calls[0]["duration_ms"] == 3 * DEVICE_FRAME_DURATION_MS
    assert calls[-1] == {"text": "", "duration_ms": 0}
    kinds = [e[0] for e in gw.esp32.events]
    assert kinds.index("subtitle") < kinds.index("frame")
    assert kinds[-1] == "subtitle"
