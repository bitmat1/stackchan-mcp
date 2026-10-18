"""Tests for the MIDI synth tools (play_midi / midi_send)."""

from __future__ import annotations

import base64
import json

import pytest
from mcp.types import CallToolRequest, CallToolRequestParams

from stackchan_mcp.midi import (
    TICKS_PER_BEAT,
    load_midi_bytes,
    melody_to_midi,
    note_number,
    parse_send_bytes,
)


def _events(smf: bytes) -> list[tuple[int, bytes]]:
    """(absolute tick, event bytes) for the single track of ``smf``."""
    assert smf[:4] == b"MThd" and smf[14:18] == b"MTrk"
    track = smf[22 : 22 + int.from_bytes(smf[18:22], "big")]
    out, i, tick = [], 0, 0
    while i < len(track):
        delta = 0
        while True:
            b = track[i]
            i += 1
            delta = (delta << 7) | (b & 0x7F)
            if not b & 0x80:
                break
        tick += delta
        if track[i] == 0xFF:
            length = track[i + 2]
            out.append((tick, track[i : i + 3 + length]))
            i += 3 + length
        else:
            size = 2 if track[i] & 0xF0 == 0xC0 else 3
            out.append((tick, track[i : i + size]))
            i += size
    return out


@pytest.mark.parametrize(
    ("name", "number"),
    [("C4", 60), ("A4", 69), ("c#4", 61), ("Bb3", 58), ("C-1", 0), ("G9", 127), ("64", 64)],
)
def test_note_number(name, number):
    assert note_number(name) == number


@pytest.mark.parametrize("bad", ["H4", "C", "C#", "G#9", "200", ""])
def test_note_number_rejects(bad):
    with pytest.raises(ValueError):
        note_number(bad)


def test_melody_timing_rests_and_chords():
    smf = melody_to_midi("C4 R:0.5 E4+G4:2", tempo=100, program=11)
    events = _events(smf)
    assert events[0] == (0, b"\xff\x51\x03" + (600_000).to_bytes(3, "big"))
    assert events[1] == (0, bytes([0xC0, 11]))
    on = [(t, e[1]) for t, e in events if len(e) == 3 and e[0] == 0x90]
    off = [(t, e[1]) for t, e in events if len(e) == 3 and e[0] == 0x80]
    # C4 for a beat, half a beat of rest, then a two-beat chord.
    assert on == [(0, 60), (720, 64), (720, 67)]
    assert off[0][1] == 60 and off[0][0] < TICKS_PER_BEAT
    assert {n for _, n in off[1:]} == {64, 67}
    assert all(720 < t < 720 + 2 * TICKS_PER_BEAT for t, _ in off[1:])
    assert events[-1][1] == b"\xff\x2f\x00"
    assert events[-1][0] == 720 + 2 * TICKS_PER_BEAT


@pytest.mark.parametrize(
    "kwargs",
    [{"notes": ""}, {"notes": "C4:0"}, {"notes": "C4:x"}, {"notes": "C4", "tempo": 5}, {"notes": "C4", "program": 128}],
)
def test_melody_rejects(kwargs):
    with pytest.raises(ValueError):
        melody_to_midi(**kwargs)


def test_parse_send_bytes():
    assert parse_send_bytes("90 3c 64, 80:3C:00") == bytes([0x90, 0x3C, 0x64, 0x80, 0x3C, 0x00])
    for bad in ["", "9", "zz", "90 3"]:
        with pytest.raises(ValueError):
            parse_send_bytes(bad)


@pytest.mark.asyncio
async def test_load_midi_bytes_from_path_and_base64(tmp_path):
    smf = melody_to_midi("C4")
    path = tmp_path / "tune.mid"
    path.write_bytes(smf)
    assert await load_midi_bytes(str(path)) == smf
    assert await load_midi_bytes(base64.b64encode(smf).decode()) == smf
    with pytest.raises(ValueError):
        await load_midi_bytes(base64.b64encode(b"not midi at all").decode())


def _fake_gateway(monkeypatch):
    from stackchan_mcp import gateway as gateway_mod

    staged, calls = {}, []

    async def fake_stage(app, mode, payload):
        staged.update(mode=mode, payload=payload)
        return "abc", "tok", "sha256:x"

    class FakeEsp32:
        device_facing_host = None

        async def call_tool(self, name, args):
            calls.append((name, args))
            return {"content": [{"type": "text", "text": '{"ok":true}'}]}, None

    monkeypatch.setattr(gateway_mod, "stage_avatar_set", fake_stage)
    gw = gateway_mod.Gateway()
    gw.esp32 = FakeEsp32()
    gw._capture_app = object()
    return gw, staged, calls


@pytest.mark.asyncio
async def test_gateway_play_midi_melody_stages_and_calls_the_robot(monkeypatch):
    gw, staged, calls = _fake_gateway(monkeypatch)
    result = await gw.play_midi(notes="C4 E4 G4", tempo=90, program=73, loops=1, volume=60)
    assert result["ok"]
    assert staged["mode"] == "midi"
    assert staged["payload"] == melody_to_midi("C4 E4 G4", tempo=90, program=73)
    name, args = calls[0]
    assert name == "self.audio.midi_play"
    assert args["url"].endswith("/avatar_set/abc")
    assert (args["token"], args["loops"], args["volume"]) == ("tok", 1, 60)


@pytest.mark.asyncio
async def test_gateway_play_midi_needs_input(monkeypatch):
    gw, _, calls = _fake_gateway(monkeypatch)
    assert not (await gw.play_midi())["ok"]
    assert not (await gw.play_midi(notes="Q9"))["ok"]
    assert calls == []


def _request(name, arguments):
    return CallToolRequest(
        method="tools/call", params=CallToolRequestParams(name=name, arguments=arguments)
    )


@pytest.mark.asyncio
async def test_midi_send_validates_and_forwards(monkeypatch):
    from stackchan_mcp import stdio_server

    calls = []

    class FakeEsp32:
        device_connected = True

        async def call_tool(self, name, args):
            calls.append((name, args))
            return {"content": [{"type": "text", "text": '{"ok":true}'}]}, None

    class FakeGateway:
        esp32 = FakeEsp32()

    monkeypatch.setattr(stdio_server, "get_gateway", lambda: FakeGateway())
    server = stdio_server.create_server()
    handler = server.request_handlers[CallToolRequest]

    result = await handler(_request("midi_send", {"hex": "903C64"}))
    assert "error" not in result.root.content[0].text
    assert calls == [("self.audio.midi_send", {"hex": "90 3c 64", "volume": 80})]

    result = await handler(_request("midi_send", {"hex": "90 3"}))
    assert json.loads(result.root.content[0].text)["ok"] is False
    assert len(calls) == 1
