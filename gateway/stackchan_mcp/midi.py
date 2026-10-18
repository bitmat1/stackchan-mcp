"""MIDI for the robot's General MIDI synth (play_midi / midi_send).

The firmware plays Standard MIDI Files with TinySoundFont and the microGM
SoundFont in its assets partition. This module loads a MIDI file from a
path, URL or base64, or writes one from a simple text melody, and turns
readable note lists into raw bytes for midi_send.

Melody text is a list of space-separated tokens, each a note, a chord or
a rest with an optional length in beats after a colon:

    "C4 D4 E4:2 R:0.5 G#4:0.5 C4+E4+G4:2 Bb3:1.5"

Notes are a letter A-G, an optional # or b and an octave (C4 = middle C,
MIDI 60), or a plain MIDI number. "R" is a rest. A chord joins notes with
"+". The default length is one beat.
"""

from __future__ import annotations

import asyncio
import base64
import os
import re

# The firmware refuses larger files.
MAX_MIDI_BYTES = 512 * 1024
TICKS_PER_BEAT = 480

_NOTE_RE = re.compile(r"^([A-Ga-g])([#b]?)(-?\d+)$")
_SEMITONES = {"c": 0, "d": 2, "e": 4, "f": 5, "g": 7, "a": 9, "b": 11}


def note_number(name: str) -> int:
    """MIDI number for a note name such as ``C4``, ``F#3``, ``Bb2`` or ``60``."""
    name = name.strip()
    if name.isdigit():
        n = int(name)
    else:
        m = _NOTE_RE.match(name)
        if not m:
            raise ValueError(f"bad note {name!r}: use e.g. C4, F#3, Bb2 or a MIDI number")
        letter, accidental, octave = m.groups()
        n = (int(octave) + 1) * 12 + _SEMITONES[letter.lower()]
        n += {"#": 1, "b": -1}.get(accidental, 0)
    if not 0 <= n <= 127:
        raise ValueError(f"note {name!r} is outside MIDI 0..127")
    return n


def _vlq(value: int) -> bytes:
    """MIDI variable-length quantity."""
    out = [value & 0x7F]
    value >>= 7
    while value:
        out.append(0x80 | (value & 0x7F))
        value >>= 7
    return bytes(reversed(out))


def melody_to_midi(
    notes: str,
    tempo: int = 120,
    program: int = 0,
    velocity: int = 100,
    channel: int = 0,
) -> bytes:
    """Write a one-track Standard MIDI File from melody text (see module doc).

    ``program`` is the General MIDI instrument (0 = piano, 40 = violin,
    73 = flute ...). Channel 9 is the drum kit, where notes pick drums.
    """
    if not 20 <= tempo <= 400:
        raise ValueError("tempo must be 20..400 beats per minute")
    if not 0 <= program <= 127:
        raise ValueError("program must be 0..127")
    if not 1 <= velocity <= 127:
        raise ValueError("velocity must be 1..127")
    if not 0 <= channel <= 15:
        raise ValueError("channel must be 0..15")
    tokens = notes.split()
    if not tokens:
        raise ValueError("notes is empty")

    track = bytearray()
    track += b"\x00\xff\x51\x03" + (60_000_000 // tempo).to_bytes(3, "big")
    track += bytes([0x00, 0xC0 | channel, program])
    pending = 0  # ticks of rest before the next event
    for token in tokens:
        pitch, _, beats_text = token.partition(":")
        try:
            beats = float(beats_text) if beats_text else 1.0
        except ValueError:
            raise ValueError(f"bad length in {token!r}") from None
        if not 0 < beats <= 64:
            raise ValueError(f"length in {token!r} must be above 0 and at most 64 beats")
        ticks = max(1, round(beats * TICKS_PER_BEAT))
        if pitch.upper() == "R":
            pending += ticks
            continue
        chord = [note_number(p) for p in pitch.split("+")]
        for i, n in enumerate(chord):
            track += _vlq(pending if i == 0 else 0) + bytes([0x90 | channel, n, velocity])
        pending = 0
        # Release a little early so repeated notes are heard separately.
        gap = min(ticks // 10, 48)
        for i, n in enumerate(chord):
            track += _vlq(ticks - gap if i == 0 else 0) + bytes([0x80 | channel, n, 0])
        pending = gap
    track += _vlq(pending) + b"\xff\x2f\x00"

    header = b"MThd" + (6).to_bytes(4, "big") + (0).to_bytes(2, "big") + (1).to_bytes(2, "big")
    header += TICKS_PER_BEAT.to_bytes(2, "big")
    return header + b"MTrk" + len(track).to_bytes(4, "big") + bytes(track)


def check_midi_file(data: bytes) -> None:
    if len(data) < 14 or data[:4] != b"MThd":
        raise ValueError("not a Standard MIDI File (no MThd header)")
    if len(data) > MAX_MIDI_BYTES:
        raise ValueError(f"MIDI file is larger than {MAX_MIDI_BYTES // 1024} KB")


async def load_midi_bytes(source: str) -> bytes:
    """Read a MIDI file from a path, an http(s) URL or base64 / data: URL."""
    source = source.strip()
    if not source:
        raise ValueError("midi source is empty")
    if source.startswith("data:"):
        data = base64.b64decode(source.partition(",")[2], validate=False)
    elif source.startswith(("http://", "https://")):
        import aiohttp

        timeout = aiohttp.ClientTimeout(total=20)
        async with aiohttp.ClientSession(timeout=timeout) as session:
            async with session.get(source) as resp:
                if resp.status != 200:
                    raise ValueError(f"download failed: HTTP {resp.status}")
                data = await resp.content.read(MAX_MIDI_BYTES + 1)
    else:
        path = os.path.expanduser(source)
        if os.path.isfile(path):
            if os.path.getsize(path) > MAX_MIDI_BYTES:
                raise ValueError(f"MIDI file is larger than {MAX_MIDI_BYTES // 1024} KB")
            data = await asyncio.to_thread(lambda: open(path, "rb").read())
        else:
            try:
                data = base64.b64decode(source, validate=True)
            except ValueError as exc:
                raise ValueError("midi must be a file path, an http(s) URL or base64 data") from exc
    check_midi_file(data)
    return data


def parse_send_bytes(hex_text: str) -> bytes:
    """Validate midi_send hex ("90 3C 64 ...") and return the bytes."""
    cleaned = re.sub(r"[\s,:]", "", hex_text)
    if not cleaned or len(cleaned) % 2 or not re.fullmatch(r"[0-9a-fA-F]+", cleaned):
        raise ValueError("hex must be pairs of hex digits, e.g. '90 3C 64'")
    data = bytes.fromhex(cleaned)
    if len(data) > 4096:
        raise ValueError("at most 4096 bytes per midi_send")
    return data
