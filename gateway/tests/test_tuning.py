"""Tests for tuning.py: pitch measurement, patch assessment and record_audio."""

from __future__ import annotations

import wave

import numpy as np
import pytest

from stackchan_mcp import audio_stream, tuning

SR = 16000


def synth(frequency, seconds=2.0, harmonics=8, vibrato_cents=0.0, vibrato_hz=5.0, noise=0.0, seed=1):
    t = np.arange(int(SR * seconds)) / SR
    ratio = 2 ** (vibrato_cents * np.sin(2 * np.pi * vibrato_hz * t) / 1200)
    phase = 2 * np.pi * np.cumsum(frequency * ratio) / SR
    wave_ = sum(np.sin(k * phase) / k for k in range(1, harmonics + 1) if k * frequency < SR / 2)
    wave_ = 0.3 * wave_ / np.max(np.abs(wave_))
    wave_ = wave_ + noise * np.random.default_rng(seed).standard_normal(len(t))
    return (np.clip(wave_, -1, 1) * 32767).astype("<i2").tobytes()


@pytest.mark.parametrize("frequency", [55.0, 110.0, 261.63, 440.0, 880.0, 1760.0])
def test_measures_frequency_within_half_a_cent(frequency):
    reading = tuning.analyse(synth(frequency * 2 ** (7 / 1200)))
    assert reading is not None and reading.valid
    assert 1200 * np.log2(reading.frequency / frequency) == pytest.approx(7, abs=0.5)


def test_names_note_and_cents():
    reading = tuning.analyse(synth(440.0 * 2 ** (-12 / 1200)))
    assert reading.note == "A4"
    assert reading.midi == 69
    assert reading.cents == pytest.approx(-12, abs=0.5)


def test_a_semitone_drop_is_visible():
    reading = tuning.analyse(synth(261.63 * 2 ** (-1 / 12)))
    assert reading.note == "B3"


def test_steady_note_survives_room_noise():
    reading = tuning.analyse(synth(330.0, noise=0.02))
    assert reading.valid
    assert reading.note == "E4"


def test_silence_and_noise_give_no_valid_reading():
    assert tuning.analyse(bytes(SR * 3)) is None
    noise = (np.random.default_rng(2).standard_normal(SR * 2) * 3000).astype("<i2").tobytes()
    reading = tuning.analyse(noise)
    assert reading is None or not reading.valid


def test_heavy_vibrato_is_not_valid():
    reading = tuning.analyse(synth(440.0, seconds=2.0, vibrato_cents=60.0, vibrato_hz=1.5))
    assert reading is None or not reading.valid


def test_octave_errors_in_a_take_are_dropped():
    steady = np.frombuffer(synth(220.0, seconds=4.0), dtype="<i2")
    octave_up = np.frombuffer(synth(440.0, seconds=0.5), dtype="<i2")
    reading = tuning.analyse(np.concatenate((steady, octave_up, steady)).tobytes())
    assert reading.note == "A3"
    assert reading.cents == pytest.approx(0, abs=0.5)
    assert reading.valid


def test_take_split_between_two_pitches_does_not_crash():
    low = np.frombuffer(synth(220.0, seconds=1.0), dtype="<i2")
    high = np.frombuffer(synth(1500.0, seconds=1.0), dtype="<i2")
    reading = tuning.analyse(np.concatenate((low, high)).tobytes())
    assert reading is None or reading.note in {"A3", "F#6"}


def test_slow_movement_averages_out_over_a_take():
    reading = tuning.analyse(synth(440.0, seconds=4.0, vibrato_cents=8.0, vibrato_hz=2.0))
    assert reading.valid
    assert reading.cents == pytest.approx(0, abs=1.0)


def test_too_short_recording():
    assert tuning.analyse(synth(440.0, seconds=0.4)) is None


def test_noise_at_the_start_is_ignored():
    note = np.frombuffer(synth(440.0), dtype="<i2").copy()
    t = np.arange(int(0.12 * SR)) / SR
    note[: len(t)] = (30000 * np.sin(2 * np.pi * 690 * t)).astype("<i2")
    reading = tuning.analyse(note.tobytes())
    assert reading.note == "A4"
    assert reading.cents == pytest.approx(0, abs=0.5)


def _reading(frequency, valid=True):
    midi, note, cents = tuning.note_for_frequency(frequency)
    return tuning.PitchReading(frequency, midi, note, cents, 1.0, 0.99, 1.0, 0.1, valid)


def test_assess_accepts_steady_patch():
    assert tuning.assess_patch([_reading(440 + i * 0.05) for i in range(5)]) == (True, "ok")


def test_assess_keeps_patch_with_one_fault_press():
    readings = [_reading(440.0)] * 4 + [_reading(415.3)]
    assert tuning.assess_patch(readings)[0]


def test_assess_skips_unmeasurable_patch():
    ok, reason = tuning.assess_patch([None, None, _reading(440.0), None, _reading(440.0, valid=False)])
    assert not ok
    assert "1 of 5" in reason


def test_assess_skips_wandering_patch():
    readings = [_reading(440.0 * 2 ** (c / 1200)) for c in (-40, -20, 0, 25, 45)]
    ok, reason = tuning.assess_patch(readings)
    assert not ok
    assert "cents" in reason


def test_summary_counts_the_outlier():
    summary = tuning.summarise([_reading(440.0)] * 9 + [_reading(415.3)] + [None])
    assert summary["presses"] == 11
    assert summary["valid"] == 10
    assert summary["note"] == "A4"
    assert summary["outliers"] == 1
    assert summary["mean_cents"] == pytest.approx(-10, abs=0.2)
    assert summary["stderr_cents"] == pytest.approx(10, abs=0.2)
    assert summary["lowest_cents"] == pytest.approx(-100, abs=0.2)


class _Esp32:
    device_connected = True

    def __init__(self):
        self.connection = type("C", (), {"protocol_version": 1, "session_id": "s"})()
        self.states = []

    async def send_listen_state(self, state, mode="manual", profile="voice"):
        self.states.append((state, profile) if state == "start" else state)
        if state == "start":
            await audio_stream.handle_audio_frame(b"frame", "s")


class _Gateway:
    def __init__(self):
        self.esp32 = _Esp32()


@pytest.mark.asyncio
async def test_record_audio_writes_wav(tmp_path, monkeypatch):
    monkeypatch.setattr(tuning, "decode_opus_frames", lambda frames: b"\x01\x00" * 960)
    path = tmp_path / "take.wav"
    gateway = _Gateway()
    result = await tuning.record_audio({"duration_ms": 100, "path": str(path)}, gateway=gateway)
    assert result["ok"] and result["frame_count"] == 1
    assert gateway.esp32.states == [("start", "raw"), "stop"]
    assert not audio_stream.is_recording()
    with wave.open(str(path)) as wav:
        assert wav.getframerate() == SR
        assert wav.getnframes() == 960


@pytest.mark.asyncio
async def test_record_audio_declines_when_microphone_busy():
    audio_stream.start_recording("other", owner="beat")
    try:
        with pytest.raises(RuntimeError, match="busy"):
            await tuning.record_audio({"duration_ms": 100}, gateway=_Gateway())
    finally:
        audio_stream.stop_recording()


@pytest.mark.asyncio
async def test_record_audio_rejects_bad_duration():
    with pytest.raises(ValueError):
        await tuning.record_audio({"duration_ms": 50}, gateway=_Gateway())
