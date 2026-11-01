"""Record from the robot's microphone and measure the pitch of a held note.

``record_audio`` is the MCP tool behind ``scripts/tuning_test.py``: it saves
the microphone to a WAV file instead of transcribing it. The rest of the
module is plain analysis: ``analyse`` turns one recording into a frequency,
a note name and a cents offset, and ``assess_patch`` decides from the first
few presses whether a synth patch is steady enough to test.
"""

from __future__ import annotations

import asyncio
import logging
import os
import statistics
import tempfile
import time
import wave
from contextlib import nullcontext
from dataclasses import asdict, dataclass
from typing import TYPE_CHECKING, Any

from .audio_stream import is_recording, recording_owner, start_recording, stop_recording
from .stt.audio_utils import DEVICE_FRAME_DURATION_MS, DEVICE_SAMPLE_RATE, decode_opus_frames

if TYPE_CHECKING:
    from .gateway import Gateway

logger = logging.getLogger(__name__)

MIN_RECORD_MS = 100
MAX_RECORD_MS = 30000
LISTEN_START_DELAY_S = 0.05

NOTE_NAMES = ("C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B")

# 0.4 s resolves the partials of notes down to about 40 Hz.
SEGMENT_S = 0.4
HOP_S = 0.2
MIN_HZ = 40.0
MAX_HZ = 2000.0
MAX_HARMONIC = 10
MIN_RMS = 0.002  # About -54 dBFS.
# The microphone takes a moment to settle once recording starts.
START_SKIP_S = 0.15
MIN_CLARITY = 0.85
MIN_VOICED_FRACTION = 0.6
MAX_WITHIN_NOTE_CENTS = 30.0
# Segments further than this from the take's median are octave errors.
OUTLIER_CENTS = 50.0
TRIM_FRACTION = 0.1


async def record_audio(arguments: dict[str, Any], *, gateway: "Gateway") -> dict[str, Any]:
    """Capture ``duration_ms`` of microphone audio to a 16 kHz mono WAV file."""
    duration_ms = arguments.get("duration_ms", 1000)
    if isinstance(duration_ms, bool) or not isinstance(duration_ms, int):
        raise ValueError(f"'duration_ms' must be an integer; got {duration_ms!r}")
    if not MIN_RECORD_MS <= duration_ms <= MAX_RECORD_MS:
        raise ValueError(f"'duration_ms' must be between {MIN_RECORD_MS} and {MAX_RECORD_MS}")
    if not gateway.esp32.device_connected:
        raise RuntimeError("No ESP32 device connected; cannot record.")
    if getattr(gateway.esp32.connection, "protocol_version", 1) != 1:
        raise RuntimeError("record_audio requires WebSocket protocol v1.")

    listen_lock = getattr(gateway.esp32, "listen_lock", None)
    async with listen_lock if listen_lock is not None else nullcontext():
        if is_recording():
            raise RuntimeError(f"microphone is busy ({recording_owner()})")
        session_id = getattr(gateway.esp32.connection, "session_id", "")
        start_recording(session_id, owner="record_audio")
        listen_started = False
        try:
            # The raw profile skips the voice processing, which treats a held
            # note as noise and removes it.
            await gateway.esp32.send_listen_state("start", mode="manual", profile="raw")
            listen_started = True
            await asyncio.sleep(LISTEN_START_DELAY_S + duration_ms / 1000.0)
        except ConnectionError as exc:
            raise RuntimeError(f"Device disconnected while recording: {exc}") from exc
        finally:
            if listen_started:
                try:
                    await asyncio.shield(gateway.esp32.send_listen_state("stop"))
                except (ConnectionError, asyncio.CancelledError):
                    logger.warning("listen.stop did not reach the device")
            frames = stop_recording()

    pcm = await asyncio.to_thread(decode_opus_frames, frames) if frames else b""
    path = str(arguments.get("path") or "") or os.path.join(
        tempfile.gettempdir(), f"stackchan-record-{time.time_ns()}.wav"
    )
    with wave.open(path, "wb") as wav:
        wav.setnchannels(1)
        wav.setsampwidth(2)
        wav.setframerate(DEVICE_SAMPLE_RATE)
        wav.writeframes(pcm)
    return {
        "ok": True,
        "path": path,
        "duration_ms": len(frames) * DEVICE_FRAME_DURATION_MS,
        "frame_count": len(frames),
        "sample_rate": DEVICE_SAMPLE_RATE,
    }


@dataclass
class PitchReading:
    """One note's pitch. ``valid`` means it can be trusted for tuning."""

    frequency: float
    midi: int
    note: str
    cents: float
    spread_cents: float
    clarity: float
    voiced_fraction: float
    rms: float
    valid: bool

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def note_for_frequency(frequency: float, a4: float = 440.0) -> tuple[int, str, float]:
    """Nearest MIDI note, its name (C4 = 60) and the offset from it in cents."""
    import numpy as np

    exact = 69.0 + 12.0 * float(np.log2(frequency / a4))
    midi = int(round(exact))
    return midi, f"{NOTE_NAMES[midi % 12]}{midi // 12 - 1}", (exact - midi) * 100.0


def _coarse_pitch(segment: Any, sample_rate: int) -> tuple[float, float]:
    """McLeod-style normalised autocorrelation: (frequency, clarity)."""
    import numpy as np

    n = len(segment)
    spectrum = np.fft.rfft(segment, 2 * n)
    acf = np.fft.irfft(spectrum * np.conj(spectrum))[:n]
    squares = np.cumsum(segment**2)
    # m(tau) = sum of x[i]^2 + x[i+tau]^2 over the overlap.
    m = (squares[-1] - np.concatenate(([0.0], squares[:-1]))) + squares[::-1]
    nsdf = 2.0 * acf / np.maximum(m, 1e-12)

    low = max(2, int(sample_rate / MAX_HZ))
    high = min(n - 2, int(sample_rate / MIN_HZ))
    peaks = [
        tau
        for tau in range(low, high)
        if nsdf[tau] > nsdf[tau - 1] and nsdf[tau] >= nsdf[tau + 1] and nsdf[tau] > 0
    ]
    if not peaks:
        return 0.0, 0.0
    best = max(float(nsdf[tau]) for tau in peaks)
    tau = next(t for t in peaks if nsdf[t] >= 0.9 * best)
    a, b, c = nsdf[tau - 1], nsdf[tau], nsdf[tau + 1]
    denominator = a - 2 * b + c
    offset = 0.5 * (a - c) / denominator if denominator else 0.0
    return sample_rate / (tau + offset), float(b)


def _refine_pitch(segment: Any, sample_rate: int, coarse: float) -> float:
    """Sharpen a pitch estimate from the spectral peaks of its harmonics."""
    import numpy as np

    n = len(segment)
    size = n * 4
    log_mag = np.log(np.abs(np.fft.rfft(segment * np.hanning(n), size)) + 1e-12)
    hz_per_bin = sample_rate / size
    floor = float(log_mag.max()) - 5.0  # Partials within about 43 dB of the peak.

    estimate = coarse
    total, weight_sum = 0.0, 0.0
    for harmonic in range(1, MAX_HARMONIC + 1):
        target = estimate * harmonic
        if target >= sample_rate / 2 * 0.9:
            break
        width = max(2, int(target * 0.02 / hz_per_bin))
        centre = int(round(target / hz_per_bin))
        low, high = max(1, centre - width), min(len(log_mag) - 2, centre + width)
        if high <= low:
            break
        peak = low + int(np.argmax(log_mag[low : high + 1]))
        if log_mag[peak] < floor or peak in (low, high):
            continue
        a, b, c = log_mag[peak - 1], log_mag[peak], log_mag[peak + 1]
        denominator = a - 2 * b + c
        offset = 0.5 * (a - c) / denominator if denominator else 0.0
        weight = float(np.exp(b)) * harmonic
        total += (peak + offset) * hz_per_bin / harmonic * weight
        weight_sum += weight
        estimate = total / weight_sum
    return estimate


def analyse(pcm: bytes, sample_rate: int = DEVICE_SAMPLE_RATE, a4: float = 440.0) -> PitchReading | None:
    """Measure the note in 16-bit mono PCM, or None when no pitch is found."""
    import numpy as np

    samples = np.frombuffer(pcm, dtype="<i2").astype(np.float64) / 32768.0
    samples = samples[int(START_SKIP_S * sample_rate) :]
    size = int(SEGMENT_S * sample_rate)
    hop = int(HOP_S * sample_rate)
    if len(samples) < size:
        return None
    samples = samples - samples.mean()
    rms = float(np.sqrt(np.mean(samples**2)))

    starts = range(0, len(samples) - size + 1, hop)
    frequencies: list[float] = []
    clarities: list[float] = []
    for start in starts:
        segment = samples[start : start + size]
        if np.sqrt(np.mean(segment**2)) < MIN_RMS:
            continue
        coarse, clarity = _coarse_pitch(segment, sample_rate)
        if coarse <= 0 or clarity < MIN_CLARITY:
            continue
        frequencies.append(_refine_pitch(segment, sample_rate, coarse))
        clarities.append(clarity)
    if not frequencies:
        return None

    # median_low is always a real segment, so a take split evenly between two
    # pitches keeps one of them rather than a midpoint that rejects both.
    centre = statistics.median_low(frequencies)
    offsets = sorted(1200.0 * float(np.log2(f / centre)) for f in frequencies)
    offsets = [o for o in offsets if abs(o) <= OUTLIER_CENTS]
    # A trimmed mean over the whole take averages out slow movement in a
    # patch better than one segment's pitch can.
    trim = int(len(offsets) * TRIM_FRACTION)
    frequency = centre * 2 ** (statistics.fmean(offsets[trim : len(offsets) - trim]) / 1200.0)
    spread = offsets[-1] - offsets[0] if len(offsets) < 5 else float(
        np.percentile(offsets, 90) - np.percentile(offsets, 10)
    )
    voiced = len(offsets) / len(starts)
    clarity = statistics.median(clarities)
    midi, note, cents = note_for_frequency(frequency, a4)
    return PitchReading(
        frequency=round(float(frequency), 3),
        midi=midi,
        note=note,
        cents=round(cents, 2),
        spread_cents=round(spread, 2),
        clarity=round(clarity, 3),
        voiced_fraction=round(voiced, 2),
        rms=round(rms, 5),
        valid=voiced >= MIN_VOICED_FRACTION and clarity >= MIN_CLARITY and spread <= MAX_WITHIN_NOTE_CENTS,
    )


def read_wav(path: str) -> tuple[bytes, int]:
    with wave.open(path, "rb") as wav:
        return wav.readframes(wav.getnframes()), wav.getframerate()


def relative_cents(frequencies: list[float]) -> list[float]:
    """Each frequency's offset in cents from the median of them all."""
    import numpy as np

    centre = statistics.median(frequencies)
    return [1200.0 * float(np.log2(f / centre)) for f in frequencies]


def assess_patch(
    readings: list[PitchReading | None],
    *,
    min_valid: int = 3,
    max_deviation_cents: float = 15.0,
) -> tuple[bool, str]:
    """Is this patch testable, judging by its first presses?

    The median deviation ignores a single wild press, so the very fault
    being hunted (a sudden drop) does not get a patch skipped.
    """
    good = [r for r in readings if r is not None and r.valid]
    if len(good) < min_valid:
        return False, f"only {len(good)} of {len(readings)} presses had a steady pitch"
    offsets = relative_cents([r.frequency for r in good])
    deviation = statistics.median(abs(o) for o in offsets)
    if deviation > max_deviation_cents:
        return False, f"pitch moves {deviation:.0f} cents between presses"
    return True, "ok"


def summarise(readings: list[PitchReading | None], *, alert_cents: float = 20.0) -> dict[str, Any]:
    """Tuning statistics for one patch's presses."""
    good = [r for r in readings if r is not None and r.valid]
    result: dict[str, Any] = {"presses": len(readings), "valid": len(good)}
    if not good:
        return result
    frequencies = [r.frequency for r in good]
    offsets = relative_cents(frequencies)
    centre = statistics.median(frequencies)
    midi, note, cents = note_for_frequency(centre)
    mean_offset = statistics.fmean(offsets)
    deviation = statistics.stdev(offsets) if len(offsets) > 1 else 0.0
    result.update(
        median_hz=round(float(centre), 3),
        note=note,
        midi=midi,
        cents=round(cents, 2),
        mean_cents=round(cents + mean_offset, 2),
        stderr_cents=round(deviation / len(offsets) ** 0.5, 2),
        stdev_cents=round(deviation, 2),
        lowest_cents=round(min(offsets), 2),
        highest_cents=round(max(offsets), 2),
        outliers=sum(1 for o in offsets if abs(o) >= alert_cents),
    )
    return result
