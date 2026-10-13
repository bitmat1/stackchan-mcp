"""Head pitch limits shared by every tool, gesture and the web panel.

Defaults are the official kit's recommended 5..85 degrees. A robot with
a different mechanical range (the RT ver. head here travels 50..90) sets
``STACKCHAN_PITCH_MIN`` / ``STACKCHAN_PITCH_MAX`` in ``.env``; the
firmware's own clamp is configured to match
(``CONFIG_STACKCHAN_PITCH_MIN`` / ``_MAX``).
"""

from __future__ import annotations

import os

DEFAULT_PITCH_MIN = 5
DEFAULT_PITCH_MAX = 85
YAW_MIN = -90
YAW_MAX = 90


def _env_int(name: str, default: int) -> int:
    raw = os.getenv(name, "").strip()
    try:
        return int(raw) if raw else default
    except ValueError:
        return default


def pitch_min() -> int:
    return _env_int("STACKCHAN_PITCH_MIN", DEFAULT_PITCH_MIN)


def pitch_max() -> int:
    return _env_int("STACKCHAN_PITCH_MAX", DEFAULT_PITCH_MAX)


def pitch_mid() -> int:
    return (pitch_min() + pitch_max()) // 2


def clamp_pitch(value: float) -> float:
    return max(pitch_min(), min(pitch_max(), value))
