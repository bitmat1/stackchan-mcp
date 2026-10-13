"""Named head gestures (nod, shake, tilt ...) built from head moves.

Each gesture is a list of (yaw offset, pitch offset, speed deg/s, hold ms)
steps relative to where the head is when it starts, finishing back at
that pose. Pitch grows upward (45 is roughly level), yaw grows to the
robot's left. Offsets are scaled by ``intensity`` and clamped to the
head's range (yaw -90..90, pitch from :mod:`stackchan_mcp.head_limits`).
"""

from __future__ import annotations

import asyncio
from typing import TYPE_CHECKING, Any

from .head_limits import clamp_pitch, pitch_mid
from .stt.orchestrator import _call_device_tool, _extract_head_angles

if TYPE_CHECKING:
    from .gateway import Gateway

Step = tuple[float, float, int, int]

GESTURES: dict[str, list[Step]] = {
    "nod": [(0, -14, 120, 60), (0, 6, 120, 60), (0, -12, 120, 60), (0, 0, 90, 0)],
    "shake": [(-22, 0, 150, 40), (22, 0, 150, 40), (-18, 0, 150, 40), (0, 0, 120, 0)],
    "tilt": [(14, 10, 60, 900), (0, 0, 60, 0)],
    "look_around": [(-45, 4, 60, 500), (45, 4, 60, 500), (0, 0, 60, 0)],
    "look_up": [(0, 28, 60, 1000), (0, 0, 60, 0)],
    "bow": [(0, -30, 50, 900), (0, 0, 50, 0)],
    "wiggle": [(-10, 4, 200, 0), (10, 4, 200, 0), (-10, 4, 200, 0), (10, 4, 200, 0), (0, 0, 150, 0)],
    "think": [(18, 18, 45, 1500), (0, 0, 45, 0)],
    "sigh": [(0, -18, 30, 700), (0, 0, 40, 0)],
    "startle": [(0, 16, 300, 400), (0, 0, 80, 0)],
}

GESTURE_NAMES = tuple(GESTURES)


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


async def run_gesture(gateway: "Gateway", name: str, intensity: float = 1.0) -> dict[str, Any]:
    steps = GESTURES.get(name)
    if steps is None:
        raise ValueError(f"unknown gesture {name!r}; choose from {', '.join(GESTURE_NAMES)}")
    intensity = _clamp(float(intensity), 0.2, 2.0)

    try:
        yaw0, pitch0 = _extract_head_angles(
            await _call_device_tool(gateway, "self.robot.get_head_angles", {})
        )
    except (RuntimeError, ValueError):
        yaw0, pitch0 = 0.0, float(pitch_mid())
    yaw0 = _clamp(yaw0, -90, 90)
    pitch0 = clamp_pitch(pitch0)

    yaw, pitch = yaw0, pitch0
    for dyaw, dpitch, dps, hold_ms in steps:
        target_yaw = round(_clamp(yaw0 + dyaw * intensity, -90, 90))
        target_pitch = round(clamp_pitch(pitch0 + dpitch * intensity))
        await _call_device_tool(
            gateway,
            "self.robot.set_head_angles",
            {"yaw": target_yaw, "pitch": target_pitch, "speed_dps": dps},
        )
        travel = max(abs(target_yaw - yaw), abs(target_pitch - pitch))
        await asyncio.sleep(travel / dps + hold_ms / 1000)
        yaw, pitch = target_yaw, target_pitch
    return {"ok": True, "gesture": name, "intensity": intensity}
