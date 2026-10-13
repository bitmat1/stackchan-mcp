"""Avatar face names the connected firmware understands.

Stock firmware has six faces. The RT build (and any build with
CONFIG_STACKCHAN_PROCEDURAL_FACE) draws the face procedurally and adds
21 more. ``STACKCHAN_EXTENDED_FACES=0`` limits the gateway to the stock
six, for devices still running the image-based avatar; unknown faces are
rejected by the device either way.
"""

from __future__ import annotations

import os

BASE_FACES: tuple[str, ...] = (
    "idle",
    "happy",
    "thinking",
    "sad",
    "surprised",
    "embarrassed",
)

EXTENDED_FACES: tuple[str, ...] = (
    "laughing",
    "excited",
    "love",
    "wink",
    "smug",
    "cool",
    "crying",
    "angry",
    "annoyed",
    "shocked",
    "confused",
    "doubtful",
    "sleepy",
    "bored",
    "nervous",
    "scared",
    "curious",
    "dizzy",
    "relieved",
    "listening",
    "kiss",
)


def extended_faces_enabled() -> bool:
    return os.getenv("STACKCHAN_EXTENDED_FACES", "1").strip().lower() not in {"0", "false", "no", "off"}


def available_faces() -> tuple[str, ...]:
    """Faces to offer to MCP clients, in display order (without "off")."""
    return BASE_FACES + EXTENDED_FACES if extended_faces_enabled() else BASE_FACES
