"""Avatar face names the connected firmware understands.

Stock firmware has six faces. The RT build (and any build with
CONFIG_STACKCHAN_PROCEDURAL_FACE) draws the face procedurally and adds
21 more. ``STACKCHAN_EXTENDED_FACES=0`` limits the gateway to the stock
six, for devices still running the image-based avatar; unknown faces are
rejected by the device either way.
"""

from __future__ import annotations

import json
import logging
import os
from pathlib import Path

logger = logging.getLogger(__name__)

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


FACE_STYLES: tuple[str, ...] = ("stackchan", "bender", "hal")
_STATE_PATH = Path.home() / ".stackchan" / "face.json"

_face = "idle"
_style = "stackchan"


def _load_style() -> None:
    """The robot keeps its style across restarts, so the gateway does too."""
    global _style
    try:
        saved = json.loads(_STATE_PATH.read_text()).get("style")
    except (OSError, ValueError, AttributeError):
        return
    if saved in FACE_STYLES:
        _style = saved


def note_device_call(name: str, arguments: dict) -> None:
    """Remember the face sent to the robot; the firmware cannot report it."""
    global _face, _style
    if name == "self.display.set_avatar":
        face = arguments.get("face")
        if face in BASE_FACES + EXTENDED_FACES:
            _face = face
    elif name == "self.display.set_face_style":
        style = arguments.get("style")
        if style in FACE_STYLES:
            _style = style
            try:
                _STATE_PATH.parent.mkdir(parents=True, exist_ok=True)
                _STATE_PATH.write_text(json.dumps({"style": style}))
            except OSError as exc:
                logger.warning("could not save the face style: %s", exc)


def current_face() -> dict[str, str]:
    return {"face": _face, "style": _style}


_load_style()
