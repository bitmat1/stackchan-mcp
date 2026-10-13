"""Emoji-driven avatar expression helpers for ``say`` text."""

from __future__ import annotations

import re


EMOJI_FACE_GROUPS: dict[str, tuple[str, ...]] = {
    "happy": ("😊", "😄", "😀", "😁", "🙂", "😆", "🥰", "😍", "😋", "🤗"),
    "sad": ("😢", "😭", "😞", "😔", "☹️", "🙁", "😿"),
    "surprised": ("😲", "😮", "😯", "😱", "🤯"),
    "embarrassed": ("😳", "😅", "🫣"),
    "thinking": ("🤔", "🧐", "💭"),
}

#: Extra mappings for firmware with the procedural face (see
#: :mod:`stackchan_mcp.faces`). They take precedence over the groups above,
#: so e.g. 😍 shows "love" instead of the nearest stock face "happy".
EXTENDED_EMOJI_FACE_GROUPS: dict[str, tuple[str, ...]] = {
    "laughing": ("😂", "🤣", "😆", "😹"),
    "excited": ("🤩", "🥳", "🎉", "✨"),
    "love": ("😍", "🥰", "❤️", "💕", "💖", "😻"),
    "kiss": ("😘", "😚", "😗", "😙", "💋"),
    "wink": ("😉", "😜"),
    "smug": ("😏",),
    "cool": ("😎",),
    "crying": ("😭",),
    "angry": ("😠", "😡", "🤬", "💢"),
    "annoyed": ("😤", "😒", "🙄"),
    "shocked": ("😱", "🤯", "😨"),
    "confused": ("😕", "😵‍💫", "❓"),
    "doubtful": ("🤨",),
    "sleepy": ("😴", "😪", "🥱", "💤"),
    "bored": ("😑", "😐", "😶"),
    "nervous": ("😬", "😰", "😓", "😅"),
    "scared": ("😧", "😦", "🫨"),
    "curious": ("🧐", "👀"),
    "dizzy": ("😵", "🌀"),
    "relieved": ("😌", "😮‍💨"),
    "listening": ("👂",),
}


def _build_map(*groups: dict[str, tuple[str, ...]]) -> dict[str, str]:
    mapping: dict[str, str] = {}
    for group in groups:
        for face, emojis in group.items():
            for emoji in emojis:
                mapping[emoji] = face
    return mapping


_EMOJI_TO_FACE: dict[str, str] = _build_map(EMOJI_FACE_GROUPS)
_EMOJI_TO_FACE_EXTENDED: dict[str, str] = _build_map(
    EMOJI_FACE_GROUPS, EXTENDED_EMOJI_FACE_GROUPS
)
_MAPPED_EMOJIS = tuple(
    sorted(_EMOJI_TO_FACE, key=lambda emoji: len(emoji), reverse=True)
)
_MAPPED_EMOJIS_EXTENDED = tuple(
    sorted(_EMOJI_TO_FACE_EXTENDED, key=lambda emoji: len(emoji), reverse=True)
)

_EMOJI_BASE = (
    "["
    "\u00a9\u00ae"
    "\u2600-\u27bf"
    "\U0001f000-\U0001faff"
    "]"
)
_EMOJI_MODIFIER = "[\ufe0f\U0001f3fb-\U0001f3ff]*"
_EMOJI_SEQUENCE_RE = re.compile(
    f"(?:[0-9#*]\ufe0f?\u20e3|"
    f"{_EMOJI_BASE}{_EMOJI_MODIFIER}"
    f"(?:\u200d{_EMOJI_BASE}{_EMOJI_MODIFIER})*)"
)
_EMOJI_RESIDUE_RE = re.compile("[\ufe0f\u200d\U0001f3fb-\U0001f3ff\u20e3]")
_WHITESPACE_RE = re.compile(r"\s+")


def detect_emoji_face(text: str) -> str | None:
    """Return the face mapped from the first supported emoji in ``text``."""
    from ..faces import extended_faces_enabled

    extended = extended_faces_enabled()
    mapped = _MAPPED_EMOJIS_EXTENDED if extended else _MAPPED_EMOJIS
    emoji_to_face = _EMOJI_TO_FACE_EXTENDED if extended else _EMOJI_TO_FACE
    best_index: int | None = None
    best_emoji: str | None = None

    for emoji in mapped:
        index = text.find(emoji)
        if index < 0:
            continue
        if (
            best_index is None
            or index < best_index
            or (
                index == best_index
                and best_emoji is not None
                and len(emoji) > len(best_emoji)
            )
        ):
            best_index = index
            best_emoji = emoji

    if best_emoji is None:
        return None
    return emoji_to_face[best_emoji]


def contains_emoji(text: str) -> bool:
    """Return whether ``text`` contains any Unicode emoji-like sequence."""
    return _EMOJI_SEQUENCE_RE.search(text) is not None


def strip_emoji_for_plain_tts(text: str) -> str:
    """Remove emoji for engines that do not interpret them as style cues."""
    if not contains_emoji(text):
        return text

    stripped = _EMOJI_SEQUENCE_RE.sub(" ", text)
    stripped = _EMOJI_RESIDUE_RE.sub(" ", stripped)
    return _WHITESPACE_RE.sub(" ", stripped).strip()
