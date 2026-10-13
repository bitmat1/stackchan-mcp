"""Speech subtitles in the robot's bottom strip.

say() shows the spoken text while the audio plays and clears it after.
On by default; ``STACKCHAN_SUBTITLES=0`` turns them off at start-up and
the ``set_subtitles`` tool toggles them at runtime.
"""

from __future__ import annotations

import logging
import os
from typing import Any

logger = logging.getLogger(__name__)

_override: bool | None = None


def subtitles_enabled() -> bool:
    if _override is not None:
        return _override
    return os.getenv("STACKCHAN_SUBTITLES", "1").strip().lower() not in {"0", "false", "no", "off"}


def set_subtitles_enabled(enabled: bool) -> None:
    global _override
    _override = bool(enabled)


async def show_subtitle(esp32: Any, text: str, duration_ms: int) -> None:
    """Best effort: a failure must never stop the speech itself."""
    try:
        _result, error = await esp32.call_tool(
            "self.display.set_subtitle", {"text": text, "duration_ms": int(duration_ms)}
        )
        if error:
            logger.debug("set_subtitle failed: %s", error)
    except Exception as exc:
        logger.debug("set_subtitle raised: %s", exc)
