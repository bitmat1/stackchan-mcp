"""Kokoro engine — local neural TTS via ``kokoro-onnx``.

Kokoro-82M runs on the CPU through ONNX Runtime and sounds far more
natural than the macOS system voices, while still keeping all audio on
this machine. It needs the ``[tts-kokoro]`` extra and two model files
(about 340 MB in total) from the kokoro-onnx releases page:

    kokoro-v1.0.onnx  and  voices-v1.0.bin
    https://github.com/thewh1teagle/kokoro-onnx/releases

Configuration (environment variables):

    STACKCHAN_KOKORO_MODEL_DIR
        Directory holding the two model files.
        Default ``~/.cache/stackchan-mcp/kokoro``.

    STACKCHAN_KOKORO_VOICE
        Voice used when the say() tool does not pass speaker_name.
        Default "bf_emma" (British English, female). Other examples:
        "af_heart", "am_michael", "bm_george".

    STACKCHAN_KOKORO_LANG
        Language code passed to Kokoro. Default "en-gb".

    STACKCHAN_KOKORO_SPEED
        Speaking speed multiplier. Default 1.0.

The model is loaded on first use and kept in memory, so the first say()
after gateway start takes a few seconds longer than the rest.
"""

from __future__ import annotations

import asyncio
import logging
import os
from pathlib import Path
from typing import Any

from .audio_utils import DEVICE_SAMPLE_RATE, resample_pcm16_linear
from .base import TTSEngine

logger = logging.getLogger(__name__)

DEFAULT_KOKORO_MODEL_DIR = Path.home() / ".cache" / "stackchan-mcp" / "kokoro"
MODEL_FILENAME = "kokoro-v1.0.onnx"
VOICES_FILENAME = "voices-v1.0.bin"
DEFAULT_KOKORO_VOICE = "bf_emma"
DEFAULT_KOKORO_LANG = "en-gb"


def _model_dir() -> Path:
    raw = os.getenv("STACKCHAN_KOKORO_MODEL_DIR")
    return Path(raw).expanduser() if raw and raw.strip() else DEFAULT_KOKORO_MODEL_DIR


class KokoroEngine(TTSEngine):
    """Synthesise text with Kokoro-82M, return 16 kHz mono PCM."""

    name = "kokoro"

    def __init__(self) -> None:
        # Import here so a missing extra skips registration cleanly.
        import kokoro_onnx  # noqa: F401
        import numpy  # noqa: F401

        self._kokoro: Any = None
        self._load_lock = asyncio.Lock()

    async def _ensure_loaded(self) -> Any:
        async with self._load_lock:
            if self._kokoro is None:
                from kokoro_onnx import Kokoro

                model_dir = _model_dir()
                model = model_dir / MODEL_FILENAME
                voices = model_dir / VOICES_FILENAME
                missing = [p.name for p in (model, voices) if not p.exists()]
                if missing:
                    raise RuntimeError(
                        f"Kokoro model files missing in {model_dir}: {', '.join(missing)}. "
                        "Download them from "
                        "https://github.com/thewh1teagle/kokoro-onnx/releases "
                        "or set STACKCHAN_KOKORO_MODEL_DIR."
                    )
                self._kokoro = await asyncio.to_thread(Kokoro, str(model), str(voices))
                logger.info("kokoro: loaded model from %s", model_dir)
            return self._kokoro

    async def synthesize(self, text: str, **opts: Any) -> bytes:
        """Run Kokoro and return 16 kHz mono signed-16-bit PCM.

        Recognised opts:

            speaker_name: str
                Kokoro voice id (e.g. "bf_emma"). Falls back to
                STACKCHAN_KOKORO_VOICE, then "bf_emma".
            speaker_id: int | None
                Ignored; accepted for a uniform orchestrator argument set.
        """
        if not text:
            raise ValueError("text must not be empty")

        import numpy as np

        speaker_name = opts.get("speaker_name")
        voice = (
            speaker_name
            if isinstance(speaker_name, str) and speaker_name
            else os.getenv("STACKCHAN_KOKORO_VOICE") or DEFAULT_KOKORO_VOICE
        )
        lang = os.getenv("STACKCHAN_KOKORO_LANG") or DEFAULT_KOKORO_LANG
        speed = float(os.getenv("STACKCHAN_KOKORO_SPEED") or 1.0)

        kokoro = await self._ensure_loaded()
        samples, sample_rate = await asyncio.to_thread(
            kokoro.create, text, voice=voice, speed=speed, lang=lang
        )
        pcm = (np.clip(samples, -1.0, 1.0) * 32767).astype("<i2").tobytes()
        pcm = resample_pcm16_linear(pcm, int(sample_rate), DEVICE_SAMPLE_RATE)
        logger.info(
            "kokoro synthesised %d bytes PCM for voice=%s, text=%r",
            len(pcm),
            voice,
            text[:60],
        )
        return pcm
