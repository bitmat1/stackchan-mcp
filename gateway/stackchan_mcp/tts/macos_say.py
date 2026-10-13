"""macOS ``say`` engine — fully local speech synthesis, no extras needed.

Runs Apple's built-in ``/usr/bin/say`` with ``--data-format=LEI16@16000``
so it writes 16 kHz mono signed-16-bit WAV directly, which is exactly the
PCM shape the orchestrator expects. Nothing leaves the machine and no
model download or Python dependency is involved.

Voice quality depends on the voices installed in System Settings >
Accessibility > Spoken Content > System Voice > Manage Voices. The
"(Enhanced)" / "(Premium)" variants sound much better than the defaults;
list what is installed with ``say -v '?'``.

Configuration (environment variables):

    STACKCHAN_MACOS_SAY_VOICE
        Voice used when the say() tool does not pass speaker_name.
        Default "Daniel" (British English, installed on every Mac).
        Read lazily on each call, like the edge-tts engine.

    STACKCHAN_MACOS_SAY_RATE
        Optional speaking rate in words per minute (``say -r``).
"""

from __future__ import annotations

import asyncio
import logging
import os
import sys
import tempfile
from pathlib import Path
from typing import Any

from .audio_utils import DEVICE_SAMPLE_RATE, wav_to_pcm16_mono
from .base import TTSEngine

logger = logging.getLogger(__name__)

#: Default voice: British English, present on every macOS install.
DEFAULT_MACOS_SAY_VOICE = "Daniel"

#: Timeout for the say subprocess.
DEFAULT_SUBPROCESS_TIMEOUT_SECONDS = 30.0


class MacOSSayEngine(TTSEngine):
    """Synthesise text with macOS ``say``, return 16 kHz mono PCM."""

    name = "macos-say"

    def __init__(
        self,
        default_voice: str | None = None,
        timeout_seconds: float = DEFAULT_SUBPROCESS_TIMEOUT_SECONDS,
        say_binary: str | None = None,
    ) -> None:
        self._default_voice_override = default_voice
        self._timeout_seconds = timeout_seconds
        self._say_binary = say_binary or "/usr/bin/say"

    @property
    def default_voice(self) -> str:
        """Constructor override, else STACKCHAN_MACOS_SAY_VOICE, else Daniel."""
        if self._default_voice_override is not None:
            return self._default_voice_override
        return os.getenv("STACKCHAN_MACOS_SAY_VOICE") or DEFAULT_MACOS_SAY_VOICE

    async def synthesize(self, text: str, **opts: Any) -> bytes:
        """Run ``say`` and return 16 kHz mono signed-16-bit PCM.

        Recognised opts:

            speaker_name: str
                macOS voice name (e.g. "Daniel", "Flo (English (UK))").
                Falls back to ``default_voice``.
            speaker_id: int | None
                Ignored; accepted for a uniform orchestrator argument set.
        """
        if not text:
            raise ValueError("text must not be empty")

        speaker_name = opts.get("speaker_name")
        voice = (
            speaker_name
            if isinstance(speaker_name, str) and speaker_name
            else self.default_voice
        )

        with tempfile.TemporaryDirectory() as tmpdir:
            wav_path = Path(tmpdir) / "out.wav"
            args = [
                self._say_binary,
                "-v", voice,
                "--file-format=WAVE",
                f"--data-format=LEI16@{DEVICE_SAMPLE_RATE}",
                "-o", str(wav_path),
                "-f", "-",  # read text from stdin, so no argv quoting issues
            ]
            rate = os.getenv("STACKCHAN_MACOS_SAY_RATE")
            if rate and rate.strip():
                args[1:1] = ["-r", rate.strip()]

            proc = await asyncio.create_subprocess_exec(
                *args,
                stdin=asyncio.subprocess.PIPE,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE,
            )
            _, stderr = await asyncio.wait_for(
                proc.communicate(text.encode("utf-8")), timeout=self._timeout_seconds
            )
            if proc.returncode != 0:
                raise RuntimeError(
                    f"say failed (code {proc.returncode}): "
                    f"{stderr.decode(errors='replace')}"
                )
            sample_rate, pcm = wav_to_pcm16_mono(wav_path.read_bytes())

        if sample_rate != DEVICE_SAMPLE_RATE:
            raise RuntimeError(
                f"say produced {sample_rate} Hz audio, expected {DEVICE_SAMPLE_RATE} Hz"
            )
        logger.info(
            "macos-say synthesised %d bytes PCM for voice=%s, text=%r",
            len(pcm),
            voice,
            text[:60],
        )
        return pcm


def is_available() -> bool:
    """True on macOS with the ``say`` binary present."""
    return sys.platform == "darwin" and Path("/usr/bin/say").exists()
