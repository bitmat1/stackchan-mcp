"""Convert pictures for the robot's 320x240 screen (show_image tool).

Accepts a local file path, an http(s) URL or base64 (optionally a
``data:image/...;base64,`` URL), and returns raw RGB565 little-endian
pixels, the format the firmware's ``self.display.show_image`` expects.
Requires Pillow (``[screen]`` extra).
"""

from __future__ import annotations

import asyncio
import base64
import io
import os
from typing import Literal

SCREEN_WIDTH = 320
SCREEN_HEIGHT = 240
SCREEN_BYTES = SCREEN_WIDTH * SCREEN_HEIGHT * 2
MAX_SOURCE_BYTES = 20 * 1024 * 1024

Fit = Literal["fit", "fill"]


async def load_image_bytes(source: str) -> bytes:
    """Read the encoded picture behind ``source`` (path, URL or base64)."""
    source = source.strip()
    if not source:
        raise ValueError("image source is empty")
    if source.startswith("data:"):
        _, _, payload = source.partition(",")
        return base64.b64decode(payload, validate=False)
    if source.startswith(("http://", "https://")):
        import aiohttp

        timeout = aiohttp.ClientTimeout(total=20)
        async with aiohttp.ClientSession(timeout=timeout) as session:
            async with session.get(source) as resp:
                if resp.status != 200:
                    raise ValueError(f"download failed: HTTP {resp.status}")
                data = await resp.content.read(MAX_SOURCE_BYTES + 1)
                if len(data) > MAX_SOURCE_BYTES:
                    raise ValueError("image is larger than 20 MB")
                return data
    path = os.path.expanduser(source)
    if os.path.isfile(path):
        if os.path.getsize(path) > MAX_SOURCE_BYTES:
            raise ValueError("image is larger than 20 MB")
        return await asyncio.to_thread(lambda: open(path, "rb").read())
    try:
        return base64.b64decode(source, validate=True)
    except ValueError as exc:
        raise ValueError(
            "image must be a file path, an http(s) URL or base64 data"
        ) from exc


def to_rgb565(encoded: bytes, fit: Fit = "fit", background: tuple[int, int, int] = (0, 0, 0)) -> bytes:
    """Decode, scale to 320x240 and pack as RGB565 little-endian.

    ``fit`` letterboxes the whole picture onto ``background``; ``fill``
    covers the screen and crops the overflow.
    """
    try:
        from PIL import Image, ImageOps
    except ImportError as exc:  # pragma: no cover - depends on extras
        raise RuntimeError(
            "show_image needs Pillow: uv sync --extra screen"
        ) from exc

    with Image.open(io.BytesIO(encoded)) as img:
        img = ImageOps.exif_transpose(img).convert("RGB")
        size = (SCREEN_WIDTH, SCREEN_HEIGHT)
        if fit == "fill":
            frame = ImageOps.fit(img, size, Image.LANCZOS)
        else:
            frame = Image.new("RGB", size, background)
            scaled = ImageOps.contain(img, size, Image.LANCZOS)
            frame.paste(
                scaled,
                ((SCREEN_WIDTH - scaled.width) // 2, (SCREEN_HEIGHT - scaled.height) // 2),
            )

    rgb = frame.tobytes()
    out = bytearray(SCREEN_BYTES)
    for i in range(0, len(rgb), 3):
        r, g, b = rgb[i], rgb[i + 1], rgb[i + 2]
        v = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)
        j = i // 3 * 2
        out[j] = v & 0xFF
        out[j + 1] = v >> 8
    return bytes(out)


#: The robot keeps the whole GIF in PSRAM; stay well below its 3 MB limit.
MAX_GIF_BYTES = 3 * 1024 * 1024
MAX_GIF_FRAMES = 400


def prepare_gif(encoded: bytes, fit: Fit = "fit") -> tuple[bytes, dict[str, int]]:
    """Re-encode an animated picture as a GIF for the robot's screen.

    Every frame is composited (Pillow applies the GIF's disposal rules),
    scaled to fit (or fill) 320x240 and saved again as a looping GIF.
    Pillow stores only each frame's changed rectangle, which keeps the file
    small and decoding on the robot fast. If the result is still over
    MAX_GIF_BYTES it is retried at 75 % and 50 % size (centred on the
    robot). Returns the GIF bytes and ``{"frames", "width", "height"}``.
    """
    try:
        from PIL import Image, ImageOps, ImageSequence
    except ImportError as exc:  # pragma: no cover - depends on extras
        raise RuntimeError("show_gif needs Pillow: uv sync --extra screen") from exc

    with Image.open(io.BytesIO(encoded)) as img:
        frames: list[Image.Image] = []
        durations: list[int] = []
        for frame in ImageSequence.Iterator(img):
            frames.append(frame.convert("RGBA"))
            durations.append(int(frame.info.get("duration", img.info.get("duration", 100)) or 100))
            if len(frames) >= MAX_GIF_FRAMES:
                break
    if not frames:
        raise ValueError("no frames in the picture")

    for scale in (1.0, 0.75, 0.5):
        size = (int(SCREEN_WIDTH * scale), int(SCREEN_HEIGHT * scale))
        out_frames = []
        for frame in frames:
            if fit == "fill":
                fitted = ImageOps.fit(frame, size, Image.LANCZOS)
            else:
                fitted = Image.new("RGBA", size, (0, 0, 0, 255))
                scaled = ImageOps.contain(frame, size, Image.LANCZOS)
                fitted.alpha_composite(scaled, ((size[0] - scaled.width) // 2, (size[1] - scaled.height) // 2))
            # Flatten onto black: the robot draws transparent pixels black anyway.
            flat = Image.new("RGB", size, (0, 0, 0))
            flat.paste(fitted, mask=fitted.getchannel("A"))
            out_frames.append(flat)
        buf = io.BytesIO()
        out_frames[0].save(
            buf,
            format="GIF",
            save_all=True,
            append_images=out_frames[1:],
            duration=durations,
            loop=0,
            disposal=1,
            optimize=False,
        )
        data = buf.getvalue()
        if len(data) <= MAX_GIF_BYTES:
            return data, {"frames": len(out_frames), "width": size[0], "height": size[1]}
    raise ValueError(f"GIF is too large for the robot even at half size ({len(data) // 1024} KB)")
