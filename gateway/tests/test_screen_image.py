"""Tests for show_image picture conversion."""

from __future__ import annotations

import base64
import io

import pytest

PIL = pytest.importorskip("PIL")
from PIL import Image  # noqa: E402

from stackchan_mcp import screen_image  # noqa: E402
from stackchan_mcp.screen_image import SCREEN_BYTES, load_image_bytes, to_rgb565  # noqa: E402


def _png(color: tuple[int, int, int], size=(100, 50)) -> bytes:
    buf = io.BytesIO()
    Image.new("RGB", size, color).save(buf, "PNG")
    return buf.getvalue()


def _pixel(frame: bytes, x: int, y: int) -> int:
    i = (y * 320 + x) * 2
    return frame[i] | (frame[i + 1] << 8)


def test_fit_letterboxes_wide_picture():
    frame = to_rgb565(_png((255, 0, 0)), "fit")
    assert len(frame) == SCREEN_BYTES
    assert _pixel(frame, 160, 120) == 0xF800  # red in the middle
    assert _pixel(frame, 160, 5) == 0x0000    # black bar on top


def test_fill_covers_screen():
    frame = to_rgb565(_png((0, 0, 255)), "fill")
    assert _pixel(frame, 160, 5) == 0x001F
    assert _pixel(frame, 0, 0) == 0x001F


@pytest.mark.asyncio
async def test_load_from_base64_and_data_url(tmp_path):
    png = _png((0, 255, 0))
    b64 = base64.b64encode(png).decode()
    assert await load_image_bytes(b64) == png
    assert await load_image_bytes("data:image/png;base64," + b64) == png
    path = tmp_path / "x.png"
    path.write_bytes(png)
    assert await load_image_bytes(str(path)) == png


@pytest.mark.asyncio
async def test_rejects_garbage():
    with pytest.raises(ValueError):
        await load_image_bytes("not an image or path!")


def _animation(n: int, size: tuple[int, int], noise: bool = False) -> bytes:
    import os

    frames = []
    for i in range(n):
        if noise:  # incompressible frames, to force the size fallback
            frames.append(Image.frombytes("RGB", size, os.urandom(size[0] * size[1] * 3)))
        else:
            frame = Image.new("RGB", size, (0, 0, 0))
            frame.paste((255, 40 * i % 256, 0), (i * 4, 0, i * 4 + 20, 20))
            frames.append(frame)
    buf = io.BytesIO()
    frames[0].save(buf, format="GIF", save_all=True, append_images=frames[1:], duration=[50] * n, loop=0)
    return buf.getvalue()


def test_prepare_gif_scales_and_keeps_frames_and_timing():
    data, info = screen_image.prepare_gif(_animation(6, (640, 120)))
    assert info == {"frames": 6, "width": 320, "height": 240}
    with Image.open(io.BytesIO(data)) as gif:
        assert gif.size == (320, 240)
        assert gif.n_frames == 6
        assert gif.info.get("loop") == 0
        assert gif.info.get("duration") == 50


def test_prepare_gif_shrinks_when_too_big(monkeypatch):
    big = _animation(4, (320, 240), noise=True)
    full, _ = screen_image.prepare_gif(big)
    monkeypatch.setattr(screen_image, "MAX_GIF_BYTES", len(full) - 1)
    _, info = screen_image.prepare_gif(big)
    assert (info["width"], info["height"]) in {(240, 180), (160, 120)}


def test_prepare_gif_accepts_a_still_picture():
    buf = io.BytesIO()
    Image.new("RGB", (50, 50), (0, 128, 255)).save(buf, format="PNG")
    _, info = screen_image.prepare_gif(buf.getvalue())
    assert info["frames"] == 1


@pytest.mark.asyncio
async def test_gateway_show_gif_stages_and_calls_the_robot(monkeypatch):
    from stackchan_mcp import gateway as gateway_mod

    staged = {}

    async def fake_stage(app, mode, payload):
        staged.update(mode=mode, size=len(payload))
        return "abc", "tok", "sha256:x"

    calls = []

    class FakeEsp32:
        device_facing_host = None

        async def call_tool(self, name, args):
            calls.append((name, args))
            return {"content": [{"type": "text", "text": '{"ok":true}'}]}, None

    monkeypatch.setattr(gateway_mod, "stage_avatar_set", fake_stage)
    gw = gateway_mod.Gateway()
    gw.esp32 = FakeEsp32()
    gw._capture_app = object()
    data = _animation(3, (100, 100))
    result = await gw.show_gif(base64.b64encode(data).decode(), seconds=5, loops=2, fast=True)
    assert result["ok"] and result["frames"] == 3
    assert staged["mode"] == "gif"
    name, args = calls[0]
    assert name == "self.display.show_gif"
    assert args["url"].endswith("/avatar_set/abc")
    assert (args["token"], args["seconds"], args["loops"], args["fast"]) == ("tok", 5, 2, True)
