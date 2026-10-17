"""Tests for the drawing canvas tools (shape args -> self.display.canvas ops)."""

from __future__ import annotations

import json
from typing import Any

import pytest

from stackchan_mcp import stdio_server
from stackchan_mcp.canvas import canvas_ops, normalise_color


@pytest.mark.parametrize(
    "value,expected",
    [
        ("orange", "#ff8c00"),
        ("#ABCDEF", "#abcdef"),
        ("#fa0", "#ffaa00"),
        ("Light Grey", "#d3d3d3"),
        ([255, 0, 300], "#ff00ff"),
    ],
)
def test_normalise_color(value, expected):
    assert normalise_color(value) == expected


@pytest.mark.parametrize("value", ["teal-ish", "#12345", [1, 2], True, ["1", 2, 3]])
def test_normalise_color_rejects(value):
    with pytest.raises(ValueError):
        normalise_color(value)


def test_single_shape_tool_is_a_one_op_batch():
    assert canvas_ops("draw_line", {"x1": 0, "y1": 1, "x2": 30, "y2": 40, "color": "red", "width": 3}) == [
        {"op": "line", "x1": 0, "y1": 1, "x2": 30, "y2": 40, "width": 3, "color": "#ff0000"}
    ]
    assert canvas_ops("canvas_clear", {}) == [{"op": "clear"}]


def test_batch_keeps_order_and_drops_unknown_fields():
    ops = canvas_ops(
        "canvas_draw",
        {
            "shapes": [
                {"shape": "clear", "color": "white"},
                {"shape": "circle", "x": 160, "y": 120, "r": 50, "fill": True, "color": "#00f", "w": 9},
                {"shape": "text", "x": 10, "y": 10, "text": "Hi"},
            ]
        },
    )
    assert ops == [
        {"op": "clear", "color": "#ffffff"},
        {"op": "circle", "x": 160, "y": 120, "r": 50, "fill": True, "color": "#0000ff"},
        {"op": "text", "x": 10, "y": 10, "text": "Hi"},
    ]


@pytest.mark.parametrize(
    "shapes,message",
    [
        ([{"shape": "star"}], "shape 0: unknown shape"),
        ([{"shape": "text", "x": 0, "y": 0}], "shape 0: a text shape needs"),
        ([{"shape": "pixel", "x": 0, "y": 0}, {"shape": "rect", "color": "nope"}], "shape 1: unknown colour"),
        ([{"shape": "pixel"}] * 257, "too many shapes"),
    ],
)
def test_batch_errors(shapes, message):
    with pytest.raises(ValueError, match=message):
        canvas_ops("canvas_draw", {"shapes": shapes})


class _FakeESP32:
    device_connected = True

    def __init__(self) -> None:
        self.calls: list[tuple[str, dict[str, Any]]] = []

    async def call_tool(self, name: str, args: dict[str, Any]):
        self.calls.append((name, args))
        return {"content": [{"type": "text", "text": '{"ok":true}'}]}, None


class _FakeGateway:
    def __init__(self) -> None:
        self.esp32 = _FakeESP32()


@pytest.mark.asyncio
async def test_canvas_tools_reach_the_device():
    gateway = _FakeGateway()
    await stdio_server._dispatch_mcp_tool_inner(
        "draw_circle", {"x": 5, "y": 6, "r": 7, "color": "gold"}, gateway
    )
    await stdio_server._dispatch_mcp_tool_inner("canvas_hide", {}, gateway)
    (name, args), hide = gateway.esp32.calls
    assert name == "self.display.canvas"
    assert json.loads(args["ops"]) == [{"op": "circle", "x": 5, "y": 6, "r": 7, "color": "#ffd700"}]
    assert hide == ("self.display.canvas_hide", {})


@pytest.mark.asyncio
async def test_bad_shape_is_reported_without_calling_the_device():
    gateway = _FakeGateway()
    content = await stdio_server._dispatch_mcp_tool_inner("draw_text", {"x": 1, "y": 2}, gateway)
    assert "text shape" in json.loads(content[0].text)["error"]
    assert gateway.esp32.calls == []


def test_canvas_tools_are_listed():
    names = {tool.name for tool in stdio_server._canvas_tools()}
    assert names == {
        "canvas_draw", "canvas_clear", "draw_pixel", "draw_line",
        "draw_rect", "draw_circle", "draw_text", "canvas_hide",
    }
