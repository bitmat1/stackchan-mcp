"""Drawing canvas tools: shape arguments -> the robot's canvas ops.

The robot draws a JSON array of ops (``self.display.canvas``) in one call,
so a whole picture costs one round trip with ``canvas_draw``. The
single-shape tools (``draw_line`` etc.) send a one-op batch. Colours are
normalised here to ``"#rrggbb"``, the only form the firmware parses.
"""

from __future__ import annotations

from typing import Any

#: Gateway tool -> canvas op for the single-shape tools.
SHAPE_TOOLS = {
    "canvas_clear": "clear",
    "draw_pixel": "pixel",
    "draw_line": "line",
    "draw_rect": "rect",
    "draw_circle": "circle",
    "draw_text": "text",
}

#: Tools that become a ``self.display.canvas`` call.
CANVAS_TOOLS = frozenset({"canvas_draw", *SHAPE_TOOLS})

#: Fields each op passes through to the firmware (besides ``op``).
OP_FIELDS = {
    "clear": ("color",),
    "pixel": ("x", "y", "size", "color"),
    "line": ("x1", "y1", "x2", "y2", "width", "color"),
    "rect": ("x", "y", "w", "h", "fill", "radius", "width", "color"),
    "circle": ("x", "y", "r", "fill", "width", "color"),
    "text": ("x", "y", "text", "color"),
}

MAX_OPS = 256

NAMED_COLORS = {
    "black": "#000000",
    "white": "#ffffff",
    "red": "#ff0000",
    "green": "#00c000",
    "lime": "#00ff00",
    "blue": "#0000ff",
    "yellow": "#ffff00",
    "orange": "#ff8c00",
    "purple": "#800080",
    "pink": "#ff69b4",
    "cyan": "#00ffff",
    "magenta": "#ff00ff",
    "brown": "#8b4513",
    "grey": "#808080",
    "gray": "#808080",
    "lightgrey": "#d3d3d3",
    "lightgray": "#d3d3d3",
    "darkgrey": "#404040",
    "darkgray": "#404040",
    "navy": "#000080",
    "skyblue": "#87ceeb",
    "gold": "#ffd700",
}


def normalise_color(value: Any) -> str:
    """Return ``value`` as ``"#rrggbb"``.

    Accepts ``"#rrggbb"``, ``"#rgb"``, a colour name such as ``"orange"``,
    or an ``[r, g, b]`` list. Raises ``ValueError`` for anything else.
    """
    if isinstance(value, (list, tuple)) and len(value) == 3:
        if all(isinstance(c, int) and not isinstance(c, bool) for c in value):
            return "#" + "".join(f"{max(0, min(255, c)):02x}" for c in value)
    if isinstance(value, str):
        text = value.strip().lower().replace(" ", "")
        if text in NAMED_COLORS:
            return NAMED_COLORS[text]
        digits = text.removeprefix("#")
        if all(ch in "0123456789abcdef" for ch in digits):
            if len(digits) == 3:
                return "#" + "".join(ch * 2 for ch in digits)
            if len(digits) == 6:
                return "#" + digits
    raise ValueError(f"unknown colour {value!r}: use a name, '#rrggbb' or [r, g, b]")


def to_op(kind: str, fields: dict[str, Any]) -> dict[str, Any]:
    """Build one firmware op of ``kind`` from tool or shape fields."""
    if kind not in OP_FIELDS:
        raise ValueError(f"unknown shape {kind!r}: use one of {', '.join(OP_FIELDS)}")
    op: dict[str, Any] = {"op": kind}
    for key in OP_FIELDS[kind]:
        if key in fields and fields[key] is not None:
            op[key] = normalise_color(fields[key]) if key == "color" else fields[key]
    if kind == "text" and not isinstance(op.get("text"), str):
        raise ValueError("a text shape needs a 'text' string")
    return op


def canvas_ops(tool: str, arguments: dict[str, Any]) -> list[dict[str, Any]]:
    """Ops for one canvas tool call. Raises ``ValueError`` on bad input."""
    if tool in SHAPE_TOOLS:
        return [to_op(SHAPE_TOOLS[tool], arguments)]
    shapes = arguments.get("shapes")
    if not isinstance(shapes, list):
        raise ValueError("shapes must be a list of shape objects")
    if len(shapes) > MAX_OPS:
        raise ValueError(f"too many shapes ({len(shapes)}); send at most {MAX_OPS} per call")
    ops = []
    for i, shape in enumerate(shapes):
        if not isinstance(shape, dict):
            raise ValueError(f"shape {i} is not an object")
        try:
            ops.append(to_op(str(shape.get("shape", shape.get("op", ""))), shape))
        except ValueError as exc:
            raise ValueError(f"shape {i}: {exc}") from None
    return ops
