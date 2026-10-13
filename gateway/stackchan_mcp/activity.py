"""In-memory activity feed for the control panel.

Records what happens while the gateway runs - what the person said, what
the robot said, every tool call (and who made it), physical events and
connection changes - in a bounded ring buffer that the panel polls via
``GET /api/activity?after=<id>``.

``source`` says who caused an entry: "claude" (an MCP client), "panel"
(the web control panel), "ollama" (the local brain) or "robot". Callers
that dispatch tools set :data:`source_var` so the central dispatcher can
attribute the call without threading a parameter through every layer.
"""

from __future__ import annotations

import itertools
import json
import time
from collections import deque
from contextvars import ContextVar
from typing import Any

MAX_ENTRIES = 1000
MAX_TEXT = 400
KINDS = ("heard", "said", "tool", "error", "event", "system")

source_var: ContextVar[str] = ContextVar("activity_source", default="claude")
_entries: deque[dict[str, Any]] = deque(maxlen=MAX_ENTRIES)
_ids = itertools.count(1)


def record(kind: str, text: str, *, source: str | None = None) -> None:
    text = " ".join(str(text).split())
    if len(text) > MAX_TEXT:
        text = text[: MAX_TEXT - 1] + "…"
    _entries.append(
        {
            "id": next(_ids),
            "ts": time.time(),
            "kind": kind,
            "source": source or source_var.get(),
            "text": text,
        }
    )


def record_tool(name: str, arguments: dict[str, Any]) -> None:
    args = json.dumps(arguments, ensure_ascii=False, default=str) if arguments else ""
    record("tool", f"{name} {args}".strip())


def since(after: int = 0, limit: int = 300) -> list[dict[str, Any]]:
    """Entries with id > ``after``, oldest first (at most ``limit``)."""
    out = [e for e in _entries if e["id"] > after]
    return out[-limit:]


def source_for_session(client_session_id: str) -> str:
    if client_session_id == "web-ui":
        return "panel"
    if client_session_id == "ollama-brain":
        return "ollama"
    return "claude"
