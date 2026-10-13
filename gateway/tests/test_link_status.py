"""Tests for pushing Claude connectivity to the robot's status icon."""

from __future__ import annotations

import asyncio
from typing import Any

import pytest

from stackchan_mcp import link_status, stdio_server


class _ESP32:
    def __init__(self) -> None:
        self.device_connected = True
        self.connection = object()
        self.calls: list[dict[str, Any]] = []

    async def call_tool(self, name: str, arguments: dict[str, Any]):
        assert name == "self.display.set_link_status"
        self.calls.append(arguments)
        return {"ok": True}, None


class _Gateway:
    def __init__(self) -> None:
        self.esp32 = _ESP32()


@pytest.mark.asyncio
async def test_pushes_changes_and_reconnects(monkeypatch):
    sessions: dict[int, Any] = {}
    monkeypatch.setattr(stdio_server, "_active_sessions", sessions)
    gw = _Gateway()
    task = asyncio.create_task(link_status.run_link_status_reporter(gw, interval_s=0.01))
    try:
        await asyncio.sleep(0.05)
        assert gw.esp32.calls == [{"claude_connected": False}]
        sessions[1] = object()
        await asyncio.sleep(0.05)
        assert gw.esp32.calls[-1] == {"claude_connected": True}
        n = len(gw.esp32.calls)
        await asyncio.sleep(0.05)
        assert len(gw.esp32.calls) == n  # no repeats while unchanged
        gw.esp32.connection = object()  # device reconnected
        await asyncio.sleep(0.05)
        assert len(gw.esp32.calls) == n + 1
    finally:
        task.cancel()
