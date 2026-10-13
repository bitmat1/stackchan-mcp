"""Tests for the stdio channel bridge's daemon-side client."""

from __future__ import annotations

from typing import Any, cast

import anyio
import pytest
from mcp import types
from mcp.server.lowlevel import NotificationOptions, Server
from mcp.shared.message import SessionMessage

from stackchan_mcp.channel_bridge import _DaemonClientSession
from stackchan_mcp.stdio_server import StackChanChannelNotification


@pytest.mark.asyncio
async def test_channel_notifications_reach_callback():
    server: Server = Server("fake-daemon")

    @server.list_tools()
    async def list_tools() -> list[types.Tool]:
        return [types.Tool(name="ping", inputSchema={"type": "object"})]

    @server.call_tool()
    async def call_tool(name: str, arguments: dict[str, Any]) -> list[types.TextContent]:
        await server.request_context.session.send_notification(
            cast(
                Any,
                StackChanChannelNotification(
                    params={"content": "hello robot", "meta": {"event_type": "voice"}}
                ),
            )
        )
        return [types.TextContent(type="text", text="pong")]

    s2c_send, s2c_recv = anyio.create_memory_object_stream[SessionMessage | Exception](10)
    c2s_send, c2s_recv = anyio.create_memory_object_stream[SessionMessage | Exception](10)
    received: list[dict[str, Any]] = []

    async def on_channel(params: Any) -> None:
        received.append(dict(params))

    async with anyio.create_task_group() as tg:
        tg.start_soon(
            lambda: server.run(
                c2s_recv,
                s2c_send,
                server.create_initialization_options(NotificationOptions()),
            )
        )
        async with _DaemonClientSession(s2c_recv, c2s_send, on_channel=on_channel) as client:
            await client.initialize()
            result = await client.call_tool("ping", {})
            assert result.content[0].text == "pong"
            with anyio.fail_after(2):
                while not received:
                    await anyio.sleep(0.01)
        tg.cancel_scope.cancel()

    assert received[0]["content"] == "hello robot"
    assert received[0]["meta"] == {"event_type": "voice"}
