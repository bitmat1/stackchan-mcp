"""stdio channel bridge: Claude Code <-> the Streamable HTTP daemon.

Claude Code channels must be stdio subprocesses that Claude Code spawns
itself, but only one process can own the ESP32 link, and the daemon
(``stackchan-mcp serve --transport streamable-http``) is the one that
also serves Claude Desktop and the browser panel. This bridge resolves
that: it is a stdio MCP server towards Claude Code and an MCP client of
the daemon.

* Tools: ``tools/list`` and ``tools/call`` are proxied to the daemon, so
  Claude Code sees exactly the daemon's tool set and its calls go through
  the daemon's single-flight queue alongside every other client.
* Channel: the daemon pushes ``notifications/claude/channel`` to every
  connected session (voice transcripts, touch events). The bridge
  declares the ``claude/channel`` capability and re-emits them to Claude
  Code, so speech to the robot shows up in the session.

Run it from Claude Code's MCP config, e.g. in ``.mcp.json``::

    {"mcpServers": {"stackchan": {"command": "uv", "args": [
        "run", "--directory", "/path/to/gateway",
        "stackchan-mcp", "channel"]}}}

then start an interactive session with
``claude --dangerously-load-development-channels server:stackchan``.

Environment: ``STACKCHAN_DAEMON_URL`` (default
``http://127.0.0.1:8767/mcp``) and ``STACKCHAN_TOKEN`` for the daemon's
bearer token, if one is set.
"""

from __future__ import annotations

import asyncio
import logging
import os
from typing import Any, Union, cast

from mcp import types
from mcp.client.session import ClientSession
from mcp.client.streamable_http import streamablehttp_client
from mcp.server.lowlevel import NotificationOptions, Server
from mcp.server.stdio import stdio_server
from pydantic import RootModel

from .stdio_server import (
    CHANNEL_CAPABILITY,
    CHANNEL_NOTIFICATION_METHOD,
    STACKCHAN_CHANNEL_INSTRUCTIONS,
    STACKCHAN_VOICE_INSTRUCTIONS,
    StackChanChannelNotification,
)

logger = logging.getLogger(__name__)

DEFAULT_DAEMON_URL = "http://127.0.0.1:8767/mcp"
RECONNECT_DELAY_SECONDS = 3.0


class _BridgeNotification(
    RootModel[Union[types.ServerNotification, StackChanChannelNotification]]
):
    """Accepts the standard server notifications plus Claude channel events."""


class _DaemonClientSession(ClientSession):
    """ClientSession that hands channel notifications to a callback.

    The stock ClientSession validates notifications against the MCP
    ServerNotification union, which rejects the Claude-specific
    ``notifications/claude/channel`` method, so it would be dropped.
    """

    def __init__(self, *args: Any, on_channel: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self._receive_notification_type = cast(Any, _BridgeNotification)
        self._on_channel = on_channel

    async def _received_notification(self, notification: Any) -> None:
        root = notification.root
        if isinstance(root, StackChanChannelNotification):
            await self._on_channel(root.params)
            return
        await super()._received_notification(root)


class ChannelBridge:
    def __init__(self, url: str, token: str | None) -> None:
        self.url = url
        self.headers = {"Authorization": f"Bearer {token}"} if token else None
        self.daemon: ClientSession | None = None
        self.daemon_ready = asyncio.Event()
        self.daemon_lost = asyncio.Event()
        self.claude_session: Any | None = None
        self.pending: list[dict[str, Any]] = []
        self.server = self._build_server()

    def _build_server(self) -> Server:
        server: Server = Server(
            "stackchan",
            instructions=f"{STACKCHAN_VOICE_INSTRUCTIONS}\n\n{STACKCHAN_CHANNEL_INSTRUCTIONS}",
        )

        @server.list_tools()
        async def list_tools() -> list[types.Tool]:
            self._remember_session(server)
            daemon = await self._wait_for_daemon()
            if daemon is None:
                return []
            return (await daemon.list_tools()).tools

        @server.call_tool(validate_input=False)
        async def call_tool(name: str, arguments: dict[str, Any]) -> types.CallToolResult:
            self._remember_session(server)
            daemon = await self._wait_for_daemon()
            if daemon is None:
                return types.CallToolResult(
                    content=[
                        types.TextContent(
                            type="text",
                            text=(
                                "stackchan daemon is not reachable at "
                                f"{self.url}. Start it with "
                                "`stackchan-mcp serve --transport streamable-http`."
                            ),
                        )
                    ],
                    isError=True,
                )
            try:
                return await daemon.call_tool(name, arguments)
            except Exception as exc:
                # A restarted daemon forgets our MCP session ("Session
                # terminated"); reconnect and retry once.
                logger.warning("daemon call %s failed (%s); reconnecting", name, exc)
                self._drop_daemon(daemon)
                daemon = await self._wait_for_daemon()
                if daemon is None:
                    raise
                return await daemon.call_tool(name, arguments)

        return server

    def _remember_session(self, server: Server) -> None:
        if self.claude_session is None:
            self.claude_session = server.request_context.session
            asyncio.get_running_loop().create_task(self._flush_pending())

    def _drop_daemon(self, daemon: ClientSession) -> None:
        if self.daemon is daemon:
            self.daemon = None
            self.daemon_ready.clear()
            self.daemon_lost.set()

    async def _wait_for_daemon(self, timeout: float = 10.0) -> ClientSession | None:
        try:
            await asyncio.wait_for(self.daemon_ready.wait(), timeout)
        except asyncio.TimeoutError:
            return None
        return self.daemon

    async def _on_channel(self, params: Any) -> None:
        payload = params if isinstance(params, dict) else dict(params or {})
        if self.claude_session is None:
            self.pending.append(payload)
            return
        await self._send_channel(payload)

    async def _flush_pending(self) -> None:
        pending, self.pending = self.pending, []
        for payload in pending:
            await self._send_channel(payload)

    async def _send_channel(self, payload: dict[str, Any]) -> None:
        try:
            await self.claude_session.send_notification(
                cast(Any, StackChanChannelNotification(params=payload))
            )
            logger.info("bridged channel event: %r", str(payload.get("content", ""))[:80])
        except Exception as exc:  # pragma: no cover - transport failure
            logger.warning("failed to forward channel event: %s", exc)

    async def run_daemon_client(self) -> None:
        """Keep an MCP client connection to the daemon, reconnecting forever."""
        while True:
            try:
                async with streamablehttp_client(self.url, headers=self.headers) as (
                    read,
                    write,
                    _get_session_id,
                ):
                    async with _DaemonClientSession(
                        read, write, on_channel=self._on_channel
                    ) as session:
                        await session.initialize()
                        self.daemon = session
                        self.daemon_lost.clear()
                        self.daemon_ready.set()
                        logger.info("connected to stackchan daemon at %s", self.url)
                        await self.daemon_lost.wait()
                        continue
            except asyncio.CancelledError:
                raise
            except Exception as exc:
                logger.warning(
                    "daemon connection to %s failed (%s); retrying in %.0fs",
                    self.url,
                    exc,
                    RECONNECT_DELAY_SECONDS,
                )
            finally:
                self.daemon = None
                self.daemon_ready.clear()
            await asyncio.sleep(RECONNECT_DELAY_SECONDS)

    async def run(self) -> None:
        client_task = asyncio.create_task(self.run_daemon_client())
        try:
            async with stdio_server() as (read, write):
                await self.server.run(
                    read,
                    write,
                    self.server.create_initialization_options(
                        notification_options=NotificationOptions(),
                        experimental_capabilities={CHANNEL_CAPABILITY: {}},
                    ),
                )
        finally:
            client_task.cancel()


def run_channel_bridge() -> None:
    logging.basicConfig(
        level=os.getenv("STACKCHAN_LOG_LEVEL", "INFO"),
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )
    url = os.getenv("STACKCHAN_DAEMON_URL") or DEFAULT_DAEMON_URL
    token = os.getenv("STACKCHAN_TOKEN") or os.getenv("BEARER_TOKEN") or None
    assert CHANNEL_NOTIFICATION_METHOD == "notifications/claude/channel"
    asyncio.run(ChannelBridge(url, token).run())
