"""Keep the robot's Wi-Fi icon colour in sync with Claude connectivity.

The firmware colours its Wi-Fi icon red when it has no gateway link and
orange when the gateway is up but no Claude client (Claude Code via the
channel bridge, Claude Desktop, ...) has an MCP session open. Only the
gateway knows the latter, so this task pushes it with
``self.display.set_link_status`` whenever it changes or the device
reconnects. The browser panel does not open MCP sessions, so it does not
count as a Claude client.
"""

from __future__ import annotations

import asyncio
import logging
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .gateway import Gateway

logger = logging.getLogger(__name__)

POLL_INTERVAL_S = 2.0


def claude_client_count() -> int:
    from .stdio_server import _active_sessions

    return len(_active_sessions)


async def run_link_status_reporter(gateway: "Gateway", interval_s: float = POLL_INTERVAL_S) -> None:
    last_sent: tuple[object, bool] | None = None
    while True:
        try:
            connection = gateway.esp32.connection
            if gateway.esp32.device_connected and connection is not None:
                state = (connection, claude_client_count() > 0)
                if state != last_sent:
                    _result, error = await gateway.esp32.call_tool(
                        "self.display.set_link_status",
                        {"claude_connected": state[1]},
                    )
                    if error:
                        logger.debug("set_link_status failed: %s", error)
                    else:
                        last_sent = state
                        logger.info("link status: claude_connected=%s", state[1])
            else:
                last_sent = None
        except asyncio.CancelledError:
            raise
        except Exception as exc:  # keep reporting through transient errors
            logger.debug("link status reporter error: %s", exc)
        await asyncio.sleep(interval_s)
