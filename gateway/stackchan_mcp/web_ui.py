"""Browser control panel for the Streamable HTTP daemon.

Serves a single-page UI at ``/ui`` plus a small JSON API that runs the
same MCP tools Claude uses, through the same single-flight command queue,
so a button press in the browser and a tool call from Claude never talk
over each other on the ESP32 link.

Endpoints:

    GET  /ui                  the control panel page
    GET  /api/tools           tool names, descriptions and input schemas
    GET  /api/timers          pending timers, reminders and alarms
    POST /api/tool/{name}     run a tool; JSON body = tool arguments

Results come back as ``{"content": [...]}`` where each item is either
``{"type": "text", "text": ...}`` or ``{"type": "image", "mimeType": ...,
"data": <base64>}``. ``/api/*`` shares the daemon's bearer-token check.
"""

from __future__ import annotations

import asyncio
import json
import os
import signal
import time
import uuid
from pathlib import Path
from typing import Any, Awaitable, Callable

import jsonschema
from mcp.types import ImageContent, ListToolsRequest, TextContent
from starlette.requests import Request
from starlette.responses import FileResponse, JSONResponse
from starlette.routing import Route

from . import activity, brain, timers
from .queue import CommandQueue, QueueFull, QueueItem

WEB_DIR = Path(__file__).parent / "web"
WEB_CLIENT_SESSION_ID = "web-ui"
BRAIN_SESSION_ID = "ollama-brain"
TIMER_SESSION_ID = "timers"
TOOL_TIMEOUT_SECONDS = 180.0

class RobotBusy(RuntimeError):
    """The command queue is full."""


DirectDispatch = Callable[[str, dict[str, Any]], Awaitable[list[TextContent | ImageContent]]]


def _serialize(content: list[Any]) -> list[dict[str, Any]]:
    out: list[dict[str, Any]] = []
    for item in content:
        if isinstance(item, ImageContent):
            out.append({"type": "image", "mimeType": item.mimeType, "data": item.data})
        elif isinstance(item, TextContent):
            out.append({"type": "text", "text": item.text})
        else:
            out.append({"type": "text", "text": str(item)})
    return out


HEALTH_TTL_S = 15.0
_health_cache: dict[str, Any] = {}
_health_lock = asyncio.Lock()
#: Device tools read for the health card, by result key.
HEALTH_TOOLS = {
    "device": "self.get_device_status",
    "head": "self.robot.get_head_angles",
    "motion": "self.sensors.get_motion",
    "proximity": "self.sensors.get_proximity",
}


async def _device_json(gateway: Any, tool: str) -> dict[str, Any] | None:
    try:
        result, error = await asyncio.wait_for(gateway.esp32.call_tool(tool, {}), timeout=12)
    except Exception:
        return None
    if error or not isinstance(result, dict):
        return None
    for item in result.get("content", []):
        if isinstance(item, dict) and item.get("type") == "text":
            try:
                data = json.loads(item.get("text", ""))
            except ValueError:
                return None
            return data if isinstance(data, dict) else None
    return None


async def robot_health(gateway: Any, queue: CommandQueue) -> dict[str, Any]:
    """Battery, Wi-Fi, memory, head and sensors, cached for HEALTH_TTL_S.

    Reads the device directly (not through the command queue) so the
    panel's periodic poll neither waits behind long tool calls nor floods
    the activity feed. Device calls are cheap reads on their own lanes.
    """
    async with _health_lock:
        now = time.monotonic()
        if _health_cache and now - _health_cache["at"] < HEALTH_TTL_S:
            return _health_cache["data"]
        esp32 = getattr(gateway, "esp32", None)
        connected = bool(esp32 and esp32.device_connected)
        data: dict[str, Any] = {
            "connected": connected,
            "queue_depth": queue.depth,
            "brain": brain.brain_mode(),
            "checked_at": time.time(),
        }
        if connected:
            available = set(esp32.get_status().get("tools", []))
            for key, tool in HEALTH_TOOLS.items():
                if tool in available:
                    data[key] = await _device_json(gateway, tool)
        _health_cache.update(at=now, data=data)
        return data


def build_web_routes(
    *,
    server: Any,
    queue: CommandQueue,
    bypass_tools: frozenset[str],
    direct_dispatch: DirectDispatch,
    gateway: Any = None,
) -> list[Route]:
    tools_cache: dict[str, Any] = {}

    async def load_tools() -> dict[str, Any]:
        if not tools_cache:
            handler = server.request_handlers[ListToolsRequest]
            result = await handler(ListToolsRequest(method="tools/list"))
            for tool in result.root.tools:
                tools_cache[tool.name] = tool
        return tools_cache

    async def run_tool(
        name: str, arguments: dict[str, Any], session_id: str = WEB_CLIENT_SESSION_ID
    ) -> list[Any]:
        """Run a tool through the shared command queue (or directly for
        bypass tools). Raises RuntimeError with a user-facing message."""
        if name in bypass_tools:
            activity.source_var.set(activity.source_for_session(session_id))
            return await direct_dispatch(name, arguments)
        future: asyncio.Future[Any] = asyncio.get_running_loop().create_future()
        item = QueueItem(
            correlation_id=str(uuid.uuid4()),
            client_session_id=session_id,
            client_request_id=str(uuid.uuid4()),
            tool_name=name,
            arguments=arguments,
            response_future=future,
            enqueued_at=time.monotonic(),
        )
        try:
            queue.enqueue(item)
        except QueueFull as exc:
            raise RobotBusy("robot is busy, try again") from exc
        try:
            result = await asyncio.wait_for(future, timeout=TOOL_TIMEOUT_SECONDS)
        except asyncio.TimeoutError as exc:
            raise TimeoutError("timed out") from exc
        if isinstance(result, list):
            return result
        # ErrorData from the dispatcher.
        raise RuntimeError(getattr(result, "message", str(result)))

    async def tool_schemas() -> dict[str, dict[str, Any]]:
        return {
            t.name: {"description": t.description or "", "inputSchema": t.inputSchema}
            for t in (await load_tools()).values()
        }

    async def brain_tool(name: str, arguments: dict[str, Any]) -> list[Any]:
        return await run_tool(name, arguments, BRAIN_SESSION_ID)

    brain.register_tool_runner(brain_tool, tool_schemas)

    async def timer_tool(name: str, arguments: dict[str, Any]) -> list[Any]:
        return await run_tool(name, arguments, TIMER_SESSION_ID)

    timers.register_tool_runner(timer_tool)

    async def page(_request: Request) -> FileResponse:
        return FileResponse(WEB_DIR / "index.html", headers={"Cache-Control": "no-store"})

    async def list_tools(_request: Request) -> JSONResponse:
        tools = await load_tools()
        return JSONResponse(
            {
                "tools": [
                    {
                        "name": t.name,
                        "description": t.description or "",
                        "inputSchema": t.inputSchema,
                    }
                    for t in tools.values()
                ]
            }
        )

    async def call_tool(request: Request) -> JSONResponse:
        name = request.path_params["name"]
        tools = await load_tools()
        tool = tools.get(name)
        if tool is None:
            return JSONResponse({"error": f"unknown tool {name!r}"}, status_code=404)
        try:
            body = await request.body()
            arguments = (await request.json()) if body else {}
        except ValueError:
            return JSONResponse({"error": "body must be JSON"}, status_code=400)
        if not isinstance(arguments, dict):
            return JSONResponse({"error": "body must be a JSON object"}, status_code=400)
        try:
            jsonschema.validate(instance=arguments, schema=tool.inputSchema)
        except jsonschema.ValidationError as exc:
            return JSONResponse({"error": f"invalid arguments: {exc.message}"}, status_code=400)

        try:
            content = await run_tool(name, arguments)
        except TimeoutError:
            return JSONResponse({"error": "timed out"}, status_code=504)
        except RobotBusy as exc:
            return JSONResponse({"error": str(exc)}, status_code=503)
        except RuntimeError as exc:
            return JSONResponse({"error": str(exc)}, status_code=502)
        return JSONResponse({"content": _serialize(content)})

    async def get_brain(_request: Request) -> JSONResponse:
        reachable, models = await brain.list_ollama_models()
        return JSONResponse(
            {
                "brain": brain.brain_mode(),
                "brains": list(brain.BRAINS),
                "model": brain.ollama_model(models),
                "ollama_url": brain.ollama_url(),
                "ollama_running": reachable,
                "models": models,
            }
        )

    async def set_brain(request: Request) -> JSONResponse:
        try:
            body = await request.json()
        except ValueError:
            return JSONResponse({"error": "body must be JSON"}, status_code=400)
        if not isinstance(body, dict):
            return JSONResponse({"error": "body must be a JSON object"}, status_code=400)
        try:
            brain.set_brain(body.get("brain"), body.get("model"))
        except (ValueError, AttributeError) as exc:
            return JSONResponse({"error": str(exc)}, status_code=400)
        return await get_brain(request)

    async def ask_brain(request: Request) -> JSONResponse:
        try:
            body = await request.json()
        except ValueError:
            return JSONResponse({"error": "body must be JSON"}, status_code=400)
        text = body.get("text") if isinstance(body, dict) else None
        if not isinstance(text, str) or not text.strip():
            return JSONResponse({"error": "text is required"}, status_code=400)
        reply = await brain.respond(text.strip())
        return JSONResponse({"reply": reply})

    async def claude_session(_request: Request) -> JSONResponse:
        from .app import open_claude_voice_session

        try:
            await asyncio.to_thread(open_claude_voice_session)
        except Exception as exc:
            return JSONResponse({"error": str(exc)}, status_code=500)
        return JSONResponse({"ok": True})

    async def get_activity(request: Request) -> JSONResponse:
        try:
            after = int(request.query_params.get("after", "0"))
        except ValueError:
            after = 0
        entries = activity.since(after)
        return JSONResponse({"entries": entries, "last_id": entries[-1]["id"] if entries else after})

    async def get_timers(_request: Request) -> JSONResponse:
        """Read straight from the schedule, so the panel's once-a-few-
        seconds poll stays out of the command queue and the activity feed."""
        return JSONResponse(
            {"now": time.time(), "timers": [t.describe() for t in timers.pending()]}
        )

    async def get_health(_request: Request) -> JSONResponse:
        if gateway is None:
            return JSONResponse({"connected": False})
        return JSONResponse(await robot_health(gateway, queue))

    async def get_launcher(_request: Request) -> JSONResponse:
        url = os.getenv("STACKCHAN_LAUNCHER_URL", "")
        token = os.getenv("STACKCHAN_LAUNCHER_TOKEN", "")
        return JSONResponse({"available": bool(url and token), "url": url, "token": token})

    async def shutdown(_request: Request) -> JSONResponse:
        """Stop this gateway gracefully (uvicorn's SIGTERM handler), after
        the response has gone out."""
        activity.record("system", "gateway closed from the panel", source="panel")
        asyncio.get_running_loop().call_later(0.3, os.kill, os.getpid(), signal.SIGTERM)
        return JSONResponse({"ok": True})

    return [
        Route("/ui", endpoint=page, methods=["GET"]),
        Route("/api/gateway/shutdown", endpoint=shutdown, methods=["POST"]),
        Route("/api/activity", endpoint=get_activity, methods=["GET"]),
        Route("/api/health", endpoint=get_health, methods=["GET"]),
        Route("/api/timers", endpoint=get_timers, methods=["GET"]),
        Route("/api/launcher", endpoint=get_launcher, methods=["GET"]),
        Route("/api/tools", endpoint=list_tools, methods=["GET"]),
        Route("/api/tool/{name}", endpoint=call_tool, methods=["POST"]),
        Route("/api/brain", endpoint=get_brain, methods=["GET"]),
        Route("/api/brain", endpoint=set_brain, methods=["POST"]),
        Route("/api/brain/ask", endpoint=ask_brain, methods=["POST"]),
        Route("/api/brain/claude-session", endpoint=claude_session, methods=["POST"]),
    ]
