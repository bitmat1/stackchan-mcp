"""Tests for the browser control panel routes on the HTTP daemon."""

from __future__ import annotations

from typing import Any

import pytest
from mcp.types import TextContent
from starlette.testclient import TestClient

from stackchan_mcp.http_server import build_app
from stackchan_mcp.queue import CommandQueue, QueueItem


class _FakeESP32:
    device_connected = True

    def get_status(self) -> dict[str, Any]:
        return {"status": "ok"}


class _FakeGateway:
    esp32 = _FakeESP32()


def _client(token: str | None = None) -> tuple[TestClient, list[QueueItem]]:
    seen: list[QueueItem] = []

    async def dispatch(item: QueueItem) -> list[TextContent]:
        seen.append(item)
        return [TextContent(type="text", text=f"ran {item.tool_name}")]

    app = build_app(
        CommandQueue(),
        gateway=_FakeGateway(),
        owner_id="test",
        host="127.0.0.1",
        port=8767,
        token=token,
        dispatch_fn=dispatch,
    )
    return TestClient(app, base_url="http://127.0.0.1:8767"), seen


def test_ui_page_served():
    client, _ = _client()
    with client:
        res = client.get("/ui")
    assert res.status_code == 200
    assert "Stack-chan" in res.text


def test_tools_listed():
    client, _ = _client()
    with client:
        names = {t["name"] for t in client.get("/api/tools").json()["tools"]}
    assert {"move_head", "say", "take_photo", "set_avatar"} <= names


def test_tool_runs_through_queue():
    client, seen = _client()
    with client:
        res = client.post("/api/tool/set_avatar", json={"face": "happy"})
    assert res.status_code == 200
    assert res.json()["content"] == [{"type": "text", "text": "ran set_avatar"}]
    assert seen[0].client_session_id == "web-ui"
    assert seen[0].arguments == {"face": "happy"}


def test_invalid_arguments_rejected():
    client, seen = _client()
    with client:
        res = client.post("/api/tool/set_avatar", json={"face": "not-a-face"})
    assert res.status_code == 400
    assert seen == []


def test_unknown_tool_404():
    client, _ = _client()
    with client:
        assert client.post("/api/tool/nope", json={}).status_code == 404


@pytest.mark.parametrize("auth,status", [(None, 401), ("Bearer secret", 200)])
def test_api_requires_token_when_set(auth, status):
    client, _ = _client(token="secret")
    headers = {"Authorization": auth} if auth else {}
    with client:
        assert client.get("/api/tools", headers=headers).status_code == status


def test_new_screen_tools_listed():
    client, _ = _client()
    with client:
        names = {t["name"] for t in client.get("/api/tools").json()["tools"]}
    assert {"show_image", "show_gif", "show_text", "clear_screen_overlay", "set_gaze", "gesture"} <= names


def test_body_sensor_tools_listed():
    client, _ = _client()
    with client:
        names = {t["name"] for t in client.get("/api/tools").json()["tools"]}
    assert {"get_motion", "get_proximity"} <= names


def test_brain_routes_switch_and_report(monkeypatch, tmp_path):
    from stackchan_mcp import brain

    monkeypatch.setattr(brain, "settings_path", lambda: tmp_path / "brain.json")
    monkeypatch.delenv("STACKCHAN_BRAIN", raising=False)
    brain._override.clear()

    async def models():
        return True, ["llama3.2:latest"]

    monkeypatch.setattr(brain, "list_ollama_models", models)
    client, _ = _client()
    try:
        with client:
            assert client.get("/api/brain").json()["brain"] == "claude"
            res = client.post("/api/brain", json={"brain": "ollama", "model": "llama3.2:latest"})
            assert res.status_code == 200
            body = res.json()
            assert (body["brain"], body["model"], body["ollama_running"]) == (
                "ollama",
                "llama3.2:latest",
                True,
            )
            assert client.post("/api/brain", json={"brain": "gpt"}).status_code == 400
            assert client.post("/api/brain/ask", json={}).status_code == 400
    finally:
        brain._override.clear()
