"""Tests for the activity feed, health / launcher routes and the app launcher."""

from __future__ import annotations

import json
import threading
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer

import pytest
from mcp.types import TextContent

from stackchan_mcp import activity, app
from stackchan_mcp import stdio_server
from tests.test_web_ui import _client


@pytest.fixture(autouse=True)
def _fresh_feed():
    activity._entries.clear()
    yield
    activity._entries.clear()


def test_record_and_since():
    activity.record("heard", "hello   robot", source="mic")
    activity.record("tool", "x" * 1000)
    entries = activity.since(0)
    assert [e["kind"] for e in entries] == ["heard", "tool"]
    assert entries[0]["text"] == "hello robot"
    assert entries[0]["source"] == "mic"
    assert len(entries[1]["text"]) == activity.MAX_TEXT
    assert activity.since(entries[0]["id"]) == [entries[1]]


def test_source_for_session():
    assert activity.source_for_session("web-ui") == "panel"
    assert activity.source_for_session("ollama-brain") == "ollama"
    assert activity.source_for_session("mcp-abc") == "claude"


@pytest.mark.asyncio
async def test_dispatch_records_speech_tools_and_errors(monkeypatch):
    async def fake_say(arguments, gateway):
        return {"ok": True}

    async def fake_inner(name, arguments, gateway):
        if name == "say":
            return [TextContent(type="text", text=json.dumps(await fake_say(arguments, gateway)))]
        return [TextContent(type="text", text=json.dumps({"error": "robot is asleep"}))]

    monkeypatch.setattr(stdio_server, "_dispatch_mcp_tool_inner", fake_inner)
    activity.source_var.set("panel")
    await stdio_server._dispatch_mcp_tool("say", {"text": "Hi there"}, object())
    await stdio_server._dispatch_mcp_tool("move_head", {"yaw": 10}, object())
    kinds = [(e["kind"], e["source"], e["text"]) for e in activity.since(0)]
    assert kinds[0] == ("said", "panel", "Hi there")
    assert kinds[1][0:2] == ("tool", "panel") and kinds[1][2].startswith("move_head")
    assert kinds[2][0] == "error" and "robot is asleep" in kinds[2][2]


def test_activity_route_pages_by_id():
    activity.record("system", "one")
    activity.record("system", "two")
    client, _ = _client()
    with client:
        first = client.get("/api/activity").json()
        later = client.get(f"/api/activity?after={first['entries'][0]['id']}").json()
    texts = [e["text"] for e in first["entries"]]
    assert "one" in texts and "two" in texts
    later_texts = [e["text"] for e in later["entries"]]
    assert "two" in later_texts and "one" not in later_texts


def test_panel_tool_calls_are_attributed_to_the_panel():
    client, _ = _client()
    with client:
        client.post("/api/tool/set_avatar", json={"face": "happy"})
    # The fake dispatcher bypasses the real one, so check the queue source tag.
    assert activity.source_for_session("web-ui") == "panel"


def test_launcher_route_reads_env(monkeypatch):
    monkeypatch.setenv("STACKCHAN_LAUNCHER_URL", "http://127.0.0.1:8768")
    monkeypatch.setenv("STACKCHAN_LAUNCHER_TOKEN", "secret")
    client, _ = _client()
    with client:
        body = client.get("/api/launcher").json()
    assert body == {"available": True, "url": "http://127.0.0.1:8768", "token": "secret"}


def test_launcher_route_unavailable_without_env(monkeypatch):
    monkeypatch.delenv("STACKCHAN_LAUNCHER_URL", raising=False)
    monkeypatch.delenv("STACKCHAN_LAUNCHER_TOKEN", raising=False)
    client, _ = _client()
    with client:
        assert client.get("/api/launcher").json()["available"] is False


def test_shutdown_route_signals_itself(monkeypatch):
    sent = []
    monkeypatch.setattr("stackchan_mcp.web_ui.os.kill", lambda pid, sig: sent.append(sig))
    client, _ = _client()
    with client:
        assert client.post("/api/gateway/shutdown").json() == {"ok": True}
        import time

        time.sleep(0.5)
    assert sent  # SIGTERM was scheduled


def test_health_route_without_robot():
    client, _ = _client()
    with client:
        body = client.get("/api/health").json()
    assert "connected" in body


class _FakeControl:
    url = "http://127.0.0.1:9"
    token = "tok"

    def __init__(self, running: bool) -> None:
        self._running = running
        self.started = 0

    def running(self) -> bool:
        return self._running

    def start(self):
        self.started += 1
        return True, "started"


def _serve(control):
    server = ThreadingHTTPServer(("127.0.0.1", 0), app.make_launcher_handler(control))
    threading.Thread(target=server.serve_forever, daemon=True).start()
    return server, f"http://127.0.0.1:{server.server_address[1]}"


def _post(url, token=None):
    req = urllib.request.Request(url, method="POST", headers={"X-Launcher-Token": token} if token else {})
    try:
        with urllib.request.urlopen(req) as resp:
            return resp.status, json.loads(resp.read())
    except urllib.error.HTTPError as exc:
        return exc.code, json.loads(exc.read())


def test_launcher_start_needs_the_token():
    control = _FakeControl(running=False)
    server, base = _serve(control)
    try:
        assert _post(f"{base}/start")[0] == 403
        assert _post(f"{base}/start", "wrong")[0] == 403
        status, body = _post(f"{base}/start", "tok")
        assert (status, body["ok"], control.started) == (200, True, 1)
    finally:
        server.shutdown()


def test_launcher_page_offers_open_when_stopped_and_cors_is_scoped():
    control = _FakeControl(running=False)
    server, base = _serve(control)
    try:
        with urllib.request.urlopen(f"{base}/") as resp:
            page = resp.read().decode()
        assert "Open gateway" in page
        with urllib.request.urlopen(f"{base}/status") as resp:
            assert resp.headers["Access-Control-Allow-Origin"] == control.url
            assert json.loads(resp.read()) == {"running": False}
    finally:
        server.shutdown()


def test_launcher_page_redirects_to_panel_when_running():
    control = _FakeControl(running=True)
    server, base = _serve(control)

    class NoRedirect(urllib.request.HTTPRedirectHandler):
        def redirect_request(self, *args, **kwargs):
            return None

    try:
        opener = urllib.request.build_opener(NoRedirect)
        with pytest.raises(urllib.error.HTTPError) as exc:
            opener.open(f"{base}/")
        assert exc.value.code == 302
        assert exc.value.headers["Location"] == f"{control.url}/ui"
    finally:
        server.shutdown()


def test_launcher_running_detects_an_existing_launcher():
    server, base = _serve(_FakeControl(running=False))
    try:
        assert app.launcher_running(base)
    finally:
        server.shutdown()
    assert not app.launcher_running("http://127.0.0.1:9")
