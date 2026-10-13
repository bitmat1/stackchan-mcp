"""Tests for the gateway's countdown timers, reminders and alarms."""

from __future__ import annotations

import json
import time
from datetime import datetime
from typing import Any

import pytest
from mcp.types import TextContent

from stackchan_mcp import brain, stdio_server, timers
from stackchan_mcp.http_server import BYPASS_TOOLS


@pytest.fixture(autouse=True)
def _isolated_timers(tmp_path, monkeypatch):
    monkeypatch.setenv("STACKCHAN_TIMERS_FILE", str(tmp_path / "timers.json"))
    monkeypatch.setattr(brain, "settings_path", lambda: tmp_path / "brain.json")
    monkeypatch.setenv("STACKCHAN_BRAIN", "ollama")  # no Claude channel notification
    timers._reset_for_tests()
    yield
    timers._reset_for_tests()


def _record_tools(say_result: str = '{"spoke": true}') -> list[tuple[str, dict[str, Any]]]:
    calls: list[tuple[str, dict[str, Any]]] = []

    async def runner(name: str, args: dict[str, Any]) -> list[TextContent]:
        calls.append((name, args))
        return [TextContent(type="text", text=say_result if name == "say" else "ok")]

    timers.register_tool_runner(runner)
    return calls


async def _call(name: str, **arguments: Any) -> dict[str, Any]:
    content = await timers.handle_tool(name, arguments)
    return json.loads(content[0].text)


def _spoken(calls: list[tuple[str, dict[str, Any]]]) -> list[str]:
    return [args["text"] for name, args in calls if name == "say"]


# ---- parsing --------------------------------------------------------------


@pytest.mark.parametrize(
    ("text", "expected"),
    [
        ("7:30", (7, 30)),
        ("07:05", (7, 5)),
        ("19.45", (19, 45)),
        ("7pm", (19, 0)),
        ("7:30 am", (7, 30)),
        ("12am", (0, 0)),
        ("12 p.m.", (12, 0)),
        ("24:00", None),
        ("13pm", None),
        ("soon", None),
    ],
)
def test_clock_parsing(text, expected):
    assert timers._clock(text) == expected


def test_parse_at_uses_next_occurrence_and_iso_dates():
    now = datetime(2026, 10, 13, 18, 0)
    assert timers.parse_at("18:30", now) == datetime(2026, 10, 13, 18, 30)
    assert timers.parse_at("9am", now) == datetime(2026, 10, 14, 9, 0)
    assert timers.parse_at("2026-10-20 08:15", now) == datetime(2026, 10, 20, 8, 15)
    with pytest.raises(ValueError, match="past"):
        timers.parse_at("2026-10-01 08:00", now)
    with pytest.raises(ValueError, match="could not read"):
        timers.parse_at("teatime", now)


def test_next_occurrence_honours_repeat():
    friday_evening = datetime(2026, 10, 16, 20, 0)  # a Friday
    assert timers.next_occurrence(7, 0, "weekdays", friday_evening) == datetime(2026, 10, 19, 7, 0)
    assert timers.next_occurrence(7, 0, "weekends", friday_evening) == datetime(2026, 10, 17, 7, 0)
    assert timers.next_occurrence(21, 0, "daily", friday_evening) == datetime(2026, 10, 16, 21, 0)


# ---- tools ----------------------------------------------------------------


@pytest.mark.asyncio
async def test_set_timer_adds_and_saves():
    result = await _call("set_timer", minutes=5, message="Tea is ready!")
    assert result["ok"] and result["kind"] == "timer"
    assert result["length"] == "5 minutes"
    assert 295 <= result["seconds_left"] <= 300
    saved = json.loads(timers.timers_path().read_text())
    assert saved[0]["message"] == "Tea is ready!"


@pytest.mark.asyncio
async def test_set_timer_rejects_bad_durations():
    assert (await _call("set_timer"))["ok"] is False
    assert (await _call("set_timer", hours=200))["ok"] is False


@pytest.mark.asyncio
async def test_set_reminder_needs_exactly_one_time():
    assert (await _call("set_reminder", message="call mum"))["ok"] is False
    assert (await _call("set_reminder", message="call mum", at="18:00", in_minutes=5))["ok"] is False
    result = await _call("set_reminder", message="call mum", in_minutes=10)
    assert result["ok"] and result["kind"] == "reminder"


@pytest.mark.asyncio
async def test_set_alarm_validates_time_and_repeat():
    assert (await _call("set_alarm", time="breakfast"))["ok"] is False
    assert (await _call("set_alarm", time="7am", repeat="hourly"))["ok"] is False
    result = await _call("set_alarm", time="7:00", repeat="weekdays")
    assert result["ok"] and result["repeat"] == "weekdays"
    assert result["due_local"].endswith("07:00:00")


@pytest.mark.asyncio
async def test_list_and_cancel():
    a = await _call("set_timer", seconds=60)
    await _call("set_timer", seconds=30)
    listed = await _call("list_timers")
    assert [t["seconds_left"] <= 30 for t in listed["timers"]] == [True, False]
    assert (await _call("cancel_timer", id=a["id"]))["cancelled"][0]["id"] == a["id"]
    assert (await _call("cancel_timer", id="nope"))["ok"] is False
    assert len((await _call("cancel_timer", id="all"))["cancelled"]) == 1
    assert (await _call("list_timers"))["timers"] == []


@pytest.mark.asyncio
async def test_timers_survive_a_restart():
    await _call("set_alarm", time="6:45", message="Feed the cat")
    timers._reset_for_tests()
    assert [t.message for t in timers.pending()] == ["Feed the cat"]


def test_timer_tools_are_listed_and_bypass_the_device_queue():
    assert timers.TOOL_NAMES <= BYPASS_TOOLS
    assert {"set_timer", "set_reminder", "set_alarm", "list_timers", "cancel_timer"} == timers.TOOL_NAMES


@pytest.mark.asyncio
async def test_dispatch_handles_timer_tools_without_a_device():
    class NoDevice:
        class esp32:
            device_connected = False

    content = await stdio_server._dispatch_mcp_tool("set_timer", {"seconds": 10}, NoDevice())
    assert json.loads(content[0].text)["ok"] is True


# ---- firing ---------------------------------------------------------------


@pytest.mark.asyncio
async def test_due_timer_is_spoken_and_removed():
    calls = _record_tools()
    timer = timers.add("timer", "Tea is ready!", time.time() - 1)
    await timers.run_due()
    assert _spoken(calls) == ["Tea is ready!"]
    assert calls[0] == ("set_avatar", {"face": "surprised"})
    assert calls[-1] == ("set_avatar", {"face": "idle"})
    assert timer.id not in {t.id for t in timers.pending()}


@pytest.mark.asyncio
async def test_future_timer_waits():
    calls = _record_tools()
    timers.add("timer", "", time.time() + 30)
    delay = await timers.run_due()
    assert calls == []
    assert 25 <= delay <= 30


@pytest.mark.asyncio
async def test_reminder_and_alarm_texts():
    calls = _record_tools()
    timers.add("reminder", "call mum", time.time() - 1)
    timers.add("alarm", "", time.time() - 1)
    await timers.run_due()
    spoken = _spoken(calls)
    assert spoken[0] == "Reminder: call mum"
    # Alarms say their text twice, with a wiggle between.
    assert spoken[1] == spoken[2] and spoken[1].endswith("This is your alarm!")
    assert ("gesture", {"name": "wiggle"}) in calls


@pytest.mark.asyncio
async def test_late_timer_apologises_and_very_late_one_is_dropped():
    calls = _record_tools()
    timers.add("timer", "Tea!", time.time() - 600)
    timers.add("timer", "Old", time.time() - timers.LATE_GRACE_S - 60)
    await timers.run_due()
    assert _spoken(calls) == ["Sorry, this is 10 minutes late. Tea!"]
    assert timers.pending() == []


@pytest.mark.asyncio
async def test_repeating_alarm_moves_to_next_day():
    _record_tools()
    due = datetime.now().replace(second=0, microsecond=0)
    alarm = timers.add("alarm", "Morning!", due.timestamp() - 1, repeat="daily")
    await timers.run_due()
    (left,) = timers.pending()
    assert left.id == alarm.id
    assert left.due > time.time()
    assert datetime.fromtimestamp(left.due).strftime("%H:%M") == datetime.fromtimestamp(alarm.due).strftime("%H:%M")


@pytest.mark.asyncio
async def test_failed_say_keeps_the_timer_for_a_retry():
    calls = _record_tools(say_result='{"error": "No ESP32 device connected."}')
    timers.add("timer", "Tea!", time.time() - 1)
    delay = await timers.run_due()
    assert delay == timers.RETRY_S
    assert len(timers.pending()) == 1
    assert _spoken(calls) == ["Tea!"]


@pytest.mark.asyncio
async def test_claude_is_told_what_was_said(monkeypatch):
    monkeypatch.setenv("STACKCHAN_BRAIN", "claude")
    sent: list[tuple[str, dict[str, Any]]] = []

    async def fake_notify(method: str, params: dict[str, Any]) -> None:
        sent.append((method, params))

    monkeypatch.setattr(stdio_server, "notify_stackchan_event", fake_notify)
    _record_tools()
    timers.add("reminder", "stretch", time.time() - 1)
    await timers.run_due()
    ((method, params),) = sent
    assert method == "notifications/claude/channel"
    assert params["meta"]["event_type"] == "timer"
    assert "already said: Reminder: stretch" in params["content"]
    assert all(isinstance(v, str) for v in params["meta"].values())
