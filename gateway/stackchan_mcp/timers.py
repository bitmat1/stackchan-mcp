"""Countdown timers, reminders and alarms that the robot speaks on time.

The gateway keeps the schedule itself, so timers keep working whichever
brain is answering (Claude Code or the local Ollama model) and with no
Claude session open at all. When one is due the gateway shows a face and
speaks its text through the shared command queue (the same path as the
web panel and the Ollama brain), then tells a connected Claude Code
session with a ``timer`` channel message so it knows what was said.

Timers are saved to ``timers.json`` in the user config directory
(~/Library/Application Support/stackchan-mcp on macOS, next to
``brain.json``), so they survive a gateway restart. One that came due
while the gateway was down is still spoken if it is less than
``LATE_GRACE_S`` late, and dropped otherwise; repeating alarms simply
move on to their next time.

Configuration (environment variables):

    STACKCHAN_TIMERS_FILE   override the timers.json path
"""

from __future__ import annotations

import asyncio
import json
import logging
import os
import re
import time
import uuid
from dataclasses import asdict, dataclass
from datetime import datetime, timedelta
from pathlib import Path
from typing import Any, Awaitable, Callable

from mcp.types import TextContent, Tool
from platformdirs import user_config_path

logger = logging.getLogger(__name__)

KINDS = ("timer", "reminder", "alarm")
REPEATS = ("once", "daily", "weekdays", "weekends")
#: How late a timer may still be spoken (e.g. after the gateway was down).
LATE_GRACE_S = 3600
#: Wait between attempts while the robot is offline or busy.
RETRY_S = 30
#: Longest the scheduler sleeps before re-checking, in case the clock jumps.
MAX_SLEEP_S = 60
MAX_TIMERS = 50
MAX_MESSAGE_CHARS = 300
MAX_DURATION_S = 7 * 24 * 3600

ToolRunner = Callable[[str, dict[str, Any]], Awaitable[Any]]


@dataclass
class Timer:
    id: str
    kind: str
    message: str
    due: float  # unix time
    repeat: str = "once"
    created: float = 0.0

    def describe(self) -> dict[str, Any]:
        local = datetime.fromtimestamp(self.due)
        return {
            **asdict(self),
            "due_local": local.strftime("%Y-%m-%d %H:%M:%S"),
            "seconds_left": max(0, round(self.due - time.time())),
        }


_timers: dict[str, Timer] = {}
_loaded = False
_runner: ToolRunner | None = None
_wake: asyncio.Event | None = None


# ---- storage --------------------------------------------------------------


def timers_path() -> Path:
    override = os.getenv("STACKCHAN_TIMERS_FILE")
    if override:
        return Path(override).expanduser()
    return user_config_path("stackchan-mcp") / "timers.json"


def _load() -> None:
    global _loaded
    if _loaded:
        return
    _loaded = True
    try:
        raw = json.loads(timers_path().read_text(encoding="utf-8"))
    except FileNotFoundError:
        return
    except (OSError, ValueError) as exc:
        logger.warning("timers: could not read %s: %s", timers_path(), exc)
        return
    for item in raw if isinstance(raw, list) else []:
        try:
            timer = Timer(**item)
        except TypeError:
            logger.warning("timers: skipping malformed entry %r", item)
            continue
        if timer.kind in KINDS and timer.repeat in REPEATS:
            _timers[timer.id] = timer


def _save() -> None:
    path = timers_path()
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        tmp = path.with_suffix(".tmp")
        tmp.write_text(
            json.dumps([asdict(t) for t in _timers.values()], indent=2), encoding="utf-8"
        )
        tmp.replace(path)
    except OSError as exc:
        logger.warning("timers: could not save %s: %s", path, exc)


def _changed() -> None:
    _save()
    if _wake is not None:
        _wake.set()


def _reset_for_tests() -> None:
    global _loaded, _runner, _wake
    _timers.clear()
    _loaded = False
    _runner = None
    _wake = None


# ---- time parsing ---------------------------------------------------------

_CLOCK = re.compile(r"^\s*(\d{1,2})(?:[:.](\d{2}))?\s*([ap])\.?\s*m?\.?\s*$|^\s*(\d{1,2})[:.](\d{2})\s*$", re.I)


def _clock(text: str) -> tuple[int, int] | None:
    """Parse "7:30", "07:30", "19.05", "7pm", "7:30 am" into (hour, minute)."""
    m = _CLOCK.match(text)
    if not m:
        return None
    if m.group(4) is not None:
        hour, minute = int(m.group(4)), int(m.group(5))
    else:
        hour, minute = int(m.group(1)), int(m.group(2) or 0)
        if not 1 <= hour <= 12:
            return None
        hour = hour % 12 + (12 if m.group(3).lower() == "p" else 0)
    if hour > 23 or minute > 59:
        return None
    return hour, minute


def _matches(day: datetime, repeat: str) -> bool:
    if repeat == "weekdays":
        return day.weekday() < 5
    if repeat == "weekends":
        return day.weekday() >= 5
    return True


def next_occurrence(hour: int, minute: int, repeat: str, after: datetime) -> datetime:
    """The first local time at hour:minute after ``after`` that suits ``repeat``."""
    candidate = after.replace(hour=hour, minute=minute, second=0, microsecond=0)
    if candidate <= after:
        candidate += timedelta(days=1)
    while not _matches(candidate, repeat):
        candidate += timedelta(days=1)
    return candidate


def parse_at(text: str, now: datetime | None = None) -> datetime:
    """A local wall-clock time: "HH:MM" / "7pm" (next occurrence) or an ISO
    date and time such as "2026-10-14 07:30". Raises ValueError."""
    now = now or datetime.now()
    text = str(text).strip()
    clock = _clock(text)
    if clock is not None:
        return next_occurrence(*clock, "once", now)
    try:
        when = datetime.fromisoformat(text)
    except ValueError:
        raise ValueError(
            f"could not read the time {text!r}; use 24-hour HH:MM, '7pm' or 'YYYY-MM-DD HH:MM'"
        ) from None
    if when.tzinfo is not None:
        when = when.astimezone().replace(tzinfo=None)
    if when <= now:
        raise ValueError(f"{text} is in the past")
    return when


def _duration_s(arguments: dict[str, Any]) -> float:
    total = 0.0
    for key, scale in (("hours", 3600), ("minutes", 60), ("seconds", 1)):
        value = arguments.get(key, 0) or 0
        if not isinstance(value, (int, float)) or isinstance(value, bool) or value < 0:
            raise ValueError(f"{key} must be a non-negative number")
        total += value * scale
    if total < 1:
        raise ValueError("give a duration of at least 1 second (hours, minutes and/or seconds)")
    if total > MAX_DURATION_S:
        raise ValueError("durations are limited to 7 days")
    return total


def _say_duration(seconds: float) -> str:
    seconds = round(seconds)
    parts = []
    for unit, size in (("hour", 3600), ("minute", 60), ("second", 1)):
        count, seconds = divmod(seconds, size)
        if count:
            parts.append(f"{count} {unit}{'' if count == 1 else 's'}")
    return " and ".join(parts[:2]) if parts else "0 seconds"


# ---- the schedule ---------------------------------------------------------


def _message(arguments: dict[str, Any], *, required: bool) -> str:
    message = arguments.get("message", "")
    if not isinstance(message, str):
        raise ValueError("message must be text")
    message = " ".join(message.split())
    if required and not message:
        raise ValueError("message is required")
    if len(message) > MAX_MESSAGE_CHARS:
        raise ValueError(f"message is limited to {MAX_MESSAGE_CHARS} characters")
    return message


def add(kind: str, message: str, due: float, repeat: str = "once") -> Timer:
    _load()
    if len(_timers) >= MAX_TIMERS:
        raise ValueError(f"too many timers (limit {MAX_TIMERS}); cancel some first")
    timer = Timer(
        id=uuid.uuid4().hex[:6],
        kind=kind,
        message=message,
        due=due,
        repeat=repeat,
        created=time.time(),
    )
    _timers[timer.id] = timer
    _changed()
    logger.info("timers: added %s %s due %s", kind, timer.id, timer.describe()["due_local"])
    return timer


def pending() -> list[Timer]:
    _load()
    return sorted(_timers.values(), key=lambda t: t.due)


def cancel(timer_id: str) -> list[Timer]:
    _load()
    if timer_id == "all":
        removed = list(_timers.values())
        _timers.clear()
    else:
        removed = [_timers.pop(timer_id)] if timer_id in _timers else []
    if removed:
        _changed()
    return removed


def spoken_text(timer: Timer, *, late_s: float = 0) -> str:
    if timer.kind == "timer":
        text = timer.message or "Time's up! Your timer has finished."
    elif timer.kind == "alarm":
        clock = datetime.fromtimestamp(timer.due).strftime("%H:%M")
        text = timer.message or f"It's {clock}. This is your alarm!"
    else:
        text = f"Reminder: {timer.message}" if timer.message else "This is your reminder."
    if late_s >= 120:
        text = f"Sorry, this is {_say_duration(late_s)} late. {text}"
    return text


def register_tool_runner(runner: ToolRunner) -> None:
    """How the scheduler drives the robot (the shared command queue)."""
    global _runner
    _runner = runner


async def _run(name: str, args: dict[str, Any]) -> str:
    if _runner is None:
        raise RuntimeError("no tool runner registered")
    result = await _runner(name, args)
    texts = [getattr(item, "text", "") for item in result] if isinstance(result, list) else [str(result)]
    return " ".join(t for t in texts if t)


async def _announce(timer: Timer, late_s: float) -> bool:
    """Speak one timer. True once the robot has said it."""
    text = spoken_text(timer, late_s=late_s)
    try:
        await _run("set_avatar", {"face": "surprised"})
        result = await _run("say", {"text": text})
        if timer.kind == "alarm" and '"error"' not in result:
            await _run("gesture", {"name": "wiggle"})
            await _run("say", {"text": text})
        await _run("set_avatar", {"face": "idle"})
    except Exception as exc:  # robot busy, offline or the queue timed out
        logger.warning("timers: could not announce %s: %s", timer.id, exc)
        return False
    if '"error"' in result:
        logger.warning("timers: say failed for %s: %s", timer.id, result[:200])
        return False

    from . import activity

    activity.record("system", f"{timer.kind} went off: {text}", source="timer")
    await _notify_claude(timer, text)
    return True


async def _notify_claude(timer: Timer, text: str) -> None:
    from .brain import brain_mode

    if brain_mode() == "ollama":
        return
    from .stdio_server import notify_stackchan_event

    try:
        await notify_stackchan_event(
            "notifications/claude/channel",
            {
                "content": f"The {timer.kind} went off and the robot has already said: {text}",
                "meta": {
                    "event_type": "timer",
                    "kind": timer.kind,
                    "timer_id": timer.id,
                    "repeat": timer.repeat,
                    "ts_unix": str(time.time()),
                },
            },
        )
    except Exception as exc:  # pragma: no cover - transport failure
        logger.warning("timers: channel notification failed: %s", exc)


def _reschedule_or_remove(timer: Timer, now: float) -> None:
    if timer.kind == "alarm" and timer.repeat != "once":
        due = datetime.fromtimestamp(timer.due)
        timer.due = next_occurrence(
            due.hour, due.minute, timer.repeat, max(due, datetime.fromtimestamp(now))
        ).timestamp()
    else:
        _timers.pop(timer.id, None)
    _changed()


async def run_due(now: float | None = None) -> float:
    """Announce everything that is due; returns seconds until the next check."""
    _load()
    now = time.time() if now is None else now
    for timer in pending():
        if timer.due > now:
            break
        late_s = now - timer.due
        if late_s > LATE_GRACE_S:
            logger.info("timers: %s %s missed by %.0f s, skipping", timer.kind, timer.id, late_s)
            from . import activity

            activity.record("system", f"missed {timer.kind}: {timer.message}", source="timer")
            _reschedule_or_remove(timer, now)
            continue
        if not await _announce(timer, late_s):
            return RETRY_S
        _reschedule_or_remove(timer, time.time())
    upcoming = pending()
    if not upcoming:
        return MAX_SLEEP_S
    return min(MAX_SLEEP_S, max(0.0, upcoming[0].due - time.time()))


async def run_scheduler() -> None:
    """Background task: sleep until the next timer is due, then announce it."""
    global _wake
    _wake = asyncio.Event()
    _load()
    logger.info("timers: scheduler started with %d pending", len(_timers))
    while True:
        try:
            delay = await run_due()
        except asyncio.CancelledError:
            raise
        except Exception:  # keep the scheduler alive through any one bad timer
            logger.exception("timers: scheduler error")
            delay = RETRY_S
        _wake.clear()
        try:
            await asyncio.wait_for(_wake.wait(), timeout=delay)
        except asyncio.TimeoutError:
            pass


# ---- MCP tools ------------------------------------------------------------

_MESSAGE_PROP = {
    "type": "string",
    "maxLength": MAX_MESSAGE_CHARS,
}

TOOLS = [
    Tool(
        name="set_timer",
        description=(
            "Start a countdown timer. When it finishes the robot says "
            "`message` out loud (default: \"Time's up!\"), even if no "
            "Claude session is open. Give the length with hours, minutes "
            "and/or seconds. Returns the timer id."
        ),
        inputSchema={
            "type": "object",
            "properties": {
                "hours": {"type": "number", "minimum": 0},
                "minutes": {"type": "number", "minimum": 0},
                "seconds": {"type": "number", "minimum": 0},
                "message": {
                    **_MESSAGE_PROP,
                    "description": "Exactly what to say when it finishes, e.g. 'Your tea is ready!'.",
                },
            },
        },
    ),
    Tool(
        name="set_reminder",
        description=(
            "Remind the person about something later: the robot says "
            "\"Reminder: <message>\" out loud at that time. Use `at` for a "
            "local clock time ('18:30', '6:30pm' = next occurrence, or "
            "'2026-10-14 09:00'), or `in_minutes` for a delay."
        ),
        inputSchema={
            "type": "object",
            "properties": {
                "message": {**_MESSAGE_PROP, "description": "What to remind them about."},
                "at": {"type": "string", "description": "Local time, 24-hour HH:MM, '7pm' or 'YYYY-MM-DD HH:MM'."},
                "in_minutes": {"type": "number", "exclusiveMinimum": 0},
            },
            "required": ["message"],
        },
    ),
    Tool(
        name="set_alarm",
        description=(
            "Set an alarm for a local clock time. The robot shows a "
            "surprised face, wiggles and says `message` twice (default: "
            "\"It's HH:MM. This is your alarm!\"). `repeat` can make it go "
            "off every day, on weekdays or at weekends."
        ),
        inputSchema={
            "type": "object",
            "properties": {
                "time": {"type": "string", "description": "Local time: 24-hour HH:MM or '7am'."},
                "message": {**_MESSAGE_PROP, "description": "What to say when it goes off."},
                "repeat": {"type": "string", "enum": list(REPEATS), "default": "once"},
            },
            "required": ["time"],
        },
    ),
    Tool(
        name="list_timers",
        description="List pending timers, reminders and alarms, soonest first, with ids and local due times.",
        inputSchema={"type": "object", "properties": {}},
    ),
    Tool(
        name="cancel_timer",
        description="Cancel a timer, reminder or alarm by id (from list_timers), or every one with id 'all'.",
        inputSchema={
            "type": "object",
            "properties": {"id": {"type": "string"}},
            "required": ["id"],
        },
    ),
]
TOOL_NAMES = frozenset(tool.name for tool in TOOLS)


def _reply(payload: dict[str, Any]) -> list[TextContent]:
    return [TextContent(type="text", text=json.dumps(payload))]


async def handle_tool(name: str, arguments: dict[str, Any]) -> list[TextContent]:
    try:
        if name == "set_timer":
            seconds = _duration_s(arguments)
            message = _message(arguments, required=False)
            timer = add("timer", message, time.time() + seconds)
            return _reply({"ok": True, "length": _say_duration(seconds), **timer.describe()})

        if name == "set_reminder":
            message = _message(arguments, required=True)
            at, in_minutes = arguments.get("at"), arguments.get("in_minutes")
            if (at is None) == (in_minutes is None):
                raise ValueError("give either `at` or `in_minutes`")
            if at is not None:
                due = parse_at(at).timestamp()
            else:
                due = time.time() + _duration_s({"minutes": in_minutes})
            return _reply({"ok": True, **add("reminder", message, due).describe()})

        if name == "set_alarm":
            repeat = arguments.get("repeat", "once")
            if repeat not in REPEATS:
                raise ValueError(f"repeat must be one of {', '.join(REPEATS)}")
            clock = _clock(str(arguments.get("time", "")))
            if clock is None:
                raise ValueError("time must be a local clock time like '07:30' or '7am'")
            due = next_occurrence(*clock, repeat, datetime.now()).timestamp()
            message = _message(arguments, required=False)
            return _reply({"ok": True, **add("alarm", message, due, repeat).describe()})

        if name == "list_timers":
            return _reply({"ok": True, "timers": [t.describe() for t in pending()]})

        if name == "cancel_timer":
            timer_id = str(arguments.get("id", "")).strip()
            removed = cancel(timer_id)
            if not removed:
                raise ValueError(f"no timer with id {timer_id!r}")
            return _reply({"ok": True, "cancelled": [t.describe() for t in removed]})
    except ValueError as exc:
        return _reply({"ok": False, "error": str(exc)})
    return _reply({"ok": False, "error": f"unknown timer tool {name!r}"})
