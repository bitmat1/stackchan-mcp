#!/usr/bin/env python3
"""Test a synth's tuning stability with Stack-chan pressing a key.

The robot looks down so its lever holds a key, records the note on its
microphone, and shows a tuning meter on the gateway's control page
(http://127.0.0.1:8767/ui). The Mac does all the analysis. Each
Access Virus patch gets --presses presses, then a MIDI program change moves
to the next. A patch whose first --probe presses have no steady pitch (heavy
modulation, noise, silence) is skipped.

The gateway must be running with the record_audio tool. Run from the repo:

    uv run --directory gateway --with python-rtmidi python ../scripts/tuning_test.py --list-midi
    uv run --directory gateway --with python-rtmidi python ../scripts/tuning_test.py --midi-port Virus

Every press goes to presses.csv and each patch's summary to patches.csv in
--out. Ctrl-C stops cleanly; --start-bank and --start-program resume.
"""

from __future__ import annotations

import argparse
import asyncio
import csv
import json
import os
import sys
import tempfile
import time
from pathlib import Path

import httpx
from dotenv import load_dotenv
from mcp import ClientSession
from mcp.client.streamable_http import streamablehttp_client

from stackchan_mcp.tuning import PitchReading, analyse, assess_patch, read_wav, relative_cents, summarise


class Robot:
    """The gateway's MCP tools."""

    def __init__(self, session: ClientSession) -> None:
        self.session = session

    async def call(self, tool: str, **arguments) -> dict:
        result = await self.session.call_tool(tool, arguments)
        text = next((c.text for c in result.content if hasattr(c, "text")), "{}")
        try:
            data = json.loads(text)
        except ValueError:
            data = {"text": text}
        if result.isError or (isinstance(data, dict) and data.get("error")):
            raise RuntimeError(f"{tool}: {data.get('error', text) if isinstance(data, dict) else text}")
        return data

    async def wait_until_back(self, timeout_s: float = 180.0) -> None:
        """Ride out a robot reboot; the gateway reports when it reconnects."""
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            await asyncio.sleep(2)
            try:
                if (await self.call("get_status")).get("connected"):
                    await asyncio.sleep(5)  # Let it finish starting up.
                    return
            except RuntimeError:
                pass
        raise RuntimeError(f"robot did not come back within {timeout_s:.0f} s")

    async def press_and_measure(self, args: argparse.Namespace, wav_path: str) -> PitchReading | None:
        try:
            await self.call("move_head", yaw=args.yaw, pitch=args.press_pitch, speed=args.speed)
            try:
                await asyncio.sleep(args.settle_ms / 1000)
                await self.call("record_audio", duration_ms=args.record_ms, path=wav_path)
            finally:
                await self.call("move_head", yaw=args.yaw, pitch=args.rest_pitch, speed=args.speed)
            await asyncio.sleep(args.release_ms / 1000)
            pcm, sample_rate = read_wav(wav_path)
        finally:
            # Only the measurements are kept, never the audio.
            Path(wav_path).unlink(missing_ok=True)
        return analyse(pcm, sample_rate, a4=args.a4)


class Virus:
    """Patch changes over CoreMIDI. Without a port, patches stay as they are."""

    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.out = None
        if not args.midi_port:
            return
        import rtmidi

        out = rtmidi.MidiOut()
        ports = out.get_ports()
        match = next((i for i, p in enumerate(ports) if args.midi_port.lower() in p.lower()), None)
        if match is None:
            sys.exit(f"No MIDI output matching {args.midi_port!r}. Found: {ports}")
        out.open_port(match)
        self.out = out
        print(f"MIDI: {ports[match]}, channel {args.channel}")

    def select(self, bank: int, program: int) -> None:
        if self.out is None:
            return
        channel = self.args.channel - 1
        for cc in self.args.bank_cc:
            self.out.send_message([0xB0 | channel, cc, bank])
        self.out.send_message([0xC0 | channel, program])

    def close(self) -> None:
        if self.out is not None:
            self.out.close_port()


class Tuner:
    """Publishes readings to the control page's meter."""

    def __init__(self, client: httpx.AsyncClient, url: str) -> None:
        self.client = client
        self.url = url
        self.last: dict = {}

    async def publish(self, title: str, press: int, presses: int, reading, stats: dict) -> None:
        shown = reading and {
            "note": reading.note,
            "frequency": float(reading.frequency),
            "cents": float(reading.cents),
        }
        self.last = {"title": title, "press": press, "presses": presses, "active": True, "reading": shown, "stats": stats}
        await self._send()

    async def finish(self) -> None:
        """Leave the last result on the page, marked finished."""
        self.last["active"] = False
        await self._send()

    async def _send(self) -> None:
        try:
            await self.client.post(self.url, json=self.last)
        except httpx.HTTPError as exc:
            print(f"  control page: {exc}")


def describe(reading: PitchReading | None) -> str:
    if reading is None:
        return "no pitch"
    text = f"{reading.note} {reading.cents:+.1f}c {reading.frequency:.2f} Hz"
    return text if reading.valid else f"{text} (unsteady, spread {reading.spread_cents:.0f}c)"


def writer_for(path: Path, fields: list[str]) -> tuple[csv.DictWriter, object]:
    is_new = not path.exists()
    handle = path.open("a", newline="")
    writer = csv.DictWriter(handle, fieldnames=fields, extrasaction="ignore")
    if is_new:
        writer.writeheader()
    return writer, handle


async def test_patch(robot: Robot, virus: Virus, tuner: Tuner, args, bank: int, program: int, presses_csv, press_file) -> dict:
    label = f"{bank}:{program:03d}"
    virus.select(bank, program)
    await asyncio.sleep(args.patch_settle_ms / 1000)

    readings: list[PitchReading | None] = []
    wav_path = str(Path(tempfile.gettempdir()) / f"stackchan-tuning-{os.getpid()}.wav")
    title = f"Patch {label}" if virus.out is not None else "Current patch"
    outcome = "tested"
    await tuner.publish(title, 0, args.presses, None, {})
    for press in range(1, args.presses + 1):
        try:
            reading = await robot.press_and_measure(args, wav_path)
        except RuntimeError as exc:
            # A knock can claim the microphone for a moment, and the robot
            # can reboot; either way, carry on from the next press.
            print(f"  press {press}: {exc}")
            await robot.wait_until_back()
            reading = None
        readings.append(reading)
        print(f"{label} press {press}/{args.presses}: {describe(reading)}")

        row = {"time": time.strftime("%H:%M:%S"), "bank": bank, "program": program, "press": press}
        if reading is not None:
            row.update(reading.to_dict())
        presses_csv.writerow(row)
        press_file.flush()

        stats = summarise(readings, alert_cents=args.alert_cents)
        await tuner.publish(title, press, args.presses, reading, stats)
        await asyncio.sleep(args.show_ms / 1000)

        if press == args.probe:
            ok, reason = assess_patch(
                readings, min_valid=args.probe_min_valid, max_deviation_cents=args.probe_max_cents
            )
            if not ok:
                print(f"{label} skipped: {reason}")
                outcome = f"skipped: {reason}"
                break

    summary = summarise(readings, alert_cents=args.alert_cents)
    good = [r for r in readings if r is not None and r.valid]
    if good:
        for reading, offset in zip(good, relative_cents([r.frequency for r in good])):
            if abs(offset) >= args.alert_cents:
                print(f"{label} OUT OF TUNE: {reading.note} {reading.frequency:.2f} Hz, {offset:+.1f}c from the patch median")
    return {"bank": bank, "program": program, "outcome": outcome, **summary}


async def run(args: argparse.Namespace) -> None:
    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    press_fields = ["time", "bank", "program", "press", *PitchReading.__dataclass_fields__]
    patch_fields = [
        "bank", "program", "outcome", "presses", "valid", "note", "midi", "median_hz", "cents",
        "mean_cents", "stderr_cents", "stdev_cents", "lowest_cents", "highest_cents", "outliers",
    ]
    presses_csv, press_file = writer_for(out_dir / "presses.csv", press_fields)
    patches_csv, patch_file = writer_for(out_dir / "patches.csv", patch_fields)

    token = os.getenv("STACKCHAN_TOKEN") or os.getenv("BEARER_TOKEN")
    headers = {"Authorization": f"Bearer {token}"} if token else None
    virus = Virus(args)
    http = httpx.AsyncClient(headers=headers, timeout=5)
    tuner = Tuner(http, args.url.rsplit("/", 1)[0] + "/api/tuner")
    async with streamablehttp_client(args.url, headers=headers) as (read, write, _):
        async with ClientSession(read, write) as session:
            await session.initialize()
            robot = Robot(session)
            await robot.call("move_head", yaw=args.yaw, pitch=args.rest_pitch, speed=args.speed)
            await asyncio.sleep(1)
            try:
                bank, program = args.start_bank, args.start_program
                for _ in range(args.patches):
                    summary = await test_patch(robot, virus, tuner, args, bank, program, presses_csv, press_file)
                    patches_csv.writerow(summary)
                    patch_file.flush()
                    print(summary)
                    program += 1
                    if program > 127:
                        bank, program = bank + 1, 0
            finally:
                # Leave the lever up, whatever happened.
                await robot.call("move_head", yaw=args.yaw, pitch=args.rest_pitch, speed=args.speed)
    await tuner.finish()
    await http.aclose()
    virus.close()
    press_file.close()
    patch_file.close()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--list-midi", action="store_true", help="list MIDI outputs and exit")
    parser.add_argument("--midi-port", help="part of the MIDI output's name; omit to test the current patch only")
    parser.add_argument("--channel", type=int, default=1, help="Virus MIDI channel, 1-16")
    parser.add_argument("--bank-cc", type=int, nargs="*", default=[0, 32], help="bank select controllers to send")
    parser.add_argument("--start-bank", type=int, default=0)
    parser.add_argument("--start-program", type=int, default=0)
    parser.add_argument("--patches", type=int, default=128, help="how many patches to test")
    parser.add_argument("--presses", type=int, default=20, help="presses per patch")
    parser.add_argument("--probe", type=int, default=5, help="presses before deciding whether to skip")
    parser.add_argument("--probe-min-valid", type=int, default=3, help="steady presses needed in the probe")
    parser.add_argument("--probe-max-cents", type=float, default=15.0, help="largest median wander in the probe")
    parser.add_argument("--alert-cents", type=float, default=20.0, help="report presses this far from the median")
    parser.add_argument("--a4", type=float, default=440.0)
    parser.add_argument("--yaw", type=int, default=0)
    parser.add_argument("--press-pitch", type=int, default=55, help="head pitch that holds the key down")
    parser.add_argument("--rest-pitch", type=int, default=75, help="head pitch with the key released")
    parser.add_argument("--speed", default="mid", help="head speed: low, mid or high")
    parser.add_argument("--settle-ms", type=int, default=600, help="wait after pressing, past the attack")
    parser.add_argument("--record-ms", type=int, default=3000, help="take length; longer averages out patch movement")
    parser.add_argument("--release-ms", type=int, default=1000, help="wait after releasing, past the note's tail")
    parser.add_argument("--show-ms", type=int, default=1000, help="how long each result stays up before the next press")
    parser.add_argument("--patch-settle-ms", type=int, default=800, help="wait after a patch change")
    parser.add_argument("--url", default="http://127.0.0.1:8767/mcp")
    parser.add_argument("--out", default=str(Path.home() / ".stackchan" / "tuning"))
    args = parser.parse_args()
    if args.probe > args.presses or args.probe_min_valid > args.probe:
        parser.error("need probe-min-valid <= probe <= presses")
    return args


def main() -> None:
    load_dotenv(Path(__file__).resolve().parent.parent / "gateway" / ".env")
    args = parse_args()
    if args.list_midi:
        import rtmidi

        print("\n".join(rtmidi.MidiOut().get_ports()) or "No MIDI outputs")
        return
    if not args.midi_port:
        args.patches = 1
    try:
        asyncio.run(run(args))
    except KeyboardInterrupt:
        print("Stopped.")


if __name__ == "__main__":
    main()
