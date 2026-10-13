# Stack-chan RT ver. as a voice for Claude

This fork runs on rt-net's **Stack-chan RT ver.** (M5Stack CoreS3 + two
DYNAMIXEL XL330-M288-T servos) and connects it to Claude on a Mac, with
speech and listening done locally.

```
 you ──speech──▶ CoreS3 mic ─Wi-Fi─▶ gateway daemon ──local Whisper──▶ Claude Code (channel)
                                       │      ▲                             │
 web panel  ◀──────── /ui ─────────────┤      └──── say / set_avatar / ◀────┘
 Claude Desktop ◀──── MCP ─────────────┘           move_head / take_photo
```

## What changed from upstream

| Area | Change |
| --- | --- |
| Servos | `CONFIG_STACKCHAN_SERVO_DYNAMIXEL`: DYNAMIXEL Protocol 2.0 driver (`firmware/components/dynamixel_bus`), RT pins TX=7 / RX=6, current-based position mode with time-based profiles. |
| Face | `CONFIG_STACKCHAN_PROCEDURAL_FACE`: 27 drawn expressions with animated overlays (`firmware/components/procedural_face`). Preview: `docs/images/procedural-faces.png`. |
| Build | New `stackchan-rt` build variant in `firmware/main/boards/stackchan/config.json` with both options on. |
| Voice out | Local TTS engines `macos-say` (built in) and `kokoro` (neural, optional). |
| Voice in | `STACKCHAN_VOICE_CHANNEL=1`: wake word / button capture → local faster-whisper → Claude Code channel message. |
| Claude Code | `stackchan-mcp channel`: stdio bridge that proxies tools to the daemon and relays channel events. |
| Web panel | `http://127.0.0.1:8767/ui` on the daemon: face, head, speech, listening, camera, settings and every MCP tool. |

## 1. Build and flash the firmware

```bash
cd firmware
docker run --rm --cpus=4 --ulimit nofile=65536:65536 -v $PWD:/project -w /project \
  espressif/idf:v5.5.2 python ./scripts/release.py stackchan --name stackchan-rt
uvx esptool --chip esp32s3 --port /dev/cu.usbmodem1101 -b 460800 write_flash 0x0 build/merged-binary.bin
```

On first boot the robot opens a Wi-Fi setup hotspot (xiaozhi's standard
flow): join it from a phone and enter your Wi-Fi details. It then finds the
gateway on the LAN by mDNS.

If the head moves the wrong way, flip `CONFIG_STACKCHAN_DXL_YAW_INVERT` /
`CONFIG_STACKCHAN_DXL_PITCH_INVERT`; tilt range and servo current limits are
also in `idf.py menuconfig` under the StackChan board options.

## 2. Run the gateway daemon on the Mac

```bash
cd gateway
uv sync --extra tts --extra stt --extra stt-faster-whisper
uv run stackchan-mcp serve --transport streamable-http
```

Settings live in `gateway/.env` (gitignored). The important ones:

| Variable | Purpose |
| --- | --- |
| `STACKCHAN_TTS_ENGINE` | `macos-say` (default here) or `kokoro`. |
| `STACKCHAN_MACOS_SAY_VOICE` | Any voice from `say -v '?'`; Enhanced/Premium voices sound best. |
| `STACKCHAN_STT_LANGUAGE` | Default language for `listen()`, e.g. `en`. |
| `STACKCHAN_VOICE_CHANNEL` | `1` sends robot-initiated speech to Claude Code. |
| `STACKCHAN_VOICE_STT_MODEL` | faster-whisper model, e.g. `small.en` (downloaded on first use). |
| `VISION_HOST` | This Mac's LAN IP, so the robot can upload photos. |

Open the control panel at <http://127.0.0.1:8767/ui>.

### Optional: Kokoro neural voice

```bash
uv sync --extra tts --extra stt --extra stt-faster-whisper --extra tts-kokoro
mkdir -p ~/.cache/stackchan-mcp/kokoro
# download kokoro-v1.0.onnx and voices-v1.0.bin from
# https://github.com/thewh1teagle/kokoro-onnx/releases into that folder
```

Then set `STACKCHAN_TTS_ENGINE=kokoro` (voices such as `bf_emma`,
`bm_george`, `af_heart`).

## 3. Hands-free conversation (Claude Code channel)

Channels need an interactive Claude Code terminal session (v2.1.80+, signed
in with claude.ai) and are a research preview, so the development flag is
required for this custom channel:

```bash
cd voice-session
claude --dangerously-load-development-channels server:stackchan
```

`voice-session/.mcp.json` starts the bridge and `voice-session/CLAUDE.md`
sets the robot persona. Press the robot's button (or say the wake word) and
speak; the transcript arrives in the session as a `<channel>` message and
Claude answers through `say`, with a matching face and head gesture.

## 4. Claude Desktop

Add the bridge to `~/Library/Application Support/Claude/claude_desktop_config.json`
(Claude Desktop ignores channel events, but gets every robot tool):

```json
{
  "mcpServers": {
    "stackchan": {
      "command": "/opt/homebrew/bin/uv",
      "args": ["run", "--directory", "/Users/mat/CODE/stackchan-claude/gateway",
               "stackchan-mcp", "channel"]
    }
  }
}
```

Restart Claude Desktop, then ask it to "say hello through Stack-chan and look
happy" or "take a photo and tell me what you see".

## 5. One app: Stack-chan.app

```bash
scripts/make-app.sh          # builds ~/Applications/Stack-chan.app
```

Double-clicking **Stack-chan.app** starts the gateway (unless one is already
running), opens the control panel in its own window (an app window in
Chrome / Edge / Brave if installed, otherwise a browser tab) and, with the
Claude brain, opens the Claude voice session in Terminal. Quitting the app
stops the gateway it started. The same thing runs from a terminal with
`uv run stackchan-mcp app` or `scripts/start-stackchan.command`. Logs:
`~/Library/Logs/stackchan-app.log` and `~/Library/Logs/stackchan-gateway.log`.
Re-run `make-app.sh` after moving the repository.

The app also runs a small launcher on `127.0.0.1:8768` (override with
`STACKCHAN_LAUNCHER_PORT`) that outlives the gateway. The panel's **Close
gateway** button shuts the gateway down; **Open gateway** asks the launcher
to start it again (with a per-launch token the panel reads from the gateway,
so other web pages cannot). The app window opens the launcher page, which
forwards to the panel whenever the gateway is up. A gateway started outside
the app can be closed from the panel but must be reopened from the app.

The panel also shows **Robot health** (battery, Wi-Fi with RSSI, memory and
its low point, uptime, chip temperature, head angles, held / someone-near,
queue and brain; refreshed every 15 s) and an **Activity** feed (minimise /
reopen, clear, hide tool calls) with everything since the gateway started:
what you said, what the robot said, every tool call and who made it
(claude / panel / ollama), robot events and connection changes.

## 6. Local brain: Ollama instead of Claude

The panel's **Brain** card switches who answers when you talk to the robot.
The choice is saved in `~/Library/Application Support/stackchan-mcp/brain.json`.

- **Claude**: speech goes to the Claude Code voice session (section 3).
- **Ollama (local)**: the gateway answers by itself, with no internet and no
  Claude session. Start the Ollama app, pick a model in the panel (by default
  the first installed llama or qwen model), then talk to the robot or use the
  card's *Ask* box.

The model replies as `[face] [gesture] text`; the gateway shows the face,
plays the gesture and speaks the text, keeping the last 8 exchanges as
memory. On an M1 Pro, `llama3.1:8b` generates ~27 tokens/s, so the answer
itself takes a few seconds; the rest of a turn is the face, gesture and
speech. Environment overrides: `STACKCHAN_BRAIN=claude|ollama`,
`STACKCHAN_OLLAMA_MODEL`, `OLLAMA_URL` (default `http://127.0.0.1:11434`).
The Ollama brain needs the HTTP daemon (`serve --transport streamable-http`,
which the app runs), because it uses the daemon's command queue.

## Previewing faces without a robot

```bash
cd firmware
c++ -std=c++17 -O2 -I components/procedural_face components/procedural_face/procedural_face.cc \
  host_test/face_preview.cc -o /tmp/face_preview
/tmp/face_preview /tmp/faces.ppm && sips -s format png /tmp/faces.ppm --out /tmp/faces.png
```
