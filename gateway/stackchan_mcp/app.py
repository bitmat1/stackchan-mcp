"""``stackchan-mcp app``: one launcher for the gateway and its control panel.

Starts the Streamable HTTP gateway (unless one is already running), opens
the web control panel in its own app-style window, and - when the Claude
brain is selected - opens the Claude Code voice session in Terminal. With
the local Ollama brain no Claude session is needed. Quitting the launcher
(Cmd-Q on Stack-chan.app, or Ctrl-C) stops the gateway it started.

The launcher also serves a tiny page on 127.0.0.1:8768 that outlives the
gateway: the panel's "Close gateway" button shuts the gateway down, and
"Open gateway" asks this launcher to start it again. The app window opens
the launcher page, which forwards to the panel whenever the gateway is up.
Starting requires a per-launch random token that the panel reads from the
gateway's own (Origin-checked) API, so other web pages cannot use it.

``scripts/make-app.sh`` wraps this command in a double-clickable
``Stack-chan.app``.
"""

from __future__ import annotations

import html
import json
import os
import secrets
import shlex
import shutil
import signal
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

GATEWAY_DIR = Path(__file__).resolve().parent.parent
VOICE_SESSION_DIR = GATEWAY_DIR.parent / "voice-session"
LOG_PATH = Path.home() / "Library" / "Logs" / "stackchan-gateway.log"
#: Variables this launcher took from .env, left out of each gateway's
#: environment so the gateway reads .env afresh and picks up edits.
_dotenv_keys: set[str] = set()
#: Chromium browsers that can open a page as a chromeless app window.
APP_WINDOW_BROWSERS = ("Google Chrome", "Microsoft Edge", "Brave Browser", "Chromium")


def base_url() -> str:
    host = os.getenv("MCP_HTTP_HOST", "127.0.0.1")
    if host in {"0.0.0.0", "::"}:
        host = "127.0.0.1"
    return f"http://{host}:{os.getenv('MCP_HTTP_PORT', '8767')}"


def gateway_healthy(url: str, timeout: float = 1.0) -> bool:
    try:
        with urllib.request.urlopen(f"{url}/healthz", timeout=timeout) as resp:
            return resp.status == 200
    except (urllib.error.URLError, OSError):
        return False


def start_gateway(extra_env: dict[str, str] | None = None) -> subprocess.Popen[bytes]:
    LOG_PATH.parent.mkdir(parents=True, exist_ok=True)
    log = open(LOG_PATH, "ab")  # noqa: SIM115 - handed to the child process
    return subprocess.Popen(
        [sys.executable, "-m", "stackchan_mcp.cli", "serve", "--transport", "streamable-http"],
        cwd=GATEWAY_DIR,
        stdout=log,
        stderr=subprocess.STDOUT,
        env={
            **{k: v for k, v in os.environ.items() if k not in _dotenv_keys},
            **(extra_env or {}),
        },
    )


class GatewayControl:
    """Starts the gateway on request and remembers the process it owns."""

    def __init__(self, url: str, launcher_url: str, token: str) -> None:
        self.url = url
        self.launcher_url = launcher_url
        self.token = token
        self.process: subprocess.Popen[bytes] | None = None
        self._lock = threading.Lock()

    def running(self) -> bool:
        return gateway_healthy(self.url)

    def start(self, wait_s: int = 60) -> tuple[bool, str]:
        with self._lock:
            if self.running():
                return True, "already running"
            self.process = start_gateway(
                {
                    "STACKCHAN_LAUNCHER_URL": self.launcher_url,
                    "STACKCHAN_LAUNCHER_TOKEN": self.token,
                    # The gateway exits if this launcher dies (see http_server).
                    "STACKCHAN_LAUNCHER_PID": str(os.getpid()),
                }
            )
            for _ in range(wait_s):
                if self.running():
                    return True, "started"
                if self.process.poll() is not None:
                    return False, f"the gateway exited; see {LOG_PATH}"
                time.sleep(1)
            return False, f"the gateway did not come up within {wait_s} s; see {LOG_PATH}"

    def stop(self) -> None:
        with self._lock:
            if self.process is not None and self.process.poll() is None:
                self.process.terminate()
                try:
                    self.process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    self.process.kill()


LAUNCHER_PAGE = """<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Stack-chan</title>
<style>
:root {{ --bg:#f6f5f2; --text:#1d1d1f; --muted:#6b6b70; --accent:#d9622b; }}
@media (prefers-color-scheme: dark) {{ :root {{ --bg:#161618; --text:#ececee; --muted:#9a9aa2; --accent:#ef7d45; }} }}
body {{ margin:0; min-height:100vh; display:grid; place-items:center; background:var(--bg);
  color:var(--text); font:16px/1.5 -apple-system, BlinkMacSystemFont, sans-serif; text-align:center; padding:16px; }}
button {{ font:inherit; padding:10px 22px; border-radius:10px; border:0; background:var(--accent);
  color:#fff; cursor:pointer; }}
button:disabled {{ opacity:.5; cursor:wait; }}
p {{ color:var(--muted); }}
</style></head><body><main>
<h1>Stack-chan</h1>
<p id="msg">The gateway is stopped.</p>
<button id="open">Open gateway</button>
</main><script>
const panel = {panel};
const token = {token};
async function poll() {{
  try {{ const s = await (await fetch("/status")).json(); if (s.running) location.replace(panel); }} catch {{}}
}}
document.getElementById("open").onclick = async (e) => {{
  e.target.disabled = true;
  document.getElementById("msg").textContent = "Starting the gateway...";
  try {{
    const r = await (await fetch("/start", {{ method: "POST", headers: {{ "X-Launcher-Token": token }} }})).json();
    if (r.ok) location.replace(panel); else document.getElementById("msg").textContent = r.message;
  }} catch (err) {{ document.getElementById("msg").textContent = String(err); }}
  e.target.disabled = false;
}};
setInterval(poll, 3000);
</script></body></html>
"""


def make_launcher_handler(control: GatewayControl) -> type[BaseHTTPRequestHandler]:
    panel_url = f"{control.url}/ui"

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_args: object) -> None:  # keep the app log quiet
            pass

        def _cors(self) -> None:
            # Only the panel (served by the gateway) may call the launcher.
            self.send_header("Access-Control-Allow-Origin", control.url)
            self.send_header("Access-Control-Allow-Headers", "X-Launcher-Token")
            self.send_header("Access-Control-Allow-Methods", "GET, POST")
            self.send_header("Vary", "Origin")

        def _json(self, status: int, body: dict[str, object]) -> None:
            data = json.dumps(body).encode()
            self.send_response(status)
            self._cors()
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_OPTIONS(self) -> None:  # noqa: N802 - http.server API
            self.send_response(204)
            self._cors()
            self.end_headers()

        def do_GET(self) -> None:  # noqa: N802
            if self.path == "/status":
                self._json(200, {"running": control.running()})
            elif self.path == "/":
                if control.running():
                    self.send_response(302)
                    self.send_header("Location", panel_url)
                    self.end_headers()
                    return
                page = LAUNCHER_PAGE.format(
                    panel=json.dumps(panel_url), token=html.escape(json.dumps(control.token), quote=False)
                ).encode()
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(page)))
                self.end_headers()
                self.wfile.write(page)
            else:
                self._json(404, {"error": "not found"})

        def do_POST(self) -> None:  # noqa: N802
            if self.path != "/start":
                self._json(404, {"error": "not found"})
                return
            if not secrets.compare_digest(self.headers.get("X-Launcher-Token", ""), control.token):
                self._json(403, {"ok": False, "message": "bad launcher token"})
                return
            ok, message = control.start()
            self._json(200 if ok else 500, {"ok": ok, "message": message})

    return Handler


def panel_window_command(url: str, apps_dir: Path = Path("/Applications")) -> list[str]:
    """``open`` command for the panel: an app window if a Chromium browser
    is installed, otherwise a tab in the default browser."""
    for name in APP_WINDOW_BROWSERS:
        if (apps_dir / f"{name}.app").exists():
            return ["open", "-na", name, "--args", f"--app={url}", "--window-size=1100,900"]
    return ["open", url]


def launcher_running(launcher_url: str) -> bool:
    try:
        with urllib.request.urlopen(f"{launcher_url}/status", timeout=2) as resp:
            return "running" in json.loads(resp.read())
    except (urllib.error.URLError, OSError, ValueError):
        return False


def claude_binary() -> str | None:
    found = shutil.which("claude")
    if found:
        return found
    fallback = Path.home() / ".local" / "bin" / "claude"
    return str(fallback) if fallback.exists() else None


def claude_session_script(claude: str, session_dir: Path = VOICE_SESSION_DIR) -> str:
    """AppleScript that opens the Claude Code voice session in Terminal."""
    shell = (
        f"cd {shlex.quote(str(session_dir))} && "
        f"{shlex.quote(claude)} --dangerously-load-development-channels server:stackchan"
    )
    escaped = shell.replace("\\", "\\\\").replace('"', '\\"')
    return f'tell application "Terminal"\n  activate\n  do script "{escaped}"\nend tell'


def open_claude_voice_session() -> None:
    """Open the Claude Code voice session in a new Terminal window (macOS)."""
    if sys.platform != "darwin":
        raise RuntimeError("opening the voice session is only supported on macOS")
    claude = claude_binary()
    if claude is None:
        raise RuntimeError("the claude command was not found; install Claude Code first")
    if not VOICE_SESSION_DIR.is_dir():
        raise RuntimeError(f"{VOICE_SESSION_DIR} is missing")
    subprocess.run(["osascript", "-e", claude_session_script(claude)], check=True)


def _alert(message: str) -> None:
    print(message, file=sys.stderr)
    if sys.platform == "darwin":
        script = f'display alert "Stack-chan" message "{message.replace(chr(34), chr(39))}"'
        subprocess.run(["osascript", "-e", script], check=False)


def run_app() -> int:
    from .brain import brain_mode
    from .cli import _load_dotenv

    before = set(os.environ)
    _load_dotenv()
    _dotenv_keys.update(set(os.environ) - before)
    url = base_url()
    port = int(os.getenv("STACKCHAN_LAUNCHER_PORT", "8768"))
    launcher_url = f"http://127.0.0.1:{port}"
    control = GatewayControl(url, launcher_url, secrets.token_urlsafe(24))
    try:
        server = ThreadingHTTPServer(("127.0.0.1", port), make_launcher_handler(control))
    except OSError as exc:
        if launcher_running(launcher_url):
            print("Stack-chan is already running; opening its window.")
            subprocess.run(panel_window_command(launcher_url), check=False)
            return 0
        _alert(f"Port {port} is busy ({exc}) and is not a Stack-chan launcher.")
        return 1
    threading.Thread(target=server.serve_forever, daemon=True).start()

    if control.running():
        print("Gateway already running (started outside the app; Open gateway will not restart it).")
    else:
        print(f"Starting gateway (log: {LOG_PATH})...")
        ok, message = control.start()
        if not ok:
            _alert(f"The gateway failed to start: {message}")

    subprocess.run(panel_window_command(launcher_url), check=False)
    if brain_mode() == "claude":
        try:
            open_claude_voice_session()
        except (RuntimeError, subprocess.CalledProcessError) as exc:
            print(f"Claude voice session not opened: {exc}", file=sys.stderr)

    quit_event = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: quit_event.set())
    signal.signal(signal.SIGINT, lambda *_: quit_event.set())
    print(f"Stack-chan is running ({launcher_url}). Quit the app or press Ctrl-C to stop.")
    quit_event.wait()
    server.shutdown()
    control.stop()
    return 0
