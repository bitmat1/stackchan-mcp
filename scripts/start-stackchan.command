#!/bin/zsh
# Double-click to start Stack-chan from Terminal: the gateway, the control
# panel and (with the Claude brain) the Claude voice session in a second
# Terminal window. Ctrl+C here stops the gateway if this script started it.
# Same as Stack-chan.app (scripts/make-app.sh) and `stackchan-mcp app`.

ROOT="/Users/mat/CODE/stackchan-claude"
export PATH="/opt/homebrew/bin:$HOME/.local/bin:$PATH"

cd "$ROOT/gateway" && exec uv run stackchan-mcp app
