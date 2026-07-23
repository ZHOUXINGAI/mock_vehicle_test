#!/usr/bin/env bash
set -euo pipefail

agent="${1:-}"
case "$agent" in
  orin1-carrier)
    expected_user=jetson
    display="${DISPLAY:-:0}"
    ;;
  orin2-mini)
    expected_user=seeed
    display="${DISPLAY:-:0}"
    ;;
  *)
    echo "usage: $0 <orin1-carrier|orin2-mini>" >&2
    exit 2
    ;;
esac

if [[ "$(id -un)" != "$expected_user" ]]; then
  echo "expected user $expected_user, got $(id -un)" >&2
  exit 1
fi

repo="$HOME/mock_vehicle_test"
viewer="$repo/codex_ops/scripts/watch_agent_chat.sh"
runtime_dir="/run/user/$(id -u)"
export DISPLAY="$display"
export XDG_RUNTIME_DIR="$runtime_dir"
export DBUS_SESSION_BUS_ADDRESS="unix:path=$runtime_dir/bus"

if [[ ! -S "$runtime_dir/bus" ]]; then
  echo "desktop D-Bus is unavailable at $runtime_dir/bus" >&2
  exit 1
fi

nohup gnome-terminal \
  --title="$agent Codex Chat" \
  -- "$viewer" "$agent" \
  >"$repo/codex_ops/local/$agent/chat-window-launch.log" 2>&1 &

printf 'Opened local %s Codex Chat window on DISPLAY=%s\n' "$agent" "$DISPLAY"
