#!/usr/bin/env bash
set -euo pipefail

agent="${1:-}"
case "$agent" in
  orin1-carrier)
    expected_user=jetson
    home=/home/jetson
    ;;
  orin2-mini)
    expected_user=seeed
    home=/home/seeed
    ;;
  *)
    echo "usage: $0 <orin1-carrier|orin2-mini>" >&2
    exit 2
    ;;
esac

if [[ "$(id -un)" != "$expected_user" ]]; then
  echo "refusing to launch $agent as $(id -un); expected $expected_user" >&2
  exit 1
fi

if [[ "${CODEX_APP_BRIDGE_LOGIN_ENV:-}" != 1 ]]; then
  export CODEX_APP_BRIDGE_LOGIN_ENV=1
  export CODEX_APP_BRIDGE_SCRIPT
  export CODEX_APP_BRIDGE_AGENT="$agent"
  CODEX_APP_BRIDGE_SCRIPT="$(readlink -f "$0")"
  exec bash -lic 'exec "$CODEX_APP_BRIDGE_SCRIPT" "$CODEX_APP_BRIDGE_AGENT"'
fi

service="codex-agentd-$agent.service"
if systemctl is-active --quiet "$service"; then
  echo "refusing competing task consumers: $service is still active" >&2
  echo "stop only that coordination service before launching this bridge" >&2
  exit 1
fi

repo="$home/mock_vehicle_test"
installed_config="/etc/codex-agentd/$agent.json"
local_dir="$repo/codex_ops/local/$agent"
bridge_config="$local_dir/visible-app-bridge.json"
mkdir -p "$local_dir"

python3 - "$installed_config" "$bridge_config" "$agent" "$repo" <<'PY'
import json
import os
import pathlib
import sys

source = pathlib.Path(sys.argv[1])
target = pathlib.Path(sys.argv[2])
agent = sys.argv[3]
repo = pathlib.Path(sys.argv[4])
config = json.loads(source.read_text(encoding="utf-8"))
if config.get("agent_id") != agent:
    raise SystemExit("refusing agent mismatch")
if config.get("policy", {}).get("mode") != "observe":
    raise SystemExit("refusing non-observe policy")
binary = pathlib.Path(str(config.get("codex", {}).get("binary", "")))
if not binary.is_absolute() or str(binary).startswith("/mnt/") or not os.access(binary, os.X_OK):
    raise SystemExit(f"refusing non-native Codex binary: {binary}")
config["codex"]["enabled"] = True
config["codex"]["backend"] = "app-server"
config["codex"]["model"] = ""
config["codex"]["session_file"] = str(
    repo / "codex_ops/local" / agent / "app-server-session.json"
)
temporary = target.with_suffix(".tmp")
temporary.write_text(json.dumps(config, indent=2) + "\n", encoding="utf-8")
temporary.chmod(0o600)
temporary.replace(target)
PY

printf 'Visible Codex app-server bridge for %s\n' "$agent"
printf 'Backend: official local stdio app-server\n'
printf 'Policy: observe / read-only / no approvals\n'
printf 'Cross-machine transport: mTLS NATS\n'
printf 'Press Ctrl-C to stop the bridge. No vehicle service is started.\n\n'

export CODEX_HOME="$home/.codex"
exec "$repo/codex_ops/local/venv/bin/python" \
  -m codex_ops.realtime.agentd \
  --config "$bridge_config"
