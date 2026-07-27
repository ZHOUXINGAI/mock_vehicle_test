#!/usr/bin/env bash
set -euo pipefail

if [[ $EUID -ne 0 || $# -ne 2 ]]; then
  echo "usage: sudo $0 <orin1-carrier|orin2-mini> <existing-codex-session-id>" >&2
  exit 2
fi

agent="$1"
session_id="$2"
case "$agent" in
  orin1-carrier)
    user=jetson
    home=/home/jetson
    ;;
  orin2-mini)
    user=seeed
    home=/home/seeed
    ;;
  *)
    echo "unknown agent: $agent" >&2
    exit 2
    ;;
esac

repo="$home/mock_vehicle_test"
config="/etc/codex-agentd/$agent.json"
service="codex-agentd-$agent.service"
unit_source="$repo/codex_ops/deploy/$service"
session_file="$repo/codex_ops/local/$agent/app-server-session.json"

if pgrep -u "$user" -f 'codex_ops/local/.*/visible-app-bridge.json' >/dev/null; then
  echo "refusing while a visible app Bridge is running" >&2
  exit 1
fi
if systemctl is-active --quiet "$service"; then
  echo "refusing while $service is already active" >&2
  exit 1
fi

python3 - "$config" "$agent" <<'PY'
import json
import pathlib
import sys

path = pathlib.Path(sys.argv[1])
agent = sys.argv[2]
config = json.loads(path.read_text(encoding="utf-8"))
if config.get("agent_id") != agent:
    raise SystemExit("refusing agent mismatch")
if config.get("policy", {}).get("mode") != "observe":
    raise SystemExit("refusing non-observe policy")
binary = pathlib.Path(str(config.get("codex", {}).get("binary", "")))
if not binary.is_absolute() or str(binary).startswith("/mnt/") or not binary.is_file():
    raise SystemExit(f"refusing non-native Codex binary: {binary}")
PY

sudo -u "$user" python3 "$repo/codex_ops/scripts/pin_agent_codex_session.py" \
  --session-file "$session_file" \
  --codex-home "$home/.codex" \
  --agent-id "$agent" \
  --session-id "$session_id"

stamp="$(date -u +%Y%m%dT%H%M%SZ)"
cp -p "$config" "$config.bak-$stamp"
python3 - "$config" "$agent" "$session_file" <<'PY'
import json
import os
import pathlib
import tempfile
import sys

path = pathlib.Path(sys.argv[1])
agent = sys.argv[2]
session_file = sys.argv[3]
original = path.stat()
config = json.loads(path.read_text(encoding="utf-8"))
if config.get("agent_id") != agent:
    raise SystemExit("refusing agent mismatch")
if config.get("policy", {}).get("mode") != "observe":
    raise SystemExit("refusing non-observe policy")
codex = config["codex"]
codex["enabled"] = True
codex["backend"] = "app-server"
codex["session_file"] = session_file
codex["model"] = ""
rendered = json.dumps(config, indent=2, ensure_ascii=False) + "\n"
descriptor, name = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
temporary = pathlib.Path(name)
try:
    with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
        stream.write(rendered)
        stream.flush()
        os.fsync(stream.fileno())
    os.chmod(temporary, original.st_mode)
    os.chown(temporary, original.st_uid, original.st_gid)
    os.replace(temporary, path)
finally:
    temporary.unlink(missing_ok=True)
PY

install -m 0644 "$unit_source" "/etc/systemd/system/$service"
systemctl daemon-reload
systemctl enable --now "$service"
sleep 2
systemctl is-active --quiet "$service"

echo "$service is enabled and active."
echo "Backend: app-server; policy: observe/read-only."
echo "Pinned Codex session: $session_id"
echo "Configuration backup: $config.bak-$stamp"
echo "No vehicle or hardware service was started."
