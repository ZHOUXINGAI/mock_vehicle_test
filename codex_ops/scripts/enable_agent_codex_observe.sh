#!/usr/bin/env bash
set -euo pipefail

if [[ $EUID -ne 0 || $# -ne 1 ]]; then
  echo "usage: sudo $0 <orin1-carrier|orin2-mini>" >&2
  exit 2
fi

agent="$1"
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

codex_binary="$(
  python3 - "$config" "$agent" <<'PY'
import json
import pathlib
import sys

path = pathlib.Path(sys.argv[1])
expected_agent = sys.argv[2]
config = json.loads(path.read_text(encoding="utf-8"))
if config.get("agent_id") != expected_agent:
    raise SystemExit("refusing agent mismatch")
if config.get("policy", {}).get("mode") != "observe":
    raise SystemExit("refusing non-observe policy")
binary = pathlib.Path(str(config.get("codex", {}).get("binary", "")))
if not binary.is_absolute():
    raise SystemExit("refusing non-absolute Codex binary")
print(binary)
PY
)"

if [[ "$codex_binary" == /mnt/* || ! -x "$codex_binary" ]]; then
  echo "configured Codex binary is not a native executable: $codex_binary" >&2
  exit 1
fi

echo "Verified native Codex binary: $codex_binary"
sudo -u "$user" env CODEX_HOME="$home/.codex" "$codex_binary" --version

python3 "$repo/codex_ops/scripts/set_agent_codex_enabled.py" \
  --config "$config" \
  --enabled true \
  --require-agent "$agent" \
  --require-mode observe

systemctl restart "$service"
systemctl is-active --quiet "$service"

echo "$service is active in observe mode with Codex execution enabled."
echo "No vehicle or hardware service was started."
