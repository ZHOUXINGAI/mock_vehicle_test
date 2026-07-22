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
source_root="$repo/codex_ops"
venv="$source_root/local/venv"
service="codex-agentd-$agent.service"
codex_binary="${CODEX_BINARY:-}"

if [[ -z "$codex_binary" ]]; then
  codex_binary="$(sudo -u "$user" bash -lc 'command -v codex' 2>/dev/null || true)"
fi
if [[ -z "$codex_binary" && -d "$home/.nvm/versions/node" ]]; then
  codex_binary="$(find "$home/.nvm/versions/node" -mindepth 3 -maxdepth 3 \
    \( -type f -o -type l \) -path '*/bin/codex' 2>/dev/null \
    | sort -V | tail -n 1)"
fi

test -d "$repo/.git"
test -f "$source_root/realtime/config/$agent.example.json"
if [[ -z "$codex_binary" || ! -x "$codex_binary" ]]; then
  echo "codex CLI is not installed for $user; install/upgrade it before agentd" >&2
  exit 1
fi
install -d -m 0750 -o "$user" -g "$user" "$source_root/local" "$source_root/runs"
sudo -u "$user" python3 -m venv "$venv"
sudo -u "$user" "$venv/bin/pip" install --upgrade pip
sudo -u "$user" "$venv/bin/pip" install -r "$source_root/realtime/requirements.txt"

install -d -m 0750 -o "$user" -g "$user" /etc/codex-agentd /etc/codex-agentd/certs
if [[ ! -e "/etc/codex-agentd/$agent.json" ]]; then
  install -m 0640 -o "$user" -g "$user" \
    "$source_root/realtime/config/$agent.example.json" "/etc/codex-agentd/$agent.json"
fi
sudo -u "$user" python3 - "/etc/codex-agentd/$agent.json" "$codex_binary" <<'PY'
import json
import pathlib
import sys

path = pathlib.Path(sys.argv[1])
config = json.loads(path.read_text(encoding="utf-8"))
config["codex"]["binary"] = sys.argv[2]
path.write_text(json.dumps(config, indent=2) + "\n", encoding="utf-8")
PY
install -m 0644 "$source_root/deploy/$service" "/etc/systemd/system/$service"
systemctl daemon-reload

echo "Installed $service but did not enable or start it."
echo "Codex CLI: $codex_binary"
echo "Configure /etc/codex-agentd/$agent.json and client certs first."
