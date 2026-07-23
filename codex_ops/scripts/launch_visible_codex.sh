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

repo="$home/mock_vehicle_test"
config="/etc/codex-agentd/$agent.json"
mapfile -t settings < <(
  python3 - "$config" "$agent" <<'PY'
import json
import pathlib
import sys

config = json.loads(pathlib.Path(sys.argv[1]).read_text(encoding="utf-8"))
if config.get("agent_id") != sys.argv[2]:
    raise SystemExit("refusing agent mismatch")
binary = pathlib.Path(str(config.get("codex", {}).get("binary", "")))
if not binary.is_absolute():
    raise SystemExit("refusing non-absolute Codex binary")
print(binary)
print("true" if config.get("codex", {}).get("enabled") else "false")
PY
)

codex_binary="${settings[0]:-}"
background_enabled="${settings[1]:-true}"
if [[ "$background_enabled" != false ]]; then
  echo "refusing concurrent interactive and agentd Codex execution" >&2
  echo "disable codex.enabled in $config first" >&2
  exit 1
fi
if [[ "$codex_binary" == /mnt/* || ! -x "$codex_binary" ]]; then
  echo "configured Codex binary is not a native executable: $codex_binary" >&2
  exit 1
fi

export CODEX_HOME="$home/.codex"
export CODEX_VISIBLE_BINARY="$codex_binary"
export CODEX_VISIBLE_REPO="$repo"

printf 'Starting visible interactive Codex for %s\n' "$agent"
printf 'Repository: %s\n' "$repo"
printf 'NATS remains notification-only; prompts are not injected into this chat.\n\n'

exec bash -lic 'cd "$CODEX_VISIBLE_REPO" && exec "$CODEX_VISIBLE_BINARY"'
