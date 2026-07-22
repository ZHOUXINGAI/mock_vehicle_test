#!/usr/bin/env bash
set -euo pipefail

ops_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
python="$ops_root/local/venv/bin/python"
if [[ ! -x "$python" ]]; then
  echo "missing coordination venv; run codex_ops/deploy/install_agentd.sh first" >&2
  exit 1
fi
exec "$python" -m codex_ops.realtime.coordctl "$@"
