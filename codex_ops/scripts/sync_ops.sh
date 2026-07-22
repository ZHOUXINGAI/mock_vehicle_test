#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REPO_ROOT="$(git -C "$ROOT_DIR" rev-parse --show-toplevel)"
cd "$REPO_ROOT"

if git remote get-url origin >/dev/null 2>&1; then
  git pull --rebase --autostash || {
    echo "warning: git pull failed; continuing with local state" >&2
  }
else
  echo "warning: no origin remote configured for mock_vehicle_test" >&2
fi

"$ROOT_DIR/scripts/codex_ops.py" doctor "$@"
