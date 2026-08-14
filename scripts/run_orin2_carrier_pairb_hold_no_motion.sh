#!/usr/bin/env bash
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_DIR"

if [[ "${CONFIRM_NO_MOTION:-}" != "true" ]]; then
  echo "REFUSED: set CONFIRM_NO_MOTION=true only after confirming no executor is connected." >&2
  exit 2
fi

exec python3 scripts/lr24_pairb_dry_run.py carrier \
  --transport mavros-router \
  --source-system 2 \
  --target-system 1 \
  --expected-source-system 1 \
  --phase hold \
  --duration-sec "${DURATION_SEC:-30}" \
  --confirm-no-motion "$@"
