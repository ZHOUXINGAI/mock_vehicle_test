#!/usr/bin/env bash
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_DIR"

if [[ "${CONFIRM_NO_MOTION:-}" != "true" ]]; then
  echo "REFUSED: set CONFIRM_NO_MOTION=true only after confirming no executor is connected." >&2
  exit 2
fi
if [[ -z "${PAIRB_PORT:-}" ]]; then
  echo "REFUSED: set PAIRB_PORT to Orin1 Pair B CP2102 stable by-id path." >&2
  exit 2
fi

MINI_STATE_SOURCE="${MINI_STATE_SOURCE:-simulated}"
state_source_args=(--state-source "$MINI_STATE_SOURCE")
if [[ "$MINI_STATE_SOURCE" == "mavros-local" ]]; then
  # shellcheck disable=SC1091
  source "$REPO_DIR/scripts/env.sh"
  state_source_args+=(
    --mavros-namespace "${MAVROS_NS:-/mavros}"
    --state-sample-timeout-sec "${STATE_SAMPLE_TIMEOUT_SEC:-2.0}"
  )
fi

exec python3 scripts/lr24_pairb_dry_run.py mini \
  --transport mavlink-serial \
  --port "$PAIRB_PORT" \
  --source-system 1 \
  --target-system 2 \
  --expected-source-system 2 \
  --vehicle-id 1 \
  --duration-sec "${DURATION_SEC:-30}" \
  --local-max-plan-ttl-ms "${PLAN_VALIDITY_MS:-120000}" \
  "${state_source_args[@]}" \
  --confirm-no-motion "$@"
