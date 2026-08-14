#!/usr/bin/env bash
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_DIR"

if [[ "${CONFIRM_NO_MOTION:-}" != "true" ]]; then
  echo "REFUSED: set CONFIRM_NO_MOTION=true only when no motion executor is connected." >&2
  exit 2
fi

MINI_SPEED_MPS="${MINI_SPEED_MPS:-0.12}"
STRAIGHT_DISTANCE_M="${STRAIGHT_DISTANCE_M:-5.0}"

exec python3 scripts/lr24_pairb_dry_run.py carrier \
  --transport mavros-router \
  --source-system 2 \
  --target-system 1 \
  --expected-source-system 1 \
  --duration-sec "${DURATION_SEC:-30}" \
  --phase trajectory \
  --v-mps "$MINI_SPEED_MPS" \
  --distance-m "$STRAIGHT_DISTANCE_M" \
  --max-speed-mps "$MINI_SPEED_MPS" \
  --max-accel-mps2 "${MAX_ACCEL_MPS2:-0.30}" \
  --valid-for-ms "${COMMAND_TTL_MS:-500}" \
  --send-staged-mission-plan \
  --staged-plan-rate-hz "${STAGED_PLAN_RATE_HZ:-1.0}" \
  --staged-plan-valid-for-ms "${PLAN_VALIDITY_MS:-120000}" \
  --staged-lead-delay-ms "${LEAD_DELAY_MS:-5000}" \
  --staged-lead-distance-m "${LEAD_DISTANCE_M:-2.0}" \
  --staged-straight-distance-m "$STRAIGHT_DISTANCE_M" \
  --staged-lateral-offset-m "${LATERAL_OFFSET_M:-6.0}" \
  --staged-turn-radius-m "${TURN_RADIUS_M:-3.0}" \
  --staged-mini-speed-mps "$MINI_SPEED_MPS" \
  --staged-carrier-speed-mps "${CARRIER_SPEED_MPS:-0.06}" \
  --allow-nonhold-command \
  --confirm-no-motion "$@"
