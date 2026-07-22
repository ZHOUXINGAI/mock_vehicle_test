#!/usr/bin/env bash

set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

ROLE="${1:-}"
if [ -z "$ROLE" ]; then
  cat >&2 <<'EOF'
Usage:
  CONFIRM_NO_MOTION=true ./scripts/run_lr24_pairb_dry_run.sh <carrier|mini> [args...]

Mini side example:
  CONFIRM_NO_MOTION=true ./scripts/run_lr24_pairb_dry_run.sh mini \
    --port /dev/serial/by-id/<B-MINI-LR24> \
    --duration-sec 120 \
    --state-rate-hz 10 \
    --simulate-orbit

Carrier side example:
  CONFIRM_NO_MOTION=true ./scripts/run_lr24_pairb_dry_run.sh carrier \
    --port /dev/serial/by-id/<B-CARRIER-LR24> \
    --duration-sec 120 \
    --command-rate-hz 2 \
    --phase hold \
    --stale-ms 300 \
    --send-corridor-plan \
    --corridor-plan-rate-hz 0.2
EOF
  exit 2
fi
shift

if [ "${CONFIRM_NO_MOTION:-false}" != "true" ]; then
  echo "Refusing to run LR24 Pair B dry-run until CONFIRM_NO_MOTION=true." >&2
  echo "This script is serial-only, but it must not be connected to motor execution." >&2
  exit 2
fi

LOG_ROOT="${LR24_PAIR_B_LOG_ROOT:-$REPO_DIR/results/lr24_pairb_dry_run}"
RUN_ID="${LR24_PAIR_B_RUN_ID:-$(date +%Y%m%d_%H%M%S)_$ROLE}"
LOG_DIR="$LOG_ROOT/$RUN_ID"
LOG_FILE="$LOG_DIR/lr24_pairb_${ROLE}.log"
CSV_FILE="$LOG_DIR/lr24_pairb_${ROLE}.csv"
mkdir -p "$LOG_DIR"
ln -sfn "$LOG_DIR" "$LOG_ROOT/latest"

echo "Saving LR24 Pair B dry-run log:"
echo "  directory: $LOG_DIR"
echo "  file:      $LOG_FILE"
echo "  csv:       $CSV_FILE"
echo "  latest:    $LOG_ROOT/latest"
echo

exec > >(tee -a "$LOG_FILE") 2>&1
echo "===== LR24 PAIR B DRY-RUN START $(date --iso-8601=seconds) ====="
echo "cwd=$PWD"
echo "role=$ROLE"
echo "args=$*"
echo

python3 "$REPO_DIR/scripts/lr24_pairb_dry_run.py" \
  --print-frame-sizes \
  "$ROLE" \
  --confirm-no-motion \
  --csv "$CSV_FILE" \
  "$@"
