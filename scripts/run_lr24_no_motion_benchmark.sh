#!/usr/bin/env bash

set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

MODE="${1:-}"
if [ -z "$MODE" ]; then
  cat >&2 <<'EOF'
Usage:
  CONFIRM_NO_MOTION=true ./scripts/run_lr24_no_motion_benchmark.sh <mode> [args...]

Modes are passed to scripts/lr24_link_benchmark.py:
  echo, ping, state-tx, state-rx, command-tx, command-rx,
  corridor-plan-tx, corridor-plan-rx

Examples:
  CONFIRM_NO_MOTION=true ./scripts/run_lr24_no_motion_benchmark.sh echo \
    --port /dev/serial/by-id/<mini-side-lr24>

  CONFIRM_NO_MOTION=true ./scripts/run_lr24_no_motion_benchmark.sh ping \
    --port /dev/serial/by-id/<carrier-side-lr24> --duration-sec 60 --rate-hz 10
EOF
  exit 2
fi
shift

if [ "${CONFIRM_NO_MOTION:-false}" != "true" ]; then
  echo "Refusing to run LR24 benchmark until CONFIRM_NO_MOTION=true." >&2
  echo "Use only with motors disabled, wheels lifted, or vehicle power kept safe." >&2
  exit 2
fi

LOG_ROOT="${LR24_BENCHMARK_LOG_ROOT:-$REPO_DIR/results/lr24_benchmark}"
RUN_ID="${LR24_BENCHMARK_RUN_ID:-$(date +%Y%m%d_%H%M%S)_$MODE}"
LOG_DIR="$LOG_ROOT/$RUN_ID"
LOG_FILE="$LOG_DIR/lr24_${MODE}.log"
mkdir -p "$LOG_DIR"
ln -sfn "$LOG_DIR" "$LOG_ROOT/latest"

echo "Saving LR24 no-motion benchmark log:"
echo "  directory: $LOG_DIR"
echo "  file:      $LOG_FILE"
echo "  latest:    $LOG_ROOT/latest"
echo

exec > >(tee -a "$LOG_FILE") 2>&1
echo "===== LR24 NO-MOTION BENCHMARK START $(date --iso-8601=seconds) ====="
echo "cwd=$PWD"
echo "mode=$MODE"
echo "args=$*"
echo

python3 "$REPO_DIR/scripts/lr24_link_benchmark.py" \
  --print-frame-sizes \
  "$MODE" \
  --confirm-no-motion \
  "$@"
