#!/usr/bin/env bash

set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

RC_WATCH_LOG_ROOT="${RC_WATCH_LOG_ROOT:-$REPO_DIR/results/rc_watch}"
RC_WATCH_RUN_ID="${RC_WATCH_RUN_ID:-$(date +%Y%m%d_%H%M%S)}"
RC_WATCH_LOG_DIR="$RC_WATCH_LOG_ROOT/$RC_WATCH_RUN_ID"
RC_WATCH_LOG_FILE="$RC_WATCH_LOG_DIR/rc_watch.log"
mkdir -p "$RC_WATCH_LOG_DIR"
ln -sfn "$RC_WATCH_LOG_DIR" "$RC_WATCH_LOG_ROOT/latest"

echo "Saving MAVROS RC I/O watch log:"
echo "  directory: $RC_WATCH_LOG_DIR"
echo "  file:      $RC_WATCH_LOG_FILE"
echo "  latest:    $RC_WATCH_LOG_ROOT/latest"
echo

exec > >(tee -a "$RC_WATCH_LOG_FILE") 2>&1

echo "===== MAVROS RC I/O WATCH LOG START $(date --iso-8601=seconds) ====="
echo "cwd=$PWD"
echo "command=$0 $*"
echo

# shellcheck disable=SC1091
source "$REPO_DIR/scripts/env.sh"

MAVROS_NS="${MAVROS_NS:-/mavros}"
DURATION_SEC="${DURATION_SEC:-45}"
CHANNELS_TO_PRINT="${CHANNELS_TO_PRINT:-8}"
PRINT_PERIOD_SEC="${PRINT_PERIOD_SEC:-1.0}"
CHANGE_THRESHOLD_US="${CHANGE_THRESHOLD_US:-15}"

python3 "$REPO_DIR/src/mavros_rc_io_watch.py" \
  --ros-args \
  -p mavros_namespace:="'$MAVROS_NS'" \
  -p duration_sec:="$DURATION_SEC" \
  -p channels_to_print:="$CHANNELS_TO_PRINT" \
  -p print_period_sec:="$PRINT_PERIOD_SEC" \
  -p change_threshold_us:="$CHANGE_THRESHOLD_US"
