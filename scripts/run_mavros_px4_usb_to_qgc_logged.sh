#!/usr/bin/env bash

set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG_ROOT="${MAVROS_LOG_ROOT:-$REPO_DIR/results/mavros}"
RUN_ID="${MAVROS_RUN_ID:-$(date +%Y%m%d_%H%M%S)}"
LOG_DIR="$LOG_ROOT/$RUN_ID"
LOG_FILE="$LOG_DIR/mavros.log"

mkdir -p "$LOG_DIR"
ln -sfn "$LOG_DIR" "$LOG_ROOT/latest"

echo "Saving MAVROS log:"
echo "  directory: $LOG_DIR"
echo "  file:      $LOG_FILE"
echo "  latest:    $LOG_ROOT/latest"
echo

exec > >(tee -a "$LOG_FILE") 2>&1

echo "===== MAVROS LOG START $(date --iso-8601=seconds) ====="
echo "cwd=$PWD"
echo "command=$0 $*"
echo

AUTO_RESTART="${MAVROS_AUTO_RESTART:-true}"
RESTART_DELAY_SEC="${MAVROS_RESTART_DELAY_SEC:-3}"
STOP_REQUESTED=false
CHILD_PID=""

stop_child() {
  STOP_REQUESTED=true
  if [ -n "$CHILD_PID" ] && kill -0 "$CHILD_PID" 2>/dev/null; then
    kill "$CHILD_PID" 2>/dev/null || true
  fi
}

trap stop_child INT TERM

run_count=0
while true; do
  run_count=$((run_count + 1))
  echo
  echo "===== MAVROS ATTEMPT $run_count START $(date --iso-8601=seconds) ====="

  set +e
  "$REPO_DIR/scripts/run_mavros_px4_usb_to_qgc.sh" "$@" &
  CHILD_PID=$!
  wait "$CHILD_PID"
  status=$?
  CHILD_PID=""
  set -e

  echo "===== MAVROS ATTEMPT $run_count EXIT status=$status $(date --iso-8601=seconds) ====="

  if [ "$STOP_REQUESTED" = "true" ]; then
    echo "Stop requested; not restarting MAVROS."
    exit "$status"
  fi

  if [ "$AUTO_RESTART" != "true" ]; then
    echo "MAVROS_AUTO_RESTART=$AUTO_RESTART; not restarting MAVROS."
    exit "$status"
  fi

  case "$status" in
    0|130|143)
      echo "MAVROS exited with status $status; not restarting."
      exit "$status"
      ;;
  esac

  echo "MAVROS exited unexpectedly, likely because PX4 USB reset during reboot."
  echo "Restarting in ${RESTART_DELAY_SEC}s. Press Ctrl-C to stop."
  sleep "$RESTART_DELAY_SEC"
done
