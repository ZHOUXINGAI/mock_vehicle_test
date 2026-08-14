#!/usr/bin/env bash
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [ "${CONFIRM_READ_ONLY_HIL:-}" != "READ_ONLY_HIL" ]; then
  echo "Refusing to start without CONFIRM_READ_ONLY_HIL=READ_ONLY_HIL" >&2
  echo "This mode requires Carrier MANUAL+disarmed and publishes RViz topics only." >&2
  exit 2
fi

# shellcheck disable=SC1091
source "$REPO_DIR/scripts/env.sh"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

REPLAY_DIR="${REPLAY_DIR:-}"
if [ -z "$REPLAY_DIR" ]; then
  REPLAY_SUMMARY="$(find "$REPO_DIR/results/pairb_cooperative_docking_offline" -path '*/nominal/summary.json' -type f -print 2>/dev/null | sort | tail -1)"
  if [ -n "$REPLAY_SUMMARY" ]; then
    REPLAY_DIR="$(dirname "$REPLAY_SUMMARY")"
  fi
fi
if [ -z "$REPLAY_DIR" ] || [ ! -f "$REPLAY_DIR/timeline.csv" ]; then
  echo "No nominal replay found. Run the offline cooperative simulation first." >&2
  exit 3
fi

RVIZ_PID=""
cleanup() {
  if [ -n "$RVIZ_PID" ] && kill -0 "$RVIZ_PID" 2>/dev/null; then
    kill "$RVIZ_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

rviz2 -d "$REPO_DIR/config/rviz/pairb_cooperative_virtual_mini.rviz" &
RVIZ_PID=$!

python3 "$REPO_DIR/scripts/run_pairb_virtual_mini_hil_rviz.py" \
  --replay-dir "$REPLAY_DIR" \
  --time-scale "${TIME_SCALE:-4.0}" \
  --allowed-disarmed-mode "${HIL_ALLOWED_DISARMED_MODE:-MANUAL}"
