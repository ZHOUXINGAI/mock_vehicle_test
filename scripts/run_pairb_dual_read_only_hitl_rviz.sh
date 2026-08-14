#!/usr/bin/env bash
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [ "${CONFIRM_DUAL_READ_ONLY_HITL:-}" != "DUAL_READ_ONLY_HITL" ]; then
  echo "Refusing to start without CONFIRM_DUAL_READ_ONLY_HITL=DUAL_READ_ONLY_HITL" >&2
  echo "Both Pixhawks must remain disarmed; this process publishes RViz topics only." >&2
  exit 2
fi

export ROS_LOCALHOST_ONLY=0
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-99}"
# shellcheck disable=SC1091
source "$REPO_DIR/scripts/env.sh"

PAIRB_PORT="${PAIRB_PORT:-/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0}"
if [ ! -e "$PAIRB_PORT" ]; then
  echo "Pair B Ground port not found: $PAIRB_PORT" >&2
  exit 4
fi

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
  --carrier-namespace "${CARRIER_MAVROS_NS:-/carrier/mavros}" \
  --mini-pairb-port "$PAIRB_PORT" \
  --mini-pairb-baud "${PAIRB_BAUD:-115200}" \
  --allowed-disarmed-mode "${HITL_ALLOWED_DISARMED_MODE:-MANUAL}" \
  --require-real-mini
