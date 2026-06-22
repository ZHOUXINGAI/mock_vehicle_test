#!/usr/bin/env bash

set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

cat <<'EOF'
Indoor fake-vision MAVROS Offboard FORWARD-only test.

Default behavior:
  - wheels lifted only
  - starts fake external vision
  - waits for MANUAL arm from RC/QGC
  - requests OFFBOARD
  - drives forward only, then stops and disarms

Observe:
  - all drive wheels should indicate forward motion
  - abort with RC disarm/kill, QGC disarm, physical power cutoff, or Ctrl+C
EOF

FORWARD_ONLY_LOG_DISABLE="${FORWARD_ONLY_LOG_DISABLE:-false}"
if [ "$FORWARD_ONLY_LOG_DISABLE" != "true" ] \
  && [ "${FORWARD_ONLY_LOG_ACTIVE:-false}" != "true" ]; then
  FORWARD_ONLY_LOG_ROOT="${FORWARD_ONLY_LOG_ROOT:-$REPO_DIR/results/fake_vision_offboard_forward}"
  FORWARD_ONLY_RUN_ID="${FORWARD_ONLY_RUN_ID:-$(date +%Y%m%d_%H%M%S)}"
  FORWARD_ONLY_LOG_DIR="$FORWARD_ONLY_LOG_ROOT/$FORWARD_ONLY_RUN_ID"
  FORWARD_ONLY_LOG_FILE="$FORWARD_ONLY_LOG_DIR/forward.log"
  mkdir -p "$FORWARD_ONLY_LOG_DIR"
  ln -sfn "$FORWARD_ONLY_LOG_DIR" "$FORWARD_ONLY_LOG_ROOT/latest"

  echo
  echo "Saving indoor fake-vision Offboard forward-only log:"
  echo "  directory: $FORWARD_ONLY_LOG_DIR"
  echo "  file:      $FORWARD_ONLY_LOG_FILE"
  echo "  latest:    $FORWARD_ONLY_LOG_ROOT/latest"
  echo

  export FORWARD_ONLY_LOG_ACTIVE=true
  export FAKE_VISION_MOTION_LOG_ACTIVE=true
  export FAKE_VISION_MOTION_LOG_DIR="$FORWARD_ONLY_LOG_DIR"
  export FAKE_VISION_MOTION_LOG_FILE="$FORWARD_ONLY_LOG_FILE"
  exec > >(tee -a "$FORWARD_ONLY_LOG_FILE") 2>&1
  echo "===== INDOOR FAKE-VISION OFFBOARD FORWARD TEST LOG START $(date --iso-8601=seconds) ====="
  echo "cwd=$PWD"
  echo "command=$0 $*"
  echo
fi

export ROS_LOG_DIR="${ROS_LOG_DIR:-$REPO_DIR/results/ros_logs}"

export FORWARD_SEC="${FORWARD_SEC:-10.0}"
export BACKWARD_SEC=0.0
export TURN_SEC=0.0
export STOP_SEC="${STOP_SEC:-2.0}"
export FINAL_STOP_SEC="${FINAL_STOP_SEC:-2.0}"
export LINEAR_SPEED_MPS="${LINEAR_SPEED_MPS:-0.35}"
export LINEAR_DIRECTION_SIGN="${LINEAR_DIRECTION_SIGN:--1.0}"
export TURN_LINEAR_SPEED_MPS=0.0
export TURN_LATERAL_SPEED_MPS=0.0
export TURN_YAW_RATE_RADPS=0.0
export MAX_LINEAR_SPEED_MPS="${MAX_LINEAR_SPEED_MPS:-0.60}"
export MAX_YAW_RATE_RADPS="${MAX_YAW_RATE_RADPS:-0.60}"

echo "Effective forward-only settings:"
echo "  FORWARD_SEC=$FORWARD_SEC"
echo "  LINEAR_SPEED_MPS=$LINEAR_SPEED_MPS"
echo "  LINEAR_DIRECTION_SIGN=$LINEAR_DIRECTION_SIGN"
echo "  STOP_SEC=$STOP_SEC"
echo "  BACKWARD_SEC=$BACKWARD_SEC"
echo "  TURN_SEC=$TURN_SEC"
echo

"$REPO_DIR/scripts/run_real_rover_mavros_indoor_fake_vision_offboard_motion_task.sh"
