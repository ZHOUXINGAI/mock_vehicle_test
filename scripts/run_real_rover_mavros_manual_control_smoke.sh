#!/usr/bin/env bash

set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

MANUAL_CONTROL_LOG_DISABLE="${MANUAL_CONTROL_LOG_DISABLE:-false}"
if [ "$MANUAL_CONTROL_LOG_DISABLE" != "true" ] \
  && [ "${MANUAL_CONTROL_LOG_ACTIVE:-false}" != "true" ]; then
  MANUAL_CONTROL_LOG_ROOT="${MANUAL_CONTROL_LOG_ROOT:-$REPO_DIR/results/manual_control}"
  MANUAL_CONTROL_RUN_ID="${MANUAL_CONTROL_RUN_ID:-$(date +%Y%m%d_%H%M%S)}"
  MANUAL_CONTROL_LOG_DIR="$MANUAL_CONTROL_LOG_ROOT/$MANUAL_CONTROL_RUN_ID"
  MANUAL_CONTROL_LOG_FILE="$MANUAL_CONTROL_LOG_DIR/manual_control.log"
  mkdir -p "$MANUAL_CONTROL_LOG_DIR"
  ln -sfn "$MANUAL_CONTROL_LOG_DIR" "$MANUAL_CONTROL_LOG_ROOT/latest"
  export MANUAL_CONTROL_LOG_ACTIVE=true
  export MANUAL_CONTROL_LOG_DIR
  export MANUAL_CONTROL_LOG_FILE

  echo "Saving MAVROS manual-control smoke log:"
  echo "  directory: $MANUAL_CONTROL_LOG_DIR"
  echo "  file:      $MANUAL_CONTROL_LOG_FILE"
  echo "  latest:    $MANUAL_CONTROL_LOG_ROOT/latest"
  echo

  exec > >(tee -a "$MANUAL_CONTROL_LOG_FILE") 2>&1
  echo "===== MANUAL CONTROL SMOKE LOG START $(date --iso-8601=seconds) ====="
  echo "cwd=$PWD"
  echo "command=$0 $*"
  echo
fi

MAVROS_NS="${MAVROS_NS:-/mavros}"
MANUAL_CONTROL_TOPIC="${MANUAL_CONTROL_TOPIC:-manual_control/send}"
COMMAND_RATE_HZ="${COMMAND_RATE_HZ:-20}"
WARMUP_SEC="${WARMUP_SEC:-2.0}"
STOP_SEC="${STOP_SEC:-1.0}"
FORWARD_SEC="${FORWARD_SEC:-1.0}"
BACKWARD_SEC="${BACKWARD_SEC:-1.0}"
TURN_SEC="${TURN_SEC:-0.5}"
FINAL_STOP_SEC="${FINAL_STOP_SEC:-2.0}"

FORWARD_AXIS="${FORWARD_AXIS:-x}"
TURN_AXIS="${TURN_AXIS:-y}"
FORWARD_VALUE_RAW="${FORWARD_VALUE_RAW:-120}"
TURN_VALUE_RAW="${TURN_VALUE_RAW:-120}"
FORWARD_SIGN="${FORWARD_SIGN:-1.0}"
TURN_SIGN="${TURN_SIGN:-1.0}"
NEUTRAL_Z_RAW="${NEUTRAL_Z_RAW:-0}"
MAX_ABS_XY_R_RAW="${MAX_ABS_XY_R_RAW:-250}"
MIN_Z_RAW="${MIN_Z_RAW:-0}"
MAX_Z_RAW="${MAX_Z_RAW:-1000}"

ALLOWED_MODES="${ALLOWED_MODES:-MANUAL}"
REQUIRE_CONNECTED="${REQUIRE_CONNECTED:-true}"
REQUIRE_ARMED="${REQUIRE_ARMED:-true}"
ABORT_ON_MODE_EXIT="${ABORT_ON_MODE_EXIT:-true}"
ABORT_ON_DISARM="${ABORT_ON_DISARM:-true}"
MAX_WAIT_FOR_READY_SEC="${MAX_WAIT_FOR_READY_SEC:-60}"
STOP_BURST_SEC="${STOP_BURST_SEC:-0.8}"

TEST_SURFACE="${TEST_SURFACE:-wheels_lifted}"
CONFIRM_WHEELS_LIFTED="${CONFIRM_WHEELS_LIFTED:-false}"
CONFIRM_GROUND_AREA_CLEAR="${CONFIRM_GROUND_AREA_CLEAR:-false}"
CONFIRM_LOW_SPEED_GROUND_TEST="${CONFIRM_LOW_SPEED_GROUND_TEST:-false}"
CONFIRM_RC_READY="${CONFIRM_RC_READY:-false}"
CONFIRM_PARAM_BACKUP="${CONFIRM_PARAM_BACKUP:-false}"

missing_confirmations=()

case "$TEST_SURFACE" in
  wheels_lifted)
    if [ "$CONFIRM_WHEELS_LIFTED" != "true" ]; then
      missing_confirmations+=("CONFIRM_WHEELS_LIFTED=true")
    fi
    ;;
  ground)
    if [ "$CONFIRM_GROUND_AREA_CLEAR" != "true" ]; then
      missing_confirmations+=("CONFIRM_GROUND_AREA_CLEAR=true")
    fi
    if [ "$CONFIRM_LOW_SPEED_GROUND_TEST" != "true" ]; then
      missing_confirmations+=("CONFIRM_LOW_SPEED_GROUND_TEST=true")
    fi
    ;;
  *)
    echo "TEST_SURFACE must be 'wheels_lifted' or 'ground'." >&2
    exit 2
    ;;
esac

if [ "$CONFIRM_RC_READY" != "true" ]; then
  missing_confirmations+=("CONFIRM_RC_READY=true")
fi
if [ "$CONFIRM_PARAM_BACKUP" != "true" ]; then
  missing_confirmations+=("CONFIRM_PARAM_BACKUP=true")
fi

if [ "${#missing_confirmations[@]}" -gt 0 ]; then
  {
    echo "Refusing to start real rover MAVROS manual-control smoke test."
    echo "Required confirmations:"
    for item in "${missing_confirmations[@]}"; do
      echo "  $item"
    done
  } >&2
  exit 2
fi

# shellcheck disable=SC1091
source "$REPO_DIR/scripts/env.sh"

mkdir -p "$REPO_DIR/results/ros_logs"
export ROS_LOG_DIR="${ROS_LOG_DIR:-$REPO_DIR/results/ros_logs}"

if ! ros2 pkg prefix mavros >/dev/null 2>&1; then
  echo "MAVROS is not installed in the current ROS 2 environment." >&2
  echo "Run: ./scripts/install_mavros_humble.sh" >&2
  exit 1
fi

python3 "$REPO_DIR/src/real_rover_mavros_manual_control_smoke.py" \
  --ros-args \
  -p mavros_namespace:="'$MAVROS_NS'" \
  -p manual_control_topic:="'$MANUAL_CONTROL_TOPIC'" \
  -p command_rate_hz:="$COMMAND_RATE_HZ" \
  -p warmup_sec:="$WARMUP_SEC" \
  -p stop_sec:="$STOP_SEC" \
  -p forward_sec:="$FORWARD_SEC" \
  -p backward_sec:="$BACKWARD_SEC" \
  -p turn_sec:="$TURN_SEC" \
  -p final_stop_sec:="$FINAL_STOP_SEC" \
  -p forward_axis:="'$FORWARD_AXIS'" \
  -p turn_axis:="'$TURN_AXIS'" \
  -p forward_value_raw:="$FORWARD_VALUE_RAW" \
  -p turn_value_raw:="$TURN_VALUE_RAW" \
  -p forward_sign:="$FORWARD_SIGN" \
  -p turn_sign:="$TURN_SIGN" \
  -p neutral_z_raw:="$NEUTRAL_Z_RAW" \
  -p max_abs_xy_r_raw:="$MAX_ABS_XY_R_RAW" \
  -p min_z_raw:="$MIN_Z_RAW" \
  -p max_z_raw:="$MAX_Z_RAW" \
  -p allowed_modes:="'$ALLOWED_MODES'" \
  -p require_connected:="$REQUIRE_CONNECTED" \
  -p require_armed:="$REQUIRE_ARMED" \
  -p abort_on_mode_exit:="$ABORT_ON_MODE_EXIT" \
  -p abort_on_disarm:="$ABORT_ON_DISARM" \
  -p max_wait_for_ready_sec:="$MAX_WAIT_FOR_READY_SEC" \
  -p stop_burst_sec:="$STOP_BURST_SEC" \
  -p test_surface:="'$TEST_SURFACE'" \
  -p confirm_wheels_lifted:="$CONFIRM_WHEELS_LIFTED" \
  -p confirm_ground_area_clear:="$CONFIRM_GROUND_AREA_CLEAR" \
  -p confirm_low_speed_ground_test:="$CONFIRM_LOW_SPEED_GROUND_TEST" \
  -p confirm_rc_ready:="$CONFIRM_RC_READY" \
  -p confirm_param_backup:="$CONFIRM_PARAM_BACKUP"
