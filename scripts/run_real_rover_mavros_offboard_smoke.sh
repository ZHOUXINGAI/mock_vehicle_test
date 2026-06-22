#!/usr/bin/env bash

set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

OFFBOARD_LOG_DISABLE="${OFFBOARD_LOG_DISABLE:-false}"
if [ "$OFFBOARD_LOG_DISABLE" != "true" ] && [ "${OFFBOARD_LOG_ACTIVE:-false}" != "true" ]; then
  OFFBOARD_LOG_ROOT="${OFFBOARD_LOG_ROOT:-$REPO_DIR/results/offboard}"
  OFFBOARD_RUN_ID="${OFFBOARD_RUN_ID:-$(date +%Y%m%d_%H%M%S)}"
  OFFBOARD_LOG_DIR="$OFFBOARD_LOG_ROOT/$OFFBOARD_RUN_ID"
  OFFBOARD_LOG_FILE="$OFFBOARD_LOG_DIR/offboard.log"
  mkdir -p "$OFFBOARD_LOG_DIR"
  ln -sfn "$OFFBOARD_LOG_DIR" "$OFFBOARD_LOG_ROOT/latest"
  export OFFBOARD_LOG_ACTIVE=true
  export OFFBOARD_LOG_DIR
  export OFFBOARD_LOG_FILE

  echo "Saving Offboard smoke log:"
  echo "  directory: $OFFBOARD_LOG_DIR"
  echo "  file:      $OFFBOARD_LOG_FILE"
  echo "  latest:    $OFFBOARD_LOG_ROOT/latest"
  echo

  exec > >(tee -a "$OFFBOARD_LOG_FILE") 2>&1
  echo "===== OFFBOARD SMOKE LOG START $(date --iso-8601=seconds) ====="
  echo "cwd=$PWD"
  echo "command=$0 $*"
  echo
fi

MAVROS_NS="${MAVROS_NS:-/mavros}"
MAVROS_NS="${MAVROS_NS%/}"
ARMING_SERVICE="${ARMING_SERVICE:-$MAVROS_NS/cmd/arming}"
SET_MODE_SERVICE="${SET_MODE_SERVICE:-$MAVROS_NS/set_mode}"
PARAM_SET_SERVICE="${PARAM_SET_SERVICE:-$MAVROS_NS/param/set}"
COMMAND_RATE_HZ="${COMMAND_RATE_HZ:-20}"
PUBLISH_UNSTAMPED_CMD_VEL="${PUBLISH_UNSTAMPED_CMD_VEL:-true}"
SETPOINT_VELOCITY_PARAM_NODE="${SETPOINT_VELOCITY_PARAM_NODE:-$MAVROS_NS/setpoint_velocity}"
SETPOINT_VELOCITY_MAV_FRAME_SERVICE="${SETPOINT_VELOCITY_MAV_FRAME_SERVICE:-$MAVROS_NS/setpoint_velocity/mav_frame}"
SETPOINT_VELOCITY_MAV_FRAME="${SETPOINT_VELOCITY_MAV_FRAME:-BODY_NED}"
WARMUP_SEC="${WARMUP_SEC:-2.0}"
INITIAL_STOP_SEC="${INITIAL_STOP_SEC:--1.0}"
STOP_SEC="${STOP_SEC:-1.0}"
FORWARD_SEC="${FORWARD_SEC:-1.0}"
BACKWARD_SEC="${BACKWARD_SEC:-1.0}"
TURN_SEC="${TURN_SEC:-0.5}"
TURN_LEFT_SEC="${TURN_LEFT_SEC:--1.0}"
TURN_RIGHT_SEC="${TURN_RIGHT_SEC:--1.0}"
FINAL_STOP_SEC="${FINAL_STOP_SEC:-2.0}"
LINEAR_SPEED_MPS="${LINEAR_SPEED_MPS:-0.12}"
LINEAR_DIRECTION_SIGN="${LINEAR_DIRECTION_SIGN:--1.0}"
TURN_LINEAR_SPEED_MPS="${TURN_LINEAR_SPEED_MPS:-0.0}"
TURN_LINEAR_DIRECTION_SIGN="${TURN_LINEAR_DIRECTION_SIGN:-1.0}"
TURN_LATERAL_SPEED_MPS="${TURN_LATERAL_SPEED_MPS:-0.0}"
TURN_YAW_RATE_RADPS="${TURN_YAW_RATE_RADPS:-0.25}"
TURN_SIGN="${TURN_SIGN:-1.0}"
MAX_LINEAR_SPEED_MPS="${MAX_LINEAR_SPEED_MPS:-0.30}"
MAX_YAW_RATE_RADPS="${MAX_YAW_RATE_RADPS:-0.70}"

MODE_CHANGE_ON_START="${MODE_CHANGE_ON_START:-false}"
ARM_ON_START="${ARM_ON_START:-false}"
REQUIRE_ARMED_BEFORE_MODE_CHANGE="${REQUIRE_ARMED_BEFORE_MODE_CHANGE:-false}"
MODE_REQUEST_RETRY_SEC="${MODE_REQUEST_RETRY_SEC:-2.0}"
ARM_REQUEST_RETRY_SEC="${ARM_REQUEST_RETRY_SEC:-2.0}"
DISARM_ON_FINISH="${DISARM_ON_FINISH:-false}"
REQUIRE_CONNECTED="${REQUIRE_CONNECTED:-true}"
REQUIRE_OFFBOARD_MODE="${REQUIRE_OFFBOARD_MODE:-true}"
REQUIRE_ARMED="${REQUIRE_ARMED:-true}"
ABORT_ON_MODE_EXIT="${ABORT_ON_MODE_EXIT:-true}"
ABORT_ON_DISARM="${ABORT_ON_DISARM:-true}"
ABORT_ON_ARM_REJECTED="${ABORT_ON_ARM_REJECTED:-true}"
MAX_WAIT_FOR_READY_SEC="${MAX_WAIT_FOR_READY_SEC:-60}"
STOP_BURST_SEC="${STOP_BURST_SEC:-0.8}"
AUTO_RESTORE_OUTPUT_MAPPING="${AUTO_RESTORE_OUTPUT_MAPPING:-true}"
DISCOVERY_TIMEOUT_SEC="${DISCOVERY_TIMEOUT_SEC:-4s}"

TEST_SURFACE="${TEST_SURFACE:-wheels_lifted}"
CONFIRM_WHEELS_LIFTED="${CONFIRM_WHEELS_LIFTED:-false}"
CONFIRM_GROUND_AREA_CLEAR="${CONFIRM_GROUND_AREA_CLEAR:-false}"
CONFIRM_LOW_SPEED_GROUND_TEST="${CONFIRM_LOW_SPEED_GROUND_TEST:-false}"
CONFIRM_RC_READY="${CONFIRM_RC_READY:-false}"
CONFIRM_PARAM_BACKUP="${CONFIRM_PARAM_BACKUP:-false}"
CONFIRM_QGC_DISARM_READY="${CONFIRM_QGC_DISARM_READY:-false}"
CONFIRM_PHYSICAL_POWER_CUTOFF_READY="${CONFIRM_PHYSICAL_POWER_CUTOFF_READY:-false}"

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
if [ "$AUTO_RESTORE_OUTPUT_MAPPING" = "true" ]; then
  if [ "$CONFIRM_QGC_DISARM_READY" != "true" ]; then
    missing_confirmations+=("CONFIRM_QGC_DISARM_READY=true")
  fi
  if [ "$CONFIRM_PHYSICAL_POWER_CUTOFF_READY" != "true" ]; then
    missing_confirmations+=("CONFIRM_PHYSICAL_POWER_CUTOFF_READY=true")
  fi
fi

if [ "${#missing_confirmations[@]}" -gt 0 ]; then
  {
    echo "Refusing to start real rover MAVROS Offboard smoke test."
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

cleanup() {
  local status=$?
  trap - EXIT INT TERM
  set +e
  echo
  echo "Real rover MAVROS Offboard smoke cleanup..."
  echo "Requesting MAVROS disarm..."
  timeout 5s ros2 service call \
    "$ARMING_SERVICE" \
    mavros_msgs/srv/CommandBool \
    "{value: false}" || true
  echo "Requesting MANUAL mode..."
  timeout 5s ros2 service call \
    "$SET_MODE_SERVICE" \
    mavros_msgs/srv/SetMode \
    "{base_mode: 0, custom_mode: 'MANUAL'}" || true
  if [ "$AUTO_RESTORE_OUTPUT_MAPPING" = "true" ]; then
    service_list="$(timeout "$DISCOVERY_TIMEOUT_SEC" ros2 service list --spin-time 2 || true)"
    if printf '%s\n' "$service_list" | grep -Fxq "$PARAM_SET_SERVICE"; then
      echo "Restoring RC passthrough output mapping..."
      sleep 1
      TEST_SURFACE="$TEST_SURFACE" \
      CONFIRM_PARAM_BACKUP=true \
      CONFIRM_WHEELS_LIFTED="$CONFIRM_WHEELS_LIFTED" \
      CONFIRM_GROUND_AREA_CLEAR="$CONFIRM_GROUND_AREA_CLEAR" \
      CONFIRM_LOW_SPEED_GROUND_TEST="$CONFIRM_LOW_SPEED_GROUND_TEST" \
      CONFIRM_VEHICLE_DISARMED=true \
      CONFIRM_QGC_DISARM_READY=true \
      CONFIRM_PHYSICAL_POWER_CUTOFF_READY=true \
        "$REPO_DIR/scripts/set_px4_rover_output_mapping.sh" restore-baseline || true
    else
      echo "Skipping output-mapping restore because MAVROS param services are unavailable."
    fi
  fi
  echo "Final MAVROS state snapshot:"
  timeout 5s ros2 topic echo --once "$MAVROS_NS/state" || true
  exit "$status"
}
trap cleanup EXIT INT TERM

if ! ros2 pkg prefix mavros >/dev/null 2>&1; then
  echo "MAVROS is not installed in the current ROS 2 environment." >&2
  echo "Run: ./scripts/install_mavros_humble.sh" >&2
  exit 1
fi

case "$SETPOINT_VELOCITY_MAV_FRAME" in
  LOCAL_NED) SETPOINT_VELOCITY_MAV_FRAME_ID=1 ;;
  LOCAL_OFFSET_NED) SETPOINT_VELOCITY_MAV_FRAME_ID=7 ;;
  BODY_NED) SETPOINT_VELOCITY_MAV_FRAME_ID=8 ;;
  BODY_OFFSET_NED) SETPOINT_VELOCITY_MAV_FRAME_ID=9 ;;
  BODY_FRD) SETPOINT_VELOCITY_MAV_FRAME_ID=12 ;;
  *)
    echo "Unsupported SETPOINT_VELOCITY_MAV_FRAME: $SETPOINT_VELOCITY_MAV_FRAME" >&2
    echo "Supported: LOCAL_NED, LOCAL_OFFSET_NED, BODY_NED, BODY_OFFSET_NED, BODY_FRD" >&2
    exit 2
    ;;
esac

echo "Configuring MAVROS setpoint_velocity frame:"
echo "  target: $SETPOINT_VELOCITY_MAV_FRAME ($SETPOINT_VELOCITY_MAV_FRAME_ID)"
service_list="$(ros2 service list --spin-time 2 || true)"
if printf '%s\n' "$service_list" | grep -Fxq "$SETPOINT_VELOCITY_MAV_FRAME_SERVICE"; then
  timeout 5s ros2 service call \
    "$SETPOINT_VELOCITY_MAV_FRAME_SERVICE" \
    mavros_msgs/srv/SetMavFrame \
    "{mav_frame: $SETPOINT_VELOCITY_MAV_FRAME_ID}"
else
  echo "  mav_frame service not discovered; trying ROS parameter instead."
  timeout 5s ros2 param set \
    "$SETPOINT_VELOCITY_PARAM_NODE" \
    mav_frame \
    "$SETPOINT_VELOCITY_MAV_FRAME"
fi
echo "Current setpoint_velocity mav_frame:"
timeout 5s ros2 param get "$SETPOINT_VELOCITY_PARAM_NODE" mav_frame || true
echo

python3 "$REPO_DIR/src/real_rover_mavros_offboard_smoke.py" \
  --ros-args \
  -p mavros_namespace:="$MAVROS_NS" \
  -p command_rate_hz:="$COMMAND_RATE_HZ" \
  -p publish_unstamped_cmd_vel:="$PUBLISH_UNSTAMPED_CMD_VEL" \
  -p warmup_sec:="$WARMUP_SEC" \
  -p initial_stop_sec:="$INITIAL_STOP_SEC" \
  -p stop_sec:="$STOP_SEC" \
  -p forward_sec:="$FORWARD_SEC" \
  -p backward_sec:="$BACKWARD_SEC" \
  -p turn_sec:="$TURN_SEC" \
  -p turn_left_sec:="$TURN_LEFT_SEC" \
  -p turn_right_sec:="$TURN_RIGHT_SEC" \
  -p final_stop_sec:="$FINAL_STOP_SEC" \
  -p linear_speed_mps:="$LINEAR_SPEED_MPS" \
  -p linear_direction_sign:="$LINEAR_DIRECTION_SIGN" \
  -p turn_linear_speed_mps:="$TURN_LINEAR_SPEED_MPS" \
  -p turn_linear_direction_sign:="$TURN_LINEAR_DIRECTION_SIGN" \
  -p turn_lateral_speed_mps:="$TURN_LATERAL_SPEED_MPS" \
  -p turn_yaw_rate_radps:="$TURN_YAW_RATE_RADPS" \
  -p turn_sign:="$TURN_SIGN" \
  -p max_linear_speed_mps:="$MAX_LINEAR_SPEED_MPS" \
  -p max_yaw_rate_radps:="$MAX_YAW_RATE_RADPS" \
  -p mode_change_on_start:="$MODE_CHANGE_ON_START" \
  -p arm_on_start:="$ARM_ON_START" \
  -p require_armed_before_mode_change:="$REQUIRE_ARMED_BEFORE_MODE_CHANGE" \
  -p mode_request_retry_sec:="$MODE_REQUEST_RETRY_SEC" \
  -p arm_request_retry_sec:="$ARM_REQUEST_RETRY_SEC" \
  -p disarm_on_finish:="$DISARM_ON_FINISH" \
  -p require_connected:="$REQUIRE_CONNECTED" \
  -p require_offboard_mode:="$REQUIRE_OFFBOARD_MODE" \
  -p require_armed:="$REQUIRE_ARMED" \
  -p abort_on_mode_exit:="$ABORT_ON_MODE_EXIT" \
  -p abort_on_disarm:="$ABORT_ON_DISARM" \
  -p abort_on_arm_rejected:="$ABORT_ON_ARM_REJECTED" \
  -p max_wait_for_ready_sec:="$MAX_WAIT_FOR_READY_SEC" \
  -p stop_burst_sec:="$STOP_BURST_SEC" \
  -p test_surface:="$TEST_SURFACE" \
  -p confirm_wheels_lifted:="$CONFIRM_WHEELS_LIFTED" \
  -p confirm_ground_area_clear:="$CONFIRM_GROUND_AREA_CLEAR" \
  -p confirm_low_speed_ground_test:="$CONFIRM_LOW_SPEED_GROUND_TEST" \
  -p confirm_rc_ready:="$CONFIRM_RC_READY" \
  -p confirm_param_backup:="$CONFIRM_PARAM_BACKUP"
