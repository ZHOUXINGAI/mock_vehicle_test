#!/usr/bin/env bash

set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

LOG_ROOT="${DIFF_OFFBOARD_L_TURN_LOG_ROOT:-$REPO_DIR/results/differential_offboard_l_turn}"
RUN_ID="${DIFF_OFFBOARD_L_TURN_RUN_ID:-$(date +%Y%m%d_%H%M%S)}"
LOG_DIR="$LOG_ROOT/$RUN_ID"
LOG_FILE="$LOG_DIR/differential_offboard_l_turn.log"
mkdir -p "$LOG_DIR"
ln -sfn "$LOG_DIR" "$LOG_ROOT/latest"

echo "Saving differential Offboard L-turn log:"
echo "  directory: $LOG_DIR"
echo "  file:      $LOG_FILE"
echo "  latest:    $LOG_ROOT/latest"
echo

exec > >(tee -a "$LOG_FILE") 2>&1
echo "===== DIFFERENTIAL OFFBOARD L-TURN LOG START $(date --iso-8601=seconds) ====="
echo "cwd=$PWD"
echo "command=$0 $*"
echo

cat <<'EOF'
PX4 v1.17 differential-rover MAVROS Offboard body-frame L-turn test.

Mission:
  1. capture current local pose/yaw
  2. drive forward in BODY_NED, so PX4 should not pre-correct yaw
  3. command body lateral velocity until local yaw changes by the target angle
  4. drive forward in BODY_NED again

This is not an in-place spin test. It is a low-speed arc turn that avoids the
previous LOCAL_NED behavior where PX4 corrected yaw before driving forward.
EOF

MAVROS_NS="${MAVROS_NS:-/mavros}"
MAVROS_NS="${MAVROS_NS%/}"
STATE_TOPIC="${STATE_TOPIC:-$MAVROS_NS/state}"
LOCAL_POSE_TOPIC="${LOCAL_POSE_TOPIC:-$MAVROS_NS/local_position/pose}"
ARMING_SERVICE="${ARMING_SERVICE:-$MAVROS_NS/cmd/arming}"
SET_MODE_SERVICE="${SET_MODE_SERVICE:-$MAVROS_NS/set_mode}"
SETPOINT_VELOCITY_PARAM_NODE="${SETPOINT_VELOCITY_PARAM_NODE:-$MAVROS_NS/setpoint_velocity}"
SETPOINT_VELOCITY_MAV_FRAME_SERVICE="${SETPOINT_VELOCITY_MAV_FRAME_SERVICE:-$MAVROS_NS/setpoint_velocity/mav_frame}"
STATE_TIMEOUT_SEC="${STATE_TIMEOUT_SEC:-6s}"

CONFIRM_GROUND_AREA_CLEAR="${CONFIRM_GROUND_AREA_CLEAR:-false}"
CONFIRM_LOW_SPEED_GROUND_TEST="${CONFIRM_LOW_SPEED_GROUND_TEST:-false}"
CONFIRM_VEHICLE_DISARMED="${CONFIRM_VEHICLE_DISARMED:-false}"
CONFIRM_RC_READY="${CONFIRM_RC_READY:-false}"
CONFIRM_PARAM_BACKUP="${CONFIRM_PARAM_BACKUP:-false}"
CONFIRM_QGC_DISARM_READY="${CONFIRM_QGC_DISARM_READY:-false}"
CONFIRM_PHYSICAL_POWER_CUTOFF_READY="${CONFIRM_PHYSICAL_POWER_CUTOFF_READY:-false}"
CONFIRM_REAL_LOCAL_POSITION="${CONFIRM_REAL_LOCAL_POSITION:-false}"
CONFIRM_CURRENT_DIFF_MAPPING="${CONFIRM_CURRENT_DIFF_MAPPING:-false}"
CONFIRM_WHEELS_INSTALLED="${CONFIRM_WHEELS_INSTALLED:-false}"

SETPOINT_VELOCITY_MAV_FRAME="${SETPOINT_VELOCITY_MAV_FRAME:-BODY_NED}"
FIRST_DISTANCE_M="${FIRST_DISTANCE_M:-3.0}"
SECOND_DISTANCE_M="${SECOND_DISTANCE_M:-3.0}"
LINEAR_SPEED_MPS="${LINEAR_SPEED_MPS:-0.12}"
TURN_ANGLE_DEG="${TURN_ANGLE_DEG:-90.0}"
TURN_DIRECTION_SIGN="${TURN_DIRECTION_SIGN:--1.0}"
TURN_LATERAL_SPEED_MPS="${TURN_LATERAL_SPEED_MPS:-0.10}"
TURN_FORWARD_SPEED_MPS="${TURN_FORWARD_SPEED_MPS:-0.0}"
YAW_TOLERANCE_DEG="${YAW_TOLERANCE_DEG:-12.0}"
DISTANCE_TOLERANCE_M="${DISTANCE_TOLERANCE_M:-0.12}"
FIRST_LEG_MAX_SEC="${FIRST_LEG_MAX_SEC:-35.0}"
TURN_MAX_SEC="${TURN_MAX_SEC:-12.0}"
SECOND_LEG_MAX_SEC="${SECOND_LEG_MAX_SEC:-35.0}"
MAX_LINEAR_SPEED_MPS="${MAX_LINEAR_SPEED_MPS:-0.20}"
MAX_WAIT_FOR_READY_SEC="${MAX_WAIT_FOR_READY_SEC:-60}"

missing=()
for item in \
  CONFIRM_GROUND_AREA_CLEAR \
  CONFIRM_LOW_SPEED_GROUND_TEST \
  CONFIRM_VEHICLE_DISARMED \
  CONFIRM_RC_READY \
  CONFIRM_PARAM_BACKUP \
  CONFIRM_QGC_DISARM_READY \
  CONFIRM_PHYSICAL_POWER_CUTOFF_READY \
  CONFIRM_REAL_LOCAL_POSITION \
  CONFIRM_CURRENT_DIFF_MAPPING \
  CONFIRM_WHEELS_INSTALLED
do
  if [ "${!item}" != "true" ]; then
    missing+=("$item=true")
  fi
done

if [ "${#missing[@]}" -gt 0 ]; then
  {
    echo "Refusing to start differential Offboard L-turn task."
    echo "Required confirmations:"
    for item in "${missing[@]}"; do
      echo "  $item"
    done
  } >&2
  exit 2
fi

# shellcheck disable=SC1091
source "$REPO_DIR/scripts/env.sh"

cleanup() {
  local status=$?
  trap - EXIT INT TERM
  set +e
  echo
  echo "Differential Offboard L-turn cleanup..."
  echo "Requesting MAVROS disarm..."
  timeout 5s ros2 service call "$ARMING_SERVICE" mavros_msgs/srv/CommandBool "{value: false}" || true
  echo "Requesting MANUAL mode..."
  timeout 5s ros2 service call "$SET_MODE_SERVICE" mavros_msgs/srv/SetMode "{base_mode: 0, custom_mode: 'MANUAL'}" || true
  echo "Final MAVROS state snapshot:"
  timeout 5s ros2 topic echo --once "$STATE_TOPIC" || true
  exit "$status"
}
trap cleanup EXIT INT TERM

topic_list="$(ros2 topic list --spin-time 2 || true)"
if ! printf '%s\n' "$topic_list" | grep -Fxq "$STATE_TOPIC"; then
  echo "MAVROS state topic was not discovered: $STATE_TOPIC" >&2
  echo "Start MAVROS first: ./scripts/run_mavros_px4_usb_to_qgc_logged.sh" >&2
  exit 1
fi
if ! printf '%s\n' "$topic_list" | grep -Fxq "$LOCAL_POSE_TOPIC"; then
  echo "Local pose topic was not discovered: $LOCAL_POSE_TOPIC" >&2
  echo "This task needs a real local position estimate from PX4/MAVROS." >&2
  exit 1
fi

echo
echo "Checking MAVROS state before L-turn task..."
state_snapshot="$(timeout "$STATE_TIMEOUT_SEC" ros2 topic echo --once "$STATE_TOPIC" || true)"
printf '%s\n' "$state_snapshot"
state_armed="$(printf '%s\n' "$state_snapshot" | awk '$1 == "armed:" {print $2; exit}')"
state_manual_input="$(printf '%s\n' "$state_snapshot" | awk '$1 == "manual_input:" {print $2; exit}')"
state_mode="$(printf '%s\n' "$state_snapshot" | sed -n 's/^mode: //p' | head -n 1)"
case "$state_armed" in False|false) ;; *) echo "Refusing to start while armed." >&2; exit 2 ;; esac
case "$state_manual_input" in True|true) ;; *) echo "Refusing unless manual_input=true." >&2; exit 2 ;; esac
case "$state_mode" in MANUAL|manual) ;; *) echo "Refusing unless mode=MANUAL." >&2; exit 2 ;; esac

echo
echo "Checking one local pose sample..."
timeout "$STATE_TIMEOUT_SEC" ros2 topic echo --once "$LOCAL_POSE_TOPIC"

case "$SETPOINT_VELOCITY_MAV_FRAME" in
  LOCAL_NED) SETPOINT_VELOCITY_MAV_FRAME_ID=1 ;;
  LOCAL_OFFSET_NED) SETPOINT_VELOCITY_MAV_FRAME_ID=7 ;;
  BODY_NED) SETPOINT_VELOCITY_MAV_FRAME_ID=8 ;;
  BODY_OFFSET_NED) SETPOINT_VELOCITY_MAV_FRAME_ID=9 ;;
  BODY_FRD) SETPOINT_VELOCITY_MAV_FRAME_ID=12 ;;
  *)
    echo "Unsupported SETPOINT_VELOCITY_MAV_FRAME: $SETPOINT_VELOCITY_MAV_FRAME" >&2
    exit 2
    ;;
esac

echo
echo "Configuring MAVROS setpoint_velocity frame:"
echo "  target: $SETPOINT_VELOCITY_MAV_FRAME ($SETPOINT_VELOCITY_MAV_FRAME_ID)"
service_list="$(ros2 service list --spin-time 2 || true)"
if printf '%s\n' "$service_list" | grep -Fxq "$SETPOINT_VELOCITY_MAV_FRAME_SERVICE"; then
  timeout 5s ros2 service call \
    "$SETPOINT_VELOCITY_MAV_FRAME_SERVICE" \
    mavros_msgs/srv/SetMavFrame \
    "{mav_frame: $SETPOINT_VELOCITY_MAV_FRAME_ID}"
else
  timeout 5s ros2 param set "$SETPOINT_VELOCITY_PARAM_NODE" mav_frame "$SETPOINT_VELOCITY_MAV_FRAME"
fi
timeout 5s ros2 param get "$SETPOINT_VELOCITY_PARAM_NODE" mav_frame || true

cat <<EOF

Effective L-turn settings:
  FIRST_DISTANCE_M=$FIRST_DISTANCE_M
  SECOND_DISTANCE_M=$SECOND_DISTANCE_M
  LINEAR_SPEED_MPS=$LINEAR_SPEED_MPS
  TURN_ANGLE_DEG=$TURN_ANGLE_DEG
  TURN_DIRECTION_SIGN=$TURN_DIRECTION_SIGN (-1 means left on this PX4/MAVROS BODY_NED setup)
  TURN_LATERAL_SPEED_MPS=$TURN_LATERAL_SPEED_MPS
  TURN_FORWARD_SPEED_MPS=$TURN_FORWARD_SPEED_MPS
  YAW_TOLERANCE_DEG=$YAW_TOLERANCE_DEG
  FIRST_LEG_MAX_SEC=$FIRST_LEG_MAX_SEC
  TURN_MAX_SEC=$TURN_MAX_SEC
  SECOND_LEG_MAX_SEC=$SECOND_LEG_MAX_SEC
  SETPOINT_VELOCITY_MAV_FRAME=$SETPOINT_VELOCITY_MAV_FRAME
  ARM_ON_START=${ARM_ON_START:-true}

Expected path: forward ${FIRST_DISTANCE_M}m, arc/turn left toward a
${TURN_ANGLE_DEG}-degree heading change, then forward ${SECOND_DISTANCE_M}m.
EOF

mkdir -p "$REPO_DIR/results/ros_logs"
export ROS_LOG_DIR="${ROS_LOG_DIR:-$REPO_DIR/results/ros_logs}"

python3 "$REPO_DIR/src/real_rover_mavros_offboard_l_turn.py" \
  --ros-args \
  -p mavros_namespace:="$MAVROS_NS" \
  -p command_rate_hz:="${COMMAND_RATE_HZ:-20}" \
  -p publish_unstamped_cmd_vel:="${PUBLISH_UNSTAMPED_CMD_VEL:-true}" \
  -p warmup_sec:="${WARMUP_SEC:-2.0}" \
  -p initial_stop_sec:="${INITIAL_STOP_SEC:-0.5}" \
  -p stop_after_first_sec:="${STOP_AFTER_FIRST_SEC:-0.3}" \
  -p stop_after_turn_sec:="${STOP_AFTER_TURN_SEC:-0.3}" \
  -p final_stop_sec:="${FINAL_STOP_SEC:-1.0}" \
  -p first_distance_m:="$FIRST_DISTANCE_M" \
  -p second_distance_m:="$SECOND_DISTANCE_M" \
  -p linear_speed_mps:="$LINEAR_SPEED_MPS" \
  -p turn_angle_deg:="$TURN_ANGLE_DEG" \
  -p turn_direction_sign:="$TURN_DIRECTION_SIGN" \
  -p turn_lateral_speed_mps:="$TURN_LATERAL_SPEED_MPS" \
  -p turn_forward_speed_mps:="$TURN_FORWARD_SPEED_MPS" \
  -p yaw_tolerance_deg:="$YAW_TOLERANCE_DEG" \
  -p distance_tolerance_m:="$DISTANCE_TOLERANCE_M" \
  -p first_leg_max_sec:="$FIRST_LEG_MAX_SEC" \
  -p turn_max_sec:="$TURN_MAX_SEC" \
  -p second_leg_max_sec:="$SECOND_LEG_MAX_SEC" \
  -p max_linear_speed_mps:="$MAX_LINEAR_SPEED_MPS" \
  -p max_pose_age_sec:="${MAX_POSE_AGE_SEC:-1.0}" \
  -p mode_change_on_start:="${MODE_CHANGE_ON_START:-true}" \
  -p arm_on_start:="${ARM_ON_START:-true}" \
  -p require_armed_before_mode_change:="${REQUIRE_ARMED_BEFORE_MODE_CHANGE:-false}" \
  -p mode_request_retry_sec:="${MODE_REQUEST_RETRY_SEC:-2.0}" \
  -p arm_request_retry_sec:="${ARM_REQUEST_RETRY_SEC:-2.0}" \
  -p disarm_on_finish:="${DISARM_ON_FINISH:-true}" \
  -p require_connected:="${REQUIRE_CONNECTED:-true}" \
  -p require_offboard_mode:="${REQUIRE_OFFBOARD_MODE:-true}" \
  -p require_armed:="${REQUIRE_ARMED:-true}" \
  -p abort_on_mode_exit:="${ABORT_ON_MODE_EXIT:-true}" \
  -p abort_on_disarm:="${ABORT_ON_DISARM:-true}" \
  -p abort_on_arm_rejected:="${ABORT_ON_ARM_REJECTED:-true}" \
  -p max_wait_for_ready_sec:="$MAX_WAIT_FOR_READY_SEC" \
  -p stop_burst_sec:="${STOP_BURST_SEC:-0.8}" \
  -p test_surface:="ground" \
  -p confirm_ground_area_clear:=true \
  -p confirm_low_speed_ground_test:=true \
  -p confirm_rc_ready:=true \
  -p confirm_param_backup:=true \
  -p confirm_real_local_position:=true
