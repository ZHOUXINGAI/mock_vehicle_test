#!/usr/bin/env bash

set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

LOG_ROOT="${DIFF_OFFBOARD_OUT_BACK_LOG_ROOT:-$REPO_DIR/results/differential_offboard_out_and_back}"
RUN_ID="${DIFF_OFFBOARD_OUT_BACK_RUN_ID:-$(date +%Y%m%d_%H%M%S)}"
LOG_DIR="$LOG_ROOT/$RUN_ID"
LOG_FILE="$LOG_DIR/differential_offboard_out_and_back.log"
mkdir -p "$LOG_DIR"
ln -sfn "$LOG_DIR" "$LOG_ROOT/latest"

echo "Saving differential Offboard out-and-back log:"
echo "  directory: $LOG_DIR"
echo "  file:      $LOG_FILE"
echo "  latest:    $LOG_ROOT/latest"
echo

exec > >(tee -a "$LOG_FILE") 2>&1
echo "===== DIFFERENTIAL OFFBOARD OUT-AND-BACK LOG START $(date --iso-8601=seconds) ====="
echo "cwd=$PWD"
echo "command=$0 $*"
echo

cat <<'EOF'
PX4 v1.17 differential-rover MAVROS true Offboard task.

Mission:
  1. drive forward 5 m using real /mavros/local_position/pose distance
  2. turn 180 degrees using local yaw
  3. drive forward 5 m again, returning near the start line

Do not run this with fake vision. It requires a real local position estimate
that changes with rover motion.
EOF

MAVROS_NS="${MAVROS_NS:-/mavros}"
MAVROS_NS="${MAVROS_NS%/}"
STATE_TOPIC="${STATE_TOPIC:-$MAVROS_NS/state}"
LOCAL_POSE_TOPIC="${LOCAL_POSE_TOPIC:-$MAVROS_NS/local_position/pose}"
ARMING_SERVICE="${ARMING_SERVICE:-$MAVROS_NS/cmd/arming}"
SET_MODE_SERVICE="${SET_MODE_SERVICE:-$MAVROS_NS/set_mode}"
PARAM_NODE="${PARAM_NODE:-$MAVROS_NS/param}"
SETPOINT_VELOCITY_PARAM_NODE="${SETPOINT_VELOCITY_PARAM_NODE:-$MAVROS_NS/setpoint_velocity}"
SETPOINT_VELOCITY_MAV_FRAME_SERVICE="${SETPOINT_VELOCITY_MAV_FRAME_SERVICE:-$MAVROS_NS/setpoint_velocity/mav_frame}"
STATE_TIMEOUT_SEC="${STATE_TIMEOUT_SEC:-6s}"
PARAM_TIMEOUT_SEC="${PARAM_TIMEOUT_SEC:-12s}"

CONFIRM_GROUND_AREA_CLEAR="${CONFIRM_GROUND_AREA_CLEAR:-false}"
CONFIRM_LOW_SPEED_GROUND_TEST="${CONFIRM_LOW_SPEED_GROUND_TEST:-false}"
CONFIRM_VEHICLE_DISARMED="${CONFIRM_VEHICLE_DISARMED:-false}"
CONFIRM_RC_READY="${CONFIRM_RC_READY:-false}"
CONFIRM_PARAM_BACKUP="${CONFIRM_PARAM_BACKUP:-false}"
CONFIRM_QGC_DISARM_READY="${CONFIRM_QGC_DISARM_READY:-false}"
CONFIRM_PHYSICAL_POWER_CUTOFF_READY="${CONFIRM_PHYSICAL_POWER_CUTOFF_READY:-false}"
CONFIRM_REAL_LOCAL_POSITION="${CONFIRM_REAL_LOCAL_POSITION:-false}"
CONFIRM_ARDUINO_DIFFERENTIAL_PWM_BRIDGE="${CONFIRM_ARDUINO_DIFFERENTIAL_PWM_BRIDGE:-false}"

OUTPUT_MAPPING_ACTION="${OUTPUT_MAPPING_ACTION:-apply-differential-limited}"
SETPOINT_VELOCITY_MAV_FRAME="${SETPOINT_VELOCITY_MAV_FRAME:-BODY_NED}"
FORWARD_DISTANCE_M="${FORWARD_DISTANCE_M:-5.0}"
LINEAR_SPEED_MPS="${LINEAR_SPEED_MPS:-0.25}"
LINEAR_DIRECTION_SIGN="${LINEAR_DIRECTION_SIGN:-1.0}"
TURN_ANGLE_DEG="${TURN_ANGLE_DEG:-180.0}"
TURN_YAW_RATE_RADPS="${TURN_YAW_RATE_RADPS:-0.35}"
TURN_DIRECTION_SIGN="${TURN_DIRECTION_SIGN:-1.0}"
DISTANCE_TOLERANCE_M="${DISTANCE_TOLERANCE_M:-0.15}"
YAW_TOLERANCE_DEG="${YAW_TOLERANCE_DEG:-8.0}"
MAX_LINEAR_SPEED_MPS="${MAX_LINEAR_SPEED_MPS:-0.50}"
MAX_YAW_RATE_RADPS="${MAX_YAW_RATE_RADPS:-0.70}"
MAX_WAIT_FOR_READY_SEC="${MAX_WAIT_FOR_READY_SEC:-120}"

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
  CONFIRM_ARDUINO_DIFFERENTIAL_PWM_BRIDGE
do
  if [ "${!item}" != "true" ]; then
    missing+=("$item=true")
  fi
done

if [ "${#missing[@]}" -gt 0 ]; then
  {
    echo "Refusing to start differential Offboard out-and-back task."
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
  echo "Differential Offboard out-and-back cleanup..."
  echo "Requesting MAVROS disarm..."
  timeout 5s ros2 service call "$ARMING_SERVICE" mavros_msgs/srv/CommandBool "{value: false}" || true
  echo "Requesting MANUAL mode..."
  timeout 5s ros2 service call "$SET_MODE_SERVICE" mavros_msgs/srv/SetMode "{base_mode: 0, custom_mode: 'MANUAL'}" || true
  echo "Restoring observed manual RC passthrough baseline..."
  TEST_SURFACE=ground \
  CONFIRM_PARAM_BACKUP=true \
  CONFIRM_GROUND_AREA_CLEAR=true \
  CONFIRM_LOW_SPEED_GROUND_TEST=true \
  CONFIRM_VEHICLE_DISARMED=true \
  CONFIRM_QGC_DISARM_READY=true \
  CONFIRM_PHYSICAL_POWER_CUTOFF_READY=true \
    "$REPO_DIR/scripts/set_px4_rover_output_mapping.sh" restore-baseline || true
  echo "Final MAVROS state snapshot:"
  timeout 5s ros2 topic echo --once "$STATE_TOPIC" || true
  exit "$status"
}
trap cleanup EXIT INT TERM

if ! ros2 topic list --spin-time 2 | grep -Fxq "$STATE_TOPIC"; then
  echo "MAVROS state topic was not discovered: $STATE_TOPIC" >&2
  echo "Start MAVROS first: ./scripts/run_mavros_px4_usb_to_qgc_logged.sh" >&2
  exit 1
fi
if ! ros2 topic list --spin-time 2 | grep -Fxq "$LOCAL_POSE_TOPIC"; then
  echo "Local pose topic was not discovered: $LOCAL_POSE_TOPIC" >&2
  echo "This task needs a real local position estimate." >&2
  exit 1
fi

echo
echo "Checking MAVROS state before out-and-back task..."
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

echo
echo "Applying PX4 differential output mapping: $OUTPUT_MAPPING_ACTION"
TEST_SURFACE=ground \
CONFIRM_PARAM_BACKUP=true \
CONFIRM_GROUND_AREA_CLEAR=true \
CONFIRM_LOW_SPEED_GROUND_TEST=true \
CONFIRM_VEHICLE_DISARMED=true \
CONFIRM_QGC_DISARM_READY=true \
CONFIRM_PHYSICAL_POWER_CUTOFF_READY=true \
  "$REPO_DIR/scripts/set_px4_rover_output_mapping.sh" "$OUTPUT_MAPPING_ACTION"

echo
echo "Checking differential output mapping..."
for param in SYS_AUTOSTART CA_AIRFRAME CA_R_REV PWM_MAIN_FUNC1 PWM_MAIN_FUNC2 PWM_MAIN_FUNC6 PWM_MAIN_FUNC7; do
  echo "===== $param ====="
  timeout "$PARAM_TIMEOUT_SEC" ros2 param get "$PARAM_NODE" "$param" || true
done

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

Effective out-and-back settings:
  FORWARD_DISTANCE_M=$FORWARD_DISTANCE_M
  LINEAR_SPEED_MPS=$LINEAR_SPEED_MPS
  LINEAR_DIRECTION_SIGN=$LINEAR_DIRECTION_SIGN
  TURN_ANGLE_DEG=$TURN_ANGLE_DEG
  TURN_YAW_RATE_RADPS=$TURN_YAW_RATE_RADPS
  TURN_DIRECTION_SIGN=$TURN_DIRECTION_SIGN
  OUTPUT_MAPPING_ACTION=$OUTPUT_MAPPING_ACTION

Arm manually in MANUAL when ready. The script will then request OFFBOARD.
EOF

mkdir -p "$REPO_DIR/results/ros_logs"
export ROS_LOG_DIR="${ROS_LOG_DIR:-$REPO_DIR/results/ros_logs}"

python3 "$REPO_DIR/src/real_rover_mavros_offboard_out_and_back.py" \
  --ros-args \
  -p mavros_namespace:="$MAVROS_NS" \
  -p command_rate_hz:="${COMMAND_RATE_HZ:-20}" \
  -p publish_unstamped_cmd_vel:="${PUBLISH_UNSTAMPED_CMD_VEL:-true}" \
  -p warmup_sec:="${WARMUP_SEC:-2.0}" \
  -p stop_sec:="${STOP_SEC:-1.5}" \
  -p final_stop_sec:="${FINAL_STOP_SEC:-2.0}" \
  -p forward_distance_m:="$FORWARD_DISTANCE_M" \
  -p linear_speed_mps:="$LINEAR_SPEED_MPS" \
  -p linear_direction_sign:="$LINEAR_DIRECTION_SIGN" \
  -p turn_angle_deg:="$TURN_ANGLE_DEG" \
  -p turn_yaw_rate_radps:="$TURN_YAW_RATE_RADPS" \
  -p turn_direction_sign:="$TURN_DIRECTION_SIGN" \
  -p distance_tolerance_m:="$DISTANCE_TOLERANCE_M" \
  -p yaw_tolerance_deg:="$YAW_TOLERANCE_DEG" \
  -p max_linear_speed_mps:="$MAX_LINEAR_SPEED_MPS" \
  -p max_yaw_rate_radps:="$MAX_YAW_RATE_RADPS" \
  -p max_pose_age_sec:="${MAX_POSE_AGE_SEC:-1.0}" \
  -p mode_change_on_start:="${MODE_CHANGE_ON_START:-true}" \
  -p arm_on_start:="${ARM_ON_START:-false}" \
  -p require_armed_before_mode_change:="${REQUIRE_ARMED_BEFORE_MODE_CHANGE:-true}" \
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
