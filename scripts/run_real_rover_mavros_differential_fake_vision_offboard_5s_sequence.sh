#!/usr/bin/env bash

set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

LOG_ROOT="${DIFF_OFFBOARD_5S_LOG_ROOT:-$REPO_DIR/results/differential_offboard_5s}"
RUN_ID="${DIFF_OFFBOARD_5S_RUN_ID:-$(date +%Y%m%d_%H%M%S)}"
LOG_DIR="$LOG_ROOT/$RUN_ID"
LOG_FILE="$LOG_DIR/differential_offboard_5s.log"
mkdir -p "$LOG_DIR"
ln -sfn "$LOG_DIR" "$LOG_ROOT/latest"

echo "Saving differential Offboard 5s sequence log:"
echo "  directory: $LOG_DIR"
echo "  file:      $LOG_FILE"
echo "  latest:    $LOG_ROOT/latest"
echo

exec > >(tee -a "$LOG_FILE") 2>&1
echo "===== DIFFERENTIAL OFFBOARD 5S SEQUENCE LOG START $(date --iso-8601=seconds) ====="
echo "cwd=$PWD"
echo "command=$0 $*"
echo

cat <<'EOF'
PX4 v1.17 differential-rover MAVROS Offboard 5-second sequence.

Sequence:
  forward 5s, stop, backward 5s, stop, left turn 5s, stop, right turn 5s, stop.

Use only with wheels lifted for the first runs.
Requires the Arduino differential PWM bridge, where Pixhawk PWM inputs are
left/right wheel commands, not throttle/steering.
EOF

MAVROS_NS="${MAVROS_NS:-/mavros}"
MAVROS_NS="${MAVROS_NS%/}"
STATE_TOPIC="${STATE_TOPIC:-$MAVROS_NS/state}"
ARMING_SERVICE="${ARMING_SERVICE:-$MAVROS_NS/cmd/arming}"
SET_MODE_SERVICE="${SET_MODE_SERVICE:-$MAVROS_NS/set_mode}"
PARAM_NODE="${PARAM_NODE:-$MAVROS_NS/param}"
PARAM_TIMEOUT_SEC="${PARAM_TIMEOUT_SEC:-12s}"
STATE_TIMEOUT_SEC="${STATE_TIMEOUT_SEC:-6s}"

CONFIRM_WHEELS_LIFTED="${CONFIRM_WHEELS_LIFTED:-false}"
CONFIRM_VEHICLE_DISARMED="${CONFIRM_VEHICLE_DISARMED:-false}"
CONFIRM_RC_READY="${CONFIRM_RC_READY:-false}"
CONFIRM_PARAM_BACKUP="${CONFIRM_PARAM_BACKUP:-false}"
CONFIRM_QGC_DISARM_READY="${CONFIRM_QGC_DISARM_READY:-false}"
CONFIRM_PHYSICAL_POWER_CUTOFF_READY="${CONFIRM_PHYSICAL_POWER_CUTOFF_READY:-false}"
CONFIRM_FAKE_LOCAL_POSITION_ONLY="${CONFIRM_FAKE_LOCAL_POSITION_ONLY:-false}"
CONFIRM_LOW_SPEED_WHEELS_TEST="${CONFIRM_LOW_SPEED_WHEELS_TEST:-false}"
CONFIRM_ARDUINO_DIFFERENTIAL_PWM_BRIDGE="${CONFIRM_ARDUINO_DIFFERENTIAL_PWM_BRIDGE:-false}"

OUTPUT_MAPPING_ACTION="${OUTPUT_MAPPING_ACTION:-apply-differential-limited}"
FAKE_VISION_DURATION_SEC="${FAKE_VISION_DURATION_SEC:-180}"
FAKE_VISION_WARMUP_SEC="${FAKE_VISION_WARMUP_SEC:-10}"
SETPOINT_VELOCITY_MAV_FRAME="${SETPOINT_VELOCITY_MAV_FRAME:-BODY_NED}"

LINEAR_SPEED_MPS="${LINEAR_SPEED_MPS:-0.20}"
LINEAR_DIRECTION_SIGN="${LINEAR_DIRECTION_SIGN:-1.0}"
TURN_YAW_RATE_RADPS="${TURN_YAW_RATE_RADPS:-0.35}"
TURN_SIGN="${TURN_SIGN:-1.0}"
MAX_LINEAR_SPEED_MPS="${MAX_LINEAR_SPEED_MPS:-0.35}"
MAX_YAW_RATE_RADPS="${MAX_YAW_RATE_RADPS:-0.60}"

missing=()
for item in \
  CONFIRM_WHEELS_LIFTED \
  CONFIRM_VEHICLE_DISARMED \
  CONFIRM_RC_READY \
  CONFIRM_PARAM_BACKUP \
  CONFIRM_QGC_DISARM_READY \
  CONFIRM_PHYSICAL_POWER_CUTOFF_READY \
  CONFIRM_FAKE_LOCAL_POSITION_ONLY \
  CONFIRM_LOW_SPEED_WHEELS_TEST \
  CONFIRM_ARDUINO_DIFFERENTIAL_PWM_BRIDGE
do
  if [ "${!item}" != "true" ]; then
    missing+=("$item=true")
  fi
done

if [ "${#missing[@]}" -gt 0 ]; then
  {
    echo "Refusing to start differential Offboard 5s sequence."
    echo "Required confirmations:"
    for item in "${missing[@]}"; do
      echo "  $item"
    done
  } >&2
  exit 2
fi

# shellcheck disable=SC1091
source "$REPO_DIR/scripts/env.sh"

fake_vision_pid=""

cleanup() {
  local status=$?
  trap - EXIT INT TERM
  set +e
  echo
  echo "Differential Offboard 5s cleanup..."
  echo "Requesting MAVROS disarm..."
  timeout 5s ros2 service call "$ARMING_SERVICE" mavros_msgs/srv/CommandBool "{value: false}" || true
  echo "Requesting MANUAL mode..."
  timeout 5s ros2 service call "$SET_MODE_SERVICE" mavros_msgs/srv/SetMode "{base_mode: 0, custom_mode: 'MANUAL'}" || true
  echo "Restoring observed manual RC passthrough baseline..."
  TEST_SURFACE=wheels_lifted \
  CONFIRM_PARAM_BACKUP=true \
  CONFIRM_WHEELS_LIFTED=true \
  CONFIRM_VEHICLE_DISARMED=true \
  CONFIRM_QGC_DISARM_READY=true \
  CONFIRM_PHYSICAL_POWER_CUTOFF_READY=true \
    "$REPO_DIR/scripts/set_px4_rover_output_mapping.sh" restore-baseline || true
  if [ -n "$fake_vision_pid" ]; then
    echo "Stopping fake external-vision publisher process group pgid=$fake_vision_pid"
    kill -TERM -- "-$fake_vision_pid" >/dev/null 2>&1 || kill "$fake_vision_pid" >/dev/null 2>&1 || true
    sleep 0.5
    kill -KILL -- "-$fake_vision_pid" >/dev/null 2>&1 || true
    wait "$fake_vision_pid" >/dev/null 2>&1 || true
  fi
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

echo
echo "Checking MAVROS state before differential Offboard 5s sequence..."
state_snapshot="$(timeout "$STATE_TIMEOUT_SEC" ros2 topic echo --once "$STATE_TOPIC" || true)"
printf '%s\n' "$state_snapshot"
state_armed="$(printf '%s\n' "$state_snapshot" | awk '$1 == "armed:" {print $2; exit}')"
state_manual_input="$(printf '%s\n' "$state_snapshot" | awk '$1 == "manual_input:" {print $2; exit}')"
state_mode="$(printf '%s\n' "$state_snapshot" | sed -n 's/^mode: //p' | head -n 1)"
case "$state_armed" in False|false) ;; *) echo "Refusing to start while armed." >&2; exit 2 ;; esac
case "$state_manual_input" in True|true) ;; *) echo "Refusing unless manual_input=true." >&2; exit 2 ;; esac
case "$state_mode" in MANUAL|manual) ;; *) echo "Refusing unless mode=MANUAL." >&2; exit 2 ;; esac

echo
echo "Applying PX4 differential output mapping: $OUTPUT_MAPPING_ACTION"
TEST_SURFACE=wheels_lifted \
CONFIRM_PARAM_BACKUP=true \
CONFIRM_WHEELS_LIFTED=true \
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

echo
echo "Starting fake external-vision publisher..."
mkdir -p "$REPO_DIR/results/ros_logs"
export ROS_LOG_DIR="${ROS_LOG_DIR:-$REPO_DIR/results/ros_logs}"
setsid env ROS_LOG_DIR="$ROS_LOG_DIR" \
  python3 "$REPO_DIR/src/mavros_fake_external_vision.py" \
    --ros-args \
    -p mavros_namespace:="$MAVROS_NS" \
    -p rate_hz:="${FAKE_VISION_RATE_HZ:-30}" \
    -p duration_sec:="$FAKE_VISION_DURATION_SEC" \
    -p use_current_local_pose:="${USE_CURRENT_LOCAL_POSE:-true}" \
    -p current_pose_wait_sec:="${CURRENT_POSE_WAIT_SEC:-5}" &
fake_vision_pid=$!
echo "Fake external-vision process group pgid=$fake_vision_pid"

echo "Waiting ${FAKE_VISION_WARMUP_SEC}s for fake local position..."
sleep "$FAKE_VISION_WARMUP_SEC"

cat <<EOF

Effective sequence settings:
  SETPOINT_VELOCITY_MAV_FRAME=$SETPOINT_VELOCITY_MAV_FRAME
  LINEAR_SPEED_MPS=$LINEAR_SPEED_MPS
  LINEAR_DIRECTION_SIGN=$LINEAR_DIRECTION_SIGN
  TURN_YAW_RATE_RADPS=$TURN_YAW_RATE_RADPS
  TURN_SIGN=$TURN_SIGN
  OUTPUT_MAPPING_ACTION=$OUTPUT_MAPPING_ACTION

Arm manually in MANUAL when ready. The script will then request OFFBOARD.
EOF

OFFBOARD_LOG_DISABLE=true \
MAVROS_NS="$MAVROS_NS" \
SETPOINT_VELOCITY_MAV_FRAME="$SETPOINT_VELOCITY_MAV_FRAME" \
COMMAND_RATE_HZ="${COMMAND_RATE_HZ:-20}" \
PUBLISH_UNSTAMPED_CMD_VEL="${PUBLISH_UNSTAMPED_CMD_VEL:-true}" \
WARMUP_SEC="${WARMUP_SEC:-2.0}" \
STOP_SEC="${STOP_SEC:-1.0}" \
FORWARD_SEC="${FORWARD_SEC:-5.0}" \
BACKWARD_SEC="${BACKWARD_SEC:-5.0}" \
TURN_SEC="${TURN_SEC:-5.0}" \
TURN_LEFT_SEC="${TURN_LEFT_SEC:-5.0}" \
TURN_RIGHT_SEC="${TURN_RIGHT_SEC:-5.0}" \
FINAL_STOP_SEC="${FINAL_STOP_SEC:-2.0}" \
LINEAR_SPEED_MPS="$LINEAR_SPEED_MPS" \
LINEAR_DIRECTION_SIGN="$LINEAR_DIRECTION_SIGN" \
TURN_LINEAR_SPEED_MPS="${TURN_LINEAR_SPEED_MPS:-0.0}" \
TURN_LINEAR_DIRECTION_SIGN="${TURN_LINEAR_DIRECTION_SIGN:-1.0}" \
TURN_LATERAL_SPEED_MPS="${TURN_LATERAL_SPEED_MPS:-0.0}" \
TURN_YAW_RATE_RADPS="$TURN_YAW_RATE_RADPS" \
TURN_SIGN="$TURN_SIGN" \
MAX_LINEAR_SPEED_MPS="$MAX_LINEAR_SPEED_MPS" \
MAX_YAW_RATE_RADPS="$MAX_YAW_RATE_RADPS" \
MODE_CHANGE_ON_START=true \
ARM_ON_START=false \
REQUIRE_ARMED_BEFORE_MODE_CHANGE=true \
DISARM_ON_FINISH=true \
REQUIRE_CONNECTED=true \
REQUIRE_OFFBOARD_MODE=true \
REQUIRE_ARMED=true \
ABORT_ON_MODE_EXIT=true \
ABORT_ON_DISARM=true \
MAX_WAIT_FOR_READY_SEC="${MAX_WAIT_FOR_READY_SEC:-120}" \
AUTO_RESTORE_OUTPUT_MAPPING=false \
TEST_SURFACE=wheels_lifted \
CONFIRM_WHEELS_LIFTED=true \
CONFIRM_RC_READY=true \
CONFIRM_PARAM_BACKUP=true \
CONFIRM_QGC_DISARM_READY=true \
CONFIRM_PHYSICAL_POWER_CUTOFF_READY=true \
  "$REPO_DIR/scripts/run_real_rover_mavros_offboard_smoke.sh"
