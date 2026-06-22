#!/usr/bin/env bash

set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

FAKE_VISION_MOTION_LOG_DISABLE="${FAKE_VISION_MOTION_LOG_DISABLE:-false}"
if [ "$FAKE_VISION_MOTION_LOG_DISABLE" != "true" ] \
  && [ "${FAKE_VISION_MOTION_LOG_ACTIVE:-false}" != "true" ]; then
  FAKE_VISION_MOTION_LOG_ROOT="${FAKE_VISION_MOTION_LOG_ROOT:-$REPO_DIR/results/fake_vision_offboard_motion}"
  FAKE_VISION_MOTION_RUN_ID="${FAKE_VISION_MOTION_RUN_ID:-$(date +%Y%m%d_%H%M%S)}"
  FAKE_VISION_MOTION_LOG_DIR="$FAKE_VISION_MOTION_LOG_ROOT/$FAKE_VISION_MOTION_RUN_ID"
  FAKE_VISION_MOTION_LOG_FILE="$FAKE_VISION_MOTION_LOG_DIR/fake_vision_offboard_motion.log"
  mkdir -p "$FAKE_VISION_MOTION_LOG_DIR"
  ln -sfn "$FAKE_VISION_MOTION_LOG_DIR" "$FAKE_VISION_MOTION_LOG_ROOT/latest"
  export FAKE_VISION_MOTION_LOG_ACTIVE=true
  export FAKE_VISION_MOTION_LOG_DIR
  export FAKE_VISION_MOTION_LOG_FILE

  echo "Saving indoor fake-vision Offboard motion log:"
  echo "  directory: $FAKE_VISION_MOTION_LOG_DIR"
  echo "  file:      $FAKE_VISION_MOTION_LOG_FILE"
  echo "  latest:    $FAKE_VISION_MOTION_LOG_ROOT/latest"
  echo

  exec > >(tee -a "$FAKE_VISION_MOTION_LOG_FILE") 2>&1
  echo "===== INDOOR FAKE-VISION OFFBOARD MOTION LOG START $(date --iso-8601=seconds) ====="
  echo "cwd=$PWD"
  echo "command=$0 $*"
  echo
fi

cat <<'EOF'
Indoor fake-vision MAVROS Offboard low-speed motion task.

Default behavior:
  - starts fixed external-vision pose/odometry input
  - waits for MANUAL arm from RC/QGC before requesting OFFBOARD
  - does not arm from the script
  - drives: forward 1s, stop, backward 1s, stop, left/right small turns
  - disarms and requests MANUAL on exit

Use only with wheels lifted.
EOF

MAVROS_NS="${MAVROS_NS:-/mavros}"
MAVROS_NS="${MAVROS_NS%/}"
STATE_TOPIC="${STATE_TOPIC:-$MAVROS_NS/state}"
PARAM_NODE="${PARAM_NODE:-$MAVROS_NS/param}"
PARAM_PULL_SERVICE="${PARAM_PULL_SERVICE:-$MAVROS_NS/param/pull}"
ARMING_SERVICE="${ARMING_SERVICE:-$MAVROS_NS/cmd/arming}"
SET_MODE_SERVICE="${SET_MODE_SERVICE:-$MAVROS_NS/set_mode}"
SETPOINT_VELOCITY_PARAM_NODE="${SETPOINT_VELOCITY_PARAM_NODE:-$MAVROS_NS/setpoint_velocity}"
SETPOINT_VELOCITY_MAV_FRAME_SERVICE="${SETPOINT_VELOCITY_MAV_FRAME_SERVICE:-$MAVROS_NS/setpoint_velocity/mav_frame}"
STATE_TIMEOUT_SEC="${STATE_TIMEOUT_SEC:-6s}"
PARAM_TIMEOUT_SEC="${PARAM_TIMEOUT_SEC:-12s}"
FAKE_VISION_WARMUP_SEC="${FAKE_VISION_WARMUP_SEC:-15}"
FAKE_VISION_DURATION_SEC="${FAKE_VISION_DURATION_SEC:-180}"
RC_WATCH_ENABLED="${RC_WATCH_ENABLED:-true}"
RC_WATCH_DURATION_SEC="${RC_WATCH_DURATION_SEC:-150}"
RC_WATCH_CHANNELS_TO_PRINT="${RC_WATCH_CHANNELS_TO_PRINT:-8}"
RC_WATCH_PRINT_PERIOD_SEC="${RC_WATCH_PRINT_PERIOD_SEC:-1.0}"
RC_WATCH_CHANGE_THRESHOLD_US="${RC_WATCH_CHANGE_THRESHOLD_US:-5}"
AUTO_RESTORE_OUTPUT_MAPPING="${AUTO_RESTORE_OUTPUT_MAPPING:-true}"

mav_frame_id() {
  case "$1" in
    LOCAL_NED) echo 1 ;;
    LOCAL_OFFSET_NED) echo 7 ;;
    BODY_NED) echo 8 ;;
    BODY_OFFSET_NED) echo 9 ;;
    BODY_FRD) echo 12 ;;
    *)
      echo "Unsupported SETPOINT_VELOCITY_MAV_FRAME: $1" >&2
      echo "Supported: LOCAL_NED, LOCAL_OFFSET_NED, BODY_NED, BODY_OFFSET_NED, BODY_FRD" >&2
      return 2
      ;;
  esac
}

SETPOINT_VELOCITY_MAV_FRAME="${SETPOINT_VELOCITY_MAV_FRAME:-BODY_NED}"
SETPOINT_VELOCITY_MAV_FRAME_ID="${SETPOINT_VELOCITY_MAV_FRAME_ID:-$(mav_frame_id "$SETPOINT_VELOCITY_MAV_FRAME")}"

COMMAND_RATE_HZ="${COMMAND_RATE_HZ:-20}"
PUBLISH_UNSTAMPED_CMD_VEL="${PUBLISH_UNSTAMPED_CMD_VEL:-true}"
WARMUP_SEC="${WARMUP_SEC:-2.0}"
STOP_SEC="${STOP_SEC:-1.0}"
FORWARD_SEC="${FORWARD_SEC:-1.0}"
BACKWARD_SEC="${BACKWARD_SEC:-1.0}"
TURN_SEC="${TURN_SEC:-0.5}"
FINAL_STOP_SEC="${FINAL_STOP_SEC:-2.0}"
LINEAR_SPEED_MPS="${LINEAR_SPEED_MPS:-0.45}"
LINEAR_DIRECTION_SIGN="${LINEAR_DIRECTION_SIGN:--1.0}"
TURN_LINEAR_SPEED_MPS="${TURN_LINEAR_SPEED_MPS:-0.25}"
TURN_LINEAR_DIRECTION_SIGN="${TURN_LINEAR_DIRECTION_SIGN:-1.0}"
TURN_LATERAL_SPEED_MPS="${TURN_LATERAL_SPEED_MPS:-0.0}"
TURN_YAW_RATE_RADPS="${TURN_YAW_RATE_RADPS:-0.35}"
MAX_LINEAR_SPEED_MPS="${MAX_LINEAR_SPEED_MPS:-0.60}"
MAX_YAW_RATE_RADPS="${MAX_YAW_RATE_RADPS:-0.60}"
MAX_WAIT_FOR_READY_SEC="${MAX_WAIT_FOR_READY_SEC:-90}"

CONFIRM_WHEELS_LIFTED="${CONFIRM_WHEELS_LIFTED:-false}"
CONFIRM_VEHICLE_DISARMED="${CONFIRM_VEHICLE_DISARMED:-false}"
CONFIRM_RC_READY="${CONFIRM_RC_READY:-false}"
CONFIRM_PARAM_BACKUP="${CONFIRM_PARAM_BACKUP:-false}"
CONFIRM_QGC_DISARM_READY="${CONFIRM_QGC_DISARM_READY:-false}"
CONFIRM_PHYSICAL_POWER_CUTOFF_READY="${CONFIRM_PHYSICAL_POWER_CUTOFF_READY:-false}"
CONFIRM_FAKE_LOCAL_POSITION_ONLY="${CONFIRM_FAKE_LOCAL_POSITION_ONLY:-false}"
CONFIRM_LOW_SPEED_WHEELS_TEST="${CONFIRM_LOW_SPEED_WHEELS_TEST:-false}"

missing_confirmations=()
if [ "$CONFIRM_WHEELS_LIFTED" != "true" ]; then
  missing_confirmations+=("CONFIRM_WHEELS_LIFTED=true")
fi
if [ "$CONFIRM_VEHICLE_DISARMED" != "true" ]; then
  missing_confirmations+=("CONFIRM_VEHICLE_DISARMED=true")
fi
if [ "$CONFIRM_RC_READY" != "true" ]; then
  missing_confirmations+=("CONFIRM_RC_READY=true")
fi
if [ "$CONFIRM_PARAM_BACKUP" != "true" ]; then
  missing_confirmations+=("CONFIRM_PARAM_BACKUP=true")
fi
if [ "$CONFIRM_QGC_DISARM_READY" != "true" ]; then
  missing_confirmations+=("CONFIRM_QGC_DISARM_READY=true")
fi
if [ "$CONFIRM_PHYSICAL_POWER_CUTOFF_READY" != "true" ]; then
  missing_confirmations+=("CONFIRM_PHYSICAL_POWER_CUTOFF_READY=true")
fi
if [ "$CONFIRM_FAKE_LOCAL_POSITION_ONLY" != "true" ]; then
  missing_confirmations+=("CONFIRM_FAKE_LOCAL_POSITION_ONLY=true")
fi
if [ "$CONFIRM_LOW_SPEED_WHEELS_TEST" != "true" ]; then
  missing_confirmations+=("CONFIRM_LOW_SPEED_WHEELS_TEST=true")
fi
if [ "${#missing_confirmations[@]}" -gt 0 ]; then
  {
    echo "Refusing to start indoor fake-vision Offboard motion task."
    echo "Required confirmations:"
    for item in "${missing_confirmations[@]}"; do
      echo "  $item"
    done
  } >&2
  exit 2
fi

# shellcheck disable=SC1091
source "$REPO_DIR/scripts/env.sh"

fake_vision_pid=""
rc_watch_pid=""

cleanup() {
  local status=$?
  trap - EXIT INT TERM
  echo
  echo "Indoor fake-vision Offboard motion cleanup..."
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
    echo "Restoring RC passthrough output mapping..."
    sleep 1
    CONFIRM_PARAM_BACKUP=true \
    CONFIRM_WHEELS_LIFTED=true \
    CONFIRM_VEHICLE_DISARMED=true \
    CONFIRM_QGC_DISARM_READY=true \
    CONFIRM_PHYSICAL_POWER_CUTOFF_READY=true \
      "$REPO_DIR/scripts/set_px4_rover_output_mapping.sh" restore-baseline || true
  fi
  if [ -n "$rc_watch_pid" ]; then
    echo "Stopping RC I/O watcher process group pgid=$rc_watch_pid"
    kill -TERM -- "-$rc_watch_pid" >/dev/null 2>&1 || \
      kill "$rc_watch_pid" >/dev/null 2>&1 || true
    wait "$rc_watch_pid" >/dev/null 2>&1 || true
  fi
  if [ -n "$fake_vision_pid" ]; then
    echo "Stopping fake external-vision publisher process group pgid=$fake_vision_pid"
    kill -TERM -- "-$fake_vision_pid" >/dev/null 2>&1 || \
      kill "$fake_vision_pid" >/dev/null 2>&1 || true
    sleep 0.5
    kill -KILL -- "-$fake_vision_pid" >/dev/null 2>&1 || true
    wait "$fake_vision_pid" >/dev/null 2>&1 || true
  fi
  exit "$status"
}
trap cleanup EXIT INT TERM

if ! ros2 topic list --spin-time 2 | grep -Fxq "$STATE_TOPIC"; then
  echo "MAVROS state topic was not discovered: $STATE_TOPIC" >&2
  echo "Start MAVROS first: ./scripts/run_mavros_px4_usb_to_qgc_logged.sh" >&2
  exit 1
fi

echo
echo "Checking MAVROS state before fake-vision Offboard motion..."
state_snapshot="$(timeout "$STATE_TIMEOUT_SEC" ros2 topic echo --once "$STATE_TOPIC" || true)"
if [ -z "$state_snapshot" ]; then
  echo "Could not read $STATE_TOPIC." >&2
  exit 1
fi
state_connected="$(printf '%s\n' "$state_snapshot" | awk '$1 == "connected:" {print $2; exit}')"
state_armed="$(printf '%s\n' "$state_snapshot" | awk '$1 == "armed:" {print $2; exit}')"
state_manual_input="$(printf '%s\n' "$state_snapshot" | awk '$1 == "manual_input:" {print $2; exit}')"
state_mode="$(printf '%s\n' "$state_snapshot" | sed -n 's/^mode: //p' | head -n 1)"
echo "Current MAVROS state:"
echo "  connected=${state_connected:-unknown}"
echo "  armed=${state_armed:-unknown}"
echo "  manual_input=${state_manual_input:-unknown}"
echo "  mode=${state_mode:-unknown}"

case "$state_armed" in
  False|false) ;;
  *)
    echo "Refusing to start while the vehicle is armed." >&2
    exit 2
    ;;
esac
case "$state_manual_input" in
  True|true) ;;
  *)
    echo "Refusing to start unless manual_input is true." >&2
    echo "Return the RC/mode switch to MANUAL and verify /mavros/state first." >&2
    exit 2
    ;;
esac
case "$state_mode" in
  MANUAL|manual) ;;
  *)
    echo "Refusing to start unless mode is MANUAL." >&2
    echo "Run: ros2 service call $SET_MODE_SERVICE mavros_msgs/srv/SetMode \"{base_mode: 0, custom_mode: 'MANUAL'}\"" >&2
    exit 2
    ;;
esac

echo
echo "Checking COM_RC_IN_MODE and EKF external-vision parameters..."
timeout "$PARAM_TIMEOUT_SEC" ros2 service call \
  "$PARAM_PULL_SERVICE" \
  mavros_msgs/srv/ParamPull \
  "{force_pull: false}" >/dev/null
for param in COM_RC_IN_MODE EKF2_EV_CTRL COM_ARM_WO_GPS; do
  echo "===== $param ====="
  timeout "$PARAM_TIMEOUT_SEC" ros2 param get "$PARAM_NODE" "$param" || true
done
current_rc_mode="$(timeout "$PARAM_TIMEOUT_SEC" ros2 param get "$PARAM_NODE" COM_RC_IN_MODE || true)"
if ! printf '%s\n' "$current_rc_mode" | grep -Eq '(^|[^0-9])3([^0-9]|$)'; then
  echo "Refusing to run unless COM_RC_IN_MODE is 3." >&2
  echo "Restore first: CONFIRM_PARAM_BACKUP=true ./scripts/set_px4_com_rc_in_mode.sh 3" >&2
  exit 2
fi
main1_func="$(timeout "$PARAM_TIMEOUT_SEC" ros2 param get "$PARAM_NODE" PWM_MAIN_FUNC1 || true)"
main2_func="$(timeout "$PARAM_TIMEOUT_SEC" ros2 param get "$PARAM_NODE" PWM_MAIN_FUNC2 || true)"
if ! printf '%s\n' "$main1_func" | grep -Eq '(^|[^0-9])101([^0-9]|$)' \
  || ! printf '%s\n' "$main2_func" | grep -Eq '(^|[^0-9])201([^0-9]|$)'; then
  cat >&2 <<'EOF'
Refusing to run because PX4 MAIN1/MAIN2 are not mapped to rover controller outputs.

This fake-vision Offboard task needs:
  PWM_MAIN_FUNC1=101  Motor 1 / throttle
  PWM_MAIN_FUNC2=201  Servo 1 / steering

Apply the wheels-lifted PX4 output mapping first:

  CONFIRM_PARAM_BACKUP=true \
  CONFIRM_WHEELS_LIFTED=true \
  CONFIRM_VEHICLE_DISARMED=true \
  CONFIRM_QGC_DISARM_READY=true \
  CONFIRM_PHYSICAL_POWER_CUTOFF_READY=true \
  ./scripts/set_px4_rover_output_mapping.sh apply

Then verify MANUAL with wheels lifted before re-running this motion task.
EOF
  echo "Current PWM_MAIN_FUNC1:" >&2
  printf '%s\n' "$main1_func" >&2
  echo "Current PWM_MAIN_FUNC2:" >&2
  printf '%s\n' "$main2_func" >&2
  exit 2
fi

echo
echo "Configuring MAVROS setpoint_velocity frame..."
echo "  target frame: $SETPOINT_VELOCITY_MAV_FRAME ($SETPOINT_VELOCITY_MAV_FRAME_ID)"
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

cat <<EOF

Effective indoor fake-vision Offboard motion settings:
  MAVROS_NS=$MAVROS_NS
  SETPOINT_VELOCITY_MAV_FRAME=$SETPOINT_VELOCITY_MAV_FRAME
  FAKE_VISION_WARMUP_SEC=$FAKE_VISION_WARMUP_SEC
  FAKE_VISION_DURATION_SEC=$FAKE_VISION_DURATION_SEC
  LINEAR_SPEED_MPS=$LINEAR_SPEED_MPS
  LINEAR_DIRECTION_SIGN=$LINEAR_DIRECTION_SIGN
  TURN_LINEAR_SPEED_MPS=$TURN_LINEAR_SPEED_MPS
  TURN_LINEAR_DIRECTION_SIGN=$TURN_LINEAR_DIRECTION_SIGN
  TURN_LATERAL_SPEED_MPS=$TURN_LATERAL_SPEED_MPS
  TURN_YAW_RATE_RADPS=$TURN_YAW_RATE_RADPS
  FORWARD_SEC=$FORWARD_SEC
  BACKWARD_SEC=$BACKWARD_SEC
  TURN_SEC=$TURN_SEC
  MAX_WAIT_FOR_READY_SEC=$MAX_WAIT_FOR_READY_SEC
  RC_WATCH_ENABLED=$RC_WATCH_ENABLED

Note:
  Default lifted-wheel speed is intentionally above the Arduino bridge deadband.
  Lower it later after the PX4-to-bridge mapping is verified.
EOF

mkdir -p "$REPO_DIR/results/ros_logs"
export ROS_LOG_DIR="${ROS_LOG_DIR:-$REPO_DIR/results/ros_logs}"

if [ "$RC_WATCH_ENABLED" = "true" ]; then
  echo
  echo "Starting MAVROS RC I/O watcher..."
  setsid env \
    ROS_LOG_DIR="$ROS_LOG_DIR" \
    python3 "$REPO_DIR/src/mavros_rc_io_watch.py" \
      --ros-args \
      -p mavros_namespace:="$MAVROS_NS" \
      -p duration_sec:="$RC_WATCH_DURATION_SEC" \
      -p channels_to_print:="$RC_WATCH_CHANNELS_TO_PRINT" \
      -p print_period_sec:="$RC_WATCH_PRINT_PERIOD_SEC" \
      -p change_threshold_us:="$RC_WATCH_CHANGE_THRESHOLD_US" &
  rc_watch_pid=$!
  echo "RC I/O watcher process group pgid=$rc_watch_pid"
fi

echo
echo "Starting fake external-vision publisher..."
setsid env \
  ROS_LOG_DIR="$ROS_LOG_DIR" \
  python3 "$REPO_DIR/src/mavros_fake_external_vision.py" \
    --ros-args \
    -p mavros_namespace:="$MAVROS_NS" \
    -p rate_hz:="${FAKE_VISION_RATE_HZ:-30}" \
    -p duration_sec:="$FAKE_VISION_DURATION_SEC" \
    -p use_current_local_pose:="${USE_CURRENT_LOCAL_POSE:-true}" \
    -p current_pose_wait_sec:="${CURRENT_POSE_WAIT_SEC:-5}" \
    -p publish_vision_pose:="${PUBLISH_VISION_POSE:-true}" \
    -p publish_vision_pose_cov:="${PUBLISH_VISION_POSE_COV:-true}" \
    -p publish_odometry:="${PUBLISH_ODOMETRY:-true}" \
    -p pose_covariance:="${POSE_COVARIANCE:-0.02}" \
    -p orientation_covariance:="${ORIENTATION_COVARIANCE:-0.02}" \
    -p velocity_covariance:="${VELOCITY_COVARIANCE:-0.05}" &
fake_vision_pid=$!

echo "Fake external-vision process group pgid=$fake_vision_pid"
echo "Warming up fake external vision for ${FAKE_VISION_WARMUP_SEC}s..."
sleep "$FAKE_VISION_WARMUP_SEC"

echo
cat <<'EOF'
Running low-speed Offboard motion task...

Operator action:
  1. Keep the RC mode switch in MANUAL.
  2. Arm from RC or QGC while QGC still shows MANUAL.
  3. After MAVROS sees armed=true, this script requests OFFBOARD.
  4. Watch the lifted wheels.
  5. Use RC disarm/kill, QGC disarm, physical power cutoff, or Ctrl+C to abort.
EOF

OFFBOARD_LOG_ACTIVE=true \
OFFBOARD_LOG_DIR="$FAKE_VISION_MOTION_LOG_DIR" \
OFFBOARD_LOG_FILE="$FAKE_VISION_MOTION_LOG_FILE" \
TEST_SURFACE=wheels_lifted \
MODE_CHANGE_ON_START=true \
ARM_ON_START=false \
REQUIRE_ARMED_BEFORE_MODE_CHANGE=true \
MODE_REQUEST_RETRY_SEC="${MODE_REQUEST_RETRY_SEC:-2.0}" \
ARM_REQUEST_RETRY_SEC="${ARM_REQUEST_RETRY_SEC:-2.0}" \
DISARM_ON_FINISH=true \
REQUIRE_CONNECTED=true \
REQUIRE_OFFBOARD_MODE=true \
REQUIRE_ARMED=true \
ABORT_ON_MODE_EXIT=true \
ABORT_ON_DISARM=true \
ABORT_ON_ARM_REJECTED=true \
MAX_WAIT_FOR_READY_SEC="$MAX_WAIT_FOR_READY_SEC" \
COMMAND_RATE_HZ="$COMMAND_RATE_HZ" \
PUBLISH_UNSTAMPED_CMD_VEL="$PUBLISH_UNSTAMPED_CMD_VEL" \
WARMUP_SEC="$WARMUP_SEC" \
STOP_SEC="$STOP_SEC" \
FORWARD_SEC="$FORWARD_SEC" \
BACKWARD_SEC="$BACKWARD_SEC" \
TURN_SEC="$TURN_SEC" \
FINAL_STOP_SEC="$FINAL_STOP_SEC" \
LINEAR_SPEED_MPS="$LINEAR_SPEED_MPS" \
LINEAR_DIRECTION_SIGN="$LINEAR_DIRECTION_SIGN" \
TURN_LINEAR_SPEED_MPS="$TURN_LINEAR_SPEED_MPS" \
TURN_LINEAR_DIRECTION_SIGN="$TURN_LINEAR_DIRECTION_SIGN" \
TURN_LATERAL_SPEED_MPS="$TURN_LATERAL_SPEED_MPS" \
TURN_YAW_RATE_RADPS="$TURN_YAW_RATE_RADPS" \
MAX_LINEAR_SPEED_MPS="$MAX_LINEAR_SPEED_MPS" \
MAX_YAW_RATE_RADPS="$MAX_YAW_RATE_RADPS" \
CONFIRM_WHEELS_LIFTED=true \
CONFIRM_RC_READY=true \
CONFIRM_PARAM_BACKUP=true \
CONFIRM_QGC_DISARM_READY=true \
CONFIRM_PHYSICAL_POWER_CUTOFF_READY=true \
AUTO_RESTORE_OUTPUT_MAPPING="$AUTO_RESTORE_OUTPUT_MAPPING" \
"$REPO_DIR/scripts/run_real_rover_mavros_offboard_smoke.sh"
