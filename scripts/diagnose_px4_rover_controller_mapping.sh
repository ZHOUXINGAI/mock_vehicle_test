#!/usr/bin/env bash

set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

MAVROS_NS="${MAVROS_NS:-/mavros}"
MAVROS_NS="${MAVROS_NS%/}"
PARAM_SET_SERVICE="${PARAM_SET_SERVICE:-$MAVROS_NS/param/set}"
PARAM_PULL_SERVICE="${PARAM_PULL_SERVICE:-$MAVROS_NS/param/pull}"
STATE_TOPIC="${STATE_TOPIC:-$MAVROS_NS/state}"
ARM_SERVICE="${ARM_SERVICE:-$MAVROS_NS/cmd/arming}"
SET_MODE_SERVICE="${SET_MODE_SERVICE:-$MAVROS_NS/set_mode}"
PARAM_NODE="${PARAM_NODE:-$MAVROS_NS/param}"

WATCH_SEC="${WATCH_SEC:-35}"
ARM_HOLD_SEC="${ARM_HOLD_SEC:-8}"
ARDUINO_PORT="${ARDUINO_PORT:-/dev/ttyUSB0}"
AUTO_ARM="${AUTO_ARM:-true}"
RESTORE_ON_EXIT="${RESTORE_ON_EXIT:-true}"

CONFIRM_PARAM_BACKUP="${CONFIRM_PARAM_BACKUP:-false}"
CONFIRM_WHEELS_LIFTED="${CONFIRM_WHEELS_LIFTED:-false}"
CONFIRM_MOTOR_POWER_DISCONNECTED="${CONFIRM_MOTOR_POWER_DISCONNECTED:-false}"
CONFIRM_QGC_DISARM_READY="${CONFIRM_QGC_DISARM_READY:-false}"
CONFIRM_PHYSICAL_POWER_CUTOFF_READY="${CONFIRM_PHYSICAL_POWER_CUTOFF_READY:-false}"
CONFIRM_AUTO_ARM="${CONFIRM_AUTO_ARM:-false}"

LOG_ROOT="${CONTROLLER_MAPPING_DIAG_LOG_ROOT:-$REPO_DIR/results/controller_mapping_diag}"
RUN_ID="${CONTROLLER_MAPPING_DIAG_RUN_ID:-$(date +%Y%m%d_%H%M%S)}"
LOG_DIR="$LOG_ROOT/$RUN_ID"
LOG_FILE="$LOG_DIR/diagnosis.log"
ARDUINO_LOG_FILE="$LOG_DIR/arduino_serial.log"

mkdir -p "$LOG_DIR"
ln -sfn "$LOG_DIR" "$LOG_ROOT/latest"

exec > >(tee -a "$LOG_FILE") 2>&1

echo "===== PX4 ROVER CONTROLLER MAPPING DIAGNOSIS START $(date --iso-8601=seconds) ====="
echo "cwd=$PWD"
echo "log_dir=$LOG_DIR"
echo "arduino_log=$ARDUINO_LOG_FILE"
echo

missing=()
if [ "$CONFIRM_PARAM_BACKUP" != "true" ]; then
  missing+=("CONFIRM_PARAM_BACKUP=true")
fi
if [ "$CONFIRM_WHEELS_LIFTED" != "true" ]; then
  missing+=("CONFIRM_WHEELS_LIFTED=true")
fi
if [ "$CONFIRM_MOTOR_POWER_DISCONNECTED" != "true" ]; then
  missing+=("CONFIRM_MOTOR_POWER_DISCONNECTED=true")
fi
if [ "$CONFIRM_QGC_DISARM_READY" != "true" ]; then
  missing+=("CONFIRM_QGC_DISARM_READY=true")
fi
if [ "$CONFIRM_PHYSICAL_POWER_CUTOFF_READY" != "true" ]; then
  missing+=("CONFIRM_PHYSICAL_POWER_CUTOFF_READY=true")
fi
if [ "$AUTO_ARM" = "true" ] && [ "$CONFIRM_AUTO_ARM" != "true" ]; then
  missing+=("CONFIRM_AUTO_ARM=true")
fi

if [ "${#missing[@]}" -gt 0 ]; then
  echo "Refusing to run controller mapping diagnosis."
  echo "Required confirmations:"
  for item in "${missing[@]}"; do
    echo "  $item"
  done
  exit 2
fi

# shellcheck disable=SC1091
source "$REPO_DIR/scripts/env.sh"
mkdir -p "$REPO_DIR/results/ros_logs"
export ROS_LOG_DIR="${ROS_LOG_DIR:-$REPO_DIR/results/ros_logs}"

set_param_int() {
  local param="$1"
  local value="$2"
  echo
  echo "Setting $param=$value..."
  timeout 12s ros2 service call \
    "$PARAM_SET_SERVICE" \
    mavros_msgs/srv/ParamSetV2 \
    "{force_set: false, param_id: '$param', value: {type: 2, integer_value: $value}}"
}

request_disarm() {
  echo
  echo "Requesting DISARM..."
  timeout 6s ros2 service call \
    "$ARM_SERVICE" \
    mavros_msgs/srv/CommandBool \
    "{value: false}" || true
}

request_manual() {
  echo
  echo "Requesting MANUAL mode..."
  timeout 6s ros2 service call \
    "$SET_MODE_SERVICE" \
    mavros_msgs/srv/SetMode \
    "{base_mode: 0, custom_mode: 'MANUAL'}" || true
}

print_state_and_io() {
  echo
  echo "===== MAVROS state ====="
  timeout 8s ros2 topic echo --once "$STATE_TOPIC" || true
  echo
  echo "===== MAVROS rc/in ====="
  timeout 8s ros2 topic echo --once "$MAVROS_NS/rc/in" || true
  echo
  echo "===== MAVROS rc/out ====="
  timeout 8s ros2 topic echo --once "$MAVROS_NS/rc/out" || true
}

print_params() {
  echo
  echo "===== Relevant params ====="
  timeout 12s ros2 service call \
    "$PARAM_PULL_SERVICE" \
    mavros_msgs/srv/ParamPull \
    "{force_pull: true}" || true
  for param in \
    RC_MAP_PITCH \
    RC_MAP_THROTTLE \
    RC_MAP_YAW \
    PWM_MAIN_FUNC1 \
    PWM_MAIN_FUNC2 \
    PWM_MAIN_FUNC6 \
    PWM_MAIN_FUNC7 \
    PWM_MAIN_DIS1 \
    PWM_MAIN_DIS2 \
    PWM_MAIN_FAIL1 \
    PWM_MAIN_FAIL2
  do
    echo "--- $param ---"
    timeout 8s ros2 param get "$PARAM_NODE" "$param" || true
  done
}

cleanup() {
  local status=$?
  set +e
  if [ "${RC_WATCH_PID:-}" ]; then
    wait "$RC_WATCH_PID" || true
  fi
  if [ "${ARDUINO_WATCH_PID:-}" ]; then
    wait "$ARDUINO_WATCH_PID" || true
  fi
  request_disarm
  request_manual
  if [ "$RESTORE_ON_EXIT" = "true" ]; then
    echo
    echo "Restoring RC passthrough baseline and RC_MAP_THROTTLE=3..."
    CONFIRM_PARAM_BACKUP=true \
    CONFIRM_WHEELS_LIFTED=true \
    CONFIRM_VEHICLE_DISARMED=true \
    CONFIRM_QGC_DISARM_READY=true \
    CONFIRM_PHYSICAL_POWER_CUTOFF_READY=true \
      "$REPO_DIR/scripts/set_px4_rover_output_mapping.sh" restore-baseline || true
    set_param_int RC_MAP_THROTTLE 3 || true
    set_param_int RC_MAP_YAW 4 || true
  fi
  print_state_and_io
  print_params
  echo
  echo "Diagnosis logs:"
  echo "  $LOG_FILE"
  echo "  $ARDUINO_LOG_FILE"
  echo "  $LOG_ROOT/latest"
  exit "$status"
}
trap cleanup EXIT

echo "Safety setup:"
echo "  wheels lifted: true"
echo "  motor power disconnected: true"
echo "  auto_arm: $AUTO_ARM"
echo "  restore_on_exit: $RESTORE_ON_EXIT"
echo "  watch_sec: $WATCH_SEC"
echo "  arm_hold_sec: $ARM_HOLD_SEC"
echo
echo "Purpose:"
echo "  Verify whether PX4 rover-controller outputs 101/201 become neutral"
echo "  when RC_MAP_THROTTLE is changed from channel 3 to channel 2."

request_disarm
request_manual
print_state_and_io
print_params

echo
echo "Applying test input/output mapping:"
echo "  RC_MAP_THROTTLE=2"
echo "  RC_MAP_YAW=4"
echo "  PWM_MAIN_FUNC1=101"
echo "  PWM_MAIN_FUNC2=201"
set_param_int RC_MAP_THROTTLE 2
set_param_int RC_MAP_YAW 4

CONFIRM_PARAM_BACKUP=true \
CONFIRM_WHEELS_LIFTED=true \
CONFIRM_VEHICLE_DISARMED=true \
CONFIRM_QGC_DISARM_READY=true \
CONFIRM_PHYSICAL_POWER_CUTOFF_READY=true \
  "$REPO_DIR/scripts/set_px4_rover_output_mapping.sh" apply

print_state_and_io
print_params

echo
echo "Starting watchers..."
DURATION_SEC="$WATCH_SEC" \
CHANNELS_TO_PRINT=8 \
RC_WATCH_LOG_ROOT="$LOG_DIR/rc_watch" \
  "$REPO_DIR/scripts/watch_mavros_rc_io.sh" &
RC_WATCH_PID=$!

python3 "$REPO_DIR/scripts/arduino_serial_watch.py" \
  --port "$ARDUINO_PORT" \
  --duration "$WATCH_SEC" \
  > "$ARDUINO_LOG_FILE" 2>&1 &
ARDUINO_WATCH_PID=$!

sleep 4

if [ "$AUTO_ARM" = "true" ]; then
  echo
  echo "Auto ARM for ${ARM_HOLD_SEC}s with motor power disconnected..."
  timeout 6s ros2 service call \
    "$ARM_SERVICE" \
    mavros_msgs/srv/CommandBool \
    "{value: true}" || true
  sleep "$ARM_HOLD_SEC"
  print_state_and_io
  request_disarm
else
  echo
  echo "AUTO_ARM=false. Manually arm for a few seconds, then disarm."
  echo "Watchers continue for ${WATCH_SEC}s."
fi

wait "$RC_WATCH_PID" || true
unset RC_WATCH_PID
wait "$ARDUINO_WATCH_PID" || true
unset ARDUINO_WATCH_PID

echo
echo "Arduino serial tail:"
tail -40 "$ARDUINO_LOG_FILE" || true

echo
echo "Controller mapping diagnosis completed."
