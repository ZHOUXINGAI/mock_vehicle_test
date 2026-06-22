#!/usr/bin/env bash

set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

ACTION="${1:-${PX4_ROVER_CONTROLLER_MAPPING_ACTION:-}}"
MAVROS_NS="${MAVROS_NS:-/mavros}"
MAVROS_NS="${MAVROS_NS%/}"
PARAM_NODE="${PARAM_NODE:-$MAVROS_NS/param}"
PARAM_PULL_SERVICE="${PARAM_PULL_SERVICE:-$MAVROS_NS/param/pull}"
PARAM_SET_SERVICE="${PARAM_SET_SERVICE:-$MAVROS_NS/param/set}"
STATE_TOPIC="${STATE_TOPIC:-$MAVROS_NS/state}"
PARAM_TIMEOUT_SEC="${PARAM_TIMEOUT_SEC:-12s}"
STATE_TIMEOUT_SEC="${STATE_TIMEOUT_SEC:-5s}"
DISCOVERY_TIMEOUT_SEC="${DISCOVERY_TIMEOUT_SEC:-4s}"
FORCE_PARAM_PULL_IN_PRINT="${FORCE_PARAM_PULL_IN_PRINT:-false}"

CONFIRM_PARAM_BACKUP="${CONFIRM_PARAM_BACKUP:-false}"
CONFIRM_WHEELS_LIFTED="${CONFIRM_WHEELS_LIFTED:-false}"
CONFIRM_VEHICLE_DISARMED="${CONFIRM_VEHICLE_DISARMED:-false}"
CONFIRM_QGC_DISARM_READY="${CONFIRM_QGC_DISARM_READY:-false}"
CONFIRM_PHYSICAL_POWER_CUTOFF_READY="${CONFIRM_PHYSICAL_POWER_CUTOFF_READY:-false}"
CONFIRM_CONTROLLER_MAPPING_TESTED="${CONFIRM_CONTROLLER_MAPPING_TESTED:-false}"

LOG_ROOT="${CONTROLLER_MAPPING_LOG_ROOT:-$REPO_DIR/results/controller_mapping}"
RUN_ID="${CONTROLLER_MAPPING_RUN_ID:-$(date +%Y%m%d_%H%M%S)}"
LOG_DIR="$LOG_ROOT/$RUN_ID"
LOG_FILE="$LOG_DIR/controller_mapping.log"

usage() {
  cat <<'EOF'
Usage:
  CONFIRM_PARAM_BACKUP=true \
  CONFIRM_WHEELS_LIFTED=true \
  CONFIRM_VEHICLE_DISARMED=true \
  CONFIRM_QGC_DISARM_READY=true \
  CONFIRM_PHYSICAL_POWER_CUTOFF_READY=true \
  CONFIRM_CONTROLLER_MAPPING_TESTED=true \
  ./scripts/set_px4_rover_controller_offboard_mapping.sh apply

  CONFIRM_PARAM_BACKUP=true \
  CONFIRM_WHEELS_LIFTED=true \
  CONFIRM_VEHICLE_DISARMED=true \
  CONFIRM_QGC_DISARM_READY=true \
  CONFIRM_PHYSICAL_POWER_CUTOFF_READY=true \
  ./scripts/set_px4_rover_controller_offboard_mapping.sh restore

Actions:
  apply
    Configure the rover for PX4 controller / Offboard output:
      RC_MAP_THROTTLE=2
      RC_MAP_YAW=4
      PWM_MAIN_FUNC1=101
      PWM_MAIN_FUNC2=201
      PWM_MAIN_FUNC6=0
      PWM_MAIN_FUNC7=0

  restore
    Restore the known-safe RC passthrough baseline:
      RC_MAP_PITCH=2
      RC_MAP_YAW=4
      PWM_MAIN_FUNC1=405
      PWM_MAIN_FUNC2=403
      PWM_MAIN_FUNC6=0
      PWM_MAIN_FUNC7=0
EOF
}

case "$ACTION" in
  apply|restore|restore-baseline) ;;
  *)
    usage >&2
    exit 2
    ;;
esac
if [ "$ACTION" = "restore-baseline" ]; then
  ACTION="restore"
fi

mkdir -p "$LOG_DIR"
ln -sfn "$LOG_DIR" "$LOG_ROOT/latest"

exec > >(tee -a "$LOG_FILE") 2>&1

echo "===== PX4 ROVER CONTROLLER/OFFBOARD MAPPING LOG START $(date --iso-8601=seconds) ====="
echo "cwd=$PWD"
echo "command=$0 $*"
echo "log_dir=$LOG_DIR"
echo

missing=()
if [ "$CONFIRM_PARAM_BACKUP" != "true" ]; then
  missing+=("CONFIRM_PARAM_BACKUP=true")
fi
if [ "$CONFIRM_WHEELS_LIFTED" != "true" ]; then
  missing+=("CONFIRM_WHEELS_LIFTED=true")
fi
if [ "$CONFIRM_VEHICLE_DISARMED" != "true" ]; then
  missing+=("CONFIRM_VEHICLE_DISARMED=true")
fi
if [ "$CONFIRM_QGC_DISARM_READY" != "true" ]; then
  missing+=("CONFIRM_QGC_DISARM_READY=true")
fi
if [ "$CONFIRM_PHYSICAL_POWER_CUTOFF_READY" != "true" ]; then
  missing+=("CONFIRM_PHYSICAL_POWER_CUTOFF_READY=true")
fi
if [ "$ACTION" = "apply" ] && [ "$CONFIRM_CONTROLLER_MAPPING_TESTED" != "true" ]; then
  missing+=("CONFIRM_CONTROLLER_MAPPING_TESTED=true")
fi
if [ "${#missing[@]}" -gt 0 ]; then
  echo "Refusing to change PX4 rover controller mapping."
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

service_list="$(timeout "$DISCOVERY_TIMEOUT_SEC" ros2 service list --spin-time 2 || true)"
for service in "$PARAM_PULL_SERVICE" "$PARAM_SET_SERVICE"; do
  if ! printf '%s\n' "$service_list" | grep -Fxq "$service"; then
    echo "Required MAVROS service was not discovered: $service" >&2
    echo "Make sure ./scripts/run_mavros_px4_usb_to_qgc_logged.sh is running." >&2
    exit 1
  fi
done

echo "Checking MAVROS state before parameter change..."
state_snapshot="$(timeout "$STATE_TIMEOUT_SEC" ros2 topic echo --once "$STATE_TOPIC" || true)"
if [ -z "$state_snapshot" ]; then
  echo "Could not read $STATE_TOPIC; refusing to change mapping." >&2
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
    echo "Refusing to change mapping while vehicle is armed." >&2
    exit 2
    ;;
esac

set_param_int() {
  local param="$1"
  local value="$2"
  echo
  echo "Setting $param=$value..."
  local response
  response="$(
    timeout "$PARAM_TIMEOUT_SEC" ros2 service call \
      "$PARAM_SET_SERVICE" \
      mavros_msgs/srv/ParamSetV2 \
      "{force_set: false, param_id: '$param', value: {type: 2, integer_value: $value}}"
  )"
  printf '%s\n' "$response"
  case "$response" in
    *"success=True"*|*"success: true"*) ;;
    *)
      echo "MAVROS did not report success while setting $param=$value." >&2
      exit 1
      ;;
  esac
}

print_params() {
  echo
  if [ "$FORCE_PARAM_PULL_IN_PRINT" = "true" ]; then
    echo "Pulling PX4 parameters into MAVROS cache..."
    timeout "$PARAM_TIMEOUT_SEC" ros2 service call \
      "$PARAM_PULL_SERVICE" \
      mavros_msgs/srv/ParamPull \
      "{force_pull: true}" || true
  else
    echo "Skipping forced full parameter pull while printing mapping."
  fi
  echo
  echo "Relevant mapping:"
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
    echo "===== $param ====="
    timeout "$PARAM_TIMEOUT_SEC" ros2 param get "$PARAM_NODE" "$param" || true
  done
}

print_params

if [ "$ACTION" = "apply" ]; then
  cat <<'EOF'

Applying PX4 rover controller / Offboard mapping validated by diagnosis:
  - RC channel 2 is the rover throttle input.
  - RC channel 4 is yaw/steering input.
  - MAIN1/MAIN2 are PX4 controller outputs, not raw RC passthrough.
EOF
  set_param_int RC_MAP_THROTTLE 2
  set_param_int RC_MAP_YAW 4
  CONFIRM_PARAM_BACKUP=true \
  CONFIRM_WHEELS_LIFTED=true \
  CONFIRM_VEHICLE_DISARMED=true \
  CONFIRM_QGC_DISARM_READY=true \
  CONFIRM_PHYSICAL_POWER_CUTOFF_READY=true \
    "$REPO_DIR/scripts/set_px4_rover_output_mapping.sh" apply
else
  cat <<'EOF'

Restoring known-safe RC passthrough baseline:
  - RC channel 2 is restored as PX4 pitch/manual forward-back source.
  - RC channel 4 is restored as PX4 yaw/manual steering source.
  - MAIN1/MAIN2 return to the observed raw RC Yaw/Pitch passthrough.
EOF
  CONFIRM_PARAM_BACKUP=true \
  CONFIRM_WHEELS_LIFTED=true \
  CONFIRM_VEHICLE_DISARMED=true \
  CONFIRM_QGC_DISARM_READY=true \
  CONFIRM_PHYSICAL_POWER_CUTOFF_READY=true \
    "$REPO_DIR/scripts/set_px4_rover_output_mapping.sh" restore-baseline
  set_param_int RC_MAP_PITCH 2
  set_param_int RC_MAP_YAW 4
fi

print_params

cat <<EOF

Done.
Latest log:
  $LOG_ROOT/latest

Next:
  apply:
    1. Keep wheels lifted.
    2. Reconnect motor power only if you are ready to test MANUAL arm.
    3. ARM in MANUAL and verify there is no self-motion.
    4. Test tiny stick movement.
    5. Only then run Offboard.

  restore:
    Verify MANUAL RC passthrough again before driving.
EOF
