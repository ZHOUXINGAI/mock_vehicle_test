#!/usr/bin/env bash

set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

ACTION="${1:-${PX4_ROVER_OUTPUT_MAPPING_ACTION:-}}"
MAVROS_NS="${MAVROS_NS:-/mavros}"
MAVROS_NS="${MAVROS_NS%/}"
PARAM_NODE="${PARAM_NODE:-$MAVROS_NS/param}"
PARAM_PULL_SERVICE="${PARAM_PULL_SERVICE:-$MAVROS_NS/param/pull}"
PARAM_SET_SERVICE="${PARAM_SET_SERVICE:-$MAVROS_NS/param/set}"
STATE_TOPIC="${STATE_TOPIC:-$MAVROS_NS/state}"
PARAM_TIMEOUT_SEC="${PARAM_TIMEOUT_SEC:-12s}"
STATE_TIMEOUT_SEC="${STATE_TIMEOUT_SEC:-5s}"
DISCOVERY_TIMEOUT_SEC="${DISCOVERY_TIMEOUT_SEC:-4s}"
FORCE_PARAM_PULL_AFTER_CHANGE="${FORCE_PARAM_PULL_AFTER_CHANGE:-false}"

CONFIRM_PARAM_BACKUP="${CONFIRM_PARAM_BACKUP:-false}"
TEST_SURFACE="${TEST_SURFACE:-wheels_lifted}"
CONFIRM_WHEELS_LIFTED="${CONFIRM_WHEELS_LIFTED:-false}"
CONFIRM_GROUND_AREA_CLEAR="${CONFIRM_GROUND_AREA_CLEAR:-false}"
CONFIRM_LOW_SPEED_GROUND_TEST="${CONFIRM_LOW_SPEED_GROUND_TEST:-false}"
CONFIRM_VEHICLE_DISARMED="${CONFIRM_VEHICLE_DISARMED:-false}"
CONFIRM_QGC_DISARM_READY="${CONFIRM_QGC_DISARM_READY:-false}"
CONFIRM_PHYSICAL_POWER_CUTOFF_READY="${CONFIRM_PHYSICAL_POWER_CUTOFF_READY:-false}"

OUTPUT_REMAP_LOG_DISABLE="${OUTPUT_REMAP_LOG_DISABLE:-false}"
if [ "$OUTPUT_REMAP_LOG_DISABLE" != "true" ] \
  && [ "${OUTPUT_REMAP_LOG_ACTIVE:-false}" != "true" ]; then
  OUTPUT_REMAP_LOG_ROOT="${OUTPUT_REMAP_LOG_ROOT:-$REPO_DIR/results/param_changes}"
  OUTPUT_REMAP_RUN_ID="${OUTPUT_REMAP_RUN_ID:-$(date +%Y%m%d_%H%M%S)}"
  OUTPUT_REMAP_LOG_DIR="$OUTPUT_REMAP_LOG_ROOT/$OUTPUT_REMAP_RUN_ID"
  OUTPUT_REMAP_LOG_FILE="$OUTPUT_REMAP_LOG_DIR/px4_rover_output_mapping.log"
  mkdir -p "$OUTPUT_REMAP_LOG_DIR"
  ln -sfn "$OUTPUT_REMAP_LOG_DIR" "$OUTPUT_REMAP_LOG_ROOT/latest"
  export OUTPUT_REMAP_LOG_ACTIVE=true
  export OUTPUT_REMAP_LOG_DIR
  export OUTPUT_REMAP_LOG_FILE

  echo "Saving PX4 rover output-mapping change log:"
  echo "  directory: $OUTPUT_REMAP_LOG_DIR"
  echo "  file:      $OUTPUT_REMAP_LOG_FILE"
  echo "  latest:    $OUTPUT_REMAP_LOG_ROOT/latest"
  echo

  exec > >(tee -a "$OUTPUT_REMAP_LOG_FILE") 2>&1
  echo "===== PX4 ROVER OUTPUT MAPPING CHANGE LOG START $(date --iso-8601=seconds) ====="
  echo "cwd=$PWD"
  echo "command=$0 $*"
  echo
fi

usage() {
  cat <<'EOF'
Usage:
  CONFIRM_PARAM_BACKUP=true \
  TEST_SURFACE=wheels_lifted \
  CONFIRM_WHEELS_LIFTED=true \
  CONFIRM_VEHICLE_DISARMED=true \
  CONFIRM_QGC_DISARM_READY=true \
  CONFIRM_PHYSICAL_POWER_CUTOFF_READY=true \
  ./scripts/set_px4_rover_output_mapping.sh apply

  CONFIRM_PARAM_BACKUP=true \
  TEST_SURFACE=wheels_lifted \
  CONFIRM_WHEELS_LIFTED=true \
  CONFIRM_VEHICLE_DISARMED=true \
  CONFIRM_QGC_DISARM_READY=true \
  CONFIRM_PHYSICAL_POWER_CUTOFF_READY=true \
  ./scripts/set_px4_rover_output_mapping.sh apply-limited

  CONFIRM_PARAM_BACKUP=true \
  TEST_SURFACE=wheels_lifted \
  CONFIRM_WHEELS_LIFTED=true \
  CONFIRM_VEHICLE_DISARMED=true \
  CONFIRM_QGC_DISARM_READY=true \
  CONFIRM_PHYSICAL_POWER_CUTOFF_READY=true \
  ./scripts/set_px4_rover_output_mapping.sh apply-differential-limited

  CONFIRM_PARAM_BACKUP=true \
  CONFIRM_WHEELS_LIFTED=true \
  CONFIRM_VEHICLE_DISARMED=true \
  CONFIRM_QGC_DISARM_READY=true \
  CONFIRM_PHYSICAL_POWER_CUTOFF_READY=true \
  ./scripts/set_px4_rover_output_mapping.sh restore-baseline

Actions:
  apply
    Route PX4 rover controller outputs to the physical bridge wiring:
      PWM_MAIN_FUNC1=101  Motor 1 / reversible throttle
      PWM_MAIN_FUNC2=201  Servo 1 / steering
      PWM_MAIN_FUNC6=0
      PWM_MAIN_FUNC7=0
      PWM_MAIN_DIS1=1500
      PWM_MAIN_DIS2=1500
      PWM_MAIN_FAIL1=1500
      PWM_MAIN_FAIL2=1500
      CA_R_REV=1

  apply-limited
    Same output functions as apply, but with narrow MAIN1/MAIN2 PWM limits for
    wheels-lifted controller diagnostics:
      PWM_MAIN_MIN1=1400
      PWM_MAIN_MAX1=1600
      PWM_MAIN_MIN2=1450
      PWM_MAIN_MAX2=1550

  apply-differential
    Route PX4 differential-rover controller outputs to two reversible motor
    channels. This requires the Arduino side to interpret the two Pixhawk PWM
    inputs as left/right wheel commands, not throttle/steering:
      PWM_MAIN_FUNC1=101  Motor 1
      PWM_MAIN_FUNC2=102  Motor 2
      PWM_MAIN_FUNC6=0
      PWM_MAIN_FUNC7=0
      CA_R_REV=3

  apply-differential-limited
    Same output functions as apply-differential, but with narrow MAIN1/MAIN2
    PWM limits for first wheels-lifted tests:
      PWM_MAIN_MIN1=1400
      PWM_MAIN_MAX1=1600
      PWM_MAIN_MIN2=1400
      PWM_MAIN_MAX2=1600

  restore-baseline
    Restore the previously observed RC-passthrough mapping:
      PWM_MAIN_FUNC1=405  RC Yaw
      PWM_MAIN_FUNC2=403  RC Pitch
      PWM_MAIN_FUNC6=0    Disabled
      PWM_MAIN_FUNC7=0    Disabled
      PWM_MAIN_DIS1=1500
      PWM_MAIN_DIS2=1500
      PWM_MAIN_DIS6=1500
      PWM_MAIN_DIS7=1500
      PWM_MAIN_FAIL1=1500
      PWM_MAIN_FAIL2=1500
      PWM_MAIN_FAIL6=1500
      PWM_MAIN_FAIL7=1500
      PWM_MAIN_MIN1=1000
      PWM_MAIN_MAX1=2000
      PWM_MAIN_MIN2=1000
      PWM_MAIN_MAX2=2000
      CA_R_REV=3
EOF
}

case "$ACTION" in
  apply|apply-limited|apply-differential|apply-differential-limited|restore-baseline|restore) ;;
  *)
    usage >&2
    exit 2
    ;;
esac
if [ "$ACTION" = "restore" ]; then
  ACTION="restore-baseline"
fi

missing=()
if [ "$CONFIRM_PARAM_BACKUP" != "true" ]; then
  missing+=("CONFIRM_PARAM_BACKUP=true")
fi
case "$TEST_SURFACE" in
  wheels_lifted)
    if [ "$CONFIRM_WHEELS_LIFTED" != "true" ]; then
      missing+=("CONFIRM_WHEELS_LIFTED=true")
    fi
    ;;
  ground)
    if [ "$CONFIRM_GROUND_AREA_CLEAR" != "true" ]; then
      missing+=("CONFIRM_GROUND_AREA_CLEAR=true")
    fi
    if [ "$CONFIRM_LOW_SPEED_GROUND_TEST" != "true" ]; then
      missing+=("CONFIRM_LOW_SPEED_GROUND_TEST=true")
    fi
    ;;
  *)
    echo "TEST_SURFACE must be 'wheels_lifted' or 'ground'." >&2
    exit 2
    ;;
esac
if [ "$CONFIRM_VEHICLE_DISARMED" != "true" ]; then
  missing+=("CONFIRM_VEHICLE_DISARMED=true")
fi
if [ "$CONFIRM_QGC_DISARM_READY" != "true" ]; then
  missing+=("CONFIRM_QGC_DISARM_READY=true")
fi
if [ "$CONFIRM_PHYSICAL_POWER_CUTOFF_READY" != "true" ]; then
  missing+=("CONFIRM_PHYSICAL_POWER_CUTOFF_READY=true")
fi
if [ "${#missing[@]}" -gt 0 ]; then
  {
    echo "Refusing to change PX4 rover output mapping."
    echo "Required confirmations:"
    for item in "${missing[@]}"; do
      echo "  $item"
    done
  } >&2
  exit 2
fi

cat <<EOF
PX4 rover output mapping through MAVROS.

Action:
  $ACTION

MAVROS:
  namespace:   $MAVROS_NS
  param node:  $PARAM_NODE
  pull svc:    $PARAM_PULL_SERVICE
  set svc:     $PARAM_SET_SERVICE
  state topic: $STATE_TOPIC
  ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-unset}
  test surface: $TEST_SURFACE

Safety note:
  This changes PX4 actuator output functions only. It does not bypass PX4.
  The vehicle must stay disarmed while changing mappings.
EOF

# shellcheck disable=SC1091
source "$REPO_DIR/scripts/env.sh"

service_list="$(timeout "$DISCOVERY_TIMEOUT_SEC" ros2 service list --spin-time 2 || true)"
for service in "$PARAM_PULL_SERVICE" "$PARAM_SET_SERVICE"; do
  if ! printf '%s\n' "$service_list" | grep -Fxq "$service"; then
    echo "Required MAVROS service was not discovered: $service" >&2
    echo "Make sure ./scripts/run_mavros_px4_usb_to_qgc_logged.sh is running." >&2
    exit 1
  fi
done

echo
echo "Checking MAVROS state before parameter change..."
state_snapshot="$(timeout "$STATE_TIMEOUT_SEC" ros2 topic echo --once "$STATE_TOPIC" || true)"
if [ -z "$state_snapshot" ]; then
  echo "Could not read $STATE_TOPIC; refusing to change output mapping." >&2
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
    cat >&2 <<'EOF'
Refusing to change output mapping while the vehicle is armed.

Do this first:
  1. Disarm in QGC or from the RC switch.
  2. Verify QGC shows disarmed.
  3. Re-run this script.
EOF
    exit 2
    ;;
esac

read_param() {
  local param="$1"
  timeout "$PARAM_TIMEOUT_SEC" ros2 param get "$PARAM_NODE" "$param" || true
}

print_param() {
  local param="$1"
  echo "===== $param ====="
  read_param "$param"
  echo
}

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

print_mapping_summary() {
  for param in \
    CA_AIRFRAME \
    CA_R_REV \
    PWM_MAIN_FUNC1 \
    PWM_MAIN_FUNC2 \
    PWM_MAIN_FUNC6 \
    PWM_MAIN_FUNC7 \
    PWM_MAIN_MIN1 \
    PWM_MAIN_MAX1 \
    PWM_MAIN_MIN2 \
    PWM_MAIN_MAX2 \
    PWM_MAIN_DIS1 \
    PWM_MAIN_DIS2 \
    PWM_MAIN_DIS6 \
    PWM_MAIN_DIS7 \
    PWM_MAIN_FAIL1 \
    PWM_MAIN_FAIL2 \
    PWM_MAIN_FAIL6 \
    PWM_MAIN_FAIL7
  do
    print_param "$param"
  done
}

echo
echo "Pulling PX4 parameters into MAVROS cache before change..."
timeout "$PARAM_TIMEOUT_SEC" ros2 service call \
  "$PARAM_PULL_SERVICE" \
  mavros_msgs/srv/ParamPull \
  "{force_pull: false}" || {
    echo "Failed to pull parameters from PX4." >&2
    exit 1
  }

echo
echo "Current relevant mapping before change:"
print_mapping_summary

if [ "$ACTION" = "apply" ] || [ "$ACTION" = "apply-limited" ]; then
  if [ "$ACTION" = "apply-limited" ]; then
    MAIN1_MIN_US="${MAIN1_MIN_US:-1400}"
    MAIN1_MAX_US="${MAIN1_MAX_US:-1600}"
    MAIN2_MIN_US="${MAIN2_MIN_US:-1450}"
    MAIN2_MAX_US="${MAIN2_MAX_US:-1550}"
    cat <<EOF
Applying PX4-controlled rover mapping with narrow PWM limits:
  MAIN1 -> Motor 1 / reversible throttle, limited to ${MAIN1_MIN_US}-${MAIN1_MAX_US} us
  MAIN2 -> Servo 1 / steering, limited to ${MAIN2_MIN_US}-${MAIN2_MAX_US} us

Use this only for wheels-lifted controller diagnostics. Steering is intentionally
kept near center so a PX4 steering-controller excursion cannot command a hard
turn through the Arduino mixer.
EOF
  else
    MAIN1_MIN_US="${MAIN1_MIN_US:-1000}"
    MAIN1_MAX_US="${MAIN1_MAX_US:-2000}"
    MAIN2_MIN_US="${MAIN2_MIN_US:-1000}"
    MAIN2_MAX_US="${MAIN2_MAX_US:-2000}"
    cat <<'EOF'
Applying PX4-controlled rover mapping:
  MAIN1 -> Motor 1 / reversible throttle
  MAIN2 -> Servo 1 / steering

After this, MANUAL should still work through PX4's rover controller, but it is
no longer raw RC passthrough. Verify MANUAL with wheels lifted before Offboard.
EOF
  fi
  set_param_int CA_R_REV 1
  set_param_int PWM_MAIN_FUNC1 101
  set_param_int PWM_MAIN_FUNC2 201
  set_param_int PWM_MAIN_FUNC6 0
  set_param_int PWM_MAIN_FUNC7 0
  set_param_int PWM_MAIN_MIN1 "$MAIN1_MIN_US"
  set_param_int PWM_MAIN_MAX1 "$MAIN1_MAX_US"
  set_param_int PWM_MAIN_MIN2 "$MAIN2_MIN_US"
  set_param_int PWM_MAIN_MAX2 "$MAIN2_MAX_US"
  set_param_int PWM_MAIN_DIS1 1500
  set_param_int PWM_MAIN_DIS2 1500
  set_param_int PWM_MAIN_FAIL1 1500
  set_param_int PWM_MAIN_FAIL2 1500
elif [ "$ACTION" = "apply-differential" ] || [ "$ACTION" = "apply-differential-limited" ]; then
  if [ "$ACTION" = "apply-differential-limited" ]; then
    MAIN1_MIN_US="${MAIN1_MIN_US:-1400}"
    MAIN1_MAX_US="${MAIN1_MAX_US:-1600}"
    MAIN2_MIN_US="${MAIN2_MIN_US:-1400}"
    MAIN2_MAX_US="${MAIN2_MAX_US:-1600}"
    cat <<EOF
Applying PX4 differential-rover mapping with narrow PWM limits:
  MAIN1 -> Motor 1, limited to ${MAIN1_MIN_US}-${MAIN1_MAX_US} us
  MAIN2 -> Motor 2, limited to ${MAIN2_MIN_US}-${MAIN2_MAX_US} us

This is for wheels-lifted testing with an Arduino differential PWM bridge.
EOF
  else
    MAIN1_MIN_US="${MAIN1_MIN_US:-1000}"
    MAIN1_MAX_US="${MAIN1_MAX_US:-2000}"
    MAIN2_MIN_US="${MAIN2_MIN_US:-1000}"
    MAIN2_MAX_US="${MAIN2_MAX_US:-2000}"
    cat <<'EOF'
Applying PX4 differential-rover mapping:
  MAIN1 -> Motor 1
  MAIN2 -> Motor 2

This requires the Arduino side to treat the two Pixhawk PWM inputs as
left/right wheel commands. Do not use this with the throttle/steering bridge.
EOF
  fi
  set_param_int CA_AIRFRAME 6
  set_param_int CA_R_REV 3
  set_param_int PWM_MAIN_FUNC1 101
  set_param_int PWM_MAIN_FUNC2 102
  set_param_int PWM_MAIN_FUNC6 0
  set_param_int PWM_MAIN_FUNC7 0
  set_param_int PWM_MAIN_MIN1 "$MAIN1_MIN_US"
  set_param_int PWM_MAIN_MAX1 "$MAIN1_MAX_US"
  set_param_int PWM_MAIN_MIN2 "$MAIN2_MIN_US"
  set_param_int PWM_MAIN_MAX2 "$MAIN2_MAX_US"
  set_param_int PWM_MAIN_DIS1 1500
  set_param_int PWM_MAIN_DIS2 1500
  set_param_int PWM_MAIN_DIS6 1500
  set_param_int PWM_MAIN_DIS7 1500
  set_param_int PWM_MAIN_FAIL1 1500
  set_param_int PWM_MAIN_FAIL2 1500
  set_param_int PWM_MAIN_FAIL6 1500
  set_param_int PWM_MAIN_FAIL7 1500
elif [ "$ACTION" = "restore-baseline" ]; then
  cat <<'EOF'
Restoring the observed RC-passthrough baseline:
  MAIN1 -> RC Yaw
  MAIN2 -> RC Pitch
  MAIN6 -> Disabled
  MAIN7 -> Disabled
EOF
  set_param_int CA_R_REV 3
  set_param_int PWM_MAIN_FUNC1 405
  set_param_int PWM_MAIN_FUNC2 403
  set_param_int PWM_MAIN_FUNC6 0
  set_param_int PWM_MAIN_FUNC7 0
  set_param_int PWM_MAIN_MIN1 1000
  set_param_int PWM_MAIN_MAX1 2000
  set_param_int PWM_MAIN_MIN2 1000
  set_param_int PWM_MAIN_MAX2 2000
  set_param_int PWM_MAIN_DIS1 1500
  set_param_int PWM_MAIN_DIS2 1500
  set_param_int PWM_MAIN_DIS6 1500
  set_param_int PWM_MAIN_DIS7 1500
  set_param_int PWM_MAIN_FAIL1 1500
  set_param_int PWM_MAIN_FAIL2 1500
  set_param_int PWM_MAIN_FAIL6 1500
  set_param_int PWM_MAIN_FAIL7 1500
fi

if [ "$FORCE_PARAM_PULL_AFTER_CHANGE" = "true" ]; then
  echo
  echo "Pulling PX4 parameters into MAVROS cache after change..."
  timeout "$PARAM_TIMEOUT_SEC" ros2 service call \
    "$PARAM_PULL_SERVICE" \
    mavros_msgs/srv/ParamPull \
    "{force_pull: true}" || {
      echo "Failed to refresh parameters after setting output mapping." >&2
      exit 1
    }
else
  echo
  echo "Skipping forced full parameter pull after change."
  echo "Set responses were successful; reading key values from MAVROS cache."
fi

echo
echo "Relevant mapping after change:"
print_mapping_summary

cat <<'EOF'
Next:
  1. Keep wheels lifted.
  2. Verify MANUAL arm/disarm and small stick movement first.
  3. If MANUAL behaves wrong, restore immediately:

       CONFIRM_PARAM_BACKUP=true \
       CONFIRM_WHEELS_LIFTED=true \
       CONFIRM_VEHICLE_DISARMED=true \
       CONFIRM_QGC_DISARM_READY=true \
       CONFIRM_PHYSICAL_POWER_CUTOFF_READY=true \
       ./scripts/set_px4_rover_output_mapping.sh restore-baseline

  4. Only after MANUAL is sane, run the fake-vision Offboard motion task.
EOF
