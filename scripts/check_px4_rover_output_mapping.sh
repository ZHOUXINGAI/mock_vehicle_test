#!/usr/bin/env bash

set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

MAVROS_NS="${MAVROS_NS:-/mavros}"
MAVROS_NS="${MAVROS_NS%/}"
PARAM_NODE="${PARAM_NODE:-$MAVROS_NS/param}"
PARAM_PULL_SERVICE="${PARAM_PULL_SERVICE:-$MAVROS_NS/param/pull}"
PARAM_TIMEOUT_SEC="${PARAM_TIMEOUT_SEC:-12s}"
DISCOVERY_TIMEOUT_SEC="${DISCOVERY_TIMEOUT_SEC:-4s}"

OUTPUT_MAPPING_LOG_DISABLE="${OUTPUT_MAPPING_LOG_DISABLE:-false}"
if [ "$OUTPUT_MAPPING_LOG_DISABLE" != "true" ] \
  && [ "${OUTPUT_MAPPING_LOG_ACTIVE:-false}" != "true" ]; then
  OUTPUT_MAPPING_LOG_ROOT="${OUTPUT_MAPPING_LOG_ROOT:-$REPO_DIR/results/output_mapping}"
  OUTPUT_MAPPING_RUN_ID="${OUTPUT_MAPPING_RUN_ID:-$(date +%Y%m%d_%H%M%S)}"
  OUTPUT_MAPPING_LOG_DIR="$OUTPUT_MAPPING_LOG_ROOT/$OUTPUT_MAPPING_RUN_ID"
  OUTPUT_MAPPING_LOG_FILE="$OUTPUT_MAPPING_LOG_DIR/output_mapping.log"
  mkdir -p "$OUTPUT_MAPPING_LOG_DIR"
  ln -sfn "$OUTPUT_MAPPING_LOG_DIR" "$OUTPUT_MAPPING_LOG_ROOT/latest"
  export OUTPUT_MAPPING_LOG_ACTIVE=true
  export OUTPUT_MAPPING_LOG_DIR
  export OUTPUT_MAPPING_LOG_FILE

  echo "Saving PX4 output-mapping check log:"
  echo "  directory: $OUTPUT_MAPPING_LOG_DIR"
  echo "  file:      $OUTPUT_MAPPING_LOG_FILE"
  echo "  latest:    $OUTPUT_MAPPING_LOG_ROOT/latest"
  echo

  exec > >(tee -a "$OUTPUT_MAPPING_LOG_FILE") 2>&1
  echo "===== PX4 OUTPUT MAPPING CHECK LOG START $(date --iso-8601=seconds) ====="
  echo "cwd=$PWD"
  echo "command=$0 $*"
  echo
fi

# shellcheck disable=SC1091
source "$REPO_DIR/scripts/env.sh"

echo "Checking PX4 rover output mapping through MAVROS:"
echo "  namespace: $MAVROS_NS"
echo "  param node: $PARAM_NODE"
echo "  pull svc:   $PARAM_PULL_SERVICE"
echo "  ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-unset}"
echo

print_mavros_graph() {
  echo "Discovered MAVROS nodes:"
  timeout "$DISCOVERY_TIMEOUT_SEC" ros2 node list --spin-time 2 \
    | grep -E '(^|/)mavros|/uas' || true
  echo
  echo "Discovered MAVROS services:"
  timeout "$DISCOVERY_TIMEOUT_SEC" ros2 service list -t --spin-time 2 \
    | grep -E '(^|/)mavros|/uas' || true
  echo
}

read_param() {
  local param="$1"
  if timeout "$PARAM_TIMEOUT_SEC" ros2 param get "$PARAM_NODE" "$param"; then
    return 0
  fi

  echo "ros2 param get failed for $param; trying get_parameters service..." >&2
  timeout "$PARAM_TIMEOUT_SEC" ros2 service call \
    "$PARAM_NODE/get_parameters" \
    rcl_interfaces/srv/GetParameters \
    "{names: ['$param']}" || {
      echo "Failed to read $param within $PARAM_TIMEOUT_SEC" >&2
      return 1
    }
}

print_param() {
  local param="$1"
  echo "===== $param ====="
  read_param "$param" || true
  echo
}

print_param_series() {
  local prefix="$1"
  local first="${2:-1}"
  local last="${3:-8}"
  local index

  for index in $(seq "$first" "$last"); do
    print_param "${prefix}${index}"
  done
}

service_list="$(
  timeout "$DISCOVERY_TIMEOUT_SEC" ros2 service list --spin-time 2 || true
)"

if ! printf '%s\n' "$service_list" | grep -Fxq "$PARAM_PULL_SERVICE"; then
  echo "MAVROS parameter pull service was not discovered." >&2
  echo "Make sure ./scripts/run_mavros_px4_usb_to_qgc_logged.sh is still running." >&2
  echo "Also make sure both terminals use the same ROS_DOMAIN_ID." >&2
  echo
  print_mavros_graph
  exit 1
fi

echo "Pulling PX4 parameters into MAVROS cache..."
if ! timeout "$PARAM_TIMEOUT_SEC" ros2 service call \
  "$PARAM_PULL_SERVICE" \
  mavros_msgs/srv/ParamPull \
  "{force_pull: false}"; then
  echo "Failed to pull parameters from PX4 within $PARAM_TIMEOUT_SEC" >&2
  echo
  print_mavros_graph
  exit 1
fi
echo

cat <<'EOF'
Context from the latest indoor fake-vision Offboard motion log:
  - PX4 accepted OFFBOARD and the setpoint sequence ran.
  - /mavros/rc/out channel 1 and 2 stayed near neutral.
  - /mavros/rc/out channel 6 and 7 changed during Offboard steps.

That points to an output-function mapping mismatch: the rover Offboard actuator
output is landing on PWM outputs 6/7, while the current motor bridge is wired
to the Pixhawk outputs that show up as rc/out 1/2 in MANUAL.

This script only reads parameters.
EOF
echo

echo "### Vehicle / RC Input"
for param in \
  MAV_TYPE \
  MAV_FWDEXTSP \
  COM_RC_IN_MODE \
  COM_OF_LOSS_T \
  COM_OBL_RC_ACT \
  COM_RCL_EXCEPT \
  RC_MAP_ROLL \
  RC_MAP_PITCH \
  RC_MAP_THROTTLE \
  RC_MAP_YAW \
  RC_MAP_ARM_SW \
  RC_MAP_KILL_SW \
  RC_MAP_MODE_SW \
  RC_MAP_FLTMODE
do
  print_param "$param"
done

echo "### PWM MAIN Output Functions"
print_param_series PWM_MAIN_FUNC 1 8

echo "### PWM MAIN Output Ranges"
print_param_series PWM_MAIN_MIN 1 8
print_param_series PWM_MAIN_MAX 1 8
print_param_series PWM_MAIN_DIS 1 8
print_param_series PWM_MAIN_FAIL 1 8
print_param PWM_MAIN_REV

echo "### PWM AUX Output Functions"
print_param_series PWM_AUX_FUNC 1 8

echo "### Control Allocation / Rover Parameters"
for param in \
  CA_AIRFRAME \
  CA_METHOD \
  CA_R_REV \
  CA_ROTOR_COUNT \
  CA_R0_SLEW \
  CA_R1_SLEW \
  CA_SV_CS_COUNT \
  CA_SV0_SLEW \
  CA_SV1_SLEW \
  GND_SPEED_MAX \
  GND_SPEED_TRIM \
  GND_SPEED_THR_SC \
  GND_THR_MAX \
  GND_THR_MIN \
  GND_SP_CTRL_MODE \
  GND_WHEEL_BASE
do
  print_param "$param"
done

cat <<'EOF'
Next interpretation:
  1. Compare PWM_MAIN_FUNC1/2 with PWM_MAIN_FUNC6/7.
  2. The function values currently on the channels that moved in Offboard
     should be assigned to the physical outputs feeding the motor bridge.
  3. Do not change this blindly while armed. Keep wheels lifted, backup params,
     and verify MANUAL arm/disarm before any mapping change.
EOF
