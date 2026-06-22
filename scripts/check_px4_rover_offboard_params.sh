#!/usr/bin/env bash

set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# shellcheck disable=SC1091
source "$REPO_DIR/scripts/env.sh"

MAVROS_NS="${MAVROS_NS:-/mavros}"
MAVROS_NS="${MAVROS_NS%/}"
PARAM_NODE="${PARAM_NODE:-$MAVROS_NS/param}"
PARAM_PULL_SERVICE="${PARAM_PULL_SERVICE:-$MAVROS_NS/param/pull}"
PARAM_TIMEOUT_SEC="${PARAM_TIMEOUT_SEC:-12s}"
DISCOVERY_TIMEOUT_SEC="${DISCOVERY_TIMEOUT_SEC:-4s}"

params=(
  MAV_TYPE
  MAV_FWDEXTSP
  COM_OF_LOSS_T
  COM_OBL_RC_ACT
  COM_RC_IN_MODE
  COM_RCL_EXCEPT
  RC_MAP_THROTTLE
  RC_MAP_YAW
  RC_MAP_ROLL
  RC_MAP_PITCH
)

echo "Checking PX4 rover Offboard-related parameters through MAVROS:"
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

service_list="$(
  timeout "$DISCOVERY_TIMEOUT_SEC" ros2 service list --spin-time 2 || true
)"

if ! printf '%s\n' "$service_list" | grep -Fxq "$PARAM_PULL_SERVICE"; then
  echo "MAVROS parameter pull service was not discovered." >&2
  echo "Make sure ./scripts/run_mavros_px4_usb_to_qgc_logged.sh is still running in another terminal." >&2
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

for param in "${params[@]}"; do
  echo "===== $param ====="
  if ! timeout "$PARAM_TIMEOUT_SEC" ros2 param get "$PARAM_NODE" "$param"; then
    echo "ros2 param get failed for $param; trying get_parameters service..." >&2
    timeout "$PARAM_TIMEOUT_SEC" ros2 service call \
      "$PARAM_NODE/get_parameters" \
      rcl_interfaces/srv/GetParameters \
      "{names: ['$param']}" || {
        echo "Failed to read $param within $PARAM_TIMEOUT_SEC" >&2
      }
  fi
  echo
done

cat <<'EOF'
Expected minimum for this MAVROS rover Offboard path:
  MAV_TYPE     should be 10 for Ground rover.
  MAV_FWDEXTSP should be 1 so PX4 forwards external MAVLink setpoints to rover controllers.

Manual/RC diagnostics:
  COM_RC_IN_MODE=3 means RC or joystick, keep the first available source until reboot.
  RC_MAP_THROTTLE and RC_MAP_YAW tell which RC input channels actually drive rover motion.

This script only reads parameters. Change PX4 parameters from QGC after saving a .params backup.
EOF
