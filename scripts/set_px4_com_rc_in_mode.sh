#!/usr/bin/env bash

set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

MODE_VALUE="${1:-${COM_RC_IN_MODE_VALUE:-}}"
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
CONFIRM_WHEELS_LIFTED="${CONFIRM_WHEELS_LIFTED:-false}"
CONFIRM_PHYSICAL_POWER_CUTOFF_READY="${CONFIRM_PHYSICAL_POWER_CUTOFF_READY:-false}"
CONFIRM_QGC_DISARM_READY="${CONFIRM_QGC_DISARM_READY:-false}"
CONFIRM_VEHICLE_DISARMED="${CONFIRM_VEHICLE_DISARMED:-false}"

PARAM_CHANGE_LOG_DISABLE="${PARAM_CHANGE_LOG_DISABLE:-false}"
if [ "$PARAM_CHANGE_LOG_DISABLE" != "true" ] \
  && [ "${PARAM_CHANGE_LOG_ACTIVE:-false}" != "true" ]; then
  PARAM_CHANGE_LOG_ROOT="${PARAM_CHANGE_LOG_ROOT:-$REPO_DIR/results/param_changes}"
  PARAM_CHANGE_RUN_ID="${PARAM_CHANGE_RUN_ID:-$(date +%Y%m%d_%H%M%S)}"
  PARAM_CHANGE_LOG_DIR="$PARAM_CHANGE_LOG_ROOT/$PARAM_CHANGE_RUN_ID"
  PARAM_CHANGE_LOG_FILE="$PARAM_CHANGE_LOG_DIR/com_rc_in_mode.log"
  mkdir -p "$PARAM_CHANGE_LOG_DIR"
  ln -sfn "$PARAM_CHANGE_LOG_DIR" "$PARAM_CHANGE_LOG_ROOT/latest"
  export PARAM_CHANGE_LOG_ACTIVE=true
  export PARAM_CHANGE_LOG_DIR
  export PARAM_CHANGE_LOG_FILE

  echo "Saving COM_RC_IN_MODE change log:"
  echo "  directory: $PARAM_CHANGE_LOG_DIR"
  echo "  file:      $PARAM_CHANGE_LOG_FILE"
  echo "  latest:    $PARAM_CHANGE_LOG_ROOT/latest"
  echo

  exec > >(tee -a "$PARAM_CHANGE_LOG_FILE") 2>&1
  echo "===== COM_RC_IN_MODE CHANGE LOG START $(date --iso-8601=seconds) ====="
  echo "cwd=$PWD"
  echo "command=$0 $*"
  echo
fi

usage() {
  cat <<'EOF'
Usage:
  CONFIRM_PARAM_BACKUP=true ./scripts/set_px4_com_rc_in_mode.sh 3

  CONFIRM_PARAM_BACKUP=true \
  CONFIRM_WHEELS_LIFTED=true \
  CONFIRM_VEHICLE_DISARMED=true \
  CONFIRM_PHYSICAL_POWER_CUTOFF_READY=true \
  CONFIRM_QGC_DISARM_READY=true \
  ./scripts/set_px4_com_rc_in_mode.sh 1

Known values used in this project:
  1  Joystick/MAVLink manual-control input only. RC input checks/takeover may not work.
  3  RC or joystick, keep the first available source until reboot. This is the current baseline.
EOF
}

if [ -z "$MODE_VALUE" ]; then
  usage >&2
  exit 2
fi

case "$MODE_VALUE" in
  ''|*[!0-9]*)
    echo "COM_RC_IN_MODE value must be an integer." >&2
    exit 2
    ;;
esac

if [ "$CONFIRM_PARAM_BACKUP" != "true" ]; then
  echo "Refusing to change PX4 parameters without CONFIRM_PARAM_BACKUP=true." >&2
  exit 2
fi

if [ "$MODE_VALUE" = "1" ]; then
  missing=()
  if [ "$CONFIRM_WHEELS_LIFTED" != "true" ]; then
    missing+=("CONFIRM_WHEELS_LIFTED=true")
  fi
  if [ "$CONFIRM_VEHICLE_DISARMED" != "true" ]; then
    missing+=("CONFIRM_VEHICLE_DISARMED=true")
  fi
  if [ "$CONFIRM_PHYSICAL_POWER_CUTOFF_READY" != "true" ]; then
    missing+=("CONFIRM_PHYSICAL_POWER_CUTOFF_READY=true")
  fi
  if [ "$CONFIRM_QGC_DISARM_READY" != "true" ]; then
    missing+=("CONFIRM_QGC_DISARM_READY=true")
  fi
  if [ "${#missing[@]}" -gt 0 ]; then
    {
      echo "Refusing to set COM_RC_IN_MODE=1 until joystick-only safety is confirmed."
      echo "Required confirmations:"
      for item in "${missing[@]}"; do
        echo "  $item"
      done
    } >&2
    exit 2
  fi
fi

cat <<EOF
PX4 COM_RC_IN_MODE parameter change through MAVROS.

Target:
  COM_RC_IN_MODE=$MODE_VALUE
  namespace: $MAVROS_NS
  param node: $PARAM_NODE
  pull svc:   $PARAM_PULL_SERVICE
  set svc:    $PARAM_SET_SERVICE
  state topic: $STATE_TOPIC
  ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-unset}

Safety note:
  COM_RC_IN_MODE=1 is only for wheels-lifted joystick/manual-control diagnostics.
  In that mode RC stick/kill/disarm handling may not be accepted by PX4.
  Keep QGC/MAVROS disarm and a physical power cutoff ready.
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

if [ "$MODE_VALUE" = "1" ]; then
  echo
  echo "Checking MAVROS state before enabling joystick-only input..."
  state_snapshot="$(timeout "$STATE_TIMEOUT_SEC" ros2 topic echo --once "$STATE_TOPIC" || true)"
  if [ -z "$state_snapshot" ]; then
    echo "Could not read $STATE_TOPIC; refusing to set COM_RC_IN_MODE=1." >&2
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
Refusing to set COM_RC_IN_MODE=1 while the vehicle is armed.

Reason:
  COM_RC_IN_MODE=1 can remove RC as the accepted manual input source. If that is
  done while armed, PX4 can immediately trigger Manual control lost failsafe and
  switch to AUTO.LAND/Descend.

Do this first:
  1. Disarm in QGC or power-cycle safely.
  2. Restore baseline if needed:
       CONFIRM_PARAM_BACKUP=true ./scripts/set_px4_com_rc_in_mode.sh 3
  3. Only retry COM_RC_IN_MODE=1 while armed=false.
EOF
      exit 2
      ;;
  esac
fi

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
echo "Current COM_RC_IN_MODE before change:"
timeout "$PARAM_TIMEOUT_SEC" ros2 param get "$PARAM_NODE" COM_RC_IN_MODE || true

echo
echo "Setting COM_RC_IN_MODE=$MODE_VALUE..."
set_response="$(
  timeout "$PARAM_TIMEOUT_SEC" ros2 service call \
    "$PARAM_SET_SERVICE" \
    mavros_msgs/srv/ParamSetV2 \
    "{force_set: false, param_id: 'COM_RC_IN_MODE', value: {type: 2, integer_value: $MODE_VALUE}}"
)"
printf '%s\n' "$set_response"

case "$set_response" in
  *"success=True"*|*"success: true"*) ;;
  *)
    echo "MAVROS did not report parameter set success." >&2
    exit 1
    ;;
esac

if [ "$FORCE_PARAM_PULL_AFTER_CHANGE" = "true" ]; then
  echo
  echo "Pulling PX4 parameters into MAVROS cache after change..."
  timeout "$PARAM_TIMEOUT_SEC" ros2 service call \
    "$PARAM_PULL_SERVICE" \
    mavros_msgs/srv/ParamPull \
    "{force_pull: true}" || {
      echo "Failed to refresh parameters after setting COM_RC_IN_MODE." >&2
      exit 1
    }
else
  echo
  echo "Skipping forced full parameter pull after change."
  echo "Set response was successful; reading COM_RC_IN_MODE from MAVROS cache."
fi

echo
echo "Current COM_RC_IN_MODE after change:"
timeout "$PARAM_TIMEOUT_SEC" ros2 param get "$PARAM_NODE" COM_RC_IN_MODE

cat <<EOF

Next:
  If the value is 1, run the joystick-only manual-control test with wheels lifted.
  After the test, restore the baseline with:

    CONFIRM_PARAM_BACKUP=true ./scripts/set_px4_com_rc_in_mode.sh 3
EOF
