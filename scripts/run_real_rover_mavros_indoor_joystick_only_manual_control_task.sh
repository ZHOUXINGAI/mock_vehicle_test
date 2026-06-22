#!/usr/bin/env bash

set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

JOYSTICK_LOG_DISABLE="${JOYSTICK_LOG_DISABLE:-false}"
if [ "$JOYSTICK_LOG_DISABLE" != "true" ] \
  && [ "${JOYSTICK_LOG_ACTIVE:-false}" != "true" ]; then
  JOYSTICK_LOG_ROOT="${JOYSTICK_LOG_ROOT:-$REPO_DIR/results/joystick_only_manual_control}"
  JOYSTICK_RUN_ID="${JOYSTICK_RUN_ID:-$(date +%Y%m%d_%H%M%S)}"
  JOYSTICK_LOG_DIR="$JOYSTICK_LOG_ROOT/$JOYSTICK_RUN_ID"
  JOYSTICK_LOG_FILE="$JOYSTICK_LOG_DIR/joystick_only_manual_control.log"
  mkdir -p "$JOYSTICK_LOG_DIR"
  ln -sfn "$JOYSTICK_LOG_DIR" "$JOYSTICK_LOG_ROOT/latest"
  export JOYSTICK_LOG_ACTIVE=true
  export JOYSTICK_LOG_DIR
  export JOYSTICK_LOG_FILE

  echo "Saving joystick-only manual-control task log:"
  echo "  directory: $JOYSTICK_LOG_DIR"
  echo "  file:      $JOYSTICK_LOG_FILE"
  echo "  latest:    $JOYSTICK_LOG_ROOT/latest"
  echo

  exec > >(tee -a "$JOYSTICK_LOG_FILE") 2>&1
  echo "===== JOYSTICK-ONLY MANUAL CONTROL TASK LOG START $(date --iso-8601=seconds) ====="
  echo "cwd=$PWD"
  echo "command=$0 $*"
  echo
fi

cat <<'EOF'
Indoor MAVROS joystick-only MANUAL_CONTROL no-GPS task.

Default behavior:
  - requires COM_RC_IN_MODE=1 before it will run
  - does not request OFFBOARD
  - does not request ARM
  - expects MAVROS already connected to Pixhawk
  - expects the vehicle to be disarmed and in MANUAL before the script starts
  - publishes neutral MANUAL_CONTROL while waiting for QGC/MAVROS arm
  - sends MANUAL_CONTROL: forward 1s, stop, backward 1s, stop, left/right turns
  - aborts if mode leaves MANUAL or the vehicle disarms
  - restores COM_RC_IN_MODE=3 on exit by default

Safety:
  - wheels must stay lifted
  - RC stick/kill/disarm may not work while COM_RC_IN_MODE=1
  - keep QGC/MAVROS disarm and a physical power cutoff ready
  - restore COM_RC_IN_MODE=3 immediately after the test
EOF

TEST_SURFACE="${TEST_SURFACE:-wheels_lifted}"
ALLOWED_MODES="${ALLOWED_MODES:-MANUAL}"
REQUIRE_CONNECTED="true"
REQUIRE_ARMED="true"
ABORT_ON_MODE_EXIT="true"
ABORT_ON_DISARM="true"
MAX_WAIT_FOR_READY_SEC="${MAX_WAIT_FOR_READY_SEC:-90}"
COMMAND_RATE_HZ="${COMMAND_RATE_HZ:-20}"
WARMUP_SEC="${WARMUP_SEC:-2.0}"
STOP_SEC="${STOP_SEC:-1.0}"
FORWARD_SEC="${FORWARD_SEC:-1.0}"
BACKWARD_SEC="${BACKWARD_SEC:-1.0}"
TURN_SEC="${TURN_SEC:-0.5}"
FINAL_STOP_SEC="${FINAL_STOP_SEC:-2.0}"
FORWARD_AXIS="${FORWARD_AXIS:-z}"
TURN_AXIS="${TURN_AXIS:-r}"
FORWARD_VALUE_RAW="${FORWARD_VALUE_RAW:-250}"
TURN_VALUE_RAW="${TURN_VALUE_RAW:-250}"
FORWARD_SIGN="${FORWARD_SIGN:-1.0}"
TURN_SIGN="${TURN_SIGN:-1.0}"
NEUTRAL_Z_RAW="${NEUTRAL_Z_RAW:-0}"
MAX_ABS_XY_R_RAW="${MAX_ABS_XY_R_RAW:-300}"
MIN_Z_RAW="${MIN_Z_RAW:--300}"
MAX_Z_RAW="${MAX_Z_RAW:-1000}"
AUTO_RESTORE_COM_RC_IN_MODE="${AUTO_RESTORE_COM_RC_IN_MODE:-true}"

CONFIRM_WHEELS_LIFTED="${CONFIRM_WHEELS_LIFTED:-false}"
CONFIRM_PHYSICAL_POWER_CUTOFF_READY="${CONFIRM_PHYSICAL_POWER_CUTOFF_READY:-false}"
CONFIRM_QGC_DISARM_READY="${CONFIRM_QGC_DISARM_READY:-false}"
CONFIRM_PARAM_BACKUP="${CONFIRM_PARAM_BACKUP:-false}"

missing_confirmations=()
if [ "$TEST_SURFACE" != "wheels_lifted" ]; then
  echo "This joystick-only diagnostic is wheels-lifted only." >&2
  exit 2
fi
if [ "$CONFIRM_WHEELS_LIFTED" != "true" ]; then
  missing_confirmations+=("CONFIRM_WHEELS_LIFTED=true")
fi
if [ "$CONFIRM_PHYSICAL_POWER_CUTOFF_READY" != "true" ]; then
  missing_confirmations+=("CONFIRM_PHYSICAL_POWER_CUTOFF_READY=true")
fi
if [ "$CONFIRM_QGC_DISARM_READY" != "true" ]; then
  missing_confirmations+=("CONFIRM_QGC_DISARM_READY=true")
fi
if [ "$CONFIRM_PARAM_BACKUP" != "true" ]; then
  missing_confirmations+=("CONFIRM_PARAM_BACKUP=true")
fi

if [ "${#missing_confirmations[@]}" -gt 0 ]; then
  {
    echo "Refusing to start joystick-only manual-control test."
    echo "Required confirmations:"
    for item in "${missing_confirmations[@]}"; do
      echo "  $item"
    done
  } >&2
  exit 2
fi

MAVROS_NS="${MAVROS_NS:-/mavros}"
MAVROS_NS="${MAVROS_NS%/}"
PARAM_NODE="${PARAM_NODE:-$MAVROS_NS/param}"
PARAM_PULL_SERVICE="${PARAM_PULL_SERVICE:-$MAVROS_NS/param/pull}"
STATE_TOPIC="${STATE_TOPIC:-$MAVROS_NS/state}"
PARAM_TIMEOUT_SEC="${PARAM_TIMEOUT_SEC:-12s}"
STATE_TIMEOUT_SEC="${STATE_TIMEOUT_SEC:-5s}"
DISCOVERY_TIMEOUT_SEC="${DISCOVERY_TIMEOUT_SEC:-4s}"

# shellcheck disable=SC1091
source "$REPO_DIR/scripts/env.sh"

service_list="$(timeout "$DISCOVERY_TIMEOUT_SEC" ros2 service list --spin-time 2 || true)"
if ! printf '%s\n' "$service_list" | grep -Fxq "$PARAM_PULL_SERVICE"; then
  echo "MAVROS parameter pull service was not discovered: $PARAM_PULL_SERVICE" >&2
  echo "Make sure ./scripts/run_mavros_px4_usb_to_qgc_logged.sh is running." >&2
  exit 1
fi

echo
echo "Checking COM_RC_IN_MODE before joystick-only control..."
timeout "$PARAM_TIMEOUT_SEC" ros2 service call \
  "$PARAM_PULL_SERVICE" \
  mavros_msgs/srv/ParamPull \
  "{force_pull: false}" >/dev/null

current_mode="$(timeout "$PARAM_TIMEOUT_SEC" ros2 param get "$PARAM_NODE" COM_RC_IN_MODE || true)"
printf '%s\n' "$current_mode"
if ! printf '%s\n' "$current_mode" | grep -Eq '(^|[^0-9])1([^0-9]|$)'; then
  cat >&2 <<'EOF'
COM_RC_IN_MODE is not 1, so PX4 is still allowed to keep RC as the active input source.
Set joystick-only input first:

  CONFIRM_PARAM_BACKUP=true \
  CONFIRM_WHEELS_LIFTED=true \
  CONFIRM_VEHICLE_DISARMED=true \
  CONFIRM_PHYSICAL_POWER_CUTOFF_READY=true \
  CONFIRM_QGC_DISARM_READY=true \
  ./scripts/set_px4_com_rc_in_mode.sh 1
EOF
  exit 2
fi

echo
echo "Checking MAVROS state before joystick-only control..."
state_snapshot="$(timeout "$STATE_TIMEOUT_SEC" ros2 topic echo --once "$STATE_TOPIC" || true)"
if [ -z "$state_snapshot" ]; then
  echo "Could not read $STATE_TOPIC; refusing to start joystick-only control." >&2
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

case "${state_mode:-}" in
  MANUAL) ;;
  *)
    cat >&2 <<'EOF'
Refusing to start joystick-only control because PX4 is not in MANUAL.

If QGC shows AUTO.LAND/Descend or Manual control lost:
  1. Disarm or power-cycle safely.
  2. Restore COM_RC_IN_MODE=3:
       CONFIRM_PARAM_BACKUP=true ./scripts/set_px4_com_rc_in_mode.sh 3
  3. Reboot Pixhawk if the mode/input state does not recover.
EOF
    exit 2
    ;;
esac

case "$state_armed" in
  False|false) ;;
  *)
    cat >&2 <<'EOF'
Refusing to start joystick-only control while the vehicle is already armed.

Start sequence for this diagnostic:
  1. Vehicle disarmed.
  2. COM_RC_IN_MODE=1.
  3. Start this script; it publishes neutral MANUAL_CONTROL.
  4. Arm from QGC/MAVROS after the script is running.
EOF
    exit 2
    ;;
esac

cat <<EOF

Effective joystick-only manual-control settings:
  TEST_SURFACE=$TEST_SURFACE
  ALLOWED_MODES=$ALLOWED_MODES
  REQUIRE_ARMED=$REQUIRE_ARMED
  FORWARD_AXIS=$FORWARD_AXIS
  TURN_AXIS=$TURN_AXIS
  FORWARD_VALUE_RAW=$FORWARD_VALUE_RAW
  TURN_VALUE_RAW=$TURN_VALUE_RAW
  MAX_WAIT_FOR_READY_SEC=$MAX_WAIT_FOR_READY_SEC
  AUTO_RESTORE_COM_RC_IN_MODE=$AUTO_RESTORE_COM_RC_IN_MODE

Run sequence:
  1. Keep this script running.
  2. In QGC, keep mode MANUAL.
  3. Arm from QGC/MAVROS when ready.
  4. Watch /mavros/rc/out or the lifted wheels.
  5. Disarm immediately if anything is unexpected.
EOF

export MANUAL_CONTROL_LOG_ACTIVE=true
export MANUAL_CONTROL_LOG_DIR="$JOYSTICK_LOG_DIR"
export MANUAL_CONTROL_LOG_FILE="$JOYSTICK_LOG_FILE"

restore_com_rc_in_mode() {
  local status=$?
  trap - EXIT
  if [ "$AUTO_RESTORE_COM_RC_IN_MODE" = "true" ]; then
    echo
    echo "Restoring COM_RC_IN_MODE=3 after joystick-only test..."
    if ! PARAM_CHANGE_LOG_DISABLE=true \
      CONFIRM_PARAM_BACKUP=true \
      "$REPO_DIR/scripts/set_px4_com_rc_in_mode.sh" 3; then
      echo "WARNING: failed to restore COM_RC_IN_MODE=3 automatically." >&2
      echo "Run manually when MAVROS is available:" >&2
      echo "  CONFIRM_PARAM_BACKUP=true ./scripts/set_px4_com_rc_in_mode.sh 3" >&2
    fi
  else
    echo
    echo "AUTO_RESTORE_COM_RC_IN_MODE=false; COM_RC_IN_MODE was not restored by this script."
  fi
  exit "$status"
}
trap restore_com_rc_in_mode EXIT

env \
  TEST_SURFACE="$TEST_SURFACE" \
  ALLOWED_MODES="$ALLOWED_MODES" \
  REQUIRE_CONNECTED="$REQUIRE_CONNECTED" \
  REQUIRE_ARMED="$REQUIRE_ARMED" \
  ABORT_ON_MODE_EXIT="$ABORT_ON_MODE_EXIT" \
  ABORT_ON_DISARM="$ABORT_ON_DISARM" \
  MAX_WAIT_FOR_READY_SEC="$MAX_WAIT_FOR_READY_SEC" \
  COMMAND_RATE_HZ="$COMMAND_RATE_HZ" \
  WARMUP_SEC="$WARMUP_SEC" \
  STOP_SEC="$STOP_SEC" \
  FORWARD_SEC="$FORWARD_SEC" \
  BACKWARD_SEC="$BACKWARD_SEC" \
  TURN_SEC="$TURN_SEC" \
  FINAL_STOP_SEC="$FINAL_STOP_SEC" \
  FORWARD_AXIS="$FORWARD_AXIS" \
  TURN_AXIS="$TURN_AXIS" \
  FORWARD_VALUE_RAW="$FORWARD_VALUE_RAW" \
  TURN_VALUE_RAW="$TURN_VALUE_RAW" \
  FORWARD_SIGN="$FORWARD_SIGN" \
  TURN_SIGN="$TURN_SIGN" \
  NEUTRAL_Z_RAW="$NEUTRAL_Z_RAW" \
  MAX_ABS_XY_R_RAW="$MAX_ABS_XY_R_RAW" \
  MIN_Z_RAW="$MIN_Z_RAW" \
  MAX_Z_RAW="$MAX_Z_RAW" \
  CONFIRM_WHEELS_LIFTED=true \
  CONFIRM_RC_READY=true \
  CONFIRM_PARAM_BACKUP="$CONFIRM_PARAM_BACKUP" \
  "$REPO_DIR/scripts/run_real_rover_mavros_manual_control_smoke.sh"
