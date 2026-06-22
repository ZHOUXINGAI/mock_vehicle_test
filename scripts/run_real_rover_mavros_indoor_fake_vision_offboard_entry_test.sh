#!/usr/bin/env bash

set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

FAKE_VISION_OFFBOARD_LOG_DISABLE="${FAKE_VISION_OFFBOARD_LOG_DISABLE:-false}"
if [ "$FAKE_VISION_OFFBOARD_LOG_DISABLE" != "true" ] \
  && [ "${FAKE_VISION_OFFBOARD_LOG_ACTIVE:-false}" != "true" ]; then
  FAKE_VISION_OFFBOARD_LOG_ROOT="${FAKE_VISION_OFFBOARD_LOG_ROOT:-$REPO_DIR/results/fake_vision_offboard}"
  FAKE_VISION_OFFBOARD_RUN_ID="${FAKE_VISION_OFFBOARD_RUN_ID:-$(date +%Y%m%d_%H%M%S)}"
  FAKE_VISION_OFFBOARD_LOG_DIR="$FAKE_VISION_OFFBOARD_LOG_ROOT/$FAKE_VISION_OFFBOARD_RUN_ID"
  FAKE_VISION_OFFBOARD_LOG_FILE="$FAKE_VISION_OFFBOARD_LOG_DIR/fake_vision_offboard.log"
  mkdir -p "$FAKE_VISION_OFFBOARD_LOG_DIR"
  ln -sfn "$FAKE_VISION_OFFBOARD_LOG_DIR" "$FAKE_VISION_OFFBOARD_LOG_ROOT/latest"
  export FAKE_VISION_OFFBOARD_LOG_ACTIVE=true
  export FAKE_VISION_OFFBOARD_LOG_DIR
  export FAKE_VISION_OFFBOARD_LOG_FILE

  echo "Saving indoor fake-vision Offboard entry-test log:"
  echo "  directory: $FAKE_VISION_OFFBOARD_LOG_DIR"
  echo "  file:      $FAKE_VISION_OFFBOARD_LOG_FILE"
  echo "  latest:    $FAKE_VISION_OFFBOARD_LOG_ROOT/latest"
  echo

  exec > >(tee -a "$FAKE_VISION_OFFBOARD_LOG_FILE") 2>&1
  echo "===== INDOOR FAKE-VISION OFFBOARD ENTRY TEST LOG START $(date --iso-8601=seconds) ====="
  echo "cwd=$PWD"
  echo "command=$0 $*"
  echo
fi

cat <<'EOF'
Indoor fake-vision MAVROS Offboard manual-arm entry test.

Default behavior:
  - starts fixed external-vision pose/odometry input
  - warms up the EKF external-vision stream
  - publishes only zero-velocity Offboard setpoints
  - waits for MANUAL arm from RC/QGC before requesting OFFBOARD
  - does not arm from the script
  - does not run forward/back/turn motion
  - attempts MAVROS disarm and stops fake vision on exit

Use only with wheels lifted.
EOF

MAVROS_NS="${MAVROS_NS:-/mavros}"
MAVROS_NS="${MAVROS_NS%/}"
STATE_TOPIC="${STATE_TOPIC:-$MAVROS_NS/state}"
PARAM_NODE="${PARAM_NODE:-$MAVROS_NS/param}"
PARAM_PULL_SERVICE="${PARAM_PULL_SERVICE:-$MAVROS_NS/param/pull}"
ARMING_SERVICE="${ARMING_SERVICE:-$MAVROS_NS/cmd/arming}"
SET_MODE_SERVICE="${SET_MODE_SERVICE:-$MAVROS_NS/set_mode}"
STATE_TIMEOUT_SEC="${STATE_TIMEOUT_SEC:-6s}"
PARAM_TIMEOUT_SEC="${PARAM_TIMEOUT_SEC:-12s}"
FAKE_VISION_WARMUP_SEC="${FAKE_VISION_WARMUP_SEC:-15}"
FAKE_VISION_DURATION_SEC="${FAKE_VISION_DURATION_SEC:-120}"
AUTO_RESTORE_OUTPUT_MAPPING="${AUTO_RESTORE_OUTPUT_MAPPING:-true}"

CONFIRM_WHEELS_LIFTED="${CONFIRM_WHEELS_LIFTED:-false}"
CONFIRM_VEHICLE_DISARMED="${CONFIRM_VEHICLE_DISARMED:-false}"
CONFIRM_RC_READY="${CONFIRM_RC_READY:-false}"
CONFIRM_PARAM_BACKUP="${CONFIRM_PARAM_BACKUP:-false}"
CONFIRM_QGC_DISARM_READY="${CONFIRM_QGC_DISARM_READY:-false}"
CONFIRM_PHYSICAL_POWER_CUTOFF_READY="${CONFIRM_PHYSICAL_POWER_CUTOFF_READY:-false}"
CONFIRM_FAKE_LOCAL_POSITION_ONLY="${CONFIRM_FAKE_LOCAL_POSITION_ONLY:-false}"

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
if [ "${#missing_confirmations[@]}" -gt 0 ]; then
  {
    echo "Refusing to start indoor fake-vision Offboard entry test."
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

cleanup() {
  local status=$?
  trap - EXIT INT TERM
  echo
  echo "Indoor fake-vision Offboard cleanup..."
  if [ -n "$fake_vision_pid" ]; then
    echo "Stopping fake external-vision publisher process group pgid=$fake_vision_pid"
    kill -TERM -- "-$fake_vision_pid" >/dev/null 2>&1 || \
      kill "$fake_vision_pid" >/dev/null 2>&1 || true
    sleep 0.5
    kill -KILL -- "-$fake_vision_pid" >/dev/null 2>&1 || true
    wait "$fake_vision_pid" >/dev/null 2>&1 || true
  fi
  echo "Requesting MAVROS disarm as cleanup..."
  timeout 5s ros2 service call \
    "$ARMING_SERVICE" \
    mavros_msgs/srv/CommandBool \
    "{value: false}" || true
  echo "Requesting MANUAL mode as cleanup..."
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
  exit "$status"
}
trap cleanup EXIT INT TERM

if ! ros2 topic list --spin-time 2 | grep -Fxq "$STATE_TOPIC"; then
  echo "MAVROS state topic was not discovered: $STATE_TOPIC" >&2
  echo "Start MAVROS first: ./scripts/run_mavros_px4_usb_to_qgc_logged.sh" >&2
  exit 1
fi

echo
echo "Checking MAVROS state before fake-vision Offboard entry..."
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

echo
echo "Checking COM_RC_IN_MODE and EKF2_EV_CTRL..."
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

cat <<EOF

Effective indoor fake-vision Offboard settings:
  MAVROS_NS=$MAVROS_NS
  FAKE_VISION_WARMUP_SEC=$FAKE_VISION_WARMUP_SEC
  FAKE_VISION_DURATION_SEC=$FAKE_VISION_DURATION_SEC
  action=zero-velocity Offboard manual-arm entry only
  MAX_WAIT_FOR_READY_SEC=${MAX_WAIT_FOR_READY_SEC:-90}
EOF

echo
echo "Starting fake external-vision publisher..."
setsid env \
  FAKE_EV_LOG_ACTIVE=true \
  FAKE_EV_LOG_DIR="$FAKE_VISION_OFFBOARD_LOG_DIR" \
  FAKE_EV_LOG_FILE="$FAKE_VISION_OFFBOARD_LOG_FILE" \
  MAVROS_NS="$MAVROS_NS" \
  DURATION_SEC="$FAKE_VISION_DURATION_SEC" \
  CONFIRM_WHEELS_LIFTED=true \
  CONFIRM_VEHICLE_DISARMED=true \
  CONFIRM_RC_READY=true \
  CONFIRM_PARAM_BACKUP=true \
  CONFIRM_FAKE_LOCAL_POSITION_ONLY=true \
  "$REPO_DIR/scripts/run_mavros_fake_external_vision.sh" &
fake_vision_pid=$!

echo "Fake external-vision process group pgid=$fake_vision_pid"
echo "Warming up fake external vision for ${FAKE_VISION_WARMUP_SEC}s..."
sleep "$FAKE_VISION_WARMUP_SEC"

echo
cat <<'EOF'
Running zero-velocity Offboard manual-arm entry test...

Operator action:
  1. Keep the RC mode switch in MANUAL.
  2. Arm from RC or QGC while QGC still shows MANUAL.
  3. After MAVROS sees armed=true, this script requests OFFBOARD.
  4. Use RC disarm/kill, QGC disarm, physical power cutoff, or Ctrl+C to abort.
EOF

OFFBOARD_LOG_ACTIVE=true \
OFFBOARD_LOG_DIR="$FAKE_VISION_OFFBOARD_LOG_DIR" \
OFFBOARD_LOG_FILE="$FAKE_VISION_OFFBOARD_LOG_FILE" \
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
MAX_WAIT_FOR_READY_SEC="${MAX_WAIT_FOR_READY_SEC:-90}" \
COMMAND_RATE_HZ="${COMMAND_RATE_HZ:-20}" \
PUBLISH_UNSTAMPED_CMD_VEL="${PUBLISH_UNSTAMPED_CMD_VEL:-true}" \
WARMUP_SEC="${WARMUP_SEC:-2.0}" \
STOP_SEC="${STOP_SEC:-1.0}" \
FORWARD_SEC=0.0 \
BACKWARD_SEC=0.0 \
TURN_SEC=0.0 \
FINAL_STOP_SEC="${FINAL_STOP_SEC:-2.0}" \
LINEAR_SPEED_MPS=0.0 \
TURN_YAW_RATE_RADPS=0.0 \
MAX_LINEAR_SPEED_MPS=0.01 \
MAX_YAW_RATE_RADPS=0.01 \
CONFIRM_WHEELS_LIFTED=true \
CONFIRM_RC_READY=true \
CONFIRM_PARAM_BACKUP=true \
CONFIRM_QGC_DISARM_READY=true \
CONFIRM_PHYSICAL_POWER_CUTOFF_READY=true \
AUTO_RESTORE_OUTPUT_MAPPING="$AUTO_RESTORE_OUTPUT_MAPPING" \
"$REPO_DIR/scripts/run_real_rover_mavros_offboard_smoke.sh"
