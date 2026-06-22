#!/usr/bin/env bash

set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

FAKE_GPS_OFFBOARD_LOG_DISABLE="${FAKE_GPS_OFFBOARD_LOG_DISABLE:-false}"
if [ "$FAKE_GPS_OFFBOARD_LOG_DISABLE" != "true" ] \
  && [ "${FAKE_GPS_OFFBOARD_LOG_ACTIVE:-false}" != "true" ]; then
  FAKE_GPS_OFFBOARD_LOG_ROOT="${FAKE_GPS_OFFBOARD_LOG_ROOT:-$REPO_DIR/results/fake_gps_offboard}"
  FAKE_GPS_OFFBOARD_RUN_ID="${FAKE_GPS_OFFBOARD_RUN_ID:-$(date +%Y%m%d_%H%M%S)}"
  FAKE_GPS_OFFBOARD_LOG_DIR="$FAKE_GPS_OFFBOARD_LOG_ROOT/$FAKE_GPS_OFFBOARD_RUN_ID"
  FAKE_GPS_OFFBOARD_LOG_FILE="$FAKE_GPS_OFFBOARD_LOG_DIR/fake_gps_offboard.log"
  mkdir -p "$FAKE_GPS_OFFBOARD_LOG_DIR"
  ln -sfn "$FAKE_GPS_OFFBOARD_LOG_DIR" "$FAKE_GPS_OFFBOARD_LOG_ROOT/latest"
  export FAKE_GPS_OFFBOARD_LOG_ACTIVE=true
  export FAKE_GPS_OFFBOARD_LOG_DIR
  export FAKE_GPS_OFFBOARD_LOG_FILE

  echo "Saving indoor fake-GPS Offboard entry-test log:"
  echo "  directory: $FAKE_GPS_OFFBOARD_LOG_DIR"
  echo "  file:      $FAKE_GPS_OFFBOARD_LOG_FILE"
  echo "  latest:    $FAKE_GPS_OFFBOARD_LOG_ROOT/latest"
  echo

  exec > >(tee -a "$FAKE_GPS_OFFBOARD_LOG_FILE") 2>&1
  echo "===== INDOOR FAKE-GPS OFFBOARD ENTRY TEST LOG START $(date --iso-8601=seconds) ====="
  echo "cwd=$PWD"
  echo "command=$0 $*"
  echo
fi

cat <<'EOF'
Indoor fake-GPS MAVROS Offboard entry test.

Default behavior:
  - starts fixed fake GPS_INPUT
  - waits for PX4/MAVROS to publish position
  - runs only the zero-velocity Offboard auto-arm entry test
  - does not run forward/back/turn motion
  - attempts MAVROS disarm and stops fake GPS on exit

Use only with wheels lifted.
EOF

MAVROS_NS="${MAVROS_NS:-/mavros}"
MAVROS_NS="${MAVROS_NS%/}"
STATE_TOPIC="${STATE_TOPIC:-$MAVROS_NS/state}"
PARAM_NODE="${PARAM_NODE:-$MAVROS_NS/param}"
PARAM_PULL_SERVICE="${PARAM_PULL_SERVICE:-$MAVROS_NS/param/pull}"
ARMING_SERVICE="${ARMING_SERVICE:-$MAVROS_NS/cmd/arming}"
STATE_TIMEOUT_SEC="${STATE_TIMEOUT_SEC:-6s}"
PARAM_TIMEOUT_SEC="${PARAM_TIMEOUT_SEC:-12s}"
FAKE_GPS_WARMUP_SEC="${FAKE_GPS_WARMUP_SEC:-20}"
FAKE_GPS_DURATION_SEC="${FAKE_GPS_DURATION_SEC:-120}"

CONFIRM_WHEELS_LIFTED="${CONFIRM_WHEELS_LIFTED:-false}"
CONFIRM_VEHICLE_DISARMED="${CONFIRM_VEHICLE_DISARMED:-false}"
CONFIRM_RC_READY="${CONFIRM_RC_READY:-false}"
CONFIRM_PARAM_BACKUP="${CONFIRM_PARAM_BACKUP:-false}"
CONFIRM_QGC_DISARM_READY="${CONFIRM_QGC_DISARM_READY:-false}"
CONFIRM_PHYSICAL_POWER_CUTOFF_READY="${CONFIRM_PHYSICAL_POWER_CUTOFF_READY:-false}"
CONFIRM_FAKE_GPS_ONLY="${CONFIRM_FAKE_GPS_ONLY:-false}"

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
if [ "$CONFIRM_FAKE_GPS_ONLY" != "true" ]; then
  missing_confirmations+=("CONFIRM_FAKE_GPS_ONLY=true")
fi
if [ "${#missing_confirmations[@]}" -gt 0 ]; then
  {
    echo "Refusing to start indoor fake-GPS Offboard entry test."
    echo "Required confirmations:"
    for item in "${missing_confirmations[@]}"; do
      echo "  $item"
    done
  } >&2
  exit 2
fi

# shellcheck disable=SC1091
source "$REPO_DIR/scripts/env.sh"

fake_gps_pid=""

cleanup() {
  local status=$?
  trap - EXIT INT TERM
  echo
  echo "Indoor fake-GPS Offboard cleanup..."
  if [ -n "$fake_gps_pid" ] && kill -0 "$fake_gps_pid" >/dev/null 2>&1; then
    echo "Stopping fake GPS_INPUT publisher pid=$fake_gps_pid"
    kill "$fake_gps_pid" >/dev/null 2>&1 || true
    wait "$fake_gps_pid" >/dev/null 2>&1 || true
  fi
  echo "Requesting MAVROS disarm as cleanup..."
  timeout 5s ros2 service call \
    "$ARMING_SERVICE" \
    mavros_msgs/srv/CommandBool \
    "{value: false}" || true
  exit "$status"
}
trap cleanup EXIT INT TERM

if ! ros2 topic list --spin-time 2 | grep -Fxq "$STATE_TOPIC"; then
  echo "MAVROS state topic was not discovered: $STATE_TOPIC" >&2
  echo "Start MAVROS first: ./scripts/run_mavros_px4_usb_to_qgc_logged.sh" >&2
  exit 1
fi

echo
echo "Checking MAVROS state before fake-GPS Offboard entry..."
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
echo "Checking COM_RC_IN_MODE baseline..."
timeout "$PARAM_TIMEOUT_SEC" ros2 service call \
  "$PARAM_PULL_SERVICE" \
  mavros_msgs/srv/ParamPull \
  "{force_pull: false}" >/dev/null
current_rc_mode="$(timeout "$PARAM_TIMEOUT_SEC" ros2 param get "$PARAM_NODE" COM_RC_IN_MODE || true)"
printf '%s\n' "$current_rc_mode"
if ! printf '%s\n' "$current_rc_mode" | grep -Eq '(^|[^0-9])3([^0-9]|$)'; then
  echo "Refusing to run unless COM_RC_IN_MODE is 3." >&2
  echo "Restore first: CONFIRM_PARAM_BACKUP=true ./scripts/set_px4_com_rc_in_mode.sh 3" >&2
  exit 2
fi

cat <<EOF

Effective indoor fake-GPS Offboard settings:
  MAVROS_NS=$MAVROS_NS
  FAKE_GPS_WARMUP_SEC=$FAKE_GPS_WARMUP_SEC
  FAKE_GPS_DURATION_SEC=$FAKE_GPS_DURATION_SEC
  action=zero-velocity Offboard auto-arm entry only
EOF

echo
echo "Starting fake GPS_INPUT publisher..."
FAKE_GPS_LOG_ACTIVE=true \
FAKE_GPS_LOG_DIR="$FAKE_GPS_OFFBOARD_LOG_DIR" \
FAKE_GPS_LOG_FILE="$FAKE_GPS_OFFBOARD_LOG_FILE" \
MAVROS_NS="$MAVROS_NS" \
DURATION_SEC="$FAKE_GPS_DURATION_SEC" \
CONFIRM_WHEELS_LIFTED=true \
CONFIRM_VEHICLE_DISARMED=true \
CONFIRM_RC_READY=true \
CONFIRM_PARAM_BACKUP=true \
CONFIRM_FAKE_GPS_ONLY=true \
"$REPO_DIR/scripts/run_mavros_fake_gps_input.sh" &
fake_gps_pid=$!

echo "Fake GPS_INPUT pid=$fake_gps_pid"
echo "Warming up fake GPS for ${FAKE_GPS_WARMUP_SEC}s..."
sleep "$FAKE_GPS_WARMUP_SEC"

echo
echo "Running zero-velocity Offboard auto-arm entry test..."
OFFBOARD_LOG_ACTIVE=true \
OFFBOARD_LOG_DIR="$FAKE_GPS_OFFBOARD_LOG_DIR" \
OFFBOARD_LOG_FILE="$FAKE_GPS_OFFBOARD_LOG_FILE" \
MAX_WAIT_FOR_READY_SEC="${MAX_WAIT_FOR_READY_SEC:-45}" \
CONFIRM_WHEELS_LIFTED=true \
CONFIRM_RC_READY=true \
CONFIRM_PARAM_BACKUP=true \
"$REPO_DIR/scripts/run_real_rover_mavros_offboard_auto_arm_entry_test.sh"
