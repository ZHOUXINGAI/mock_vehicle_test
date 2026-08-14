#!/usr/bin/env bash

set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

cat <<'EOF'
PX4 v1.17 differential-rover MAVROS Offboard forward mapping test.

Mission:
  1. request OFFBOARD
  2. arm
  3. drive forward only for a short low-speed segment
  4. stop, disarm, request MANUAL

This script preserves the current differential-rover output mapping
PWM_MAIN_FUNC1=101 / PWM_MAIN_FUNC2=102. It does not restore the old RC
passthrough 405/403 baseline.
EOF

TEST_SURFACE="${TEST_SURFACE:-ground}"
CONFIRM_WHEELS_LIFTED="${CONFIRM_WHEELS_LIFTED:-false}"
CONFIRM_GROUND_AREA_CLEAR="${CONFIRM_GROUND_AREA_CLEAR:-false}"
CONFIRM_LOW_SPEED_GROUND_TEST="${CONFIRM_LOW_SPEED_GROUND_TEST:-false}"
CONFIRM_VEHICLE_DISARMED="${CONFIRM_VEHICLE_DISARMED:-false}"
CONFIRM_RC_READY="${CONFIRM_RC_READY:-false}"
CONFIRM_PARAM_BACKUP="${CONFIRM_PARAM_BACKUP:-false}"
CONFIRM_QGC_DISARM_READY="${CONFIRM_QGC_DISARM_READY:-false}"
CONFIRM_PHYSICAL_POWER_CUTOFF_READY="${CONFIRM_PHYSICAL_POWER_CUTOFF_READY:-false}"
CONFIRM_REAL_LOCAL_POSITION="${CONFIRM_REAL_LOCAL_POSITION:-false}"
CONFIRM_CURRENT_DIFF_MAPPING="${CONFIRM_CURRENT_DIFF_MAPPING:-false}"
CONFIRM_WHEELS_INSTALLED="${CONFIRM_WHEELS_INSTALLED:-false}"
CONFIRM_FRESH_USER_START="${CONFIRM_FRESH_USER_START:-false}"
ENABLE_RC_WATCH="${ENABLE_RC_WATCH:-true}"
RC_WATCH_DURATION_SEC="${RC_WATCH_DURATION_SEC:-14}"
RC_WATCH_CHANNELS_TO_PRINT="${RC_WATCH_CHANNELS_TO_PRINT:-8}"
RC_WATCH_PRINT_PERIOD_SEC="${RC_WATCH_PRINT_PERIOD_SEC:-0.5}"
RC_WATCH_CHANGE_THRESHOLD_US="${RC_WATCH_CHANGE_THRESHOLD_US:-5}"

missing=()
case "$TEST_SURFACE" in
  wheels_lifted)
    if [ "$CONFIRM_WHEELS_LIFTED" != "true" ]; then
      missing+=("CONFIRM_WHEELS_LIFTED=true")
    fi
    ;;
  ground)
    for item in CONFIRM_GROUND_AREA_CLEAR CONFIRM_LOW_SPEED_GROUND_TEST CONFIRM_WHEELS_INSTALLED; do
      if [ "${!item}" != "true" ]; then
        missing+=("$item=true")
      fi
    done
    ;;
  *)
    echo "TEST_SURFACE must be 'wheels_lifted' or 'ground'." >&2
    exit 2
    ;;
esac

for item in \
  CONFIRM_VEHICLE_DISARMED \
  CONFIRM_RC_READY \
  CONFIRM_PARAM_BACKUP \
  CONFIRM_QGC_DISARM_READY \
  CONFIRM_PHYSICAL_POWER_CUTOFF_READY \
  CONFIRM_REAL_LOCAL_POSITION \
  CONFIRM_CURRENT_DIFF_MAPPING \
  CONFIRM_FRESH_USER_START
do
  if [ "${!item}" != "true" ]; then
    missing+=("$item=true")
  fi
done

if [ "${#missing[@]}" -gt 0 ]; then
  {
    echo "Refusing to start differential Offboard forward mapping test."
    echo "Required confirmations:"
    for item in "${missing[@]}"; do
      echo "  $item"
    done
    echo
    echo "CONFIRM_FRESH_USER_START means the user gave a fresh start command for this exact run"
    echo "after checking HDMI/display/USB/power cables, field clearance, RC kill, QGC disarm,"
    echo "physical cutoff, PX4 safe state, and neutral outputs. Do not reuse an old confirmation."
  } >&2
  exit 2
fi

export OFFBOARD_LOG_ROOT="${DIFF_OFFBOARD_FORWARD_LOG_ROOT:-$REPO_DIR/results/differential_offboard_forward_mapping}"
export OFFBOARD_RUN_ID="${DIFF_OFFBOARD_FORWARD_RUN_ID:-$(date +%Y%m%d_%H%M%S)}"
export OFFBOARD_LOG_DIR="$OFFBOARD_LOG_ROOT/$OFFBOARD_RUN_ID"
mkdir -p "$OFFBOARD_LOG_DIR"
ln -sfn "$OFFBOARD_LOG_DIR" "$OFFBOARD_LOG_ROOT/latest"

RC_WATCH_PID=""
if [ "$ENABLE_RC_WATCH" = "true" ]; then
  RC_WATCH_LOG_FILE="$OFFBOARD_LOG_DIR/rc_watch.log"
  echo "Starting MAVROS RC I/O watch:"
  echo "  file: $RC_WATCH_LOG_FILE"
  (
    # shellcheck disable=SC1091
    source "$REPO_DIR/scripts/env.sh"
    python3 "$REPO_DIR/src/mavros_rc_io_watch.py" \
      --ros-args \
      -p mavros_namespace:="${MAVROS_NS:-/mavros}" \
      -p duration_sec:="$RC_WATCH_DURATION_SEC" \
      -p channels_to_print:="$RC_WATCH_CHANNELS_TO_PRINT" \
      -p print_period_sec:="$RC_WATCH_PRINT_PERIOD_SEC" \
      -p change_threshold_us:="$RC_WATCH_CHANGE_THRESHOLD_US"
  ) >"$RC_WATCH_LOG_FILE" 2>&1 &
  RC_WATCH_PID=$!
  sleep 0.5
fi

cleanup_forward_mapping() {
  local status=$?
  trap - EXIT INT TERM
  if [ -n "$RC_WATCH_PID" ]; then
    wait "$RC_WATCH_PID" || true
  fi
  exit "$status"
}
trap cleanup_forward_mapping EXIT INT TERM

TEST_SURFACE="$TEST_SURFACE" \
CONFIRM_WHEELS_LIFTED="$CONFIRM_WHEELS_LIFTED" \
CONFIRM_GROUND_AREA_CLEAR=true \
CONFIRM_LOW_SPEED_GROUND_TEST=true \
CONFIRM_RC_READY=true \
CONFIRM_PARAM_BACKUP=true \
CONFIRM_QGC_DISARM_READY=true \
CONFIRM_PHYSICAL_POWER_CUTOFF_READY=true \
CONFIRM_FRESH_USER_START=true \
SETPOINT_VELOCITY_MAV_FRAME="${SETPOINT_VELOCITY_MAV_FRAME:-BODY_NED}" \
PRESTART_FIRST_MOTION_SETPOINT="${PRESTART_FIRST_MOTION_SETPOINT:-true}" \
MODE_CHANGE_ON_START="${MODE_CHANGE_ON_START:-true}" \
ARM_ON_START="${ARM_ON_START:-true}" \
DISARM_ON_FINISH="${DISARM_ON_FINISH:-true}" \
AUTO_RESTORE_OUTPUT_MAPPING=false \
INITIAL_STOP_SEC="${INITIAL_STOP_SEC:-0.0}" \
STOP_SEC="${STOP_SEC:-0.2}" \
FORWARD_SEC="${FORWARD_SEC:-3.0}" \
BACKWARD_SEC=0 \
TURN_SEC=0 \
TURN_LEFT_SEC=0 \
TURN_RIGHT_SEC=0 \
FINAL_STOP_SEC="${FINAL_STOP_SEC:-0.0}" \
LINEAR_SPEED_MPS="${LINEAR_SPEED_MPS:-0.10}" \
LINEAR_DIRECTION_SIGN="${LINEAR_DIRECTION_SIGN:-1.0}" \
MAX_LINEAR_SPEED_MPS="${MAX_LINEAR_SPEED_MPS:-0.20}" \
STOP_BURST_SEC="${STOP_BURST_SEC:-0.2}" \
REQUIRE_OFFBOARD_MODE=true \
REQUIRE_ARMED=true \
ABORT_ON_MODE_EXIT=true \
ABORT_ON_DISARM=true \
MAX_WAIT_FOR_READY_SEC="${MAX_WAIT_FOR_READY_SEC:-45}" \
  "$REPO_DIR/scripts/run_real_rover_mavros_offboard_smoke.sh"
