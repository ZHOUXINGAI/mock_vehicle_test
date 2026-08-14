#!/usr/bin/env bash

set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

cat <<'EOF'
PX4 v1.17 differential-rover MAVROS Offboard open-loop U-turn test.

Mission:
  1. drive forward for a short timed segment
  2. command a timed right-turn arc
  3. drive forward for a second short timed segment
  4. stop, disarm, request MANUAL

This is an open-loop ground test. It does not guarantee an exact 180-degree
heading change; tune TURN_RIGHT_SEC after observing the path.
EOF

MAVROS_NS="${MAVROS_NS:-/mavros}"
MAVROS_NS="${MAVROS_NS%/}"

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

FORWARD_SEC="${FORWARD_SEC:-3.0}"
SECOND_FORWARD_SEC="${SECOND_FORWARD_SEC:-3.0}"
TURN_RIGHT_SEC="${TURN_RIGHT_SEC:-5.0}"
LINEAR_SPEED_MPS="${LINEAR_SPEED_MPS:-0.08}"
TURN_LATERAL_SPEED_MPS="${TURN_LATERAL_SPEED_MPS:-0.10}"
TURN_YAW_RATE_RADPS="${TURN_YAW_RATE_RADPS:-0.25}"
MAX_LINEAR_SPEED_MPS="${MAX_LINEAR_SPEED_MPS:-0.20}"
MAX_YAW_RATE_RADPS="${MAX_YAW_RATE_RADPS:-0.50}"
RC_WATCH_DURATION_SEC="${RC_WATCH_DURATION_SEC:-24}"
RC_WATCH_PRINT_PERIOD_SEC="${RC_WATCH_PRINT_PERIOD_SEC:-0.5}"
RC_WATCH_CHANGE_THRESHOLD_US="${RC_WATCH_CHANGE_THRESHOLD_US:-5}"

LOG_ROOT="${DIFF_OFFBOARD_OPEN_LOOP_LOG_ROOT:-$REPO_DIR/results/differential_offboard_open_loop}"
RUN_ID="${DIFF_OFFBOARD_OPEN_LOOP_RUN_ID:-open_loop_right_uturn_$(date +%Y%m%d_%H%M%S)}"
LOG_DIR="$LOG_ROOT/$RUN_ID"
RC_WATCH_LOG_FILE="$LOG_DIR/rc_watch.log"

missing=()
for item in \
  CONFIRM_GROUND_AREA_CLEAR \
  CONFIRM_LOW_SPEED_GROUND_TEST \
  CONFIRM_VEHICLE_DISARMED \
  CONFIRM_RC_READY \
  CONFIRM_PARAM_BACKUP \
  CONFIRM_QGC_DISARM_READY \
  CONFIRM_PHYSICAL_POWER_CUTOFF_READY \
  CONFIRM_REAL_LOCAL_POSITION \
  CONFIRM_CURRENT_DIFF_MAPPING \
  CONFIRM_WHEELS_INSTALLED \
  CONFIRM_FRESH_USER_START
do
  if [ "${!item}" != "true" ]; then
    missing+=("$item=true")
  fi
done

if [ "${#missing[@]}" -gt 0 ]; then
  {
    echo "Refusing to start open-loop U-turn test."
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

# shellcheck disable=SC1091
source "$REPO_DIR/scripts/env.sh"

mkdir -p "$LOG_DIR"
ln -sfn "$LOG_DIR" "$LOG_ROOT/latest"

RC_WATCH_PID=""

cleanup() {
  local status=$?
  trap - EXIT INT TERM
  set +e
  echo
  echo "Open-loop U-turn wrapper cleanup..."
  timeout 5s ros2 service call \
    "$MAVROS_NS/cmd/arming" \
    mavros_msgs/srv/CommandBool \
    "{value: false}" >/tmp/mock_vehicle_open_loop_uturn_disarm.log 2>&1 || true
  timeout 5s ros2 service call \
    "$MAVROS_NS/set_mode" \
    mavros_msgs/srv/SetMode \
    "{base_mode: 0, custom_mode: 'MANUAL'}" >/tmp/mock_vehicle_open_loop_uturn_manual.log 2>&1 || true
  if [ -n "$RC_WATCH_PID" ]; then
    wait "$RC_WATCH_PID" || true
  fi
  echo "Final MAVROS state snapshot:"
  timeout 5s ros2 topic echo --once "$MAVROS_NS/state" || true
  echo "Final MAVROS rc/out snapshot:"
  timeout 5s ros2 topic echo --once "$MAVROS_NS/rc/out" || true
  exit "$status"
}
trap cleanup EXIT INT TERM

echo
echo "Checking MAVROS state before open-loop U-turn..."
state_snapshot="$(timeout 6s ros2 topic echo --once "$MAVROS_NS/state" || true)"
printf '%s\n' "$state_snapshot"
state_armed="$(printf '%s\n' "$state_snapshot" | awk '$1 == "armed:" {print $2; exit}')"
state_manual_input="$(printf '%s\n' "$state_snapshot" | awk '$1 == "manual_input:" {print $2; exit}')"
state_mode="$(printf '%s\n' "$state_snapshot" | sed -n 's/^mode: //p' | head -n 1)"
case "$state_armed" in False|false) ;; *) echo "Refusing to start while armed." >&2; exit 2 ;; esac
case "$state_manual_input" in True|true) ;; *) echo "Refusing unless manual_input=true." >&2; exit 2 ;; esac
case "$state_mode" in MANUAL|manual) ;; *) echo "Refusing unless mode=MANUAL." >&2; exit 2 ;; esac

echo
echo "Checking neutral rc/out before open-loop U-turn..."
timeout 6s ros2 topic echo --once "$MAVROS_NS/rc/out"

cat <<EOF

Effective open-loop U-turn settings:
  FORWARD_SEC=$FORWARD_SEC
  TURN_RIGHT_SEC=$TURN_RIGHT_SEC
  SECOND_FORWARD_SEC=$SECOND_FORWARD_SEC
  LINEAR_SPEED_MPS=$LINEAR_SPEED_MPS
  TURN_LATERAL_SPEED_MPS=$TURN_LATERAL_SPEED_MPS
  TURN_YAW_RATE_RADPS=$TURN_YAW_RATE_RADPS
  MAX_LINEAR_SPEED_MPS=$MAX_LINEAR_SPEED_MPS
  MAX_YAW_RATE_RADPS=$MAX_YAW_RATE_RADPS
  log_dir=$LOG_DIR
EOF

echo
echo "Starting MAVROS RC I/O watch:"
echo "  file: $RC_WATCH_LOG_FILE"
(
  python3 "$REPO_DIR/src/mavros_rc_io_watch.py" \
    --ros-args \
    -p mavros_namespace:="$MAVROS_NS" \
    -p duration_sec:="$RC_WATCH_DURATION_SEC" \
    -p channels_to_print:=8 \
    -p print_period_sec:="$RC_WATCH_PRINT_PERIOD_SEC" \
    -p change_threshold_us:="$RC_WATCH_CHANGE_THRESHOLD_US"
) >"$RC_WATCH_LOG_FILE" 2>&1 &
RC_WATCH_PID=$!
sleep 0.5

OFFBOARD_LOG_ROOT="$LOG_ROOT" \
OFFBOARD_RUN_ID="$RUN_ID" \
TEST_SURFACE=ground \
CONFIRM_GROUND_AREA_CLEAR=true \
CONFIRM_LOW_SPEED_GROUND_TEST=true \
CONFIRM_RC_READY=true \
CONFIRM_PARAM_BACKUP=true \
CONFIRM_QGC_DISARM_READY=true \
CONFIRM_PHYSICAL_POWER_CUTOFF_READY=true \
CONFIRM_FRESH_USER_START=true \
PRESTART_FIRST_MOTION_SETPOINT=true \
SETPOINT_VELOCITY_MAV_FRAME=BODY_NED \
MODE_CHANGE_ON_START=true \
ARM_ON_START=true \
DISARM_ON_FINISH=true \
AUTO_RESTORE_OUTPUT_MAPPING=false \
INITIAL_STOP_SEC=0.0 \
STOP_SEC="${STOP_SEC:-0.2}" \
FORWARD_SEC="$FORWARD_SEC" \
BACKWARD_SEC=0 \
TURN_SEC=0 \
TURN_LEFT_SEC=0 \
TURN_RIGHT_SEC="$TURN_RIGHT_SEC" \
SECOND_FORWARD_SEC="$SECOND_FORWARD_SEC" \
FINAL_STOP_SEC=0.0 \
LINEAR_SPEED_MPS="$LINEAR_SPEED_MPS" \
LINEAR_DIRECTION_SIGN=1.0 \
TURN_LINEAR_SPEED_MPS=0.0 \
TURN_LATERAL_SPEED_MPS="$TURN_LATERAL_SPEED_MPS" \
TURN_YAW_RATE_RADPS="$TURN_YAW_RATE_RADPS" \
TURN_SIGN=-1.0 \
MAX_LINEAR_SPEED_MPS="$MAX_LINEAR_SPEED_MPS" \
MAX_YAW_RATE_RADPS="$MAX_YAW_RATE_RADPS" \
STOP_BURST_SEC="${STOP_BURST_SEC:-0.2}" \
REQUIRE_OFFBOARD_MODE=true \
REQUIRE_ARMED=true \
ABORT_ON_MODE_EXIT=true \
ABORT_ON_DISARM=true \
MAX_WAIT_FOR_READY_SEC="${MAX_WAIT_FOR_READY_SEC:-45}" \
  "$REPO_DIR/scripts/run_real_rover_mavros_offboard_smoke.sh"
