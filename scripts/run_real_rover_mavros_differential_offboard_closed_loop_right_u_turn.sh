#!/usr/bin/env bash

set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

cat <<'EOF'
PX4 v1.17 differential-rover MAVROS Offboard closed-loop right U-turn test.

Mission:
  1. drive forward until /mavros/local_position/pose reaches the target distance
  2. command a right body-frame turn until local yaw changes by the target angle
  3. drive forward again until local position reaches the second target distance
  4. stop, disarm, request MANUAL

This is a sensor-feedback test. Distance and yaw decide when each stage ends;
timeouts are only fault guards.
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

FIRST_DISTANCE_M="${FIRST_DISTANCE_M:-3.0}"
SECOND_DISTANCE_M="${SECOND_DISTANCE_M:-3.0}"
LINEAR_SPEED_MPS="${LINEAR_SPEED_MPS:-0.14}"
TURN_ANGLE_DEG="${TURN_ANGLE_DEG:-180.0}"
TURN_DIRECTION_SIGN="${TURN_DIRECTION_SIGN:-1.0}"
TURN_LATERAL_SPEED_MPS="${TURN_LATERAL_SPEED_MPS:-0.35}"
TURN_FORWARD_SPEED_MPS="${TURN_FORWARD_SPEED_MPS:-0.0}"
YAW_TOLERANCE_DEG="${YAW_TOLERANCE_DEG:-3.0}"
TURN_COMPLETION_HOLD_SEC="${TURN_COMPLETION_HOLD_SEC:-0.3}"
DISTANCE_TOLERANCE_M="${DISTANCE_TOLERANCE_M:-0.15}"
FIRST_LEG_MAX_SEC="${FIRST_LEG_MAX_SEC:-25.0}"
TURN_MAX_SEC="${TURN_MAX_SEC:-45.0}"
SECOND_LEG_MAX_SEC="${SECOND_LEG_MAX_SEC:-25.0}"
MAX_LINEAR_SPEED_MPS="${MAX_LINEAR_SPEED_MPS:-0.50}"
MAX_WAIT_FOR_READY_SEC="${MAX_WAIT_FOR_READY_SEC:-60}"
STATUSTEXT_CHECK_SEC="${STATUSTEXT_CHECK_SEC:-8}"
RC_WATCH_DURATION_SEC="${RC_WATCH_DURATION_SEC:-130}"
RC_WATCH_PRINT_PERIOD_SEC="${RC_WATCH_PRINT_PERIOD_SEC:-0.5}"
RC_WATCH_CHANGE_THRESHOLD_US="${RC_WATCH_CHANGE_THRESHOLD_US:-5}"

LOG_ROOT="${DIFF_OFFBOARD_CLOSED_LOOP_UTURN_LOG_ROOT:-$REPO_DIR/results/differential_offboard_closed_loop_u_turn}"
RUN_ID="${DIFF_OFFBOARD_CLOSED_LOOP_UTURN_RUN_ID:-closed_loop_right_uturn_$(date +%Y%m%d_%H%M%S)}"
LOG_DIR="$LOG_ROOT/$RUN_ID"
LOG_FILE="$LOG_DIR/closed_loop_right_u_turn_wrapper.log"
RC_WATCH_LOG_FILE="$LOG_DIR/rc_watch.log"
STATUSTEXT_LOG_FILE="$LOG_DIR/statustext_precheck.log"

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
    echo "Refusing to start closed-loop right U-turn test."
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

exec > >(tee -a "$LOG_FILE") 2>&1
echo "===== DIFFERENTIAL CLOSED-LOOP RIGHT U-TURN WRAPPER START $(date --iso-8601=seconds) ====="
echo "cwd=$PWD"
echo "command=$0 $*"
echo
echo "Saving logs:"
echo "  directory: $LOG_DIR"
echo "  wrapper:   $LOG_FILE"
echo "  rc_watch:  $RC_WATCH_LOG_FILE"
echo "  statustext:$STATUSTEXT_LOG_FILE"

RC_WATCH_PID=""

stop_rc_watch() {
  if [ -n "$RC_WATCH_PID" ]; then
    kill -TERM "$RC_WATCH_PID" >/dev/null 2>&1 || true
    wait "$RC_WATCH_PID" >/dev/null 2>&1 || true
    RC_WATCH_PID=""
  fi
}

cleanup() {
  local status=$?
  trap - EXIT INT TERM
  set +e
  echo
  echo "Closed-loop right U-turn wrapper cleanup..."
  timeout 5s ros2 service call \
    "$MAVROS_NS/cmd/arming" \
    mavros_msgs/srv/CommandBool \
    "{value: false}" >/tmp/mock_vehicle_closed_loop_uturn_disarm.log 2>&1 || true
  timeout 5s ros2 service call \
    "$MAVROS_NS/set_mode" \
    mavros_msgs/srv/SetMode \
    "{base_mode: 0, custom_mode: 'MANUAL'}" >/tmp/mock_vehicle_closed_loop_uturn_manual.log 2>&1 || true
  stop_rc_watch
  echo "Final MAVROS state snapshot:"
  timeout 5s ros2 topic echo --once "$MAVROS_NS/state" || true
  echo "Final MAVROS rc/out snapshot:"
  timeout 5s ros2 topic echo --once "$MAVROS_NS/rc/out" || true
  exit "$status"
}
trap cleanup EXIT INT TERM

echo
echo "Checking MAVROS state before closed-loop right U-turn..."
state_snapshot="$(timeout 6s ros2 topic echo --once "$MAVROS_NS/state" || true)"
printf '%s\n' "$state_snapshot"
state_armed="$(printf '%s\n' "$state_snapshot" | awk '$1 == "armed:" {print $2; exit}')"
state_manual_input="$(printf '%s\n' "$state_snapshot" | awk '$1 == "manual_input:" {print $2; exit}')"
state_mode="$(printf '%s\n' "$state_snapshot" | sed -n 's/^mode: //p' | head -n 1)"
case "$state_armed" in False|false) ;; *) echo "Refusing to start while armed." >&2; exit 2 ;; esac
case "$state_manual_input" in True|true) ;; *) echo "Refusing unless manual_input=true." >&2; exit 2 ;; esac
case "$state_mode" in MANUAL|manual) ;; *) echo "Refusing unless mode=MANUAL." >&2; exit 2 ;; esac

echo
echo "Checking neutral rc/out before closed-loop right U-turn..."
timeout 6s ros2 topic echo --once "$MAVROS_NS/rc/out"

echo
echo "Checking one local pose sample before closed-loop right U-turn..."
timeout 6s ros2 topic echo --once "$MAVROS_NS/local_position/pose"

echo
echo "Listening for PX4 statustext warnings for ${STATUSTEXT_CHECK_SEC}s..."
timeout "${STATUSTEXT_CHECK_SEC}s" \
  ros2 topic echo "$MAVROS_NS/statustext/recv" >"$STATUSTEXT_LOG_FILE" 2>&1 || true
cat "$STATUSTEXT_LOG_FILE" || true
if grep -Eiq 'Preflight Fail|Strong magnetic|heading estimate|magnetic interference|Yaw estimate' "$STATUSTEXT_LOG_FILE"; then
  echo "Refusing to start because PX4 reported a heading/magnetic preflight warning." >&2
  exit 2
fi

cat <<EOF

Effective closed-loop right U-turn settings:
  FIRST_DISTANCE_M=$FIRST_DISTANCE_M
  SECOND_DISTANCE_M=$SECOND_DISTANCE_M
  LINEAR_SPEED_MPS=$LINEAR_SPEED_MPS
  TURN_ANGLE_DEG=$TURN_ANGLE_DEG
  TURN_DIRECTION_SIGN=$TURN_DIRECTION_SIGN (+1 means right)
  TURN_LATERAL_SPEED_MPS=$TURN_LATERAL_SPEED_MPS
  TURN_FORWARD_SPEED_MPS=$TURN_FORWARD_SPEED_MPS
  YAW_TOLERANCE_DEG=$YAW_TOLERANCE_DEG
  TURN_COMPLETION_HOLD_SEC=$TURN_COMPLETION_HOLD_SEC
  DISTANCE_TOLERANCE_M=$DISTANCE_TOLERANCE_M
  FIRST_LEG_MAX_SEC=$FIRST_LEG_MAX_SEC
  TURN_MAX_SEC=$TURN_MAX_SEC
  SECOND_LEG_MAX_SEC=$SECOND_LEG_MAX_SEC
  MAX_LINEAR_SPEED_MPS=$MAX_LINEAR_SPEED_MPS
  log_dir=$LOG_DIR

Expected path: forward ${FIRST_DISTANCE_M}m, right turn until yaw changes
about ${TURN_ANGLE_DEG}deg, then forward ${SECOND_DISTANCE_M}m.
EOF

echo
echo "Starting MAVROS RC I/O watch:"
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

DIFF_OFFBOARD_L_TURN_LOG_ROOT="$LOG_ROOT" \
DIFF_OFFBOARD_L_TURN_RUN_ID="$RUN_ID" \
CONFIRM_GROUND_AREA_CLEAR=true \
CONFIRM_LOW_SPEED_GROUND_TEST=true \
CONFIRM_VEHICLE_DISARMED=true \
CONFIRM_RC_READY=true \
CONFIRM_PARAM_BACKUP=true \
CONFIRM_QGC_DISARM_READY=true \
CONFIRM_PHYSICAL_POWER_CUTOFF_READY=true \
CONFIRM_REAL_LOCAL_POSITION=true \
CONFIRM_CURRENT_DIFF_MAPPING=true \
CONFIRM_WHEELS_INSTALLED=true \
CONFIRM_FRESH_USER_START=true \
SETPOINT_VELOCITY_MAV_FRAME=BODY_NED \
MODE_CHANGE_ON_START=true \
ARM_ON_START=true \
DISARM_ON_FINISH=true \
INITIAL_STOP_SEC=0.0 \
PRESTART_FIRST_MOTION_SETPOINT="${PRESTART_FIRST_MOTION_SETPOINT:-true}" \
STOP_AFTER_FIRST_SEC="${STOP_AFTER_FIRST_SEC:-0.2}" \
STOP_AFTER_TURN_SEC="${STOP_AFTER_TURN_SEC:-0.2}" \
FINAL_STOP_SEC="${FINAL_STOP_SEC:-0.2}" \
STOP_BURST_SEC="${STOP_BURST_SEC:-0.4}" \
FIRST_DISTANCE_M="$FIRST_DISTANCE_M" \
SECOND_DISTANCE_M="$SECOND_DISTANCE_M" \
LINEAR_SPEED_MPS="$LINEAR_SPEED_MPS" \
TURN_ANGLE_DEG="$TURN_ANGLE_DEG" \
TURN_DIRECTION_SIGN="$TURN_DIRECTION_SIGN" \
TURN_LATERAL_SPEED_MPS="$TURN_LATERAL_SPEED_MPS" \
TURN_FORWARD_SPEED_MPS="$TURN_FORWARD_SPEED_MPS" \
YAW_TOLERANCE_DEG="$YAW_TOLERANCE_DEG" \
TURN_COMPLETION_HOLD_SEC="$TURN_COMPLETION_HOLD_SEC" \
DISTANCE_TOLERANCE_M="$DISTANCE_TOLERANCE_M" \
FIRST_LEG_MAX_SEC="$FIRST_LEG_MAX_SEC" \
TURN_MAX_SEC="$TURN_MAX_SEC" \
SECOND_LEG_MAX_SEC="$SECOND_LEG_MAX_SEC" \
MAX_LINEAR_SPEED_MPS="$MAX_LINEAR_SPEED_MPS" \
MAX_WAIT_FOR_READY_SEC="$MAX_WAIT_FOR_READY_SEC" \
  "$REPO_DIR/scripts/run_real_rover_mavros_differential_offboard_l_turn.sh"
