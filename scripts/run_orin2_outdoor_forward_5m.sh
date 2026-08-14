#!/usr/bin/env bash

set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-99}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
DEVICE="${PIXHAWK_DEVICE:-/dev/serial/by-id/usb-Auterion_PX4_FMU_v6C.x_0-if00}"
ARDUINO_DEVICE="${ARDUINO_DEVICE:-/dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_1433432373235171C182-if00}"
INDOOR_PROFILE_MARKER="${OFFBOARD_PROFILE_STATE_DIR:-$REPO_DIR/results/runtime_state}/indoor_fake_ev.active"
EXPECTED_SYSTEM_ID="${EXPECTED_SYSTEM_ID:-2}"
EXPECTED_PWM_MAIN_REV="${EXPECTED_PWM_MAIN_REV:-0}"
EXPECTED_RO_YAW_RATE_TH="${EXPECTED_RO_YAW_RATE_TH:-0.5}"
EXPECTED_RO_YAW_RATE_CORR="${EXPECTED_RO_YAW_RATE_CORR:-2.0}"
RUN_ID_PREFIX="${RUN_ID_PREFIX:-orin2}"
MISSION_PROFILE="${MISSION_PROFILE:-forward}"
DEFAULT_TRACKER_MAX_BODY_BEARING_DEG=35.0
DEFAULT_TRACKER_CURVATURE_TO_BODY_GAIN_M=0.91
DEFAULT_TRACKER_MAX_CURVATURE_CORRECTION_INV_M=0.24
mission_args=()
external_start_args=()
if [ -n "${PAIRB_START_GATE_FD:-}" ] || [ -n "${PAIRB_START_PLAN_ID:-}" ]; then
  if [ -z "${PAIRB_START_GATE_FD:-}" ] || [ -z "${PAIRB_START_PLAN_ID:-}" ]; then
    echo "REFUSED: PAIRB_START_GATE_FD and PAIRB_START_PLAN_ID must be set together" >&2
    exit 2
  fi
  external_start_args=(
    --external-start-gate-fd "$PAIRB_START_GATE_FD"
    --external-start-plan-id "$PAIRB_START_PLAN_ID"
    --external-start-gate-timeout-sec "${PAIRB_START_GATE_TIMEOUT_SEC:-300}"
  )
fi
if [ -n "${PAIRB_RUNTIME_STOP_FD:-}" ]; then
  if [ -z "${PAIRB_START_PLAN_ID:-}" ]; then
    echo "REFUSED: Pair B runtime stop FD requires the start plan ID" >&2
    exit 2
  fi
  external_start_args+=(--external-runtime-stop-fd "$PAIRB_RUNTIME_STOP_FD")
fi
runtime_hold_enabled=0
if [ -n "${PAIRB_WORKER_STATUS_FD:-}" ] \
  || [ -n "${PAIRB_RUNTIME_SHUTDOWN_FD:-}" ]; then
  if [ -z "${PAIRB_WORKER_STATUS_FD:-}" ] \
    || [ -z "${PAIRB_RUNTIME_SHUTDOWN_FD:-}" ] \
    || [ -z "${PAIRB_START_PLAN_ID:-}" ]; then
    echo "REFUSED: worker status/shutdown FDs require the Pair B start plan ID" >&2
    exit 2
  fi
  runtime_hold_enabled=1
fi
case "$MISSION_PROFILE" in
  entry_hold)
    EXECUTE_PHRASE="OUTDOOR_ENTRY_HOLD_WHEELS_LIFTED_RC_KILL_READY"
    RUN_LABEL="entry_hold"
    mission_args=(--entry-only-hold-sec "${ENTRY_ONLY_HOLD_SEC:-2.0}")
    ;;
  forward)
    EXECUTE_PHRASE="OUTDOOR_FORWARD_5M_AREA_CLEAR_RC_KILL_READY"
    RUN_LABEL="forward"
    ;;
  right_uturn)
    EXECUTE_PHRASE="OUTDOOR_6M_RIGHT_UTURN_6M_AREA_CLEAR_RC_KILL_READY"
    RUN_LABEL="right_uturn"
    mission_args=(
      --u-turn
      --turn-direction-sign 1.0
      --turn-angle-deg "${TURN_ANGLE_DEG:-180.0}"
      --turn-tolerance-deg "${TURN_TOLERANCE_DEG:-8.0}"
      --turn-forward-speed-mps "${TURN_FORWARD_SPEED_MPS:-0.04}"
      --turn-lateral-speed-mps "${TURN_LATERAL_SPEED_MPS:-0.04}"
      --turn-max-sec "${TURN_MAX_SEC:-45.0}"
      --turn-completion-hold-sec "${TURN_COMPLETION_HOLD_SEC:-0.30}"
      --turn-stall-window-sec "${TURN_STALL_WINDOW_SEC:-8.0}"
      --turn-stall-min-progress-deg "${TURN_STALL_MIN_PROGRESS_DEG:-5.0}"
      --turn-clearance-radius-m "${TURN_CLEARANCE_RADIUS_M:-3.5}"
      --turn-radius-m "${TURN_RADIUS_M:-3.0}"
    )
    ;;
  left_uturn)
    EXECUTE_PHRASE="OUTDOOR_5M_LEFT_UTURN_5M_AREA_CLEAR_RC_KILL_READY"
    RUN_LABEL="left_uturn"
    mission_args=(
      --u-turn
      --turn-direction-sign -1.0
      --turn-angle-deg "${TURN_ANGLE_DEG:-180.0}"
      --turn-tolerance-deg "${TURN_TOLERANCE_DEG:-8.0}"
      --turn-forward-speed-mps "${TURN_FORWARD_SPEED_MPS:-0.04}"
      --turn-lateral-speed-mps "${TURN_LATERAL_SPEED_MPS:-0.04}"
      --turn-max-sec "${TURN_MAX_SEC:-45.0}"
      --turn-completion-hold-sec "${TURN_COMPLETION_HOLD_SEC:-0.30}"
      --turn-stall-window-sec "${TURN_STALL_WINDOW_SEC:-8.0}"
      --turn-stall-min-progress-deg "${TURN_STALL_MIN_PROGRESS_DEG:-5.0}"
      --turn-clearance-radius-m "${TURN_CLEARANCE_RADIUS_M:-3.5}"
      --turn-radius-m "${TURN_RADIUS_M:-3.0}"
    )
    ;;
  s_bend_return)
    EXECUTE_PHRASE="OUTDOOR_S_BEND_RETURN_AREA_CLEAR_RC_KILL_READY"
    RUN_LABEL="s_bend_return"
    mission_args=(
      --s-bend-return
      --turn-forward-speed-mps "${TURN_FORWARD_SPEED_MPS:-0.04}"
      --turn-radius-m "${TURN_RADIUS_M:-3.0}"
    )
    ;;
  *)
    echo "REFUSED: unsupported MISSION_PROFILE=$MISSION_PROFILE" >&2
    exit 2
    ;;
esac
required_confirmations=(
  CONFIRM_GROUND_AREA_CLEAR
  CONFIRM_LOW_SPEED_GROUND_TEST
  CONFIRM_VEHICLE_DISARMED
  CONFIRM_RC_KILL_READY
  CONFIRM_QGC_DISARM_READY
  CONFIRM_PHYSICAL_POWER_CUTOFF_READY
  CONFIRM_REAL_GPS_3D_FIX
  CONFIRM_REAL_LOCAL_POSITION
  CONFIRM_CURRENT_DIFF_MAPPING
  CONFIRM_WHEELS_INSTALLED
  CONFIRM_CABLES_SECURED
  CONFIRM_FRESH_USER_START
)
if [ "$MISSION_PROFILE" = "entry_hold" ]; then
  required_confirmations+=(CONFIRM_WHEELS_LIFTED)
fi

if [ "${1:-}" != "--execute" ]; then
  python3 "$REPO_DIR/src/orin2_outdoor_forward_5m.py" \
    --distance-m "${FORWARD_DISTANCE_M:-5.0}" \
    --speed-mps "${LINEAR_SPEED_MPS:-0.06}" \
    --max-speed-mps "${MAX_LINEAR_SPEED_MPS:-0.15}" \
    --terminal-speed-mps "${TERMINAL_SPEED_MPS:-0.035}" \
    --terminal-slowdown-distance-m "${TERMINAL_SLOWDOWN_DISTANCE_M:-1.5}" \
    --course-calibration-speed-mps "${COURSE_CALIBRATION_SPEED_MPS:-0.04}" \
    --course-calibration-max-sec "${COURSE_CALIBRATION_MAX_SEC:-5.0}" \
    --reference-mode "${REFERENCE_MODE:-ground_course_rollout}" \
    --rviz-topic-prefix "${RVIZ_TOPIC_PREFIX:-/orin2/offboard}" \
    --body-forward-sign "${BODY_FORWARD_SIGN:-1.0}" \
    --tracker-base-lookahead-m "${TRACKER_BASE_LOOKAHEAD_M:-1.10}" \
    --tracker-speed-lookahead-gain-sec "${TRACKER_SPEED_LOOKAHEAD_GAIN_SEC:-0.80}" \
    --tracker-min-lookahead-m "${TRACKER_MIN_LOOKAHEAD_M:-0.90}" \
    --tracker-max-lookahead-m "${TRACKER_MAX_LOOKAHEAD_M:-1.80}" \
    --tracker-max-body-bearing-deg "${TRACKER_MAX_BODY_BEARING_DEG:-$DEFAULT_TRACKER_MAX_BODY_BEARING_DEG}" \
    --tracker-max-body-bearing-rate-degps "${TRACKER_MAX_BODY_BEARING_RATE_DEGPS:-45.0}" \
    --tracker-curvature-slowdown-gain "${TRACKER_CURVATURE_SLOWDOWN_GAIN:-0.0}" \
    --tracker-reference-curvature-window-m "${TRACKER_REFERENCE_CURVATURE_WINDOW_M:-0.45}" \
    --tracker-max-curvature-correction-inv-m "${TRACKER_MAX_CURVATURE_CORRECTION_INV_M:-$DEFAULT_TRACKER_MAX_CURVATURE_CORRECTION_INV_M}" \
    --tracker-curvature-to-body-gain-m "${TRACKER_CURVATURE_TO_BODY_GAIN_M:-$DEFAULT_TRACKER_CURVATURE_TO_BODY_GAIN_M}" \
    --tracker-curvature-feedback-gain-ratio "${TRACKER_CURVATURE_FEEDBACK_GAIN_RATIO:-1.28}" \
    --tracker-cross-track-integral-gain-inv-m-per-m-sec "${TRACKER_CROSS_TRACK_INTEGRAL_GAIN_INV_M_PER_M_SEC:-0.04}" \
    --tracker-cross-track-integral-limit-m-sec "${TRACKER_CROSS_TRACK_INTEGRAL_LIMIT_M_SEC:-1.0}" \
    --tracker-min-nominal-bearing-reserve-deg "${TRACKER_MIN_NOMINAL_BEARING_RESERVE_DEG:-3.0}" \
    --tracker-max-reference-curvature-rate-inv-m2 "${TRACKER_MAX_REFERENCE_CURVATURE_RATE_INV_M2:-1.3}" \
    "${external_start_args[@]}" \
    "${mission_args[@]}"
  cat <<EOF

Preparation only. This command did not start MAVROS, Arm, Offboard or motors.
For a separately authorized field run, pass:
  --execute --confirm $EXECUTE_PHRASE
and provide every CONFIRM_* variable printed by this script.
EOF
  printf '  %s=true\n' "${required_confirmations[@]}"
  exit 0
fi
if [ "${2:-}" != "--confirm" ] || [ "${3:-}" != "$EXECUTE_PHRASE" ]; then
  echo "REFUSED: exact live phrase required:" >&2
  echo "  --execute --confirm $EXECUTE_PHRASE" >&2
  exit 2
fi

missing=()
for name in "${required_confirmations[@]}"; do
  if [ "${!name:-false}" != "true" ]; then missing+=("$name=true"); fi
done
if [ "${#missing[@]}" -gt 0 ]; then
  echo "REFUSED: missing current-run confirmations:" >&2
  printf '  %s\n' "${missing[@]}" >&2
  exit 2
fi

RUN_ID="${RUN_ID_PREFIX}_outdoor_${RUN_LABEL}_$(date +%Y%m%d_%H%M%S)"
RUN_DIR="$REPO_DIR/results/${RUN_ID_PREFIX}_outdoor_forward_5m/$RUN_ID"
mkdir -p "$RUN_DIR"
ln -sfn "$RUN_DIR" "$REPO_DIR/results/${RUN_ID_PREFIX}_outdoor_forward_5m/latest"
exec > >(tee -a "$RUN_DIR/orchestrator.log") 2>&1

MAVROS_PID=""
RC_WATCH_PID=""
MISSION_PID=""
MISSION_FINISHED=0

process_group_alive() {
  local pgid="$1"
  [ -n "$pgid" ] || return 1
  kill -0 -- "-$pgid" 2>/dev/null
}

stop_group() {
  local pid="$1"
  [ -n "$pid" ] || return 0
  # ros2 launch can exit before mavros_node. Test the dedicated process group,
  # not only its original leader PID, so descendants cannot retain the FCU.
  if process_group_alive "$pid"; then
    kill -INT -- "-$pid" 2>/dev/null || true
    for _ in $(seq 1 30); do
      process_group_alive "$pid" || return 0
      sleep 0.2
    done
    kill -TERM -- "-$pid" 2>/dev/null || true
    for _ in $(seq 1 25); do
      process_group_alive "$pid" || return 0
      sleep 0.2
    done
    echo "WARNING: process group $pid still exists after bounded cleanup" >&2
  fi
}

stop_timeout_process() {
  local pid="$1"
  [ -n "$pid" ] || return 0
  if kill -0 "$pid" 2>/dev/null; then
    # Signal only GNU timeout. It forwards one SIGINT to the mission, avoiding
    # the duplicate delivery that occurs when both timeout and Python receive
    # a process-group SIGINT.
    kill -INT "$pid" 2>/dev/null || true
    for _ in $(seq 1 75); do
      kill -0 "$pid" 2>/dev/null || return 0
      sleep 0.2
    done
    kill -TERM "$pid" 2>/dev/null || true
  fi
}

cleanup() {
  local status=$?
  trap - EXIT INT TERM
  set +e
  stop_timeout_process "$MISSION_PID"
  MISSION_PID=""
  if [ "$MISSION_FINISHED" -ne 1 ] && process_group_alive "$MAVROS_PID"; then
    echo "Running fallback Disarm+MANUAL recovery..."
    timeout 30s python3 "$REPO_DIR/src/orin2_outdoor_forward_5m.py" \
      --recover-only --namespace /mavros | tee -a "$RUN_DIR/fallback_recovery.log"
  fi
  stop_group "$RC_WATCH_PID"
  stop_group "$MAVROS_PID"
  sleep 2
  echo "Residual process audit:"
  ps -eo pid=,args= | grep -E 'mavros_node|orin2_outdoor_forward_5m.py|mavros_fake_external_vision.py|mavros_fake_gps_input.py' | grep -v grep || true
  echo "RUN_DIR=$RUN_DIR"
  exit "$status"
}
trap cleanup EXIT INT TERM

cd "$REPO_DIR"
echo "===== C2 OUTDOOR ${RUN_LABEL^^} $(date --iso-8601=seconds) ====="
echo "This run forbids fake external vision, fake GPS and reverse; only the selected bounded mission is allowed."

if [ -e "$INDOOR_PROFILE_MARKER" ]; then
  echo "REFUSED: persistent INDOOR_FAKE_EV marker exists:" >&2
  echo "  $INDOOR_PROFILE_MARKER" >&2
  echo "Restore and read back EKF2_EV_CTRL=0; never delete the marker as a substitute for verification." >&2
  exit 2
fi

if [ ! -e "$DEVICE" ]; then
  echo "REFUSED: fixed Pixhawk by-id missing: $DEVICE" >&2
  exit 2
fi
PIXHAWK_RESOLVED="$(readlink -f "$DEVICE")"
if [ -e "$ARDUINO_DEVICE" ] \
  && [ "$PIXHAWK_RESOLVED" = "$(readlink -f "$ARDUINO_DEVICE")" ]; then
  echo "REFUSED: Pixhawk and Arduino by-id paths resolve to the same device" >&2
  exit 2
fi
echo "Pixhawk fixed by-id resolves to: $PIXHAWK_RESOLVED"
if ps -eo args= | grep -E 'mavros_node|mavros_fake_external_vision.py|mavros_fake_gps_input.py|orin2_outdoor_forward_5m.py' | grep -v grep; then
  echo "REFUSED: related process already exists" >&2
  exit 2
fi

python3 scripts/px4_mavlink_param.py \
  --device "$DEVICE" --baud 115200 \
  --expect-system-id "$EXPECTED_SYSTEM_ID" --expect-component-id 1 \
  --heartbeat-timeout 10 --timeout 3 --retries 4 \
  get SYS_AUTOSTART CA_AIRFRAME MAV_SYS_ID CA_R_REV PWM_MAIN_REV \
      PWM_MAIN_FUNC1 PWM_MAIN_FUNC2 PWM_MAIN_FUNC6 PWM_MAIN_FUNC7 \
      PWM_MAIN_DIS1 PWM_MAIN_DIS2 PWM_MAIN_FAIL1 PWM_MAIN_FAIL2 \
      EKF2_EV_CTRL EKF2_GPS_CTRL COM_RC_IN_MODE COM_OF_LOSS_T \
      COM_OBL_RC_ACT NAV_RCL_ACT RO_MAX_THR_SPEED RO_SPEED_LIM \
      RO_YAW_RATE_TH RO_YAW_RATE_CORR \
  | tee "$RUN_DIR/px4_preflight_params.txt"

require_param() {
  local name="$1" expected="$2"
  grep -Eq "^${name}=${expected}([ .]|$)" "$RUN_DIR/px4_preflight_params.txt" || {
    echo "REFUSED: expected ${name}=${expected}" >&2
    exit 3
  }
}
require_float_param() {
  local name="$1" expected="$2" tolerance="${3:-0.0001}"
  awk -F'[ =]' -v name="$name" -v expected="$expected" -v tolerance="$tolerance" '
    $1 == name {
      delta = $2 - expected
      if (delta < 0) delta = -delta
      if (delta <= tolerance) found = 1
    }
    END { exit(found ? 0 : 1) }
  ' "$RUN_DIR/px4_preflight_params.txt" || {
    echo "REFUSED: expected ${name}=${expected} (+/-${tolerance})" >&2
    exit 3
  }
}
require_param SYS_AUTOSTART 50000
require_param CA_AIRFRAME 6
require_param MAV_SYS_ID "$EXPECTED_SYSTEM_ID"
require_param CA_R_REV 3
require_param PWM_MAIN_REV "$EXPECTED_PWM_MAIN_REV"
require_param PWM_MAIN_FUNC1 101
require_param PWM_MAIN_FUNC2 102
require_param PWM_MAIN_FUNC6 0
require_param PWM_MAIN_FUNC7 0
require_param PWM_MAIN_DIS1 1500
require_param PWM_MAIN_DIS2 1500
require_param PWM_MAIN_FAIL1 1500
require_param PWM_MAIN_FAIL2 1500
require_param EKF2_EV_CTRL 0
require_param EKF2_GPS_CTRL 7
require_param COM_RC_IN_MODE 3
require_param COM_OF_LOSS_T 1
require_param COM_OBL_RC_ACT 0
require_param NAV_RCL_ACT 2
if [ -n "$EXPECTED_RO_YAW_RATE_TH" ]; then
  require_float_param RO_YAW_RATE_TH "$EXPECTED_RO_YAW_RATE_TH"
fi
if [ -n "$EXPECTED_RO_YAW_RATE_CORR" ]; then
  require_float_param RO_YAW_RATE_CORR "$EXPECTED_RO_YAW_RATE_CORR"
fi

# shellcheck disable=SC1091
source "$REPO_DIR/scripts/env.sh"
export ROS_LOG_DIR="$RUN_DIR/ros_logs"
mkdir -p "$ROS_LOG_DIR"

setsid env \
  MAVLINK_DEVICE="$DEVICE" MAVLINK_BAUD=115200 MAVROS_NS=mavros \
  TARGET_SYSTEM="$EXPECTED_SYSTEM_ID" TARGET_COMPONENT=1 \
  QGC_UDP_URL=udp://:14555@127.0.0.1:14550 \
  MAVROS_DISABLE_PARAM_PLUGIN=true \
  "$REPO_DIR/scripts/run_mavros_px4_usb_to_qgc.sh" \
  >"$RUN_DIR/mavros.log" 2>&1 &
MAVROS_PID=$!

setsid python3 "$REPO_DIR/src/mavros_rc_io_watch.py" \
  --ros-args -p mavros_namespace:="'/mavros'" -p duration_sec:=120.0 \
  >"$RUN_DIR/rc_watch.log" 2>&1 &
RC_WATCH_PID=$!

set +e
MISSION_PROCESS_TIMEOUT_SEC=240
if [ -n "${PAIRB_START_GATE_FD:-}" ]; then
  MISSION_PROCESS_TIMEOUT_SEC="$(awk -v gate="${PAIRB_START_GATE_TIMEOUT_SEC:-300}" \
    'BEGIN { printf "%.0f", gate + 240 }')"
fi
setsid timeout --foreground --signal=INT --kill-after=15s "${MISSION_PROCESS_TIMEOUT_SEC}s" \
  python3 "$REPO_DIR/src/orin2_outdoor_forward_5m.py" \
    --execute --confirm "$EXECUTE_PHRASE" --namespace /mavros \
    --distance-m "${FORWARD_DISTANCE_M:-5.0}" \
    --tolerance-m "${DISTANCE_TOLERANCE_M:-0.15}" \
    --speed-mps "${LINEAR_SPEED_MPS:-0.06}" \
    --max-speed-mps "${MAX_LINEAR_SPEED_MPS:-0.15}" \
    --terminal-speed-mps "${TERMINAL_SPEED_MPS:-0.035}" \
    --terminal-slowdown-distance-m "${TERMINAL_SLOWDOWN_DISTANCE_M:-1.5}" \
    --max-cross-track-m "${MAX_CROSS_TRACK_M:-1.5}" \
    --max-heading-error-deg "${MAX_HEADING_ERROR_DEG:-35.0}" \
    --max-motion-sec "${MAX_MOTION_SEC:-180.0}" \
    --stall-window-sec "${STALL_WINDOW_SEC:-8.0}" \
    --stall-min-progress-m "${STALL_MIN_PROGRESS_M:-0.08}" \
    --course-calibration-distance-m "${COURSE_CALIBRATION_DISTANCE_M:-1.0}" \
    --course-calibration-speed-mps "${COURSE_CALIBRATION_SPEED_MPS:-0.04}" \
    --course-calibration-max-sec "${COURSE_CALIBRATION_MAX_SEC:-5.0}" \
    --course-calibration-max-yaw-change-deg "${COURSE_CALIBRATION_MAX_YAW_CHANGE_DEG:-20.0}" \
    --reference-mode "${REFERENCE_MODE:-ground_course_rollout}" \
    --rviz-topic-prefix "${RVIZ_TOPIC_PREFIX:-/orin2/offboard}" \
    --trajectory-artifact-dir "$RUN_DIR" \
    --steering-trim-mps "${STEERING_TRIM_MPS:-0.0}" \
    --heading-kp "${HEADING_KP:-0.30}" \
    --heading-ki "${HEADING_KI:-0.02}" \
    --heading-kd "${HEADING_KD:-0.03}" \
    --heading-integral-limit-rad-sec "${HEADING_INTEGRAL_LIMIT_RAD_SEC:-0.25}" \
    --heading-derivative-tau-sec "${HEADING_DERIVATIVE_TAU_SEC:-0.20}" \
    --max-steering-mps "${MAX_STEERING_MPS:-0.06}" \
    --min-effective-steering-mps "${MIN_EFFECTIVE_STEERING_MPS:-0.03}" \
    --heading-deadband-deg "${HEADING_DEADBAND_DEG:-1.0}" \
    --cross-track-lookahead-m "${CROSS_TRACK_LOOKAHEAD_M:-0.8}" \
    --cross-track-deadband-m "${CROSS_TRACK_DEADBAND_M:-0.15}" \
    --cross-track-filter-tau-sec "${CROSS_TRACK_FILTER_TAU_SEC:-0.20}" \
    --max-path-heading-correction-deg "${MAX_PATH_HEADING_CORRECTION_DEG:-15.0}" \
    --body-forward-sign "${BODY_FORWARD_SIGN:-1.0}" \
    --steering-direction-sign "${STEERING_DIRECTION_SIGN:-1.0}" \
    --trajectory-spacing-m "${TRAJECTORY_SPACING_M:-0.15}" \
    --tracker-base-lookahead-m "${TRACKER_BASE_LOOKAHEAD_M:-1.10}" \
    --tracker-speed-lookahead-gain-sec "${TRACKER_SPEED_LOOKAHEAD_GAIN_SEC:-0.80}" \
    --tracker-min-lookahead-m "${TRACKER_MIN_LOOKAHEAD_M:-0.90}" \
    --tracker-max-lookahead-m "${TRACKER_MAX_LOOKAHEAD_M:-1.80}" \
    --tracker-projection-backtrack-m "${TRACKER_PROJECTION_BACKTRACK_M:-0.20}" \
    --tracker-projection-ahead-m "${TRACKER_PROJECTION_AHEAD_M:-4.0}" \
    --tracker-max-body-bearing-deg "${TRACKER_MAX_BODY_BEARING_DEG:-$DEFAULT_TRACKER_MAX_BODY_BEARING_DEG}" \
    --tracker-max-body-bearing-rate-degps "${TRACKER_MAX_BODY_BEARING_RATE_DEGPS:-45.0}" \
    --tracker-max-yaw-rate-radps "${TRACKER_MAX_YAW_RATE_RADPS:-0.35}" \
    --tracker-curvature-slowdown-gain "${TRACKER_CURVATURE_SLOWDOWN_GAIN:-0.0}" \
    --tracker-reference-curvature-window-m "${TRACKER_REFERENCE_CURVATURE_WINDOW_M:-0.45}" \
    --tracker-max-curvature-correction-inv-m "${TRACKER_MAX_CURVATURE_CORRECTION_INV_M:-$DEFAULT_TRACKER_MAX_CURVATURE_CORRECTION_INV_M}" \
    --tracker-curvature-to-body-gain-m "${TRACKER_CURVATURE_TO_BODY_GAIN_M:-$DEFAULT_TRACKER_CURVATURE_TO_BODY_GAIN_M}" \
    --tracker-curvature-feedback-gain-ratio "${TRACKER_CURVATURE_FEEDBACK_GAIN_RATIO:-1.28}" \
    --tracker-cross-track-integral-gain-inv-m-per-m-sec "${TRACKER_CROSS_TRACK_INTEGRAL_GAIN_INV_M_PER_M_SEC:-0.04}" \
    --tracker-cross-track-integral-limit-m-sec "${TRACKER_CROSS_TRACK_INTEGRAL_LIMIT_M_SEC:-1.0}" \
    --tracker-min-nominal-bearing-reserve-deg "${TRACKER_MIN_NOMINAL_BEARING_RESERVE_DEG:-3.0}" \
    --tracker-max-reference-curvature-rate-inv-m2 "${TRACKER_MAX_REFERENCE_CURVATURE_RATE_INV_M2:-1.3}" \
    "${external_start_args[@]}" \
    "${mission_args[@]}" \
  >"$RUN_DIR/mission.log" 2>&1 &
MISSION_PID=$!
wait "$MISSION_PID"
MISSION_RC=$?
MISSION_PID=""
set -e
sed -n '1,360p' "$RUN_DIR/mission.log"

set +e
timeout 30s python3 "$REPO_DIR/src/orin2_outdoor_forward_5m.py" \
  --verify-only --namespace /mavros | tee "$RUN_DIR/final_state.log"
FINAL_STATE_RC=${PIPESTATUS[0]}
set -e
if [ "$MISSION_RC" -eq 0 ] && [ "$FINAL_STATE_RC" -ne 0 ]; then
  MISSION_RC=8
fi
if [ -s "$RUN_DIR/planned_trajectory.csv" ] && [ -s "$RUN_DIR/actual_trajectory.csv" ]; then
  if ! python3 "$REPO_DIR/scripts/plot_rover_trajectory_xy.py" \
      "$RUN_DIR/planned_trajectory.csv" "$RUN_DIR/actual_trajectory.csv" \
      "$RUN_DIR/trajectory_xy.png"; then
    echo "WARNING: trajectory PNG generation failed; CSV artifacts remain valid" >&2
  fi
fi
if [ "$runtime_hold_enabled" -eq 1 ]; then
  printf 'PAIRB_WORKER_RESULT plan_id=%s rc=%s\n' \
    "$PAIRB_START_PLAN_ID" "$MISSION_RC" >&"$PAIRB_WORKER_STATUS_FD"
  if [ "$MISSION_RC" -eq 0 ]; then
    echo "PAIRB_RUNTIME_HOLD plan_id=$PAIRB_START_PLAN_ID state=MANUAL/disarmed"
    runtime_shutdown=""
    if ! IFS= read -r -t "${PAIRB_RUNTIME_HOLD_TIMEOUT_SEC:-30}" \
      runtime_shutdown <&"$PAIRB_RUNTIME_SHUTDOWN_FD"; then
      echo "REFUSED: Pair B runtime shutdown token timed out" >&2
      MISSION_RC=9
    elif [ "$runtime_shutdown" != "PAIRB_RUNTIME_SHUTDOWN plan_id=$PAIRB_START_PLAN_ID" ]; then
      echo "REFUSED: invalid Pair B runtime shutdown token" >&2
      MISSION_RC=9
    else
      echo "PAIRB_RUNTIME_SHUTDOWN_ACCEPTED plan_id=$PAIRB_START_PLAN_ID"
    fi
  fi
fi
MISSION_FINISHED=1
echo "MISSION_RC=$MISSION_RC"
if [ "$MISSION_RC" -ne 0 ]; then exit "$MISSION_RC"; fi
