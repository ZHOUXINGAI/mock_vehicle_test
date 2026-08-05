#!/usr/bin/env bash

set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEVICE="/dev/serial/by-id/usb-Auterion_PX4_FMU_v6C.x_0-if00"
ARDUINO_DEVICE="/dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_1433432373235171C182-if00"
EXECUTE_PHRASE="OUTDOOR_FORWARD_5M_AREA_CLEAR_RC_KILL_READY"
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

if [ "${1:-}" != "--execute" ]; then
  python3 "$REPO_DIR/src/orin2_outdoor_forward_5m.py" \
    --distance-m "${FORWARD_DISTANCE_M:-5.0}" \
    --speed-mps "${LINEAR_SPEED_MPS:-0.12}"
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

RUN_ID="orin2_outdoor_forward_5m_$(date +%Y%m%d_%H%M%S)"
RUN_DIR="$REPO_DIR/results/orin2_outdoor_forward_5m/$RUN_ID"
mkdir -p "$RUN_DIR"
ln -sfn "$RUN_DIR" "$REPO_DIR/results/orin2_outdoor_forward_5m/latest"
exec > >(tee -a "$RUN_DIR/orchestrator.log") 2>&1

MAVROS_PID=""
RC_WATCH_PID=""
MISSION_FINISHED=0

stop_group() {
  local pid="$1"
  [ -n "$pid" ] || return 0
  if kill -0 "$pid" 2>/dev/null; then
    kill -INT -- "-$pid" 2>/dev/null || true
    for _ in $(seq 1 30); do
      kill -0 "$pid" 2>/dev/null || return 0
      sleep 0.2
    done
    kill -TERM -- "-$pid" 2>/dev/null || true
  fi
}

cleanup() {
  local status=$?
  trap - EXIT INT TERM
  set +e
  if [ "$MISSION_FINISHED" -ne 1 ] && [ -n "$MAVROS_PID" ] && kill -0 "$MAVROS_PID" 2>/dev/null; then
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
echo "===== C2 OUTDOOR FORWARD 5M $(date --iso-8601=seconds) ====="
echo "This run forbids fake external vision, fake GPS, turns, reverse and lateral commands."

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
  --expect-system-id 2 --expect-component-id 1 \
  --heartbeat-timeout 10 --timeout 3 --retries 4 \
  get SYS_AUTOSTART CA_AIRFRAME MAV_SYS_ID CA_R_REV \
      PWM_MAIN_FUNC1 PWM_MAIN_FUNC2 PWM_MAIN_FUNC6 PWM_MAIN_FUNC7 \
      PWM_MAIN_DIS1 PWM_MAIN_DIS2 PWM_MAIN_FAIL1 PWM_MAIN_FAIL2 \
      EKF2_EV_CTRL EKF2_GPS_CTRL COM_RC_IN_MODE COM_OF_LOSS_T \
      COM_OBL_RC_ACT NAV_RCL_ACT RO_MAX_THR_SPEED RO_SPEED_LIM \
  | tee "$RUN_DIR/px4_preflight_params.txt"

require_param() {
  local name="$1" expected="$2"
  grep -Eq "^${name}=${expected}([ .]|$)" "$RUN_DIR/px4_preflight_params.txt" || {
    echo "REFUSED: expected ${name}=${expected}" >&2
    exit 3
  }
}
require_param SYS_AUTOSTART 50000
require_param CA_AIRFRAME 6
require_param MAV_SYS_ID 2
require_param CA_R_REV 3
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

# shellcheck disable=SC1091
source "$REPO_DIR/scripts/env.sh"
export ROS_LOG_DIR="$RUN_DIR/ros_logs"
mkdir -p "$ROS_LOG_DIR"

setsid env \
  MAVLINK_DEVICE="$DEVICE" MAVLINK_BAUD=115200 MAVROS_NS=mavros \
  TARGET_SYSTEM=2 TARGET_COMPONENT=1 \
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
timeout --signal=INT --kill-after=15s 240s \
  python3 "$REPO_DIR/src/orin2_outdoor_forward_5m.py" \
    --execute --confirm "$EXECUTE_PHRASE" --namespace /mavros \
    --distance-m "${FORWARD_DISTANCE_M:-5.0}" \
    --tolerance-m "${DISTANCE_TOLERANCE_M:-0.15}" \
    --speed-mps "${LINEAR_SPEED_MPS:-0.12}" \
    --max-speed-mps "${MAX_LINEAR_SPEED_MPS:-0.15}" \
    --max-cross-track-m "${MAX_CROSS_TRACK_M:-0.75}" \
    --max-heading-error-deg "${MAX_HEADING_ERROR_DEG:-35.0}" \
    --max-motion-sec "${MAX_MOTION_SEC:-75.0}" \
    --stall-window-sec "${STALL_WINDOW_SEC:-8.0}" \
    --stall-min-progress-m "${STALL_MIN_PROGRESS_M:-0.08}" \
  >"$RUN_DIR/mission.log" 2>&1
MISSION_RC=$?
set -e
sed -n '1,360p' "$RUN_DIR/mission.log"

timeout 30s python3 "$REPO_DIR/src/orin2_outdoor_forward_5m.py" \
  --verify-only --namespace /mavros | tee "$RUN_DIR/final_state.log"
MISSION_FINISHED=1
echo "MISSION_RC=$MISSION_RC"
if [ "$MISSION_RC" -ne 0 ]; then exit "$MISSION_RC"; fi
