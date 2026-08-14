#!/usr/bin/env bash

set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEVICE="${PIXHAWK_DEVICE:-/dev/serial/by-id/usb-Auterion_PX4_FMU_v6C.x_0-if00}"
EXPECTED_SYSTEM_ID="${EXPECTED_SYSTEM_ID:-2}"
PARAM_TOOL="${PX4_PARAM_TOOL:-$REPO_DIR/scripts/px4_mavlink_param.py}"
STATE_DIR="${OFFBOARD_PROFILE_STATE_DIR:-$REPO_DIR/results/runtime_state}"
MARKER="$STATE_DIR/indoor_fake_ev.active"
CONFIRM_PHRASE="INDOOR_FAKE_EV_WHEELS_LIFTED_RESTORE_GPS_ON_EXIT"
RUN_ID="${RUN_ID_PREFIX:-orin2}_indoor_fake_ev_$(date +%Y%m%d_%H%M%S)"
RUN_DIR="$REPO_DIR/results/indoor_fake_ev/$RUN_ID"

usage() {
  cat <<EOF
Usage:
  $0 --execute --confirm $CONFIRM_PHRASE -- <indoor command> [args...]

This wrapper is exclusively for indoor fake external-vision tests. It:
  1. requires the outdoor baseline EKF2_EV_CTRL=0, EKF2_GPS_CTRL=7;
  2. creates a persistent INDOOR_FAKE_EV marker;
  3. temporarily sets EKF2_EV_CTRL=15;
  4. runs the supplied command with OFFBOARD_POSITIONING_PROFILE=INDOOR_FAKE_EV;
  5. stops the child process group and restores/reads back EKF2_EV_CTRL=0.

Required environment confirmations:
  CONFIRM_WHEELS_LIFTED=true
  CONFIRM_VEHICLE_DISARMED=true
  CONFIRM_RC_KILL_READY=true
  CONFIRM_RESTORE_GPS_BASELINE=true

Never use this wrapper outdoors. Outdoor launchers reject both this marker and
any PX4 configuration where EKF2_EV_CTRL is not zero.
EOF
}

if [ "${1:-}" != "--execute" ] \
  || [ "${2:-}" != "--confirm" ] \
  || [ "${3:-}" != "$CONFIRM_PHRASE" ] \
  || [ "${4:-}" != "--" ] \
  || [ "$#" -lt 5 ]; then
  usage
  exit 2
fi
shift 4

required_confirmations=(
  CONFIRM_WHEELS_LIFTED
  CONFIRM_VEHICLE_DISARMED
  CONFIRM_RC_KILL_READY
  CONFIRM_RESTORE_GPS_BASELINE
)
for name in "${required_confirmations[@]}"; do
  if [ "${!name:-false}" != "true" ]; then
    echo "REFUSED: required confirmation missing: $name=true" >&2
    exit 2
  fi
done

if [ ! -e "$DEVICE" ]; then
  echo "REFUSED: fixed Pixhawk by-id missing: $DEVICE" >&2
  exit 2
fi
if [ -e "$MARKER" ]; then
  echo "REFUSED: indoor profile marker already exists: $MARKER" >&2
  echo "Do not delete it manually. Restore and verify EKF2_EV_CTRL=0 first." >&2
  exit 3
fi
if ps -eo args= | grep -E \
  'mavros_node|mavros_fake_external_vision.py|mavros_fake_gps_input.py|orin2_outdoor_forward_5m.py' \
  | grep -v grep; then
  echo "REFUSED: a MAVROS/fake-position/outdoor mission process already exists" >&2
  exit 3
fi

mkdir -p "$STATE_DIR" "$RUN_DIR"
ln -sfn "$RUN_DIR" "$REPO_DIR/results/indoor_fake_ev/latest"
exec > >(tee -a "$RUN_DIR/profile.log") 2>&1

param_common=(
  python3 "$PARAM_TOOL"
  --device "$DEVICE" --baud 115200
  --expect-system-id "$EXPECTED_SYSTEM_ID" --expect-component-id 1
  --heartbeat-timeout 10 --timeout 3 --retries 4
)

require_param() {
  local file="$1" name="$2" expected="$3"
  grep -Eq "^${name}=${expected}([ .]|$)" "$file" || {
    echo "REFUSED: expected ${name}=${expected}" >&2
    return 1
  }
}

restore_outdoor_baseline() {
  local attempt verify_file
  echo "Restoring outdoor GPS baseline: EKF2_EV_CTRL=0"
  for attempt in 1 2 3; do
    if "${param_common[@]}" set --type int32 EKF2_EV_CTRL 0; then
      verify_file="$RUN_DIR/px4_restore_verify_${attempt}.txt"
      if "${param_common[@]}" get MAV_SYS_ID EKF2_EV_CTRL EKF2_GPS_CTRL \
        | tee "$verify_file" \
        && require_param "$verify_file" MAV_SYS_ID "$EXPECTED_SYSTEM_ID" \
        && require_param "$verify_file" EKF2_EV_CTRL 0 \
        && require_param "$verify_file" EKF2_GPS_CTRL 7; then
        rm -f "$MARKER"
        echo "OUTDOOR_GPS_BASELINE_RESTORED=true"
        return 0
      fi
    fi
    sleep 1
  done
  echo "CRITICAL: failed to verify outdoor baseline; marker retained: $MARKER" >&2
  echo "Do not run outdoor Offboard until EKF2_EV_CTRL=0 is read back." >&2
  return 1
}

CHILD_PID=""
PROFILE_MARKER_CREATED=0
cleanup() {
  local status=$?
  trap - EXIT INT TERM
  set +e
  if [ -n "$CHILD_PID" ] && kill -0 -- "-$CHILD_PID" 2>/dev/null; then
    echo "Stopping indoor child process group pgid=$CHILD_PID"
    kill -INT -- "-$CHILD_PID" 2>/dev/null || true
    for _ in $(seq 1 30); do
      kill -0 -- "-$CHILD_PID" 2>/dev/null || break
      sleep 0.2
    done
    kill -TERM -- "-$CHILD_PID" 2>/dev/null || true
  fi
  if [ "$PROFILE_MARKER_CREATED" -eq 1 ]; then
    sleep 1
    if ! restore_outdoor_baseline; then
      status=4
    fi
  fi
  echo "RUN_DIR=$RUN_DIR"
  exit "$status"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

echo "===== INDOOR_FAKE_EV PROFILE $(date --iso-8601=seconds) ====="
echo "PROFILE=INDOOR_FAKE_EV"
echo "EXPECTED_SYSTEM_ID=$EXPECTED_SYSTEM_ID"
echo "DEVICE=$DEVICE"
echo "Indoor command: $*"

"${param_common[@]}" get MAV_SYS_ID EKF2_EV_CTRL EKF2_GPS_CTRL \
  | tee "$RUN_DIR/px4_before.txt"
require_param "$RUN_DIR/px4_before.txt" MAV_SYS_ID "$EXPECTED_SYSTEM_ID"
require_param "$RUN_DIR/px4_before.txt" EKF2_EV_CTRL 0
require_param "$RUN_DIR/px4_before.txt" EKF2_GPS_CTRL 7

cat >"$MARKER" <<EOF
profile=INDOOR_FAKE_EV
host=$(hostname)
pid=$$
started_utc=$(date --utc --iso-8601=seconds)
expected_system_id=$EXPECTED_SYSTEM_ID
device=$DEVICE
run_dir=$RUN_DIR
EOF
PROFILE_MARKER_CREATED=1

"${param_common[@]}" set --type int32 EKF2_EV_CTRL 15
"${param_common[@]}" get MAV_SYS_ID EKF2_EV_CTRL EKF2_GPS_CTRL \
  | tee "$RUN_DIR/px4_indoor_active.txt"
require_param "$RUN_DIR/px4_indoor_active.txt" MAV_SYS_ID "$EXPECTED_SYSTEM_ID"
require_param "$RUN_DIR/px4_indoor_active.txt" EKF2_EV_CTRL 15
require_param "$RUN_DIR/px4_indoor_active.txt" EKF2_GPS_CTRL 7

setsid env \
  OFFBOARD_POSITIONING_PROFILE=INDOOR_FAKE_EV \
  OFFBOARD_PROFILE_MARKER="$MARKER" \
  INDOOR_FAKE_EV_LOG_ROOT="$RUN_DIR" \
  "$@" &
CHILD_PID=$!
set +e
wait "$CHILD_PID"
CHILD_RC=$?
set -e
exit "$CHILD_RC"
