#!/usr/bin/env bash

set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

MAVLINK_DEVICE="${MAVLINK_DEVICE:-/dev/serial/by-id/usb-Auterion_PX4_FMU_v6C.x_0-if00}"
if [ ! -e "$MAVLINK_DEVICE" ] && [ -e /dev/ttyACM0 ]; then
  MAVLINK_DEVICE=/dev/ttyACM0
fi

MAVLINK_BAUD="${MAVLINK_BAUD:-115200}"
SOURCE_SYSTEM="${SOURCE_SYSTEM:-245}"
COMMAND_RATE_HZ="${COMMAND_RATE_HZ:-20}"
WARMUP_SEC="${WARMUP_SEC:-2.0}"
STOP_SEC="${STOP_SEC:-1.0}"
FORWARD_SEC="${FORWARD_SEC:-1.0}"
BACKWARD_SEC="${BACKWARD_SEC:-1.0}"
TURN_SEC="${TURN_SEC:-0.5}"
FINAL_STOP_SEC="${FINAL_STOP_SEC:-2.0}"
LINEAR_SPEED_MPS="${LINEAR_SPEED_MPS:-0.12}"
TURN_YAW_RATE_RADPS="${TURN_YAW_RATE_RADPS:-0.25}"
TURN_SIGN="${TURN_SIGN:-1.0}"
MAX_LINEAR_SPEED_MPS="${MAX_LINEAR_SPEED_MPS:-0.30}"
MAX_YAW_RATE_RADPS="${MAX_YAW_RATE_RADPS:-0.70}"

MODE_CHANGE_ON_START="${MODE_CHANGE_ON_START:-false}"
ARM_ON_START="${ARM_ON_START:-false}"
DISARM_ON_FINISH="${DISARM_ON_FINISH:-false}"
REQUIRE_OFFBOARD_MODE="${REQUIRE_OFFBOARD_MODE:-true}"
REQUIRE_ARMED="${REQUIRE_ARMED:-true}"
MAX_WAIT_FOR_READY_SEC="${MAX_WAIT_FOR_READY_SEC:-60}"
STOP_BURST_SEC="${STOP_BURST_SEC:-0.8}"

CONFIRM_WHEELS_LIFTED="${CONFIRM_WHEELS_LIFTED:-false}"
CONFIRM_RC_READY="${CONFIRM_RC_READY:-false}"
CONFIRM_PARAM_BACKUP="${CONFIRM_PARAM_BACKUP:-false}"

if [ "$CONFIRM_WHEELS_LIFTED" != "true" ] ||
   [ "$CONFIRM_RC_READY" != "true" ] ||
   [ "$CONFIRM_PARAM_BACKUP" != "true" ]; then
  {
    echo "Refusing to start real rover MAVLink Offboard smoke test."
    echo "Required confirmations:"
    echo "  CONFIRM_WHEELS_LIFTED=true"
    echo "  CONFIRM_RC_READY=true"
    echo "  CONFIRM_PARAM_BACKUP=true"
  } >&2
  exit 2
fi

python3 "$REPO_DIR/src/real_rover_mavlink_offboard_smoke.py" \
  --device "$MAVLINK_DEVICE" \
  --baud "$MAVLINK_BAUD" \
  --source-system "$SOURCE_SYSTEM" \
  --command-rate-hz "$COMMAND_RATE_HZ" \
  --warmup-sec "$WARMUP_SEC" \
  --stop-sec "$STOP_SEC" \
  --forward-sec "$FORWARD_SEC" \
  --backward-sec "$BACKWARD_SEC" \
  --turn-sec "$TURN_SEC" \
  --final-stop-sec "$FINAL_STOP_SEC" \
  --linear-speed-mps "$LINEAR_SPEED_MPS" \
  --turn-yaw-rate-radps "$TURN_YAW_RATE_RADPS" \
  --turn-sign "$TURN_SIGN" \
  --max-linear-speed-mps "$MAX_LINEAR_SPEED_MPS" \
  --max-yaw-rate-radps "$MAX_YAW_RATE_RADPS" \
  --mode-change-on-start "$MODE_CHANGE_ON_START" \
  --arm-on-start "$ARM_ON_START" \
  --disarm-on-finish "$DISARM_ON_FINISH" \
  --require-offboard-mode "$REQUIRE_OFFBOARD_MODE" \
  --require-armed "$REQUIRE_ARMED" \
  --max-wait-for-ready-sec "$MAX_WAIT_FOR_READY_SEC" \
  --stop-burst-sec "$STOP_BURST_SEC" \
  --confirm-wheels-lifted "$CONFIRM_WHEELS_LIFTED" \
  --confirm-rc-ready "$CONFIRM_RC_READY" \
  --confirm-param-backup "$CONFIRM_PARAM_BACKUP"
