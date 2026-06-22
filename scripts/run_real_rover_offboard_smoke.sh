#!/usr/bin/env bash

set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

PX4_INSTANCE="${PX4_INSTANCE:-1}"
PX4_NAMESPACE="${PX4_NAMESPACE:-/px4_${PX4_INSTANCE}}"
VEHICLE_ID="${VEHICLE_ID:-$PX4_INSTANCE}"

COMMAND_MODE="${COMMAND_MODE:-velocity}"
DIRECT_ACTUATOR_TOPIC="${DIRECT_ACTUATOR_TOPIC:-motors}"
COMMAND_RATE_HZ="${COMMAND_RATE_HZ:-20}"
WARMUP_SEC="${WARMUP_SEC:-1.0}"
STOP_SEC="${STOP_SEC:-1.0}"
FORWARD_SEC="${FORWARD_SEC:-1.0}"
BACKWARD_SEC="${BACKWARD_SEC:-1.0}"
TURN_SEC="${TURN_SEC:-0.5}"
FINAL_STOP_SEC="${FINAL_STOP_SEC:-2.0}"

LINEAR_SPEED_MPS="${LINEAR_SPEED_MPS:-0.12}"
TURN_YAW_RATE_RADPS="${TURN_YAW_RATE_RADPS:-0.25}"
TURN_WITH_LINEAR_MPS="${TURN_WITH_LINEAR_MPS:-0.0}"
LINEAR_ACTUATOR="${LINEAR_ACTUATOR:-0.12}"
TURN_ACTUATOR="${TURN_ACTUATOR:-0.10}"
MAX_LINEAR_SPEED_MPS="${MAX_LINEAR_SPEED_MPS:-0.30}"
MAX_YAW_RATE_RADPS="${MAX_YAW_RATE_RADPS:-0.70}"
MAX_ACTUATOR_ABS="${MAX_ACTUATOR_ABS:-0.25}"
TURN_SIGN="${TURN_SIGN:-1.0}"

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
    echo "Refusing to start real rover Offboard smoke test."
    echo "Required confirmations:"
    echo "  CONFIRM_WHEELS_LIFTED=true"
    echo "  CONFIRM_RC_READY=true"
    echo "  CONFIRM_PARAM_BACKUP=true"
  } >&2
  exit 2
fi

export ROS_LOG_DIR="${ROS_LOG_DIR:-$REPO_DIR/results/ros_logs}"
mkdir -p "$ROS_LOG_DIR"

# shellcheck disable=SC1091
source "$REPO_DIR/scripts/env.sh"

python3 "$REPO_DIR/src/real_rover_offboard_smoke.py" \
  --ros-args \
  -p px4_namespace:="$PX4_NAMESPACE" \
  -p vehicle_id:="$VEHICLE_ID" \
  -p command_mode:="$COMMAND_MODE" \
  -p direct_actuator_topic:="$DIRECT_ACTUATOR_TOPIC" \
  -p command_rate_hz:="$COMMAND_RATE_HZ" \
  -p warmup_sec:="$WARMUP_SEC" \
  -p stop_sec:="$STOP_SEC" \
  -p forward_sec:="$FORWARD_SEC" \
  -p backward_sec:="$BACKWARD_SEC" \
  -p turn_sec:="$TURN_SEC" \
  -p final_stop_sec:="$FINAL_STOP_SEC" \
  -p linear_speed_mps:="$LINEAR_SPEED_MPS" \
  -p turn_yaw_rate_radps:="$TURN_YAW_RATE_RADPS" \
  -p turn_with_linear_mps:="$TURN_WITH_LINEAR_MPS" \
  -p linear_actuator:="$LINEAR_ACTUATOR" \
  -p turn_actuator:="$TURN_ACTUATOR" \
  -p max_linear_speed_mps:="$MAX_LINEAR_SPEED_MPS" \
  -p max_yaw_rate_radps:="$MAX_YAW_RATE_RADPS" \
  -p max_actuator_abs:="$MAX_ACTUATOR_ABS" \
  -p turn_sign:="$TURN_SIGN" \
  -p mode_change_on_start:="$MODE_CHANGE_ON_START" \
  -p arm_on_start:="$ARM_ON_START" \
  -p disarm_on_finish:="$DISARM_ON_FINISH" \
  -p require_offboard_mode:="$REQUIRE_OFFBOARD_MODE" \
  -p require_armed:="$REQUIRE_ARMED" \
  -p max_wait_for_ready_sec:="$MAX_WAIT_FOR_READY_SEC" \
  -p stop_burst_sec:="$STOP_BURST_SEC" \
  -p confirm_wheels_lifted:="$CONFIRM_WHEELS_LIFTED" \
  -p confirm_rc_ready:="$CONFIRM_RC_READY" \
  -p confirm_param_backup:="$CONFIRM_PARAM_BACKUP"
