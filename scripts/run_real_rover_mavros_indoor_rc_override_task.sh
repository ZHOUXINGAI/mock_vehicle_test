#!/usr/bin/env bash

set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

RC_OVERRIDE_LOG_DISABLE="${RC_OVERRIDE_LOG_DISABLE:-false}"
if [ "$RC_OVERRIDE_LOG_DISABLE" != "true" ] \
  && [ "${RC_OVERRIDE_LOG_ACTIVE:-false}" != "true" ]; then
  RC_OVERRIDE_LOG_ROOT="${RC_OVERRIDE_LOG_ROOT:-$REPO_DIR/results/rc_override}"
  RC_OVERRIDE_RUN_ID="${RC_OVERRIDE_RUN_ID:-$(date +%Y%m%d_%H%M%S)}"
  RC_OVERRIDE_LOG_DIR="$RC_OVERRIDE_LOG_ROOT/$RC_OVERRIDE_RUN_ID"
  RC_OVERRIDE_LOG_FILE="$RC_OVERRIDE_LOG_DIR/rc_override.log"
  mkdir -p "$RC_OVERRIDE_LOG_DIR"
  ln -sfn "$RC_OVERRIDE_LOG_DIR" "$RC_OVERRIDE_LOG_ROOT/latest"
  export RC_OVERRIDE_LOG_ACTIVE=true
  export RC_OVERRIDE_LOG_DIR
  export RC_OVERRIDE_LOG_FILE

  echo "Saving indoor RC override task log:"
  echo "  directory: $RC_OVERRIDE_LOG_DIR"
  echo "  file:      $RC_OVERRIDE_LOG_FILE"
  echo "  latest:    $RC_OVERRIDE_LOG_ROOT/latest"
  echo

  exec > >(tee -a "$RC_OVERRIDE_LOG_FILE") 2>&1
  echo "===== INDOOR RC OVERRIDE TASK LOG START $(date --iso-8601=seconds) ====="
  echo "cwd=$PWD"
  echo "command=$0 $*"
  echo
fi

cat <<'EOF'
Indoor MAVROS RC override no-GPS task.

Default behavior:
  - does not request OFFBOARD
  - does not request ARM
  - does not change PX4 parameters
  - expects MAVROS already connected to Pixhawk
  - expects RC transmitter ready, sticks centered, and wheels lifted
  - expects the vehicle to be in MANUAL and manually armed before motion starts
  - overrides only throttle/steering channels, then releases override at the end
EOF

MAVROS_NS="${MAVROS_NS:-/mavros}"
OVERRIDE_TOPIC="${OVERRIDE_TOPIC:-rc/override}"
TEST_SURFACE="${TEST_SURFACE:-wheels_lifted}"
ALLOWED_MODES="${ALLOWED_MODES:-MANUAL}"
COMMAND_RATE_HZ="${COMMAND_RATE_HZ:-20}"
WARMUP_SEC="${WARMUP_SEC:-1.0}"
STOP_SEC="${STOP_SEC:-1.0}"
FORWARD_SEC="${FORWARD_SEC:-1.0}"
BACKWARD_SEC="${BACKWARD_SEC:-1.0}"
TURN_SEC="${TURN_SEC:-0.5}"
FINAL_STOP_SEC="${FINAL_STOP_SEC:-2.0}"
THROTTLE_CHANNEL="${THROTTLE_CHANNEL:-2}"
STEERING_CHANNEL="${STEERING_CHANNEL:-4}"
NEUTRAL_PWM_US="${NEUTRAL_PWM_US:-1500}"
FORWARD_DELTA_US="${FORWARD_DELTA_US:-150}"
TURN_DELTA_US="${TURN_DELTA_US:-150}"
FORWARD_SIGN="${FORWARD_SIGN:-1}"
TURN_SIGN="${TURN_SIGN:-1}"
MAX_DELTA_US="${MAX_DELTA_US:-180}"
MIN_PWM_US="${MIN_PWM_US:-1100}"
MAX_PWM_US="${MAX_PWM_US:-1900}"
REQUIRE_CONNECTED="true"
REQUIRE_ARMED="true"
ABORT_ON_MODE_EXIT="true"
ABORT_ON_DISARM="true"
MAX_WAIT_FOR_READY_SEC="${MAX_WAIT_FOR_READY_SEC:-60}"
RELEASE_BURST_SEC="${RELEASE_BURST_SEC:-1.0}"

cat <<EOF
Effective indoor RC override settings:
  TEST_SURFACE=$TEST_SURFACE
  ALLOWED_MODES=$ALLOWED_MODES
  THROTTLE_CHANNEL=$THROTTLE_CHANNEL
  STEERING_CHANNEL=$STEERING_CHANNEL
  NEUTRAL_PWM_US=$NEUTRAL_PWM_US
  FORWARD_DELTA_US=$FORWARD_DELTA_US
  TURN_DELTA_US=$TURN_DELTA_US
  MAX_WAIT_FOR_READY_SEC=$MAX_WAIT_FOR_READY_SEC
EOF

# shellcheck disable=SC1091
source "$REPO_DIR/scripts/env.sh"

mkdir -p "$REPO_DIR/results/ros_logs"
export ROS_LOG_DIR="${ROS_LOG_DIR:-$REPO_DIR/results/ros_logs}"

if ! ros2 pkg prefix mavros >/dev/null 2>&1; then
  echo "MAVROS is not installed in the current ROS 2 environment." >&2
  echo "Run: ./scripts/install_mavros_humble.sh" >&2
  exit 1
fi

python3 "$REPO_DIR/src/real_rover_mavros_rc_override_smoke.py" \
  --ros-args \
  -p mavros_namespace:="'$MAVROS_NS'" \
  -p override_topic:="'$OVERRIDE_TOPIC'" \
  -p command_rate_hz:="$COMMAND_RATE_HZ" \
  -p warmup_sec:="$WARMUP_SEC" \
  -p stop_sec:="$STOP_SEC" \
  -p forward_sec:="$FORWARD_SEC" \
  -p backward_sec:="$BACKWARD_SEC" \
  -p turn_sec:="$TURN_SEC" \
  -p final_stop_sec:="$FINAL_STOP_SEC" \
  -p throttle_channel:="$THROTTLE_CHANNEL" \
  -p steering_channel:="$STEERING_CHANNEL" \
  -p neutral_pwm_us:="$NEUTRAL_PWM_US" \
  -p forward_delta_us:="$FORWARD_DELTA_US" \
  -p turn_delta_us:="$TURN_DELTA_US" \
  -p forward_sign:="$FORWARD_SIGN" \
  -p turn_sign:="$TURN_SIGN" \
  -p max_delta_us:="$MAX_DELTA_US" \
  -p min_pwm_us:="$MIN_PWM_US" \
  -p max_pwm_us:="$MAX_PWM_US" \
  -p allowed_modes:="'$ALLOWED_MODES'" \
  -p require_connected:="$REQUIRE_CONNECTED" \
  -p require_armed:="$REQUIRE_ARMED" \
  -p abort_on_mode_exit:="$ABORT_ON_MODE_EXIT" \
  -p abort_on_disarm:="$ABORT_ON_DISARM" \
  -p max_wait_for_ready_sec:="$MAX_WAIT_FOR_READY_SEC" \
  -p release_burst_sec:="$RELEASE_BURST_SEC" \
  -p test_surface:="'$TEST_SURFACE'" \
  -p confirm_wheels_lifted:="${CONFIRM_WHEELS_LIFTED:-false}" \
  -p confirm_ground_area_clear:="${CONFIRM_GROUND_AREA_CLEAR:-false}" \
  -p confirm_low_speed_ground_test:="${CONFIRM_LOW_SPEED_GROUND_TEST:-false}" \
  -p confirm_rc_ready:="${CONFIRM_RC_READY:-false}" \
  -p confirm_rc_sticks_centered:="${CONFIRM_RC_STICKS_CENTERED:-false}" \
  -p confirm_param_backup:="${CONFIRM_PARAM_BACKUP:-false}"
