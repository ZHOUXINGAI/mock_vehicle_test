#!/usr/bin/env bash

set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# Orin1 uses semantic BODY_NED +x for forward. PWM_MAIN_REV=0 corrects the
# actuator-output sign; RC2_REV/RC4_REV preserve the verified manual controls.
LINEAR_DIRECTION_SIGN=1.0 \
FORWARD_SEC="${FORWARD_SEC:-1.0}" \
LINEAR_SPEED_MPS="${LINEAR_SPEED_MPS:-0.06}" \
MAX_LINEAR_SPEED_MPS="${MAX_LINEAR_SPEED_MPS:-0.10}" \
DIFF_OFFBOARD_FORWARD_LOG_ROOT="${DIFF_OFFBOARD_FORWARD_LOG_ROOT:-$REPO_DIR/results/orin1_differential_offboard_forward_mapping}" \
  exec "$REPO_DIR/scripts/run_real_rover_mavros_differential_offboard_forward_mapping.sh"

exit=0
