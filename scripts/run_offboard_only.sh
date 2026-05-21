#!/usr/bin/env bash

set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PX4_INSTANCE="${PX4_INSTANCE:-1}"
PX4_NAMESPACE="${PX4_NAMESPACE:-/px4_${PX4_INSTANCE}}"
MISSION_MODE="${MISSION_MODE:-position}"
TRAVEL_DISTANCE_M="${TRAVEL_DISTANCE_M:-3.0}"
FENCE_RADIUS_M="${FENCE_RADIUS_M:-10.0}"
FORWARD_SPEED_MPS="${FORWARD_SPEED_MPS:-0.7}"
RETURN_SPEED_MPS="${RETURN_SPEED_MPS:-0.5}"
ARM_ON_START="${ARM_ON_START:-true}"

# shellcheck disable=SC1091
source "$REPO_DIR/scripts/env.sh"

python3 "$REPO_DIR/src/mock_rover_offboard.py" \
  --ros-args \
  -p px4_namespace:="$PX4_NAMESPACE" \
  -p vehicle_id:="$PX4_INSTANCE" \
  -p arm_on_start:="$ARM_ON_START" \
  -p mission_mode:="$MISSION_MODE" \
  -p travel_distance_m:="$TRAVEL_DISTANCE_M" \
  -p fence_radius_m:="$FENCE_RADIUS_M" \
  -p forward_speed_mps:="$FORWARD_SPEED_MPS" \
  -p return_speed_mps:="$RETURN_SPEED_MPS"
