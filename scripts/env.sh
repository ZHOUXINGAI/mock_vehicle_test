#!/usr/bin/env bash

set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export MOCK_VEHICLE_REPO_DIR="$REPO_DIR"

ROS_DISTRO="${ROS_DISTRO:-humble}"
if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
  # shellcheck disable=SC1090
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
fi

if [ -f "/home/hw/easydocking/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "/home/hw/easydocking/install/setup.bash"
fi

if [ -f "$REPO_DIR/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "$REPO_DIR/install/setup.bash"
fi

export PX4_AUTOPILOT_PATH="${PX4_AUTOPILOT_PATH:-/home/hw/PX4-Autopilot}"
export UXRCE_AGENT_BIN="${UXRCE_AGENT_BIN:-/home/hw/uxrce_agent_ws/install/microxrcedds_agent/bin/MicroXRCEAgent}"
