#!/usr/bin/env bash

set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export MOCK_VEHICLE_REPO_DIR="$REPO_DIR"

source_ros_setup() {
  local setup_file="$1"
  if [ -z "$setup_file" ] || [ ! -f "$setup_file" ]; then
    return
  fi

  local nounset_was_on=0
  case "$-" in
    *u*)
      nounset_was_on=1
      set +u
      ;;
  esac

  # shellcheck disable=SC1090
  source "$setup_file"

  if [ "$nounset_was_on" -eq 1 ]; then
    set -u
  fi
}

ROS_DISTRO="${ROS_DISTRO:-humble}"
if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
  source_ros_setup "/opt/ros/${ROS_DISTRO}/setup.bash"
fi

for setup_file in \
  "/home/hw/easydocking/install/setup.bash" \
  "/home/jetson/easydocking/install/setup.bash" \
  "/home/jetson/yahboom_ws/install/setup.bash" \
  "${EXTRA_ROS_SETUP:-}"
do
  source_ros_setup "$setup_file"
done

source_ros_setup "$REPO_DIR/install/setup.bash"

export PX4_AUTOPILOT_PATH="${PX4_AUTOPILOT_PATH:-/home/hw/PX4-Autopilot}"
export UXRCE_AGENT_BIN="${UXRCE_AGENT_BIN:-/home/hw/uxrce_agent_ws/install/microxrcedds_agent/bin/MicroXRCEAgent}"
