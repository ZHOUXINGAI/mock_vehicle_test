#!/usr/bin/env bash
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-99}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
source "$REPO_DIR/scripts/env.sh"

exec rviz2 -d "$REPO_DIR/config/rviz/orin2_offboard_trajectory.rviz"
