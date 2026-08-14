#!/usr/bin/env bash

set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# Orin1 keeps physical MAV_SYS_ID=1 even though its docking role is now Mini.
# It also needs a higher velocity-request floor than Orin2: 0.05 m/s produced
# MAIN 1540 us and only about 106/255 after its measured 35 us Arduino deadband.
export EXPECTED_SYSTEM_ID=1
export EXPECTED_PWM_MAIN_REV=0
export EXPECTED_RO_YAW_RATE_TH=0.5
export EXPECTED_RO_YAW_RATE_CORR=4.0
export REFERENCE_MODE="${REFERENCE_MODE:-initial_yaw}"
export RVIZ_TOPIC_PREFIX="${RVIZ_TOPIC_PREFIX:-/orin1/offboard}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-99}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export ARDUINO_DEVICE=/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0
export BODY_FORWARD_SIGN=1.0
export LINEAR_SPEED_MPS="${LINEAR_SPEED_MPS:-0.12}"
export MAX_LINEAR_SPEED_MPS="${MAX_LINEAR_SPEED_MPS:-0.22}"
export COURSE_CALIBRATION_SPEED_MPS="${COURSE_CALIBRATION_SPEED_MPS:-0.10}"
export COURSE_CALIBRATION_MAX_SEC="${COURSE_CALIBRATION_MAX_SEC:-8.0}"
export TERMINAL_SPEED_MPS="${TERMINAL_SPEED_MPS:-0.10}"
export TURN_FORWARD_SPEED_MPS="${TURN_FORWARD_SPEED_MPS:-0.12}"
export TRACKER_CURVATURE_SLOWDOWN_GAIN="${TRACKER_CURVATURE_SLOWDOWN_GAIN:-0.0}"
# The 2026-08-08 ULog measured 0.8-0.95 m/s ground speed for a 0.15 request.
# A 3.5 m virtual wheelbase consequently commanded about 49 degrees on a 3 m
# arc and produced an approximately 1 m physical turn.  Keep path geometry
# generic and calibrate only this Orin1 BODY_NED adapter.
export TRACKER_BASE_LOOKAHEAD_M="${TRACKER_BASE_LOOKAHEAD_M:-1.20}"
export TRACKER_MIN_LOOKAHEAD_M="${TRACKER_MIN_LOOKAHEAD_M:-1.00}"
export TRACKER_MAX_LOOKAHEAD_M="${TRACKER_MAX_LOOKAHEAD_M:-2.00}"
export TRACKER_MAX_BODY_BEARING_DEG="${TRACKER_MAX_BODY_BEARING_DEG:-25.0}"
export TRACKER_MAX_BODY_BEARING_RATE_DEGPS="${TRACKER_MAX_BODY_BEARING_RATE_DEGPS:-45.0}"
export TRACKER_CURVATURE_TO_BODY_GAIN_M="${TRACKER_CURVATURE_TO_BODY_GAIN_M:-0.94}"
export TRACKER_CURVATURE_FEEDBACK_GAIN_RATIO="${TRACKER_CURVATURE_FEEDBACK_GAIN_RATIO:-1.28}"
# The 2026-08-08 21:25 U-turn exited the R=3 m arc with about 8-9 degrees
# of heading error because the former +/-0.12 1/m feedback bound could not
# sufficiently cancel the -0.332 1/m curve feed-forward.  The BODY bearing
# remains independently limited to 25 degrees; this larger feedback budget
# lets arbitrary trajectories straighten promptly after curvature changes.
export TRACKER_MAX_CURVATURE_CORRECTION_INV_M="${TRACKER_MAX_CURVATURE_CORRECTION_INV_M:-0.24}"
export TRACKER_CROSS_TRACK_INTEGRAL_GAIN_INV_M_PER_M_SEC="${TRACKER_CROSS_TRACK_INTEGRAL_GAIN_INV_M_PER_M_SEC:-0.04}"
export TRACKER_CROSS_TRACK_INTEGRAL_LIMIT_M_SEC="${TRACKER_CROSS_TRACK_INTEGRAL_LIMIT_M_SEC:-1.0}"
export TRACKER_MAX_REFERENCE_CURVATURE_RATE_INV_M2="${TRACKER_MAX_REFERENCE_CURVATURE_RATE_INV_M2:-1.0}"
export RUN_ID_PREFIX=orin1

exec "$REPO_DIR/scripts/run_orin2_outdoor_forward_5m.sh" "$@"
