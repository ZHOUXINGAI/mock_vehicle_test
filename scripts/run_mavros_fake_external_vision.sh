#!/usr/bin/env bash

set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

FAKE_EV_LOG_DISABLE="${FAKE_EV_LOG_DISABLE:-false}"
if [ "$FAKE_EV_LOG_DISABLE" != "true" ] \
  && [ "${FAKE_EV_LOG_ACTIVE:-false}" != "true" ]; then
  FAKE_EV_LOG_ROOT="${FAKE_EV_LOG_ROOT:-$REPO_DIR/results/fake_external_vision}"
  FAKE_EV_RUN_ID="${FAKE_EV_RUN_ID:-$(date +%Y%m%d_%H%M%S)}"
  FAKE_EV_LOG_DIR="$FAKE_EV_LOG_ROOT/$FAKE_EV_RUN_ID"
  FAKE_EV_LOG_FILE="$FAKE_EV_LOG_DIR/fake_external_vision.log"
  mkdir -p "$FAKE_EV_LOG_DIR"
  ln -sfn "$FAKE_EV_LOG_DIR" "$FAKE_EV_LOG_ROOT/latest"
  export FAKE_EV_LOG_ACTIVE=true
  export FAKE_EV_LOG_DIR
  export FAKE_EV_LOG_FILE

  echo "Saving MAVROS fake external-vision log:"
  echo "  directory: $FAKE_EV_LOG_DIR"
  echo "  file:      $FAKE_EV_LOG_FILE"
  echo "  latest:    $FAKE_EV_LOG_ROOT/latest"
  echo

  exec > >(tee -a "$FAKE_EV_LOG_FILE") 2>&1
  echo "===== MAVROS FAKE EXTERNAL VISION LOG START $(date --iso-8601=seconds) ====="
  echo "cwd=$PWD"
  echo "command=$0 $*"
  echo
fi

cat <<'EOF'
MAVROS fake external-vision publisher.

Default behavior:
  - publishes fixed local pose to /mavros/vision_pose/pose
  - publishes pose covariance to /mavros/vision_pose/pose_cov
  - publishes zero-velocity odometry to /mavros/odometry/out
  - seeds the fake pose from current /mavros/local_position/pose when available
  - does not arm, disarm, change mode, or change PX4 parameters
  - refuses to start if the vehicle is already armed

This is the indoor equivalent of VICON/camera/lidar local pose injection.
EOF

MAVROS_NS="${MAVROS_NS:-/mavros}"
MAVROS_NS="${MAVROS_NS%/}"
STATE_TOPIC="${STATE_TOPIC:-$MAVROS_NS/state}"
PARAM_NODE="${PARAM_NODE:-$MAVROS_NS/param}"
PARAM_PULL_SERVICE="${PARAM_PULL_SERVICE:-$MAVROS_NS/param/pull}"
STATE_TIMEOUT_SEC="${STATE_TIMEOUT_SEC:-6s}"
PARAM_TIMEOUT_SEC="${PARAM_TIMEOUT_SEC:-12s}"

RATE_HZ="${RATE_HZ:-30}"
DURATION_SEC="${DURATION_SEC:-0}"
USE_CURRENT_LOCAL_POSE="${USE_CURRENT_LOCAL_POSE:-true}"
CURRENT_POSE_WAIT_SEC="${CURRENT_POSE_WAIT_SEC:-5}"
PUBLISH_VISION_POSE="${PUBLISH_VISION_POSE:-true}"
PUBLISH_VISION_POSE_COV="${PUBLISH_VISION_POSE_COV:-true}"
PUBLISH_ODOMETRY="${PUBLISH_ODOMETRY:-true}"
POSE_COVARIANCE="${POSE_COVARIANCE:-0.02}"
ORIENTATION_COVARIANCE="${ORIENTATION_COVARIANCE:-0.02}"
VELOCITY_COVARIANCE="${VELOCITY_COVARIANCE:-0.05}"

CONFIRM_WHEELS_LIFTED="${CONFIRM_WHEELS_LIFTED:-false}"
CONFIRM_VEHICLE_DISARMED="${CONFIRM_VEHICLE_DISARMED:-false}"
CONFIRM_RC_READY="${CONFIRM_RC_READY:-false}"
CONFIRM_PARAM_BACKUP="${CONFIRM_PARAM_BACKUP:-false}"
CONFIRM_FAKE_LOCAL_POSITION_ONLY="${CONFIRM_FAKE_LOCAL_POSITION_ONLY:-false}"

missing_confirmations=()
if [ "$CONFIRM_WHEELS_LIFTED" != "true" ]; then
  missing_confirmations+=("CONFIRM_WHEELS_LIFTED=true")
fi
if [ "$CONFIRM_VEHICLE_DISARMED" != "true" ]; then
  missing_confirmations+=("CONFIRM_VEHICLE_DISARMED=true")
fi
if [ "$CONFIRM_RC_READY" != "true" ]; then
  missing_confirmations+=("CONFIRM_RC_READY=true")
fi
if [ "$CONFIRM_PARAM_BACKUP" != "true" ]; then
  missing_confirmations+=("CONFIRM_PARAM_BACKUP=true")
fi
if [ "$CONFIRM_FAKE_LOCAL_POSITION_ONLY" != "true" ]; then
  missing_confirmations+=("CONFIRM_FAKE_LOCAL_POSITION_ONLY=true")
fi
if [ "${#missing_confirmations[@]}" -gt 0 ]; then
  {
    echo "Refusing to start fake external-vision publisher."
    echo "Required confirmations:"
    for item in "${missing_confirmations[@]}"; do
      echo "  $item"
    done
  } >&2
  exit 2
fi

# shellcheck disable=SC1091
source "$REPO_DIR/scripts/env.sh"

if ! ros2 topic list --spin-time 2 | grep -Fxq "$STATE_TOPIC"; then
  echo "MAVROS state topic was not discovered: $STATE_TOPIC" >&2
  echo "Start MAVROS first: ./scripts/run_mavros_px4_usb_to_qgc_logged.sh" >&2
  exit 1
fi

echo
echo "Checking MAVROS state before fake external vision..."
state_snapshot="$(timeout "$STATE_TIMEOUT_SEC" ros2 topic echo --once "$STATE_TOPIC" || true)"
if [ -z "$state_snapshot" ]; then
  echo "Could not read $STATE_TOPIC; refusing to start fake external vision." >&2
  exit 1
fi
state_connected="$(printf '%s\n' "$state_snapshot" | awk '$1 == "connected:" {print $2; exit}')"
state_armed="$(printf '%s\n' "$state_snapshot" | awk '$1 == "armed:" {print $2; exit}')"
state_manual_input="$(printf '%s\n' "$state_snapshot" | awk '$1 == "manual_input:" {print $2; exit}')"
state_mode="$(printf '%s\n' "$state_snapshot" | sed -n 's/^mode: //p' | head -n 1)"
echo "Current MAVROS state:"
echo "  connected=${state_connected:-unknown}"
echo "  armed=${state_armed:-unknown}"
echo "  manual_input=${state_manual_input:-unknown}"
echo "  mode=${state_mode:-unknown}"

case "$state_armed" in
  False|false) ;;
  *)
    echo "Refusing to start fake external vision while the vehicle is armed." >&2
    exit 2
    ;;
esac

echo
echo "Checking external-vision EKF parameters..."
timeout "$PARAM_TIMEOUT_SEC" ros2 service call \
  "$PARAM_PULL_SERVICE" \
  mavros_msgs/srv/ParamPull \
  "{force_pull: false}" >/dev/null
for param in EKF2_EV_CTRL EKF2_HGT_REF COM_ARM_WO_GPS COM_ARM_EKF_POS COM_ARM_EKF_VEL COM_ARM_EKF_YAW; do
  echo "===== $param ====="
  timeout "$PARAM_TIMEOUT_SEC" ros2 param get "$PARAM_NODE" "$param" || true
done

cat <<EOF

Effective fake external-vision settings:
  MAVROS_NS=$MAVROS_NS
  RATE_HZ=$RATE_HZ
  DURATION_SEC=$DURATION_SEC
  USE_CURRENT_LOCAL_POSE=$USE_CURRENT_LOCAL_POSE
  PUBLISH_VISION_POSE=$PUBLISH_VISION_POSE
  PUBLISH_VISION_POSE_COV=$PUBLISH_VISION_POSE_COV
  PUBLISH_ODOMETRY=$PUBLISH_ODOMETRY

Expected success signal:
  snapshot ... subs=vision:1 vision_cov:1 odom:1 published=x=... local=x=...
EOF

mkdir -p "$REPO_DIR/results/ros_logs"
export ROS_LOG_DIR="${ROS_LOG_DIR:-$REPO_DIR/results/ros_logs}"

python3 "$REPO_DIR/src/mavros_fake_external_vision.py" \
  --ros-args \
  -p mavros_namespace:="'$MAVROS_NS'" \
  -p rate_hz:="$RATE_HZ" \
  -p duration_sec:="$DURATION_SEC" \
  -p use_current_local_pose:="$USE_CURRENT_LOCAL_POSE" \
  -p current_pose_wait_sec:="$CURRENT_POSE_WAIT_SEC" \
  -p publish_vision_pose:="$PUBLISH_VISION_POSE" \
  -p publish_vision_pose_cov:="$PUBLISH_VISION_POSE_COV" \
  -p publish_odometry:="$PUBLISH_ODOMETRY" \
  -p pose_covariance:="$POSE_COVARIANCE" \
  -p orientation_covariance:="$ORIENTATION_COVARIANCE" \
  -p velocity_covariance:="$VELOCITY_COVARIANCE"
