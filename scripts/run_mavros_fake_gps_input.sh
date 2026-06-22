#!/usr/bin/env bash

set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

FAKE_GPS_LOG_DISABLE="${FAKE_GPS_LOG_DISABLE:-false}"
if [ "$FAKE_GPS_LOG_DISABLE" != "true" ] \
  && [ "${FAKE_GPS_LOG_ACTIVE:-false}" != "true" ]; then
  FAKE_GPS_LOG_ROOT="${FAKE_GPS_LOG_ROOT:-$REPO_DIR/results/fake_gps}"
  FAKE_GPS_RUN_ID="${FAKE_GPS_RUN_ID:-$(date +%Y%m%d_%H%M%S)}"
  FAKE_GPS_LOG_DIR="$FAKE_GPS_LOG_ROOT/$FAKE_GPS_RUN_ID"
  FAKE_GPS_LOG_FILE="$FAKE_GPS_LOG_DIR/fake_gps.log"
  mkdir -p "$FAKE_GPS_LOG_DIR"
  ln -sfn "$FAKE_GPS_LOG_DIR" "$FAKE_GPS_LOG_ROOT/latest"
  export FAKE_GPS_LOG_ACTIVE=true
  export FAKE_GPS_LOG_DIR
  export FAKE_GPS_LOG_FILE

  echo "Saving MAVROS fake GPS_INPUT log:"
  echo "  directory: $FAKE_GPS_LOG_DIR"
  echo "  file:      $FAKE_GPS_LOG_FILE"
  echo "  latest:    $FAKE_GPS_LOG_ROOT/latest"
  echo

  exec > >(tee -a "$FAKE_GPS_LOG_FILE") 2>&1
  echo "===== MAVROS FAKE GPS_INPUT LOG START $(date --iso-8601=seconds) ====="
  echo "cwd=$PWD"
  echo "command=$0 $*"
  echo
fi

cat <<'EOF'
MAVROS fake GPS_INPUT publisher.

Default behavior:
  - publishes a fixed GPS_INPUT location to /mavros/gps_input/gps_input
  - watches /mavros/global_position/global and /mavros/local_position/pose
  - does not arm, disarm, change mode, or change PX4 parameters
  - refuses to start if the vehicle is already armed

Use:
  - wheels lifted
  - QGC connected
  - MAVROS running
  - vehicle disarmed before startup
EOF

MAVROS_NS="${MAVROS_NS:-/mavros}"
MAVROS_NS="${MAVROS_NS%/}"
STATE_TOPIC="${STATE_TOPIC:-$MAVROS_NS/state}"
STATE_TIMEOUT_SEC="${STATE_TIMEOUT_SEC:-6s}"
DISCOVERY_TIMEOUT_SEC="${DISCOVERY_TIMEOUT_SEC:-4s}"

RATE_HZ="${RATE_HZ:-10}"
DURATION_SEC="${DURATION_SEC:-0}"
LAT_DEG="${LAT_DEG:-22.41674}"
LON_DEG="${LON_DEG:-114.04280}"
ALT_M="${ALT_M:-20.0}"
FIX_TYPE="${FIX_TYPE:-3}"
SATELLITES_VISIBLE="${SATELLITES_VISIBLE:-14}"
HDOP="${HDOP:-0.7}"
VDOP="${VDOP:-0.9}"
SPEED_ACCURACY="${SPEED_ACCURACY:-0.1}"
HORIZ_ACCURACY="${HORIZ_ACCURACY:-0.8}"
VERT_ACCURACY="${VERT_ACCURACY:-1.2}"
YAW_CDEG="${YAW_CDEG:-0}"
GPS_ID="${GPS_ID:-0}"

CONFIRM_WHEELS_LIFTED="${CONFIRM_WHEELS_LIFTED:-false}"
CONFIRM_VEHICLE_DISARMED="${CONFIRM_VEHICLE_DISARMED:-false}"
CONFIRM_RC_READY="${CONFIRM_RC_READY:-false}"
CONFIRM_PARAM_BACKUP="${CONFIRM_PARAM_BACKUP:-false}"
CONFIRM_FAKE_GPS_ONLY="${CONFIRM_FAKE_GPS_ONLY:-false}"

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
if [ "$CONFIRM_FAKE_GPS_ONLY" != "true" ]; then
  missing_confirmations+=("CONFIRM_FAKE_GPS_ONLY=true")
fi
if [ "${#missing_confirmations[@]}" -gt 0 ]; then
  {
    echo "Refusing to start fake GPS_INPUT publisher."
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
echo "Checking MAVROS state before fake GPS_INPUT..."
state_snapshot="$(timeout "$STATE_TIMEOUT_SEC" ros2 topic echo --once "$STATE_TOPIC" || true)"
if [ -z "$state_snapshot" ]; then
  echo "Could not read $STATE_TOPIC; refusing to start fake GPS_INPUT." >&2
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
    echo "Refusing to start fake GPS_INPUT while the vehicle is armed." >&2
    exit 2
    ;;
esac

echo
cat <<EOF
Effective fake GPS_INPUT settings:
  MAVROS_NS=$MAVROS_NS
  RATE_HZ=$RATE_HZ
  DURATION_SEC=$DURATION_SEC
  LAT_DEG=$LAT_DEG
  LON_DEG=$LON_DEG
  ALT_M=$ALT_M
  FIX_TYPE=$FIX_TYPE
  SATELLITES_VISIBLE=$SATELLITES_VISIBLE
  HORIZ_ACCURACY=$HORIZ_ACCURACY
  VERT_ACCURACY=$VERT_ACCURACY

Expected success signal:
  snapshot ... gps_input_subs=1 global=<lat,lon,alt> local=x=... y=... z=...
EOF

mkdir -p "$REPO_DIR/results/ros_logs"
export ROS_LOG_DIR="${ROS_LOG_DIR:-$REPO_DIR/results/ros_logs}"

python3 "$REPO_DIR/src/mavros_fake_gps_input.py" \
  --ros-args \
  -p mavros_namespace:="'$MAVROS_NS'" \
  -p rate_hz:="$RATE_HZ" \
  -p duration_sec:="$DURATION_SEC" \
  -p lat_deg:="$LAT_DEG" \
  -p lon_deg:="$LON_DEG" \
  -p alt_m:="$ALT_M" \
  -p fix_type:="$FIX_TYPE" \
  -p satellites_visible:="$SATELLITES_VISIBLE" \
  -p hdop:="$HDOP" \
  -p vdop:="$VDOP" \
  -p speed_accuracy:="$SPEED_ACCURACY" \
  -p horiz_accuracy:="$HORIZ_ACCURACY" \
  -p vert_accuracy:="$VERT_ACCURACY" \
  -p yaw_cdeg:="$YAW_CDEG" \
  -p gps_id:="$GPS_ID"
