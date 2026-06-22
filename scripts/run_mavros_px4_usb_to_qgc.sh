#!/usr/bin/env bash

set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

MAVLINK_DEVICE="${MAVLINK_DEVICE:-/dev/serial/by-id/usb-Auterion_PX4_FMU_v6C.x_0-if00}"
if [ ! -e "$MAVLINK_DEVICE" ] && [ -e /dev/ttyACM0 ]; then
  MAVLINK_DEVICE=/dev/ttyACM0
fi

MAVLINK_BAUD="${MAVLINK_BAUD:-115200}"
MAVROS_NS="${MAVROS_NS:-mavros}"
TARGET_SYSTEM="${TARGET_SYSTEM:-1}"
TARGET_COMPONENT="${TARGET_COMPONENT:-1}"
QGC_UDP_URL="${QGC_UDP_URL:-udp://:14555@127.0.0.1:14550}"
MAVROS_DISABLE_PARAM_PLUGIN="${MAVROS_DISABLE_PARAM_PLUGIN:-true}"
MAVROS_CONFIG_YAML="${MAVROS_CONFIG_YAML:-/opt/ros/humble/share/mavros/launch/px4_config.yaml}"
if [ "$MAVROS_DISABLE_PARAM_PLUGIN" = "true" ]; then
  MAVROS_PLUGINLISTS_YAML="${MAVROS_PLUGINLISTS_YAML:-$REPO_DIR/config/mavros_px4_pluginlists_no_param.yaml}"
else
  MAVROS_PLUGINLISTS_YAML="${MAVROS_PLUGINLISTS_YAML:-/opt/ros/humble/share/mavros/launch/px4_pluginlists.yaml}"
fi

# shellcheck disable=SC1091
source "$REPO_DIR/scripts/env.sh"

if ! ros2 pkg prefix mavros >/dev/null 2>&1; then
  echo "MAVROS is not installed in the current ROS 2 environment." >&2
  echo "Run: ./scripts/install_mavros_humble.sh" >&2
  exit 1
fi

if [ ! -e "$MAVLINK_DEVICE" ]; then
  echo "Pixhawk MAVLink device not found: $MAVLINK_DEVICE" >&2
  exit 1
fi

echo "Starting MAVROS:"
echo "  FCU: $MAVLINK_DEVICE @ $MAVLINK_BAUD"
echo "  QGC UDP: $QGC_UDP_URL"
echo "  namespace: $MAVROS_NS"
echo "  pluginlist: $MAVROS_PLUGINLISTS_YAML"
echo "  config: $MAVROS_CONFIG_YAML"
echo "  param plugin disabled: $MAVROS_DISABLE_PARAM_PLUGIN"
echo
echo "Start QGroundControl after this. QGC should listen on UDP 14550 and not open Pixhawk USB."

exec ros2 launch mavros node.launch \
  pluginlists_yaml:="$MAVROS_PLUGINLISTS_YAML" \
  config_yaml:="$MAVROS_CONFIG_YAML" \
  fcu_url:="serial://${MAVLINK_DEVICE}:${MAVLINK_BAUD}" \
  gcs_url:="$QGC_UDP_URL" \
  tgt_system:="$TARGET_SYSTEM" \
  tgt_component:="$TARGET_COMPONENT" \
  namespace:="$MAVROS_NS"
