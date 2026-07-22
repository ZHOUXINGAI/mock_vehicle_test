#!/usr/bin/env bash

set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEVICE="${MINI_PIXHAWK_DEVICE:-/dev/serial/by-id/usb-Auterion_PX4_FMU_v6C.x_0-if00}"
MODE="${1:-check}"

PARAM_TOOL=(
  python3 "$REPO_DIR/scripts/px4_mavlink_param.py"
  --device "$DEVICE"
  --baud 115200
  --expect-system-id 2
  --expect-component-id 1
)

PARAMS=(
  MAV_SYS_ID
  MAV_PROTO_VER
  MAV_1_CONFIG
  MAV_1_MODE
  MAV_1_FORWARD
  MAV_1_FLOW_CTRL
  MAV_1_RADIO_CTL
  MAV_1_RATE
  SER_TEL2_BAUD
)

usage() {
  cat <<'EOF'
Usage:
  ./scripts/configure_px4_pairb_telem2.sh check

  CONFIRM_NO_MOTION=true \
  CONFIRM_MINI_SYSID_2=true \
  CONFIRM_PX4_PARAM_WRITE=true \
    ./scripts/configure_px4_pairb_telem2.sh apply

This script only configures Mini Pixhawk TELEM2 as a low-bandwidth MAVLink 2
forwarding link for LR24 Pair B. It never arms, reboots, or sends setpoints.
Stop MAVROS first because this script needs exclusive Pixhawk USB access.
EOF
}

if [ "$MODE" != "check" ] && [ "$MODE" != "apply" ]; then
  usage >&2
  exit 2
fi

if [ ! -e "$DEVICE" ]; then
  echo "Mini Pixhawk USB device not found: $DEVICE" >&2
  exit 1
fi

if command -v fuser >/dev/null 2>&1 && fuser "$DEVICE" >/dev/null 2>&1; then
  echo "Pixhawk USB is already in use. Stop MAVROS/QGC serial access first:" >&2
  fuser -v "$DEVICE" >&2 || true
  exit 1
fi

echo "Reading Mini Pixhawk Pair B parameters from $DEVICE"
"${PARAM_TOOL[@]}" get "${PARAMS[@]}"

if [ "$MODE" = "check" ]; then
  cat <<'EOF'

Required values after configuration:
  MAV_SYS_ID=2
  MAV_PROTO_VER=2
  MAV_1_CONFIG=102        # TELEM2
  MAV_1_MODE=7            # Minimal stream
  MAV_1_FORWARD=1
  MAV_1_FLOW_CTRL=0       # LR24 has no RTS/CTS
  MAV_1_RADIO_CTL=0       # LR24 does not emit RADIO_STATUS
  MAV_1_RATE=1200         # PX4-originated bytes/s budget
  SER_TEL2_BAUD=57600
EOF
  exit 0
fi

for gate in CONFIRM_NO_MOTION CONFIRM_MINI_SYSID_2 CONFIRM_PX4_PARAM_WRITE; do
  if [ "${!gate:-false}" != "true" ]; then
    echo "Refusing parameter writes until $gate=true." >&2
    exit 2
  fi
done

set_int32() {
  local name="$1"
  local value="$2"
  echo "Setting $name=$value"
  "${PARAM_TOOL[@]}" set "$name" "$value" --type int32
}

set_int32 MAV_PROTO_VER 2
set_int32 MAV_1_CONFIG 102
set_int32 MAV_1_MODE 7
set_int32 MAV_1_FORWARD 1
set_int32 MAV_1_FLOW_CTRL 0
set_int32 MAV_1_RADIO_CTL 0
set_int32 MAV_1_RATE 1200
set_int32 SER_TEL2_BAUD 57600

echo
echo "Verifying stored values"
"${PARAM_TOOL[@]}" get "${PARAMS[@]}"

cat <<'EOF'

Pair B TELEM2 parameters are stored. Reboot Mini Pixhawk from QGC or by a
controlled power cycle, then rerun this script in check mode. Do not start a
Pair B test until the post-reboot check matches all required values.
EOF
