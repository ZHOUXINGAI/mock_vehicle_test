#!/usr/bin/env bash

set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RESULTS_DIR="${RESULTS_DIR:-$REPO_DIR/results/$(date +%Y%m%d_%H%M%S)_rover_sitl}"
PX4_INSTANCE="${PX4_INSTANCE:-1}"
PX4_SYS_AUTOSTART="${PX4_SYS_AUTOSTART:-50000}"
PX4_SIM_MODEL="${PX4_SIM_MODEL:-gz_rover_differential}"
PX4_GZ_WORLD="${PX4_GZ_WORLD:-rover}"
PX4_SIMULATOR="${PX4_SIMULATOR:-gz}"
UXRCE_PORT="${UXRCE_PORT:-8888}"
PX4_NAMESPACE="${PX4_NAMESPACE:-/px4_${PX4_INSTANCE}}"
MISSION_MODE="${MISSION_MODE:-position}"
TRAVEL_DISTANCE_M="${TRAVEL_DISTANCE_M:-3.0}"
FENCE_RADIUS_M="${FENCE_RADIUS_M:-10.0}"
FORWARD_SPEED_MPS="${FORWARD_SPEED_MPS:-0.7}"
RETURN_SPEED_MPS="${RETURN_SPEED_MPS:-0.5}"
ARM_ON_START="${ARM_ON_START:-true}"

mkdir -p "$RESULTS_DIR"

# shellcheck disable=SC1091
source "$REPO_DIR/scripts/env.sh"

PX4_BIN="$PX4_AUTOPILOT_PATH/build/px4_sitl_default/bin/px4"
if [ ! -x "$PX4_BIN" ]; then
  echo "PX4 SITL binary not found: $PX4_BIN" >&2
  echo "Build PX4 first, or set PX4_AUTOPILOT_PATH." >&2
  exit 1
fi

if [ "${MOCK_ALLOW_SHARED_SYSTEM:-false}" != "true" ]; then
  if pgrep -af "[p]x4_sitl_default/bin/px4|[M]icroXRCEAgent" >/dev/null; then
    echo "PX4 or MicroXRCEAgent is already running." >&2
    echo "Refusing to start to avoid interfering with another project/session." >&2
    echo "Stop the other stack first, or set MOCK_ALLOW_SHARED_SYSTEM=true if you know it is safe." >&2
    exit 2
  fi
fi

cleanup() {
  kill "${MISSION_PID:-}" 2>/dev/null || true
  kill "${PX4_PID:-}" 2>/dev/null || true
  kill "${AGENT_PID:-}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

if [ -x "$UXRCE_AGENT_BIN" ]; then
  "$UXRCE_AGENT_BIN" udp4 -p "$UXRCE_PORT" >"$RESULTS_DIR/microxrce_agent.log" 2>&1 &
  AGENT_PID=$!
else
  echo "MicroXRCEAgent not found at $UXRCE_AGENT_BIN; continuing without starting it." | tee "$RESULTS_DIR/warnings.log"
fi

(
  cd "$PX4_AUTOPILOT_PATH"
  PX4_SYS_AUTOSTART="$PX4_SYS_AUTOSTART" \
    PX4_SIM_MODEL="$PX4_SIM_MODEL" \
    PX4_GZ_WORLD="$PX4_GZ_WORLD" \
    PX4_SIMULATOR="$PX4_SIMULATOR" \
    "$PX4_BIN" -i "$PX4_INSTANCE"
) >"$RESULTS_DIR/px4.log" 2>&1 &
PX4_PID=$!

sleep 8

python3 "$REPO_DIR/src/mock_rover_offboard.py" \
  --ros-args \
  -p px4_namespace:="$PX4_NAMESPACE" \
  -p vehicle_id:="$PX4_INSTANCE" \
  -p arm_on_start:="$ARM_ON_START" \
  -p mission_mode:="$MISSION_MODE" \
  -p travel_distance_m:="$TRAVEL_DISTANCE_M" \
  -p fence_radius_m:="$FENCE_RADIUS_M" \
  -p forward_speed_mps:="$FORWARD_SPEED_MPS" \
  -p return_speed_mps:="$RETURN_SPEED_MPS" \
  >"$RESULTS_DIR/mission.log" 2>&1 &
MISSION_PID=$!

cat <<EOF
PX4 rover SITL started.
Results: $RESULTS_DIR
QGC: open QGroundControl and connect via MAVLink UDP.
Mission log: tail -f "$RESULTS_DIR/mission.log"
Stop: Ctrl-C
EOF

wait "$MISSION_PID"
