#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

if [ "${QGC_CONFIGURE_UDP_ONLY:-true}" = "true" ]; then
  "$REPO_DIR/scripts/configure_qgc_udp_only.sh"
fi

QGC_DIR="$SCRIPT_DIR/qgroundcontrol-v4.4.5"
QGC_BIN="$QGC_DIR/build/QGroundControl"

if [ ! -x "$QGC_BIN" ]; then
  echo "QGroundControl v4.4.5 is not built: $QGC_BIN" >&2
  echo "Build it with: $SCRIPT_DIR/build-qgroundcontrol-v4.4.5.sh" >&2
  exit 1
fi

export LD_LIBRARY_PATH="$QGC_DIR/build/libs/shapelib:$QGC_DIR/build/libs/qmlglsink:${LD_LIBRARY_PATH:-}"

exec "$QGC_BIN" "$@"
