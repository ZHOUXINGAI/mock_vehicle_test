#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

if [ "${QGC_CONFIGURE_UDP_ONLY:-true}" = "true" ]; then
  "$REPO_DIR/scripts/configure_qgc_udp_only.sh"
fi

exec "$SCRIPT_DIR/qgroundcontrol-v4.4.5/build/QGroundControl" "$@"
