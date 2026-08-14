#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
QGC_SOURCE_DIR="${QGC_SOURCE_DIR:-$SCRIPT_DIR/qgroundcontrol-v4.4.5}"
QGC_BUILD_DIR="${QGC_BUILD_DIR:-$QGC_SOURCE_DIR/build}"
QGC_BUILD_JOBS="${QGC_BUILD_JOBS:-2}"
CMAKE_BIN="${CMAKE_BIN:-/usr/bin/cmake}"
QTLOCATION_SOURCE_DIR="${QTLOCATION_SOURCE_DIR:-$SCRIPT_DIR/qtlocation-5.15.3}"
QT_PRIVATE_OVERLAY="$QGC_BUILD_DIR/qt-private-overlay"

if [ "${QGC_FETCH_MISSING_SOURCES:-true}" = "true" ] && {
  [ ! -f "$QGC_SOURCE_DIR/qgroundcontrol.pro" ] ||
  [ ! -f "$QTLOCATION_SOURCE_DIR/src/location/maps/qgeomaptype_p.h" ]
}; then
  QGC_SOURCE_DIR="$QGC_SOURCE_DIR" \
  QTLOCATION_SOURCE_DIR="$QTLOCATION_SOURCE_DIR" \
    "$SCRIPT_DIR/fetch-qgroundcontrol-v4.4.5-sources.sh"
fi

if [ ! -f "$QGC_SOURCE_DIR/qgroundcontrol.pro" ]; then
  echo "QGroundControl v4.4.5 source is missing: $QGC_SOURCE_DIR" >&2
  exit 1
fi

if ! command -v qmake >/dev/null 2>&1; then
  echo "qmake is missing. Run scripts/install_orin_nano_runtime_dependencies.sh first." >&2
  exit 1
fi

QT_VERSION="${QT_VERSION:-$(qmake -query QT_VERSION)}"
QT_MKSPEC="${QT_MKSPEC:-$(qmake -query QMAKE_XSPEC)}"

case "$QT_VERSION" in
  5.15.*) ;;
  *)
    echo "QGC v4.4.5 requires Qt 5.15.x; detected $QT_VERSION." >&2
    exit 1
    ;;
esac

# Ubuntu 22.04 arm64 does not package QtLocation's private development
# headers. QGC 4.4 includes them directly, so expose headers from the matching
# QtLocation source tag through the same include layout as an official Qt SDK.
if [ ! -f "$QTLOCATION_SOURCE_DIR/src/location/maps/qgeomaptype_p.h" ]; then
  echo "QtLocation v5.15.3 private headers are missing: $QTLOCATION_SOURCE_DIR" >&2
  echo "Clone tag v5.15.3-lts-lgpl into that directory before building." >&2
  exit 1
fi

mkdir -p \
  "$QT_PRIVATE_OVERLAY/QtLocation/private" \
  "$QT_PRIVATE_OVERLAY/QtPositioning/private"

for HEADER in \
  "$QTLOCATION_SOURCE_DIR"/src/location/*.h \
  "$QTLOCATION_SOURCE_DIR"/src/location/maps/*.h
do
  ln -sfn "$HEADER" "$QT_PRIVATE_OVERLAY/QtLocation/private/$(basename "$HEADER")"
done
for HEADER in "$QTLOCATION_SOURCE_DIR"/src/positioning/*.h; do
  ln -sfn "$HEADER" "$QT_PRIVATE_OVERLAY/QtPositioning/private/$(basename "$HEADER")"
done

env QT_VERSION="$QT_VERSION" QT_MKSPEC="$QT_MKSPEC" \
  "$CMAKE_BIN" -S "$QGC_SOURCE_DIR" -B "$QGC_BUILD_DIR" \
  -G Ninja -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CXX_FLAGS="-I$QT_PRIVATE_OVERLAY"

"$CMAKE_BIN" --build "$QGC_BUILD_DIR" --parallel "$QGC_BUILD_JOBS"

if [ ! -x "$QGC_BUILD_DIR/QGroundControl" ]; then
  echo "Build finished without the expected executable: $QGC_BUILD_DIR/QGroundControl" >&2
  exit 1
fi

echo "QGroundControl build ready: $QGC_BUILD_DIR/QGroundControl"
