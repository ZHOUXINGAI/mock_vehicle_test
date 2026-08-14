#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
QGC_SOURCE_DIR="${QGC_SOURCE_DIR:-$SCRIPT_DIR/qgroundcontrol-v4.4.5}"
QTLOCATION_SOURCE_DIR="${QTLOCATION_SOURCE_DIR:-$SCRIPT_DIR/qtlocation-5.15.3}"

QGC_REF=v4.4.5
QGC_COMMIT=1ca96414cbdf9b3c0f13e1786ae132335a20be2e
QTLOCATION_REF=v5.15.3-lts-lgpl
QTLOCATION_COMMIT=1bf01b84e30aab2b87a19184ce42160e6c92d8b1

if [ ! -d "$QGC_SOURCE_DIR/.git" ]; then
  git clone --branch "$QGC_REF" --depth 1 --recurse-submodules \
    --shallow-submodules https://github.com/mavlink/qgroundcontrol.git \
    "$QGC_SOURCE_DIR"
fi

if [ "$(git -C "$QGC_SOURCE_DIR" rev-parse HEAD)" != "$QGC_COMMIT" ]; then
  echo "Unexpected QGroundControl revision in $QGC_SOURCE_DIR" >&2
  echo "Expected: $QGC_COMMIT ($QGC_REF)" >&2
  echo "Actual:   $(git -C "$QGC_SOURCE_DIR" rev-parse HEAD)" >&2
  exit 1
fi
git -C "$QGC_SOURCE_DIR" submodule update --init --recursive --depth 1

if [ ! -d "$QTLOCATION_SOURCE_DIR/.git" ]; then
  git clone --branch "$QTLOCATION_REF" --depth 1 \
    https://github.com/qt/qtlocation.git "$QTLOCATION_SOURCE_DIR"
fi

if [ "$(git -C "$QTLOCATION_SOURCE_DIR" rev-parse HEAD)" != "$QTLOCATION_COMMIT" ]; then
  echo "Unexpected QtLocation revision in $QTLOCATION_SOURCE_DIR" >&2
  echo "Expected: $QTLOCATION_COMMIT ($QTLOCATION_REF)" >&2
  echo "Actual:   $(git -C "$QTLOCATION_SOURCE_DIR" rev-parse HEAD)" >&2
  exit 1
fi

echo "QGroundControl source ready: $QGC_SOURCE_DIR ($QGC_REF)"
echo "QtLocation source ready: $QTLOCATION_SOURCE_DIR ($QTLOCATION_REF)"
