#!/usr/bin/env bash

set -euo pipefail

if [ "$(id -u)" -eq 0 ]; then
  SUDO=()
else
  SUDO=(sudo)
fi

PACKAGES=(
  build-essential
  ccache
  cmake
  geographiclib-tools
  libespeak-dev
  libfontconfig1
  libfuse2
  libgstreamer-plugins-base1.0-dev
  libgstreamer1.0-0
  libgstreamer1.0-dev
  libqt5charts5-dev
  libqt5serialport5-dev
  libqt5svg5-dev
  libqt5texttospeech5-dev
  libqt5x11extras5-dev
  libsdl2-dev
  libssl-dev
  libudev-dev
  libunwind-dev
  ninja-build
  patchelf
  pkg-config
  qml-module-qt-labs-folderlistmodel
  qml-module-qt-labs-settings
  qml-module-qtcharts
  qml-module-qtgraphicaleffects
  qml-module-qtlocation
  qml-module-qtmultimedia
  qml-module-qtpositioning
  qml-module-qtqml-models2
  qml-module-qtquick-controls
  qml-module-qtquick-controls2
  qml-module-qtquick-dialogs
  qml-module-qtquick-layouts
  qml-module-qtquick-shapes
  qml-module-qtquick-templates2
  qml-module-qtquick-window2
  qml-module-qtquick2
  qtbase5-dev
  qtbase5-private-dev
  qtconnectivity5-dev
  qtdeclarative5-dev
  qtlocation5-dev
  qtmultimedia5-dev
  qtpositioning5-dev
  qtquickcontrols2-5-dev
  ros-humble-mavros
  ros-humble-mavros-extras
  ros-humble-ros-base
  speech-dispatcher
  zlib1g-dev
)

"${SUDO[@]}" apt-get update
"${SUDO[@]}" apt-get install -y "${PACKAGES[@]}"

GEOGRAPHICLIB_INSTALLER=/opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh
if [ "${INSTALL_GEOGRAPHICLIB_DATASETS:-true}" = "true" ]; then
  if [ ! -x "$GEOGRAPHICLIB_INSTALLER" ]; then
    echo "MAVROS installed, but dataset installer is missing: $GEOGRAPHICLIB_INSTALLER" >&2
    exit 1
  fi
  "${SUDO[@]}" "$GEOGRAPHICLIB_INSTALLER"
fi

echo "Orin Nano QGroundControl and MAVROS dependencies are installed."
