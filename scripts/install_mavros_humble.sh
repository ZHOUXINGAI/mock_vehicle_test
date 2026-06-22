#!/usr/bin/env bash

set -euo pipefail

if [ "$(id -u)" -eq 0 ]; then
  SUDO=()
else
  SUDO=(sudo)
fi

"${SUDO[@]}" apt-get update
"${SUDO[@]}" apt-get install -y curl gnupg lsb-release

"${SUDO[@]}" mkdir -p /usr/share/keyrings
curl --connect-timeout 10 --max-time 30 -fsSL \
  https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /tmp/ros-archive-keyring.gpg
"${SUDO[@]}" install -m 644 /tmp/ros-archive-keyring.gpg \
  /usr/share/keyrings/ros-archive-keyring.gpg

ARCH="$(dpkg --print-architecture)"
CODENAME="$(. /etc/os-release && printf "%s" "$UBUNTU_CODENAME")"
printf "deb [arch=%s signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu %s main\n" \
  "$ARCH" "$CODENAME" | "${SUDO[@]}" tee /etc/apt/sources.list.d/ros2.list >/dev/null

"${SUDO[@]}" apt-get update
"${SUDO[@]}" apt-get install -y \
  ros-humble-mavros \
  ros-humble-mavros-extras

echo "MAVROS install complete. Open a new shell or source /opt/ros/humble/setup.bash."
