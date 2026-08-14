# Orin Nano Runtime Restore - 2026-07-21

## Current Machine

- User: `seeed`
- Repository: `/home/seeed/mock_vehicle_test`
- Git baseline: `77ec72dc0732f40d9b2610305508706308c715fd`
- Branch: `master`, tracking `origin/master`
- No reboot was required after dependency installation.

The migrated working tree includes uncommitted Lubancat motor-mapping,
hardware-debug, and outdoor Offboard results. Do not reset or discard those
changes during machine setup.

## Restored Components

### QGroundControl

- Version: `v4.4.5`
- Source commit: `1ca96414cbdf9b3c0f13e1786ae132335a20be2e`
- Source: `tools/qgroundcontrol-v4.4.5`
- Executable: `tools/qgroundcontrol-v4.4.5/build/QGroundControl`
- Executable SHA-256:
  `d3f02732f63b5feec79922d347e31718211c2c8f088be6d87bb6b6d7db10a8de`
- Build platform: native ARM64, Qt 5.15.3, Release, Ninja

Ubuntu 22.04 ARM64 does not provide the QtLocation private headers required by
this QGC release. The matching QtLocation source is therefore kept at
`tools/qtlocation-5.15.3`:

- Tag: `v5.15.3-lts-lgpl`
- Commit: `1bf01b84e30aab2b87a19184ce42160e6c92d8b1`

QGC stayed running for the full GUI smoke-test interval and exited only when
the test timeout sent SIGTERM. No missing library, QML, or plugin error was
reported. Its settings remain UDP-only so QGC does not compete with MAVROS for
the Pixhawk USB serial port.

Re-fetch and build when the ignored source/build trees are absent:

```bash
cd ~/mock_vehicle_test
./tools/fetch-qgroundcontrol-v4.4.5-sources.sh
./tools/build-qgroundcontrol-v4.4.5.sh
```

Start QGC:

```bash
cd ~/mock_vehicle_test
./tools/run-qgroundcontrol.sh
```

### ROS 2 and MAVROS

- ROS distribution: Humble
- MAVROS: `2.14.0-1jammy.20260608.191037`
- MAVROS extras: `2.14.0-1jammy.20260608.200936`
- MAVLink package: `2026.6.6`
- GeographicLib geoid, gravity, and magnetic datasets are installed under
  `/usr/share/GeographicLib`.

A loopback-only MAVROS smoke test initialized the router, UAS, and all required
plugins. Autopilot-version timeouts were expected because no FCU was connected.
The process shut down cleanly after the test timeout.

Install the runtime again on another Ubuntu 22.04 Orin Nano with:

```bash
cd ~/mock_vehicle_test
./scripts/install_orin_nano_runtime_dependencies.sh
```

### Arduino CLI

- Version: `1.5.0`
- Path: `/home/seeed/.local/bin/arduino-cli`
- Architecture: ARM64

The CLI executable runs normally. Board detection and firmware upload were not
attempted because the Arduino was not connected during restoration.

## Hardware Reconnection Procedure

No motor command, setpoint, arming command, or PX4 parameter write was issued
during this restore.

After connecting Pixhawk and Arduino:

1. Enumerate `/dev/serial/by-id` and verify each device identity without
   opening a motion script.
2. Start the USB-to-QGC bridge:

   ```bash
   cd ~/mock_vehicle_test
   ./scripts/run_mavros_px4_usb_to_qgc_logged.sh
   ```

3. In another terminal, start QGC:

   ```bash
   cd ~/mock_vehicle_test
   ./tools/run-qgroundcontrol.sh
   ```

4. Verify MAVROS reports `connected=true`, `armed=false`, `mode=MANUAL`, and
   neutral outputs before doing anything that can move a wheel.
5. Recheck the PX4 v1.17 differential-rover parameters against
   `AGENT_STATE.md` and the frozen motor mapping in
   `docs/d24a_current_motor_mapping.md`.

The motion-safety rule remains mandatory: `准备好了` permits preparation and
read-only checks only. Every actual movement run requires a fresh explicit
`确认开始` after cable, path, emergency-stop, disarm, and power-cutoff checks.

## Repeatability Helpers

- Runtime dependencies:
  `scripts/install_orin_nano_runtime_dependencies.sh`
- QGC source fetch and revision verification:
  `tools/fetch-qgroundcontrol-v4.4.5-sources.sh`
- QGC ARM64 build:
  `tools/build-qgroundcontrol-v4.4.5.sh`
- QGC UDP-only launcher:
  `tools/run-qgroundcontrol.sh`

The QGC and QtLocation source/build directories are intentionally ignored by
the project repository. Keep the fetch/build helpers tracked so another machine
can reproduce them without copying a large build tree.
