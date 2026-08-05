# Ground result: C2 indoor fake-EV automatic zero-only Offboard pass

Date: 2026-08-05 (Asia/Shanghai)

## Result

C2/Orin2 passed one bounded indoor automatic `OFFBOARD -> Arm -> zero hold ->
Disarm -> MANUAL` diagnostic using fake external vision. The diagnostic's
selected action plan is empty and its publish path contains only an immutable
six-axis zero setpoint.

Successful C2 logs:

`/home/seeed/mock_vehicle_test/results/orin2_indoor_zero_diag/fake_vision_auto_zero_20260805_113116`

## Configuration and evidence

- Pixhawk by-id:
  `/dev/serial/by-id/usb-Auterion_PX4_FMU_v6C.x_0-if00`
- Resolved device: `/dev/ttyACM1`
- MAVLink: 115200 baud, target system/component `2.1`
- MAVROS namespace: `/mavros`
- Velocity setpoint frame: `BODY_NED`, explicitly set and read back
- Fake EV: 30 Hz, seeded from current local pose, 15-second warm-up
- MAVROS had one subscriber for each of:
  `/mavros/vision_pose/pose`, `/mavros/vision_pose/pose_cov`, and
  `/mavros/odometry/out`
- A fresh finite local pose was observed before the test.

The only persistent parameter changed for the test was
`EKF2_EV_CTRL: 0 -> 15`. The following remained unchanged:

- `EKF2_HGT_REF=1`
- `COM_ARM_WO_GPS=1`
- `COM_RC_IN_MODE=3`

## Event sequence

1. Safe MANUAL/disarmed/manual-input prestate verified.
2. Two seconds of immutable zero setpoint prestream.
3. One OFFBOARD request accepted and actual OFFBOARD state observed.
4. One Arm request accepted (`result=0`) and actual armed state observed.
5. Three seconds of immutable zero hold plus one-second zero exit burst.
6. One Disarm request accepted and actual disarmed state observed.
7. One MANUAL request accepted and actual MANUAL state observed.
8. A separate guard independently verified MANUAL/disarmed.

The successful test returned `AUTO_ZERO_RC=0`. No nonzero setpoint was
published and no vehicle movement occurred.

## Safety behavior and cleanup

The first post-reboot attempt found `AUTO.LOITER` instead of the required safe
MANUAL prestate. It therefore refused before fake EV or Arm, restored
MANUAL/disarmed, and stopped MAVROS. This is a successful safety-gate action.

After the successful run, `EKF2_EV_CTRL` was restored to its original value
`0`, the Pixhawk was rebooted, and MANUAL/disarmed was restored and verified.
The final process audit found no MAVROS, fake-EV, or Offboard process.

## Scope

This closes only the C2 indoor fake-position automatic zero-motion gate. It does
not qualify nonzero wheel motion, steering direction, outdoor GPS, Pair B
runtime control, or the two-rover docking trajectory. PX4/MAVROS emitted
undecoded event IDs and `GP: No GPS fix`; these did not block this bounded test
but remain evidence to retain for outdoor diagnostics.
