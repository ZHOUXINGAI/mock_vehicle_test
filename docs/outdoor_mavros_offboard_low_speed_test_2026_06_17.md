# Outdoor MAVROS Offboard Low-Speed Test - 2026-06-17

Use this only after MAVROS, QGC UDP monitoring, manual RC control, and RC
disarm/kill abort have already been tested with the vehicle lifted.

## Goal

Run the first low-speed outdoor Offboard task:

```text
stop/hold
forward 1.0 s
stop 1.0 s
backward 1.0 s
stop 1.0 s
left small turn 0.5 s
stop 1.0 s
right small turn 0.5 s
final stop
```

Default outdoor commands:

```text
LINEAR_SPEED_MPS=0.05
TURN_YAW_RATE_RADPS=0.12
```

## Start MAVROS

Terminal 1:

```bash
cd /home/jetson/mock_vehicle_test
./scripts/run_mavros_px4_usb_to_qgc_logged.sh
```

Wait for:

```text
CON: Got HEARTBEAT, connected. FCU: PX4 Autopilot
```

Terminal 2, optional but recommended:

```bash
cd /home/jetson/mock_vehicle_test
./tools/run-qgroundcontrol.sh
```

## Outdoor Run

Keep the RC transmitter in hand. Use a clear flat area. Start in `MANUAL`,
confirm steering/throttle are stopped, then arm manually from the transmitter.
The script waits for `armed=True` before it requests `OFFBOARD`.

Terminal 3:

```bash
cd /home/jetson/mock_vehicle_test

CONFIRM_GROUND_AREA_CLEAR=true \
CONFIRM_LOW_SPEED_GROUND_TEST=true \
CONFIRM_RC_READY=true \
CONFIRM_PARAM_BACKUP=true \
./scripts/run_real_rover_mavros_outdoor_offboard_task.sh
```

The Offboard task log is saved automatically:

```text
results/offboard/<timestamp>/offboard.log
results/offboard/latest/offboard.log
```

Expected log:

```text
state changed: connected=True mode=MANUAL armed=True ...
requested OFFBOARD mode
OFFBOARD mode request response: mode_sent=True
state changed: connected=True mode=OFFBOARD armed=True ...
starting MAVROS smoke sequence
```

## Abort Expectations

The script should stop and exit if any of these happen:

```text
RC disarm/kill
QGC disarm
mode changes away from OFFBOARD
Ctrl+C in the script terminal
```

If the vehicle does not stop, use RC kill/disarm first, then power down motor
power before debugging software.

MAVROS logs are saved under:

```text
results/mavros/<timestamp>/mavros.log
results/mavros/latest/mavros.log
```

## If QGC Says No Offboard Signal

If QGC reports:

```text
switching to mode offboard is currently not possible No offboard signal
```

then PX4 has not accepted a valid Offboard setpoint stream yet. First check
that the script log shows MAVROS subscriptions:

```text
setpoint_subs=stamped:1 unstamped:1
```

If both subscription counts are nonzero but QGC still reports no Offboard
signal, check rover Offboard parameters:

```bash
cd /home/jetson/mock_vehicle_test
./scripts/check_px4_rover_offboard_params.sh
```

For this MAVROS rover Offboard path:

```text
MAV_TYPE     should be 10 for Ground rover
MAV_FWDEXTSP should be 1 so rover external MAVLink setpoints are forwarded
```

Do not change parameters in the field without saving a QGC `.params` backup.

## Auto-Arm Entry Test

Use this only with wheels lifted. It publishes zero velocity, requests
`OFFBOARD`, then requests `ARM` from MAVROS and disarms at the end:

```bash
cd /home/jetson/mock_vehicle_test

CONFIRM_WHEELS_LIFTED=true \
CONFIRM_RC_READY=true \
CONFIRM_PARAM_BACKUP=true \
./scripts/run_real_rover_mavros_offboard_auto_arm_entry_test.sh
```

This is for testing the mode/arming sequence only. It is not a ground-driving
task.
