# Indoor MAVROS Manual-Control No-GPS Test - 2026-06-18

Purpose: test Orin-to-Pixhawk motion commands indoors without GPS and without
PX4 Offboard arming requirements.

This is a conservative MANUAL_CONTROL test, not an Offboard test. It keeps PX4
in Manual mode and sends MAVLink `MANUAL_CONTROL` messages through MAVROS:

- forward 1 second
- stop
- backward 1 second
- stop
- left turn 0.5 second
- stop
- right turn 0.5 second
- final stop

## Preconditions

- MAVROS is already connected to Pixhawk.
- QGC may be open through MAVROS UDP forwarding.
- RC transmitter is on and kill/disarm is ready.
- Mode switch is on Manual.
- Wheels are lifted for the first test.
- Current working baseline and PX4 parameters have been backed up.

## Start MAVROS

Terminal 1:

```bash
cd /home/jetson/mock_vehicle_test
./scripts/run_mavros_px4_usb_to_qgc_logged.sh
```

Optional QGC terminal:

```bash
cd /home/jetson/mock_vehicle_test
./tools/run-qgroundcontrol.sh
```

## Run The Indoor Test

Terminal 2:

```bash
cd /home/jetson/mock_vehicle_test

CONFIRM_WHEELS_LIFTED=true \
CONFIRM_RC_READY=true \
CONFIRM_PARAM_BACKUP=true \
./scripts/run_real_rover_mavros_indoor_manual_control_task.sh
```

The script waits until it sees:

- MAVROS connected
- PX4 mode is Manual
- vehicle is armed
- MAVROS is subscribed to `/mavros/manual_control/send`

It does not arm the vehicle. Arm manually from RC only when ready. If the RC is
disarmed or mode leaves Manual after motion starts, the script sends neutral and
aborts.

## Logs

Each run is saved under:

```text
results/manual_control/<timestamp>/manual_control.log
results/manual_control/latest/manual_control.log
```

MAVROS logs are saved separately under:

```text
results/mavros/latest/mavros.log
```

## Axis Tuning

The default mapping follows the way QGC packs joystick data into
`MANUAL_CONTROL`: thrust goes to `z`, yaw/steering goes to `r`. Values are kept
small for lifted-wheel testing.

```text
FORWARD_AXIS=z
TURN_AXIS=r
FORWARD_VALUE_RAW=120
TURN_VALUE_RAW=120
MAX_ABS_XY_R_RAW=250
MIN_Z_RAW=-250
```

If the script reports `manual_subs=1` and starts the sequence but the wheels do
not move, do not raise speed first. Check whether PX4 is accepting
MANUAL_CONTROL while the real RC receiver is present, then test axis mapping
with small values:

```bash
FORWARD_AXIS=x TURN_AXIS=y MIN_Z_RAW=0 \
CONFIRM_WHEELS_LIFTED=true \
CONFIRM_RC_READY=true \
CONFIRM_PARAM_BACKUP=true \
./scripts/run_real_rover_mavros_indoor_manual_control_task.sh
```

Only try ground testing after the lifted-wheel behavior is clear and repeatable.
