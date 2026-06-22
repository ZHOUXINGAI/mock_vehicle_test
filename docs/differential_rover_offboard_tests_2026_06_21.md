# Differential Rover Offboard Tests 2026-06-21

Current firmware baseline:

```text
PX4 v1.17.0 rover
SYS_AUTOSTART=50000
CA_AIRFRAME=6
CA_R_REV=3
```

Manual RC restore baseline remains:

```text
PWM_MAIN_FUNC1=405
PWM_MAIN_FUNC2=403
PWM_MAIN_FUNC6=0
PWM_MAIN_FUNC7=0
```

## Required Arduino Mode

The new differential Offboard scripts expect the Arduino to interpret the two
Pixhawk PWM inputs as left/right motor commands. Use:

```text
arduino/d24a_pixhawk_differential_pwm_bridge/d24a_pixhawk_differential_pwm_bridge.ino
```

Do not run these differential Offboard scripts with the older
`d24a_pixhawk_pwm_bridge` throttle/steering mixer loaded.

## 5s Wheels-Lifted Sequence

Script:

```bash
./scripts/run_real_rover_mavros_differential_fake_vision_offboard_5s_sequence.sh
```

Sequence:

```text
forward 5s
backward 5s
left turn 5s
right turn 5s
```

It starts fake local position only for Offboard entry. It does not prove real
distance tracking.

Required confirmations include:

```bash
CONFIRM_WHEELS_LIFTED=true
CONFIRM_VEHICLE_DISARMED=true
CONFIRM_RC_READY=true
CONFIRM_PARAM_BACKUP=true
CONFIRM_QGC_DISARM_READY=true
CONFIRM_PHYSICAL_POWER_CUTOFF_READY=true
CONFIRM_FAKE_LOCAL_POSITION_ONLY=true
CONFIRM_LOW_SPEED_WHEELS_TEST=true
CONFIRM_ARDUINO_DIFFERENTIAL_PWM_BRIDGE=true
```

The script temporarily applies:

```text
PWM_MAIN_FUNC1=101
PWM_MAIN_FUNC2=102
```

Cleanup restores the manual RC baseline.

## 3m Body-Frame L-Turn

Script:

```bash
./scripts/run_real_rover_mavros_differential_offboard_l_turn.sh
```

Mission:

```text
forward 3 m in BODY_NED
left arc/turn until local yaw changes about 90 deg
forward 3 m in BODY_NED
```

This script is the current recommended low-speed ground test. It avoids the old
`LOCAL_NED` L-turn behavior where PX4 first corrected yaw toward a global
velocity vector before driving forward.

Defaults:

```text
SETPOINT_VELOCITY_MAV_FRAME=BODY_NED
FIRST_DISTANCE_M=3.0
SECOND_DISTANCE_M=3.0
LINEAR_SPEED_MPS=0.12
TURN_DIRECTION_SIGN=-1.0
TURN_LATERAL_SPEED_MPS=0.10
TURN_FORWARD_SPEED_MPS=0.0
TURN_ANGLE_DEG=90.0
TURN_MAX_SEC=12.0
```

Required confirmations include:

```bash
CONFIRM_GROUND_AREA_CLEAR=true
CONFIRM_LOW_SPEED_GROUND_TEST=true
CONFIRM_VEHICLE_DISARMED=true
CONFIRM_RC_READY=true
CONFIRM_PARAM_BACKUP=true
CONFIRM_QGC_DISARM_READY=true
CONFIRM_PHYSICAL_POWER_CUTOFF_READY=true
CONFIRM_REAL_LOCAL_POSITION=true
CONFIRM_CURRENT_DIFF_MAPPING=true
CONFIRM_WHEELS_INSTALLED=true
```

## 5m Out-And-Back

Script:

```bash
./scripts/run_real_rover_mavros_differential_offboard_out_and_back_5m.sh
```

Mission:

```text
forward 5 m
turn 180 deg
forward 5 m
```

This is not the current recommended next test on stock PX4 v1.17 differential
rover. The MAVROS velocity Offboard path maps reverse velocity to positive
forward speed plus a 180-degree yaw target, and it ignores `cmd_vel.angular.z`
for the differential rover velocity branch. Do not use this script for true
reverse or in-place turn validation until the Offboard control approach is
changed.

The script uses `/mavros/local_position/pose` to measure distance and yaw. Do
not use fake vision for this test; fake vision is fixed and will not prove 5 m
movement.

Required confirmations include:

```bash
CONFIRM_GROUND_AREA_CLEAR=true
CONFIRM_LOW_SPEED_GROUND_TEST=true
CONFIRM_VEHICLE_DISARMED=true
CONFIRM_RC_READY=true
CONFIRM_PARAM_BACKUP=true
CONFIRM_QGC_DISARM_READY=true
CONFIRM_PHYSICAL_POWER_CUTOFF_READY=true
CONFIRM_REAL_LOCAL_POSITION=true
CONFIRM_ARDUINO_DIFFERENTIAL_PWM_BRIDGE=true
```

Defaults are conservative:

```text
FORWARD_DISTANCE_M=5.0
LINEAR_SPEED_MPS=0.25
TURN_YAW_RATE_RADPS=0.35
OUTPUT_MAPPING_ACTION=apply-differential-limited
```

If lifted-wheel validation is clean but ground motion is too weak, retry with:

```bash
OUTPUT_MAPPING_ACTION=apply-differential
```
