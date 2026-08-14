# Hardware Debug Log

This file records hardware/debugging incidents that are easy to forget but
useful for future rover bring-up. Keep entries practical: symptom, evidence,
root cause, fix, and what to check next time.

## 2026-06-24 Lubancat PX4 Offboard Forward Test Moves Backward

### Symptom

During lifted-wheel Offboard "forward 3 s" testing:

- The vehicle first showed about 1 s of turn-like wheel motion.
- Then the wheels ran backward for the 3 s forward segment.
- Near the end there was another short turn-like motion.

The user's physical observation was correct. This was not a loose motor wire
or random D24A behavior.

### Relevant Baseline

Current hardware/control chain:

```text
Lubancat -> MAVROS -> Pixhawk 6C PX4 v1.17 differential rover
          -> Arduino UNO differential PWM bridge -> D24A -> four wheels
```

Current Arduino bridge:

```text
arduino/d24a_pixhawk_differential_pwm_bridge/d24a_pixhawk_differential_pwm_bridge.ino
```

Current wheel-side convention in that Arduino bridge:

```text
PWM input > 1500 us -> positive left/right command
PWM input < 1500 us -> negative left/right command

positive left/right command = physical forward
negative left/right command = physical backward
```

D24A physical forward raw command set:

```text
A:+  B:+  C:-  D:-
```

### Evidence

Run directory:

```text
results/differential_offboard_forward_mapping/20260624_171345/
```

The Offboard script did send a positive forward setpoint:

```text
forward: 3.00s vx=0.080 vy=0.000 yaw_rate=0.000
```

But `rc_watch.log` showed two separate behaviors.

Before the actual forward segment, during Offboard/arm entry:

```text
rc/out 1:1512 2:1487
rc/out 1:1522 2:1477
...
rc/out 1:1550 2:1450
```

This is differential output, so it produces a turn-like/yaw-correction motion.

During the actual forward segment:

```text
rc/out 1:1436 2:1436
```

Both sides were equal, so this was a straight command, not steering. But both
were below `1500 us`, which the current Arduino bridge interprets as negative
left/right command, i.e. physical backward.

### Root Cause

There were two issues at once:

1. Old PX4 output reversal was still active:

```text
PWM_MAIN_REV=3
```

With the current remapped Arduino bridge, this inverted both MAIN1 and MAIN2.
So a positive PX4 forward command arrived at Arduino as `<1500 us`, causing
physical backward motion.

2. The script entered Offboard while publishing zero-velocity stop setpoints.
PX4 differential rover can use this transition to correct yaw/heading, causing
the turn-like motion before and after the intended forward segment.

### Fix Applied

Stopped MAVROS to release Pixhawk USB, then changed and saved PX4:

```text
PWM_MAIN_REV=0
CA_R_REV=3 unchanged
PWM_MAIN_FUNC1=101 unchanged
PWM_MAIN_FUNC2=102 unchanged
```

Save command acknowledged:

```text
MAV_CMD_PREFLIGHT_STORAGE result=0
```

Then restarted MAVROS/QGC bridge and verified safe state:

```text
/mavros/state:
  connected: true
  armed: false
  manual_input: true
  mode: MANUAL

/mavros/rc/out:
  channels: [1500, 1500, 0, ...]
```

Also changed the forward mapping test path:

- `src/real_rover_mavros_offboard_smoke.py` now supports
  `prestart_first_motion_setpoint`.
- `scripts/run_real_rover_mavros_offboard_smoke.sh` passes that option through.
- `scripts/run_real_rover_mavros_differential_offboard_forward_mapping.sh`
  defaults to:

```text
PRESTART_FIRST_MOTION_SETPOINT=true
INITIAL_STOP_SEC=0.0
STOP_SEC=0.2
FINAL_STOP_SEC=0.0
STOP_BURST_SEC=0.2
LINEAR_DIRECTION_SIGN=1.0
```

The intent is to enter Offboard while already publishing the forward setpoint,
instead of first giving PX4 a stop/yaw-correction opportunity.

### Verification

After the fix, lifted-wheel forward was retested:

```text
results/differential_offboard_forward_mapping/20260624_172246/
```

Commanded sequence:

```text
forward: 3.00s vx=0.080 vy=0.000 yaw_rate=0.000
stop_after_forward: 0.20s
```

`rc_watch.log` showed the correct output polarity:

```text
rc/out 1:1515 2:1515
rc/out 1:1528 2:1528
...
rc/out 1:1564 2:1564
```

Both MAIN1 and MAIN2 were above `1500 us`, so the Arduino bridge received
positive left/right commands. The user confirmed the wheels physically moved
forward for 3 seconds.

Final safety state was verified afterward:

```text
mode=MANUAL
armed=false
rc/out 1:1500 2:1500
```

### Rule For Next Time

When a lifted-wheel "forward" test does not physically move forward, do not
guess from wheel motion alone. Always record `/mavros/rc/out`.

Interpretation for this current Arduino differential bridge:

```text
MAIN1 and MAIN2 both > 1500 us -> both sides positive -> physical forward
MAIN1 and MAIN2 both < 1500 us -> both sides negative -> physical backward
MAIN1 > 1500 and MAIN2 < 1500 -> turn/yaw correction
MAIN1 < 1500 and MAIN2 > 1500 -> opposite turn/yaw correction
MAIN1 and MAIN2 near 1500 -> neutral
```

If the setpoint says `vx > 0` but `rc/out` is both below `1500`, check:

```text
PWM_MAIN_REV
Arduino INVERT_LEFT_COMMAND / INVERT_RIGHT_COMMAND
Arduino SWAP_LEFT_RIGHT_INPUTS
D24A raw wheel mapping
```

For the current Lubancat/D24A differential bridge, the expected PX4 reversal is:

```text
PWM_MAIN_REV=0
```

Do not restore the older Jetson/Nano `PWM_MAIN_REV=3` assumption without
revalidating it against the current Arduino bridge and wheel mapping.

### Useful Commands

Check safe state:

```bash
source /opt/ros/humble/setup.bash
timeout 5 ros2 topic echo --once /mavros/state
timeout 5 ros2 topic echo --once /mavros/rc/out
```

Run lifted-wheel forward mapping test after fresh safety confirmation:

```bash
TEST_SURFACE=wheels_lifted \
CONFIRM_WHEELS_LIFTED=true \
CONFIRM_VEHICLE_DISARMED=true \
CONFIRM_RC_READY=true \
CONFIRM_PARAM_BACKUP=true \
CONFIRM_QGC_DISARM_READY=true \
CONFIRM_PHYSICAL_POWER_CUTOFF_READY=true \
CONFIRM_REAL_LOCAL_POSITION=true \
CONFIRM_CURRENT_DIFF_MAPPING=true \
FORWARD_SEC=3.0 \
LINEAR_SPEED_MPS=0.08 \
LINEAR_DIRECTION_SIGN=1.0 \
./scripts/run_real_rover_mavros_differential_offboard_forward_mapping.sh
```

Read PX4 reversal directly only when MAVROS is stopped and Pixhawk USB is free:

```bash
python3 scripts/px4_mavlink_param.py get PWM_MAIN_REV CA_R_REV PWM_MAIN_FUNC1 PWM_MAIN_FUNC2
```

## 2026-07-21 New Orin Nano Basic Motor Test

The migrated rover hardware was connected to `/home/seeed/mock_vehicle_test`.
USB identities were verified as:

```text
Arduino UNO  -> /dev/ttyACM0
Pixhawk 6C   -> /dev/ttyACM1
```

The `seeed` user was added to `dialout`. `ModemManager` was stopped and
disabled so it cannot claim the Pixhawk or Arduino serial ports after reboot.
The connected PX4 currently uses MAVLink system ID `2`, so this session started
MAVROS with `TARGET_SYSTEM=2`.

For a wheels-up motor-only check, Arduino was temporarily flashed with
`arduino/d24a_serial_bridge/d24a_serial_bridge.ino`. Three separate commands
were run after a fresh user confirmation for each command:

```text
forward: PWM 170, 3 seconds -> passed
left:    PWM 170, 3 seconds -> passed
right:   PWM 170, 3 seconds -> passed
```

The user confirmed the physical response was correct for all three. This test
verified `Orin Nano -> Arduino -> D24A -> four motors`; it did not validate RC
input or PX4 Offboard behavior.

After testing, Arduino was flashed back to the production differential bridge:

```text
arduino/d24a_pixhawk_differential_pwm_bridge/d24a_pixhawk_differential_pwm_bridge.ino
```

Flash verification passed. Final Arduino serial output showed both inputs near
neutral (`1488-1495 us`) with `left=0 right=0`. MAVROS showed PX4
`armed=false`, and `/mavros/rc/out` was `[1500, 1500, 0, ...]`.

Do not treat the current `AUTO.LOITER`, `manual_input=false`, `system_status=0`
state as ready for RC or Offboard testing. Diagnose receiver/mode/preflight
state first, then require a fresh motion confirmation.
