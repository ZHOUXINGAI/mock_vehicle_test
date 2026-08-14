# Lubancat Outdoor Offboard Checkpoint 2026-06-22

This is the saved state after the Lubancat D24A four-wheel remap and Arduino
bridge restore. Use this as the starting point for the next outdoor Offboard
test.

## Current Vehicle State

- Hardware chain: `Lubancat -> Pixhawk 6C -> Arduino UNO -> D24A -> 4 motors`.
- PX4 firmware: `px4_fmu-v6c_rover.px4`, PX4 v1.17 rover differential airframe.
- Current PX4 output intent: differential rover motor outputs, not RC passthrough.
- Current Arduino sketch flashed on the UNO:

```text
arduino/d24a_pixhawk_differential_pwm_bridge/d24a_pixhawk_differential_pwm_bridge.ino
```

Serial was verified after upload:

```text
D24A Pixhawk differential PWM bridge ready
in1_us/in2_us around 1490 us
left=0 right=0
```

## Critical Wiring Reminder

The D24A standby/enable line must be connected:

```text
D24A STBY -> Arduino A2
```

During remap, this jumper fell off and made `BF` look like a B-channel failure.
After reconnecting `STBY`, right-front forward worked strongly at PWM 90.

## Current D24A Wheel Mapping

Final 8-step lifted-wheel verification, PWM 90, 5 seconds each:

```text
right-front forward  = BF
right-front backward = BB

left-front forward   = DB
left-front backward  = DF

left-rear forward    = AF
left-rear backward   = AB

right-rear forward   = CB
right-rear backward  = CF
```

Physical forward raw command set:

```text
A:+  B:+  C:-  D:-
```

Left side is `D/A`; right side is `B/C`.

## PX4 Parameters To Preserve

Key known-good differential rover state:

```text
SYS_AUTOSTART=50000
CA_AIRFRAME=6
COM_RC_IN_MODE=3
RC_MAP_THROTTLE=2
RC_MAP_YAW=4
PWM_MAIN_FUNC1=101
PWM_MAIN_FUNC2=102
PWM_MAIN_FUNC6=0
PWM_MAIN_FUNC7=0
PWM_MAIN_MIN1=1300
PWM_MAIN_MAX1=1700
PWM_MAIN_MIN2=1300
PWM_MAIN_MAX2=1700
PWM_MAIN_DIS1=1500
PWM_MAIN_DIS2=1500
PWM_MAIN_FAIL1=1500
PWM_MAIN_FAIL2=1500
PWM_MAIN_REV=0
CA_R_REV=3
RD_WHEEL_TRACK=0.30
RO_MAX_THR_SPEED=0.25
RO_SPEED_LIM=0.35
RO_YAW_P=0.25
RO_YAW_RATE_LIM=90
RO_SPEED_P=0
RO_SPEED_I=0
RO_YAW_RATE_P=0
RO_YAW_RATE_I=0
RO_YAW_RATE_CORR=1
```

Remote-controller intent remains:

```text
CH2 = forward/backward throttle intent
CH4 = left/right steering intent
```

## Next Outdoor Test

Recommended first task after the 2026-06-24 calibration is a lifted-wheel
forward mapping retest, not an L-turn. The previous forward-only attempt used
`LINEAR_DIRECTION_SIGN=-1.0`; the wheels looked mostly backward first and then
the two sides diverged, which matches PX4 yaw correction plus the negative
body-frame velocity setpoint.

Update from the next 2026-06-24 lifted-wheel retest: with
`LINEAR_DIRECTION_SIGN=1.0` but the old `PWM_MAIN_REV=3`, the actual forward
segment produced `MAIN1=MAIN2=1436`, which the Arduino bridge interpreted as
physical backward. `PWM_MAIN_REV` was changed and saved to `0`. The forward
mapping wrapper now pre-publishes the first forward setpoint before OFFBOARD
entry and removes the initial long stop segment to avoid the yaw-correction
turn at the start.

Verification after the fix: run
`results/differential_offboard_forward_mapping/20260624_172246/` commanded
`forward 3.00s vx=0.080`; `rc_watch.log` showed `MAIN1=MAIN2=1564`, and the
user confirmed the wheels physically moved forward for 3 seconds. This is the
current successful lifted-wheel forward Offboard baseline.

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
CONFIRM_FRESH_USER_START=true \
FORWARD_SEC=2.0 \
LINEAR_SPEED_MPS=0.08 \
LINEAR_DIRECTION_SIGN=1.0 \
./scripts/run_real_rover_mavros_differential_offboard_forward_mapping.sh
```

Start QGC and MAVROS first:

```bash
./tools/run-qgroundcontrol.sh
./scripts/run_mavros_px4_usb_to_qgc_logged.sh
```

The forward mapping wrapper records `/mavros/rc/in` and `/mavros/rc/out` in
the same result directory as the Offboard log. Confirm MAIN1 and MAIN2 move in
the same physical-forward direction before returning to ground L-turn tests.

Use motion scripts only after confirming:

```text
Vehicle disarmed before start
RC transmitter on and ready
QGC disarm/kill ready
Physical power cutoff ready
Wheels lifted for mapping tests, or clear outdoor ground area for ground tests
GPS/local position healthy
Current D24A differential bridge loaded
Current D24A mapping above still trusted
```

After the lifted-wheel forward mapping is confirmed, the body-frame L-turn test
is:

```bash
CONFIRM_GROUND_AREA_CLEAR=true \
CONFIRM_LOW_SPEED_GROUND_TEST=true \
CONFIRM_VEHICLE_DISARMED=true \
CONFIRM_RC_READY=true \
CONFIRM_PARAM_BACKUP=true \
CONFIRM_QGC_DISARM_READY=true \
CONFIRM_PHYSICAL_POWER_CUTOFF_READY=true \
CONFIRM_REAL_LOCAL_POSITION=true \
CONFIRM_CURRENT_DIFF_MAPPING=true \
CONFIRM_WHEELS_INSTALLED=true \
CONFIRM_FRESH_USER_START=true \
./scripts/run_real_rover_mavros_differential_offboard_l_turn.sh
```

Recommended conservative first outdoor settings:

```bash
FIRST_DISTANCE_M=3.0
SECOND_DISTANCE_M=3.0
LINEAR_SPEED_MPS=0.12
TURN_DIRECTION_SIGN=-1.0
TURN_LATERAL_SPEED_MPS=0.10
TURN_FORWARD_SPEED_MPS=0.0
TURN_ANGLE_DEG=90.0
```

For the current outdoor U-turn test, prefer the closed-loop yaw/distance wrapper
instead of timed open-loop turning:

```bash
CONFIRM_GROUND_AREA_CLEAR=true \
CONFIRM_LOW_SPEED_GROUND_TEST=true \
CONFIRM_VEHICLE_DISARMED=true \
CONFIRM_RC_READY=true \
CONFIRM_PARAM_BACKUP=true \
CONFIRM_QGC_DISARM_READY=true \
CONFIRM_PHYSICAL_POWER_CUTOFF_READY=true \
CONFIRM_REAL_LOCAL_POSITION=true \
CONFIRM_CURRENT_DIFF_MAPPING=true \
CONFIRM_WHEELS_INSTALLED=true \
CONFIRM_FRESH_USER_START=true \
./scripts/run_real_rover_mavros_differential_offboard_closed_loop_right_u_turn.sh
```

Default closed-loop U-turn settings:

```bash
FIRST_DISTANCE_M=3.0
SECOND_DISTANCE_M=3.0
LINEAR_SPEED_MPS=0.14
TURN_DIRECTION_SIGN=+1.0
TURN_LATERAL_SPEED_MPS=0.35
TURN_FORWARD_SPEED_MPS=0.0
TURN_ANGLE_DEG=180.0
YAW_TOLERANCE_DEG=3.0
TURN_MAX_SEC=45.0
```

The 2026-06-24 open-loop 5 s right-turn test changed measured yaw by only about
`79-81deg`, so fixed-time 180-degree turns are not reliable on the current
surface. The closed-loop wrapper records RC output and refuses to start if live
PX4 statustext reports heading or magnetic preflight warnings.

The first closed-loop `3m -> right 180 -> 3m` attempt on 2026-06-24 reached
only about `117deg` before the old `TURN_MAX_SEC=22` guard fired. Forward legs
completed and the vehicle returned to `MANUAL`, disarmed, with neutral RC
output. Next attempt should use the stronger defaults above.

The stronger second attempt on 2026-06-24 succeeded as the current baseline:
`3m -> right 180 -> 3m`, with the turn stopping by yaw feedback at `172.7deg`
(`180deg - 8deg` tolerance) rather than by timeout. Run directory:

```text
results/differential_offboard_closed_loop_u_turn/closed_loop_right_uturn_20260624_181035/
```

Post-run state was safe: `MANUAL`, `armed=false`, `manual_input=true`, and
`/mavros/rc/out=[1500,1500,...]`.

After that run, the user requested more turn authority and a closer 180-degree
heading. The active full-response update is:

```text
Arduino differential bridge: MAX_DRIVE_PWM=255
PX4 PWM_MAIN_MIN1/MAX1=1000/2000
PX4 PWM_MAIN_MIN2/MAX2=1000/2000
PWM_MAIN_DIS1/2=1500
PWM_MAIN_FAIL1/2=1500
Closed-loop U-turn defaults: TURN_LATERAL_SPEED_MPS=0.35, YAW_TOLERANCE_DEG=3.0
Prestart first-motion setpoint enabled to avoid OFFBOARD-entry heading correction
```

Retest only after fresh ground/RC/QGC/physical-cutoff confirmation.

The first full-response retest on 2026-06-24 showed that the power path is now
open: forward outputs reached about `1780/1780`, and turn output reached
`MAIN1=2000`. The turn yaw estimate became jumpy at this speed, so the active
script now also requires:

```bash
TURN_COMPLETION_HOLD_SEC=0.3
```

This keeps the high motor authority but prevents a single transient yaw sample
from ending the turn early.

The next full-response retest exposed a separate yaw wrap bug at the +/-180deg
boundary. The local yaw progressed through about `166deg` and then wrapped to
negative values such as `-141deg`, so direct `current - start` yaw error made a
nearly complete turn look incomplete again. The user hit kill, which was the
right stop action. The active script now accumulates incremental yaw progress
across wrap and publishes a stop command while the completion hold timer is
being satisfied. After any kill event, verify `/mavros/state system_status=3`
and neutral `/mavros/rc/out` before another motion test.

The first retest after that fix completed successfully:

```text
results/differential_offboard_closed_loop_u_turn/closed_loop_right_uturn_20260624_182951/
PX4 log: /fs/microsd/log/2026-06-24/10_30_19.ulg
```

It ran 3m forward, right-turn-to-threshold, then 3m forward, and cleanup
returned to `MANUAL`, `armed=false`, `system_status=3`, with neutral
`/mavros/rc/out`. The turn no longer got stuck at yaw wrap. The user later
confirmed the rover was catching/hitting small stones during the turn, so the
non-monotonic yaw samples are explained by mechanical disturbance in the field,
not by a remaining offboard mapping failure.

Current acceptance state: this Lubancat PX4 v1.17 differential-rover Offboard
baseline is considered complete for the vehicle. Forward motion,
high-authority differential turning, yaw-wrap handling, cleanup, and safety
recovery have all been validated outdoors.

## Per-Run Motion Safety Rule

Do not reuse confirmation between real-vehicle motion tests.

`准备好了` or `ready` only means the assistant may prepare scripts and run
non-motion checks. It does not authorize the rover to move. Every wheels-down
or motor-spinning test must wait for a fresh current-run start confirmation such
as `确认开始`, given after the user has checked the physical vehicle.

Before any future motion launch, re-check and say out loud:

```text
HDMI/display cable disconnected or safely secured
USB/power/loose cables clear of the rover and wheels
ground path clear; no immediate stones/obstacles/cables in the path
RC kill/disarm ready
QGC disarm ready
physical power cutoff ready
PX4 state: MANUAL, armed=false, system_status=3
PX4 outputs neutral: /mavros/rc/out around 1500/1500
```

The confirmation expires after every run, kill/flight termination, collision,
stuck wheel, cable change, reboot, MAVROS/PX4 disconnect, or script
refusal/failure. The next motion test needs a new explicit start confirmation.

This rule was added after the 2026-06-24 outdoor test session because a second
offboard test was launched while the HDMI cable was still connected to the
screen. Treat this as a process bug, not a vehicle bug.

## Avoid For The First Outdoor Run

Do not start with the 5 m out-and-back script. Stock PX4 v1.17 differential
rover velocity Offboard does not behave like a simple reverse/in-place-turn
controller.

Also avoid scripts that restore the old RC passthrough baseline unless the goal
is explicitly to leave differential rover mode. The current outdoor baseline is
`PWM_MAIN_FUNC1=101`, `PWM_MAIN_FUNC2=102`.
