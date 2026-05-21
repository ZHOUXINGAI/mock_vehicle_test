# AGENT_STATE — mock_vehicle_test

Last updated: 2026-05-22 CST

## Read First

This file is the working-memory handoff for `/home/hw/mock_vehicle_test`.
Every agent must read it before editing or running scripts in this repo.

This repo is intentionally separate from `/home/hw/easydocking`.
Do not modify easydocking while working here.

## Project Goal

Build a low-risk PX4 rover/offboard training project before moving to real
dual-UAV offboard flight.

Short-term goal:

1. Start one PX4 rover SITL.
2. Use QGC for monitoring.
3. Run a ROS2 offboard mission.
4. Move rover forward `3 m`.
5. Return to start.
6. Enforce a `10 m` software geofence that commands return-home if exceeded.

Hardware learning goal:

- User has Pixhawk 6C.
- First real hardware step is manual RC rover control through PX4 and QGC.
- Companion computer / ROS offboard comes after manual safety control works.

## Scope Boundary

Work here only when the user says: `在mock_vehicle_test`.

Work in `/home/hw/easydocking` only when the user says: `在easydocking`.

If the user does not specify the repo and the task could affect both, ask one
short clarification before editing.

## Current Repo State

- Path: `/home/hw/mock_vehicle_test`
- Git state at takeover: no commits yet on `master`.
- Existing files are untracked because the repo is newly created.
- Main entry points:
  - `start.sh`
  - `启动.sh`
  - `scripts/start_sitl_rover.sh`
  - `scripts/run_offboard_only.sh`
  - `src/mock_rover_offboard.py`
  - `config/default.env`
  - `docs/beginner_rc_rover_step_by_step.md`
  - `docs/pixhawk6c_manual_rover.md`

## Safety Rules

- Manual RC control must remain independent of ROS/offboard.
- First hardware tests must be wheels-up / vehicle lifted.
- Do not let rover scripts kill easydocking experiments unless the user
  explicitly asks.
- Prefer scripts that refuse to start if PX4/MicroXRCEAgent is already running.
- Do not make hardware assumptions silently; if wiring or ESC type matters,
  state the assumption.

## Development Rules

- Do not commit unless the user explicitly asks.
- Keep this repo self-contained.
- It may source `/home/hw/easydocking/install/setup.bash` only to reuse local
  `px4_msgs`; it must not write into easydocking.
- Keep beginner docs practical and hardware-oriented.

## Next Useful Work

1. Confirm Pixhawk 6C rover manual setup path.
2. Verify SITL rover script starts PX4 + MicroXRCEAgent safely.
3. Verify ROS2 offboard node sends forward/return setpoints.
4. Add a small run report format under `results/`.

## Recent User Request

- 2026-05-19: User asked for the fastest safe path to build a manually
  remote-controlled rover that can move forward/backward/left/right. Priority:
  beginner-friendly hardware guidance and safety before power-on.
- 2026-05-19: User shared a motor photo. Identified as a yellow TT-style
  two-wire brushed DC gearmotor, suitable for beginner differential rover
  training with a brushed motor driver/ESC between Pixhawk and the motors.
- 2026-05-19: User said they have Arduino boards and several ESP32 development
  boards. Guidance should distinguish controller boards from required motor
  driver/H-bridge hardware and keep Pixhawk manual safety as the target path.
- 2026-05-19: User shared a photo of assorted Arduino modules. Visible
  `HW-130 motor control` board appears to be an Arduino L293D motor shield,
  usable for low-power bench testing of yellow TT brushed gearmotors with
  wheels lifted, but not a Pixhawk direct RC-PWM motor driver.
- 2026-05-19: User considered buying L298N modules and noted HW-130 is
  Arduino-specific. Recommended L298N only as cheap beginner practice; prefer
  TB6612FNG/DRV8833 for ESP32/Lubancat low-voltage TT motors, and a true
  RC-PWM brushed motor driver/ESC for direct Pixhawk manual rover control.
- 2026-05-19: User reached the bench-test wiring stage with HW-130: PWR jumper
  should be removed, two TT motors connected to M1/M2, next steps are Arduino
  USB upload first, then wheels-lifted external 5-6V motor power on EXT_PWR.
- 2026-05-19: User proposed using a 12V supply into a multi-output buck
  converter, then 5V/GND output to HW-130 EXT_PWR. This is acceptable only if
  the output is verified as regulated 5-6V with correct polarity and sufficient
  current; never feed 12V directly to TT motors/HW-130 motor input.
- 2026-05-19: User reported both TT motors now move on HW-130 after bench
  testing. Next practical step is direction calibration with wheels lifted,
  then simple forward/back/left/right open-loop motion before any Pixhawk work.
- 2026-05-19: User does not yet have a chassis and plans to 3D print one.
  Recommended next hardware focus: simple two-wheel differential base for TT
  gearmotors, rear/front caster, secure battery/Arduino mounting, and safe
  wheels-lifted testing before ground motion.
- 2026-05-19: Added Arduino IDE sketch
  `arduino/hw130_single_wheel_sequence/hw130_single_wheel_sequence.ino`.
  It tests M1/left forward 2s, M1/left backward 2s, waits 5s, then tests
  M2/right forward 2s and backward 2s, then stays stopped.
- 2026-05-20: User confirmed wheel mapping is correct but forward/backward
  direction was reversed. Updated the single-wheel sketch to define
  `WHEEL_FORWARD = BACKWARD` and `WHEEL_BACKWARD = FORWARD` instead of
  rewiring motors.
- 2026-05-20: User later reported only the right wheel forward/backward was
  reversed. Updated the single-wheel sketch to use independent direction
  constants: left forward/backward = BACKWARD/FORWARD, right forward/backward =
  FORWARD/BACKWARD.
- 2026-05-20: User tested and provided the confirmed-correct version: keep
  `WHEEL_FORWARD = BACKWARD` and `WHEEL_BACKWARD = FORWARD`; left wheel uses
  forward/backward in that order, while right wheel test calls
  backward/forward to match physical forward/backward. Repo sketch was updated
  to match the user-tested code.
- 2026-05-20: User has WFLY ET16S transmitter and RF209S receiver and asked how
  to connect RC. Added `arduino/hw130_rc_drive/hw130_rc_drive.ino` for the
  current Arduino/HW-130 path: RF209S PWM CH2 signal to Arduino A0
  (forward/back), CH1 signal to A1 (steering), receiver +5V/GND from Arduino,
  and motor power still via HW-130 EXT_PWR 5-6V.
- 2026-05-20: Confirmed from RF209S manual that each 3-pin channel header is
  Signal / Positive / Negative, and receiver linking uses the receiver's SET
  button (hold 3s until orange LED slow-flashes), not a separate bind plug.
- 2026-05-20: User reported RC control only makes motors barely move at full
  stick even after increasing PWM limits. Added
  `arduino/hw130_rc_debug/hw130_rc_debug.ino`, which prints RF209S CH1/CH2
  pulse widths and mixed motor commands while driving motors with conservative
  high minimum PWM for diagnosis.
- 2026-05-20: User serial output for claimed up/down full-stick showed CH1/A1
  moving from about 1094 to 1932 us while CH2/A0 stayed near 1508 us. This
  indicates the current stick/channel mapping does not match the assumed
  CH2=throttle, CH1=steering layout; identify all stick axes before final code.
- 2026-05-20: User confirmed the auto-centering gimbal is the aileron/elevator
  stick. On this transmitter, its vertical axis maps to CH1/A1 and its
  horizontal axis maps to CH2/A0, matching the current Arduino RC code.
- 2026-05-20: With corrected RC mapping and near-full commands, user reports
  motors still only vibrate at full stick. Since earlier single-wheel tests
  moved both motors one at a time, likely causes are insufficient motor supply
  current/voltage sag when both motors are driven together, HW-130/L293D voltage
  drop, or mechanical load/drag rather than RC command scaling.
- 2026-05-20: User found RC driving works with `MIN_MOTOR_PWM = 245` and
  `MAX_MOTOR_PWM = 255`; L293D is not getting very hot. User then asked how to
  make it work without USB. Guidance: no code change is required; USB is only
  powering Arduino/RF209S. Keep HW-130 PWR jumper removed and add regulated
  standalone logic power (prefer 5V to Arduino 5V/GND or 7-9V to barrel/VIN),
  with common ground to motor EXT_PWR.
- 2026-05-21: User's RC rover code now steers correctly, but during
  forward/backward throttle the left wheel/M1 often stalls while right wheel
  responds well, causing the vehicle to arc left. Next diagnosis should isolate
  mechanical friction, motor weakness, M1 driver channel weakness, and supply
  sag; software cannot command above PWM 255.
- 2026-05-21: User swapped M1/M2 and the fault still followed the left wheel.
  Reversing the left motor wiring/code did not help. New symptom: left motor
  runs for about 1-2 seconds, slows, then stops in both forward and reverse,
  while right motor keeps spinning normally. Likely left motor/gearbox/wheel
  mechanical binding, bad motor brush/commutator, or intermittent left motor
  wiring/solder joint; not an Arduino channel/code issue.
- 2026-05-22: User explicitly requested pushing this repo to GitHub. Local repo
  has no commits yet and no remote configured at the time of request.
