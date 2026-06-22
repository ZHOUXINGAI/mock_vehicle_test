# AGENT_STATE — mock_vehicle_test

Last updated: 2026-06-21 CST

## Read First

This file is the working-memory handoff for `/home/jetson/mock_vehicle_test`.
Older notes may still mention `/home/hw/mock_vehicle_test`; the active Jetson
workspace is `/home/jetson/mock_vehicle_test`.
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

- Path: `/home/jetson/mock_vehicle_test`
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

## Current Hardware Baseline

- 2026-06-16: User confirmed both current hardware control paths are working:
  `Orin Nano -> Pixhawk 6C -> Arduino UNO -> D24A -> motors` and
  `AT9S PRO -> R9DS -> Pixhawk 6C -> Arduino UNO -> D24A -> motors`.
- The current recoverable baseline is documented in
  `docs/current_rover_success_baseline_2026_06_16.md`.
- Main Arduino firmware for the current Pixhawk bridge path is
  `arduino/d24a_pixhawk_pwm_bridge/d24a_pixhawk_pwm_bridge.ino`.
- Current transmitter/PX4 manual-control mapping for this vehicle is fixed:
  `CH2` controls forward/backward and `CH4` controls steering. Live testing on
  2026-06-21 showed the previous `PWM_MAIN_FUNC1=403`,
  `PWM_MAIN_FUNC2=405` mapping made `CH2` steer and `CH4` throttle. The current
  recoverable passthrough baseline is the observed cross-mapping:
  `PWM_MAIN_FUNC1=405` (`RC Yaw`, `RC_MAP_YAW=4`) and
  `PWM_MAIN_FUNC2=403` (`RC Pitch`, `RC_MAP_PITCH=2`). Do not change MAIN2 to
  `RC Roll`/CH1 for this current wiring. The old direct R9DS-to-Arduino CH1
  steering document
  `docs/at9s_pro_r9ds_d24a_rc_wiring.md` and old sketch
  `arduino/d24a_r9ds_rc_drive/d24a_r9ds_rc_drive.ino` were deleted on
  2026-06-21 to avoid confusing the current PX4 chain.
- D24A pinout and four-wheel direction mapping should be treated as frozen
  until a new wheels-up calibration is done. See
  `docs/d24a_current_motor_mapping.md`.
- For PX4 differential-rover Offboard tests, do not use the normal
  throttle/steering Arduino bridge. Use
  `arduino/d24a_pixhawk_differential_pwm_bridge/d24a_pixhawk_differential_pwm_bridge.ino`
  and temporary PX4 output mapping `PWM_MAIN_FUNC1=101`,
  `PWM_MAIN_FUNC2=102`; cleanup must restore the manual RC baseline
  `PWM_MAIN_FUNC1=405`, `PWM_MAIN_FUNC2=403`. Test entry points are documented
  in `docs/differential_rover_offboard_tests_2026_06_21.md`.
- 2026-06-20 Offboard diagnosis: fake-vision Offboard was accepted and the
  setpoint sequence ran, but the motors did not move because the working manual
  baseline had `PWM_MAIN_FUNC1=403` and `PWM_MAIN_FUNC2=405` (RC Pitch/Yaw
  passthrough) on the physical Arduino bridge outputs. Offboard actuator output
  was visible on `PWM_MAIN_FUNC6/7=101`, not on MAIN1/MAIN2. Keep the PX4 path;
  use `scripts/set_px4_rover_output_mapping.sh apply` wheels-up to remap
  MAIN1/MAIN2 to `Motor 1`/`Servo 1` (`101`/`201`) before fake-vision Offboard
  motion, and `restore-baseline` to return to the prior RC passthrough mapping.
- The Arduino bridge has an `80 us` PWM deadband. The earlier `0.05 m/s`
  Offboard command only changed PX4 output by about `11 us`, so even a correct
  mapping may need a larger wheels-up command before motion is visible.
- Full Pixhawk/QGC parameters have not yet been exported. Next hardware state
  task should be exporting the working `.params` file before changing QGC
  actuator, mixer, RC, mode, or safety settings.

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
- 2026-05-20: Old HW-130/RF209S two-wheel RC wiring experiments were done
  before the current D24A/Pixhawk rover existed. Do not use those channel notes
  for the current vehicle.
- 2026-05-20: Confirmed from RF209S manual that each 3-pin channel header is
  Signal / Positive / Negative, and receiver linking uses the receiver's SET
  button (hold 3s until orange LED slow-flashes), not a separate bind plug.
- 2026-05-20: Old HW-130/RF209S RC debug sketches existed for that earlier
  two-wheel platform. They are not authoritative for the current Pixhawk 6C
  rover, whose current mapping is `CH2=forward/backward`, `CH4=steering`.
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
- 2026-06-08: On Lubancat 4, manually starting NoMachine reliably causes
  onboard PCIe Realtek `rtl8852be` Wi-Fi to disconnect and scan zero APs within
  about 20-30 seconds. Kernel log repeats
  `halbb_*_rf_reg_8852b_a is_w_busy/is_r_busy`. This is a wireless driver/chip
  hang, not a Wi-Fi password or rfkill issue. Manual recovery command:
  `sudo systemctl restart rtl8852be-reload.service`. Added
  `scripts/recover_lubancat_wifi.sh` and installed/enabled
  `lubancat-wifi-watchdog.timer`, which checks every 30s and reloads the
  driver only when `wlan0` is not connected and Wi-Fi scan returns zero APs.
  NoMachine service is installed but `nxserver.service` autostart is disabled.
- 2026-06-08: Tried a root-cause mitigation by setting
  `options 8852be rtw_lps_mode=0 rtw_low_power=0 rtw_msi_en=0`. This made the
  Realtek PCIe device worse on this Lubancat 4: kernel reported
  `Unable to change power state from D3cold to D0, device inaccessible`, and
  `wlan0` disappeared from `nmcli`. Reverted `/etc/modprobe.d/8852be-stability.conf`
  to only `rtw_lps_mode=0 rtw_low_power=0`. Do not retry `rtw_msi_en=0` on this
  board without a full reboot/cold-boot plan. Current session may need reboot
  to re-enumerate the PCIe Wi-Fi device.
- 2026-06-16: User said the current two successful paths are complete:
  `Orin Nano -> Pixhawk -> Arduino -> D24A -> motors` and
  `remote controller -> receiver -> Pixhawk -> Arduino -> D24A -> motors`.
  User asked to first freeze the working state by recording Arduino firmware,
  Pixhawk output channels, D24A wiring, motor direction, and QGC parameters.
  Added `docs/current_rover_success_baseline_2026_06_16.md`; QGC/Pixhawk
  complete params still need export as the next state-preservation step.
- 2026-06-16: User asked to design step 3: the first real Orin/Pixhawk Offboard
  small task while the car battery is charging, and asked how controllers track
  time-parameterized dynamic corridors. Added
  `docs/offboard_minimal_task_design_2026_06_16.md`. Recommendation: do not
  reuse the SITL 3m script on hardware first; create a conservative hardware
  smoke test that sends short forward/stop/backward/stop/left/right commands
  with wheels lifted, then move to low-speed ground testing. Also record that
  feasible time tracking requires measured limits: deadband, max speed,
  acceleration/deceleration, yaw rate, turning radius, latency, battery effects,
  and tracking error.
- 2026-06-16: User asked to directly create the step-3 Offboard test script.
  Added `src/real_rover_offboard_smoke.py` and
  `scripts/run_real_rover_offboard_smoke.sh`. The script defaults to ROS 2 PX4
  Offboard `velocity` setpoints, requires explicit confirmations for wheels
  lifted, RC ready, and parameter backup, and does not arm or switch modes by
  default. It can also be run with `COMMAND_MODE=direct_actuator` if PX4 output
  routing is later configured for direct actuator outputs. `scripts/env.sh` now
  also sources Jetson-side setup paths and optional `EXTRA_ROS_SETUP`.
  Verification: Python and bash syntax checks pass; default run refuses before
  ROS startup unless confirmations are set. Current Jetson environment does not
  contain `px4_msgs`, and neither `pymavlink` nor MAVSDK is installed, so the
  next runtime dependency task is building/sourcing `px4_msgs` or installing a
  MAVLink client library.
- 2026-06-16: User connected the Pixhawk. Real system check showed Pixhawk 6C
  USB at `/dev/ttyACM0` and
  `/dev/serial/by-id/usb-Auterion_PX4_FMU_v6C.x_0-if00`; Arduino CH340 is
  `/dev/ttyUSB0`. No QGroundControl or MicroXRCEAgent process was running at
  that time. Installed `pymavlink` and added MAVLink-based true PX4 Offboard
  smoke test `src/real_rover_mavlink_offboard_smoke.py` with wrapper
  `scripts/run_real_rover_mavlink_offboard_smoke.sh`. This route sends
  `SET_POSITION_TARGET_LOCAL_NED` over Pixhawk USB and avoids the current
  missing `px4_msgs`/MicroXRCEAgent dependency. It still refuses to run without
  explicit wheels-lifted, RC-ready, and parameter-backup confirmations, and does
  not switch mode or arm unless `MODE_CHANGE_ON_START=true ARM_ON_START=true`.
- 2026-06-16: User decided to use MAVROS because aircraft will also use MAVROS,
  and selected the architecture `MAVROS owns Pixhawk serial, forwards UDP to
  QGC`. Added `scripts/install_mavros_humble.sh`,
  `scripts/run_mavros_px4_usb_to_qgc.sh`,
  `src/real_rover_mavros_offboard_smoke.py`,
  `scripts/run_real_rover_mavros_offboard_smoke.sh`, and
  `docs/mavros_px4_usb_to_qgc_plan_2026_06_16.md`. Installation is currently
  blocked in Codex because `sudo apt-get` requires an interactive password.
  User should run `./scripts/install_mavros_humble.sh` in a local terminal, then
  start MAVROS before QGC so QGC uses UDP 14550 instead of the Pixhawk USB
  serial port.
- 2026-06-16: User provided sudo password. Installed ROS 2 MAVROS packages:
  `ros-humble-mavros`, `ros-humble-mavros-extras`, and dependencies. Added ROS
  2 apt source using the current Open Robotics key from `rosdistro`; the older
  `packages.ros.org/ros.key` produced `EXPKEYSIG F42ED6FBAB17C654`. MAVROS
  initially crashed because `/usr/share/GeographicLib/geoids/egm96-5.pgm` was
  missing. Downloaded `egm96-5.tar.bz2` from SourceForge and installed
  `egm96-5.pgm`. Verified `timeout 12s ./scripts/run_mavros_px4_usb_to_qgc.sh`
  starts MAVROS, opens Pixhawk USB, opens QGC UDP endpoint
  `udp://@127.0.0.1:14550`, detects remote address `1.1`, and receives PX4
  heartbeat from `PX4 Autopilot`. The timeout stopped MAVROS after verification;
  it is not left running.
- 2026-06-16: User found that starting QGC caused MAVROS to die with
  `mavconn: serial0: receive: End of file`. Diagnosis: QGC had auto-connected
  directly to Pixhawk USB and was holding `/dev/ttyACM0`, so MAVROS lost the
  serial link. Added `scripts/configure_qgc_udp_only.sh`; updated
  `tools/run-qgroundcontrol.sh` to set QGC `[LinkManager] autoConnectPixhawk=false`
  before launch; updated `scripts/run_mavros_px4_usb_to_qgc.sh` to use explicit
  GCS URL `udp://:14555@127.0.0.1:14550`.
- 2026-06-17: User raised the remaining safety-chain test: switching from
  Offboard to Manual from the transmitter while the Offboard program is running.
  Updated `src/real_rover_mavros_offboard_smoke.py` and its wrapper so that,
  after the motion sequence starts, leaving `OFFBOARD` or disarming immediately
  sends stop and aborts the script by default (`ABORT_ON_MODE_EXIT=true`,
  `ABORT_ON_DISARM=true`). If the transmitter switch does not change QGC/PX4
  mode, the issue is RC flight-mode mapping rather than MAVROS.
- 2026-06-17: User reported QGC mission transfer warnings and still not entering
  Offboard. Clarified that QGC mission warnings such as `unexpected waypoint
  index` are not the Offboard failure signal. Added service-response logging to
  `src/real_rover_mavros_offboard_smoke.py` for `set_mode` and arming requests,
  plus state-change logging. Added
  `scripts/run_real_rover_mavros_offboard_entry_test.sh`, a zero-velocity
  Offboard/arm/disarm entry test wrapper.
- 2026-06-17: First run of the entry test crashed before requesting Offboard
  because rclpy logger methods do not accept printf-style positional arguments.
  Fixed `src/real_rover_mavros_offboard_smoke.py` logger calls to use formatted
  strings. `python3 -m py_compile` and bash syntax checks pass.
- 2026-06-17: Entry test successfully requested Offboard (`mode_sent=True`,
  state changed to `OFFBOARD`), but MAVROS arming was rejected
  (`success=False result=1`) and the script kept waiting because abort logic only
  applied after the motion sequence started. Updated MAVROS smoke script so that
  after Offboard has ever been seen, switching away from Offboard aborts even in
  the pre-start wait phase; after armed has ever been seen, disarm aborts even
  pre-start; and arm rejection aborts by default (`ABORT_ON_ARM_REJECTED=true`).
  Also fixed shutdown to avoid a duplicate `rcl_shutdown already called` error
  on Ctrl+C.
- 2026-06-17: User confirmed RC disarm/kill now makes the Offboard script exit.
  Added outdoor low-speed Offboard wrapper
  `scripts/run_real_rover_mavros_outdoor_offboard_task.sh` and doc
  `docs/outdoor_mavros_offboard_low_speed_test_2026_06_17.md`. The wrapper
  defaults to `TEST_SURFACE=ground`, `LINEAR_SPEED_MPS=0.05`,
  `TURN_YAW_RATE_RADPS=0.12`, `MODE_CHANGE_ON_START=true`,
  `ARM_ON_START=false`, and requires ground-area/low-speed/RC/parameter-backup
  confirmations. The base MAVROS smoke script now supports ground confirmations
  without requiring `CONFIRM_WHEELS_LIFTED=true`.
- 2026-06-17: Outdoor run showed `OFFBOARD armed=False`; the script had
  requested Offboard before the user manually armed, so it correctly held stop
  and waited. Added `REQUIRE_ARMED_BEFORE_MODE_CHANGE`; the outdoor wrapper now
  defaults it to true, so it waits in Manual until RC/manual arm is detected,
  then requests Offboard. Added `scripts/run_mavros_px4_usb_to_qgc_logged.sh`
  to tee MAVROS output live and save each run under `results/mavros/<timestamp>/`
  with `results/mavros/latest` pointing at the newest run.
- 2026-06-17: User then saw `MANUAL armed=True` but the outdoor script never
  requested Offboard and timed out. Hardened the outdoor wrapper to force the
  critical outdoor defaults instead of inheriting stale shell environment
  variables such as `MODE_CHANGE_ON_START=false`, and made it print effective
  settings at startup. Also changed mode/arm request bookkeeping so failed
  service readiness does not suppress future retries.
- 2026-06-17: User then saw `requested OFFBOARD mode` with `mode_sent=True`,
  but PX4 remained `MANUAL armed=True` until timeout. Updated the MAVROS smoke
  node to retry Offboard mode requests every `MODE_REQUEST_RETRY_SEC` seconds
  until mode actually becomes `OFFBOARD` or the wait timeout expires, and added
  a subscription to `/mavros/statustext/recv` so PX4/QGC rejection messages are
  printed directly in the script log.
- 2026-06-17: The first outdoor retry showed
  `New publisher discovered on topic '/mavros/statustext/recv', offering
  incompatible QoS`, so the smoke node still could not see PX4 rejection text.
  Changed the `/mavros/statustext/recv` subscription to ROS 2 sensor/best-effort
  QoS. If Offboard still stays in `MANUAL`, the next run should paste
  `PX4 STATUSTEXT`, `requested OFFBOARD mode`, `OFFBOARD mode request
  response`, and `state changed` lines. Likely causes to separate are: RC mode
  switch/channel continuously forcing MANUAL, PX4 refusing Offboard because
  local position/velocity estimate is invalid, or a safety/prearm/safety-switch
  warning.
- 2026-06-17: User reported QGC says `switching to mode offboard is currently
  not possible No offboard signal` when trying to change from armed Manual to
  Offboard. This means PX4 has not accepted a recent Offboard setpoint stream,
  independent of the mode switch itself. Updated the MAVROS smoke node to publish
  both `/mavros/setpoint_velocity/cmd_vel` (`TwistStamped`) and
  `/mavros/setpoint_velocity/cmd_vel_unstamped` (`Twist`) by default, with
  `PUBLISH_UNSTAMPED_CMD_VEL=false` available to disable the second stream. The
  wait log now includes `setpoint_subs=stamped:N unstamped:N`; if both are zero,
  MAVROS is not subscribed to the setpoint topics. If subscriptions are nonzero
  but PX4 still reports `No offboard signal`, inspect `PX4 STATUSTEXT` and MAVROS
  logs for estimator/safety/setpoint rejection.
- 2026-06-17: User reran outdoor script and saw `setpoint_subs=stamped:1
  unstamped:1`, but QGC still reported `No offboard signal`. This confirms the
  Python script is publishing and MAVROS is subscribed; the remaining issue is
  MAVROS-to-PX4 rover Offboard acceptance. PX4 current docs say rover MAVLink
  setpoints are gated by `MAV_FWDEXTSP` (Forward external setpoint messages).
  Added `scripts/check_px4_rover_offboard_params.sh` to read `MAV_TYPE`,
  `MAV_FWDEXTSP`, `COM_OF_LOSS_T`, `COM_OBL_RC_ACT`, `COM_RC_IN_MODE`, and
  `COM_RCL_EXCEPT` through MAVROS without changing parameters. Also added
  automatic Offboard task logging under `results/offboard/<timestamp>/offboard.log`
  with `results/offboard/latest/offboard.log`, and added
  `scripts/run_real_rover_mavros_offboard_auto_arm_entry_test.sh`, a
  wheels-lifted zero-velocity test that requests OFFBOARD, then ARM, then
  DISARM. Do not use auto-arm for ground motion until the entry test is clean.
- 2026-06-17: User ran `scripts/check_px4_rover_offboard_params.sh` and every
  parameter read failed because the script used the old/deprecated
  `/mavros/param/get` service. Updated it to discover `/mavros/param/pull`, call
  `mavros_msgs/srv/ParamPull`, then read cached PX4 parameters using
  `ros2 param get /mavros/param <name>` with a `get_parameters` service fallback.
  The script now prints `ROS_DOMAIN_ID` and discovered MAVROS nodes/services if
  the parameter service is missing.
- 2026-06-17: User reran the fixed parameter check successfully:
  `MAV_TYPE=10`, `MAV_FWDEXTSP=1`, `COM_OF_LOSS_T=1.0`,
  `COM_OBL_RC_ACT=0`, `COM_RC_IN_MODE=3`, `COM_RCL_EXCEPT=0`. This rules out
  the two main parameter blockers for MAVROS rover Offboard
  (`MAV_TYPE` wrong, `MAV_FWDEXTSP` disabled). Next diagnostic is the
  wheels-lifted auto-arm entry test to check whether PX4 accepts
  `OFFBOARD -> ARM` when the script, not the RC transmitter, sends the arm
  command.
- 2026-06-17: Auto-arm entry test result: script published setpoints, switched
  PX4 from `MANUAL armed=False` to `OFFBOARD armed=False`, then MAVROS ARM was
  rejected (`success=False result=1`). Offboard script and MAVROS both reported
  `Arming denied: Resolve system health failures first`. MAVROS log throughout
  this run repeatedly reported `GP: No GPS fix`. This means the original
  `No offboard signal` blocker is no longer the active blocker; the active
  blocker is PX4 health/prearm checks for arming in Offboard, likely no GPS/local
  position/velocity estimate. Manual RC arming had succeeded earlier despite no
  GPS because Manual mode has weaker position-estimate requirements than
  Offboard/autonomous arming.
- 2026-06-18: User chose an indoor/no-GPS temporary control path because current
  conditions cannot satisfy PX4 Offboard health checks. Added
  `src/real_rover_mavros_manual_control_smoke.py`,
  `scripts/run_real_rover_mavros_manual_control_smoke.sh`, and
  `scripts/run_real_rover_mavros_indoor_manual_control_task.sh`. This path does
  not request Offboard and does not arm. It keeps PX4 in Manual, waits for MAVROS
  connected + Manual + armed + `/mavros/manual_control/send` subscriber, then
  sends a conservative MANUAL_CONTROL sequence: forward 1s, stop, backward 1s,
  stop, left/right turns, final stop. It logs each run to
  `results/manual_control/<timestamp>/manual_control.log` with
  `results/manual_control/latest` updated. The first test should be wheels
  lifted with `CONFIRM_WHEELS_LIFTED=true`, `CONFIRM_RC_READY=true`, and
  `CONFIRM_PARAM_BACKUP=true`. If the sequence starts but wheels do not move,
  inspect `manual_subs`, PX4 status text, and axis/source priority before
  increasing raw command values.
- 2026-06-18: First indoor MANUAL_CONTROL run reached MAVROS subscriber,
  `MANUAL armed=True`, and completed the sequence, but motors did not move. The
  initial defaults used `x` for forward and `y` for turn. QGC's joystick sender
  packs manual control as `x=pitch`, `y=roll`, `z=thrust`, `r=yaw`, so rover
  throttle/steering is more likely `z/r`. Updated the indoor wrapper defaults to
  `FORWARD_AXIS=z`, `TURN_AXIS=r`, and `MIN_Z_RAW=-250` while keeping raw values
  at 120. If z/r still has no effect, the likely remaining blocker is PX4 input
  source selection such as `COM_RC_IN_MODE`/joystick handling with the real RC
  receiver present, not ROS publication.
- 2026-06-18: User reran MANUAL_CONTROL with `z/r`; sequence completed but
  wheels still did not move. Local PX4 metadata says `COM_RC_IN_MODE=3` means
  "RC or Joystick keep first"; since the RC receiver is available first, PX4 can
  ignore later MAVLink joystick/manual-control input until reboot. Instead of
  immediately changing `COM_RC_IN_MODE=1` (which disables RC input handling and
  can weaken RC kill/disarm safety), added a safer diagnostic path:
  `src/real_rover_mavros_rc_override_smoke.py` and
  `scripts/run_real_rover_mavros_indoor_rc_override_task.sh`. It publishes
  MAVROS `/mavros/rc/override` (`RC_CHANNELS_OVERRIDE`) and only overrides
  default CH3 throttle and CH4 steering, leaving arm/mode/kill channels to the
  physical transmitter. It requires `CONFIRM_RC_STICKS_CENTERED=true`, releases
  override before/after the run, and logs under
  `results/rc_override/<timestamp>/rc_override.log`.
- 2026-06-18: User ran the RC override task; it reached `/mavros/rc/override`
  subscriber, `MANUAL armed=True`, and completed the sequence, but wheels still
  did not move. Added `src/mavros_rc_io_watch.py` and
  `scripts/watch_mavros_rc_io.sh` to compare `/mavros/rc/in` and
  `/mavros/rc/out` during manual transmitter motion and during override/script
  motion. Also extended `scripts/check_px4_rover_offboard_params.sh` to print
  `RC_MAP_THROTTLE`, `RC_MAP_YAW`, `RC_MAP_ROLL`, and `RC_MAP_PITCH`. Next
  diagnostic: if manual RC changes `rc/out` but override/manual-control does
  not, PX4 is ignoring MAVLink control input; then test `COM_RC_IN_MODE=1` only
  with wheels lifted. If override changes `rc/out` but wheels do not move, debug
  Pixhawk output channel / Arduino input path instead.
- 2026-06-20: User provided watcher log while manually moving RC sticks. Manual
  forward/back changed `rc/in` channel 2 and `rc/out` channel 1. Manual steering
  changed `rc/in` channel 4 and `rc/out` channel 2. This matches the hardware
  baseline where Pixhawk output 1 feeds Arduino D2 throttle and output 2 feeds
  Arduino D13 steering. The previous RC override run used `THROTTLE_CHANNEL=3`,
  so it was overriding the wrong input channel. Also Arduino
  `d24a_pixhawk_pwm_bridge.ino` has `DEAD_BAND_US=80`, while the old override
  delta was 80us, exactly at the deadband boundary. Updated RC override defaults
  to `THROTTLE_CHANNEL=2`, `STEERING_CHANNEL=4`,
  `FORWARD_DELTA_US=150`, and `TURN_DELTA_US=150`. Re-run the same wheels-lifted
  RC override task before changing PX4 joystick/Offboard parameters.
- 2026-06-20: User reran RC override with corrected `THROTTLE_CHANNEL=2`,
  `STEERING_CHANNEL=4`, and 150us deltas. The task ran and completed while
  `MANUAL armed=True`, but the concurrent watcher showed `rc/out` channel 1/2
  stayed near neutral (`1505/1500`) throughout the scripted sequence. Manual RC
  stick motion had already proved that `rc/out` changes when PX4 accepts an
  input, so this run confirms PX4 is ignoring MAVLink RC override rather than an
  Arduino/D24A failure. Added `scripts/set_px4_com_rc_in_mode.sh` for explicit
  `COM_RC_IN_MODE` changes and
  `scripts/run_real_rover_mavros_indoor_joystick_only_manual_control_task.sh`
  for a wheels-lifted `COM_RC_IN_MODE=1` MANUAL_CONTROL diagnostic. Treat
  `COM_RC_IN_MODE=1` as temporary only: RC input checks/takeover may not work;
  restore baseline `COM_RC_IN_MODE=3` immediately after testing.
- 2026-06-20: User set `COM_RC_IN_MODE=1` while PX4 was already armed. QGC
  reported `Manual control lost`, `Failsafe activated: Autopilot disengaged,
  switching to Descend`; MAVROS state became `AUTO.LAND armed=True guided=True`
  and watcher showed `manual_input=False`. The joystick-only script correctly did
  not run because allowed mode was only `MANUAL`, but it waited instead of
  failing early. Updated `scripts/set_px4_com_rc_in_mode.sh` so setting value
  `1` now requires `CONFIRM_VEHICLE_DISARMED=true` and refuses if
  `/mavros/state` reports `armed=true`. Updated
  `scripts/run_real_rover_mavros_indoor_joystick_only_manual_control_task.sh`
  to refuse unless state is `MANUAL` and `armed=false` before startup. Correct
  diagnostic order: disarm, set `COM_RC_IN_MODE=1`, start joystick-only script
  so MAVLink MANUAL_CONTROL neutral stream exists, arm from QGC/MAVROS, observe,
  then restore `COM_RC_IN_MODE=3`.
- 2026-06-20: After the failed joystick-only sequence, user reported the motors
  started turning without new input; local checks showed MAVROS/ROS 2 were not
  running while QGC still was. Most likely cause: `COM_RC_IN_MODE=1` remained
  active after the manual-control script was interrupted, the MAVLink manual
  stream stopped, and PX4 stayed armed/failsafe until the user recovered to RC
  direct control. Updated
  `scripts/run_real_rover_mavros_indoor_joystick_only_manual_control_task.sh` so
  it no longer `exec`s the child manual-control script; it now wraps it and, on
  normal exit or Ctrl+C, attempts to restore `COM_RC_IN_MODE=3` automatically
  unless `AUTO_RESTORE_COM_RC_IN_MODE=false` is explicitly set.
- 2026-06-20 01:10 CST: User restored `COM_RC_IN_MODE=3` through MAVROS; log
  showed before value `3`, set success, and after value `3`. Follow-up
  `/mavros/state` check with `ROS_DOMAIN_ID=99` showed
  `connected=true`, `armed=false`, `guided=false`, `manual_input=true`,
  `mode=MANUAL`. This is the recovered RC/Pixhawk safety baseline.
- 2026-06-20: User wanted an indoor fake-GPS path to avoid wasting outdoor time
  before re-testing Offboard. Added `src/mavros_fake_gps_input.py` and
  `scripts/run_mavros_fake_gps_input.sh`. The node publishes fixed
  `mavros_msgs/GPSINPUT` to `/mavros/gps_input/gps_input` and logs whether
  MAVROS/PX4 produce `/mavros/global_position/global` and
  `/mavros/local_position/pose`. Also added
  `scripts/run_real_rover_mavros_indoor_fake_gps_offboard_entry_test.sh`, which
  starts fake GPS, warms up, then runs only the zero-velocity Offboard
  auto-arm entry test and attempts disarm/cleanup on exit. It refuses unless
  the vehicle is disarmed and `COM_RC_IN_MODE=3`. This is not a safe indoor
  motion test; use it only wheels-lifted to test estimator/GPS/Offboard entry.
- 2026-06-20: Fake GPS test showed MAVROS subscribed to
  `/mavros/gps_input/gps_input` and `/mavros/local_position/pose` produced
  local pose, but `/mavros/global_position/raw/fix`,
  `/mavros/global_position/global`, and `/mavros/global_position/local` did not
  publish within timeout. User correctly reasoned that real indoor localization
  should inject local position through the PX4 external vision / mocap /
  odometry path. Current MAVROS graph has subscribers on
  `/mavros/vision_pose/pose`, `/mavros/vision_pose/pose_cov`,
  `/mavros/mocap/pose`, and `/mavros/odometry/out`; PX4 params include
  `EKF2_EV_CTRL=15`, so external vision fusion is already enabled. Added
  `src/mavros_fake_external_vision.py`,
  `scripts/run_mavros_fake_external_vision.sh`, and
  `scripts/run_real_rover_mavros_indoor_fake_vision_offboard_entry_test.sh`.
  The fake EV node publishes fixed pose/covariance/zero-velocity odometry,
  seeding from the current local pose by default to avoid estimator jumps. The
  integrated test only runs zero-velocity Offboard entry with wheels lifted and
  disarmed startup.
- 2026-06-20 01:42 CST: Ran the non-arming fake external-vision verification
  for 12s. Pre-state was `MANUAL`, `armed=false`, `manual_input=true`.
  Parameters read: `EKF2_EV_CTRL=15`, `EKF2_HGT_REF=1`,
  `COM_ARM_WO_GPS=1`, and EKF arm thresholds `0.5`. The fake EV node seeded
  from current local pose and repeatedly logged
  `subs=vision:1 vision_cov:1 odom:1` with stable published/local pose values.
  The process exited normally with no fake publisher left running; post-state
  remained `connected=true`, `armed=false`, `guided=false`,
  `manual_input=true`, `mode=MANUAL`. Did not run the auto-arm Offboard entry
  test autonomously.
- 2026-06-20 01:46 CST: User ran the integrated fake-vision Offboard entry
  wrapper. PX4 accepted the script's OFFBOARD request while disarmed, proving
  the zero-velocity setpoint signal is now valid, but the wrapper then tried to
  arm in OFFBOARD and PX4 rejected it with `Arming denied: Resolve system health
  failures first`; QGC also reported `Arming denied: switch to manual mode
  first`. Updated `scripts/run_real_rover_mavros_indoor_fake_vision_offboard_entry_test.sh`
  so it no longer auto-arms. It now starts fake external vision, waits for the
  operator to arm in MANUAL, and only then requests OFFBOARD with zero motion.
  After the failed auto-arm run, a read-only state check showed
  `connected=true`, `armed=false`, `guided=false`, `manual_input=false`,
  `mode=OFFBOARD`; restore/verify MANUAL and `manual_input=true` before the
  next attempt.
- 2026-06-20: User saw `mavros_fake_external_vision` continue printing after
  Ctrl+C at the shell prompt. Local `pgrep` later showed no fake-vision/offboard
  process left, but the integrated wrapper was still vulnerable because it
  started the fake-vision shell script in the background and cleanup only killed
  that shell PID. Updated
  `scripts/run_real_rover_mavros_indoor_fake_vision_offboard_entry_test.sh` to
  start fake external vision under `setsid` and cleanup now terminates the whole
  process group, with a SIGKILL fallback. `bash -n` passed. A read-only
  `/mavros/state` check at that moment did not find the topic, indicating MAVROS
  was not publishing/running; restart MAVROS and verify `MANUAL`, `armed=false`,
  `manual_input=true` before the next attempt.
- 2026-06-20 01:54 CST: User ran the updated manual-arm fake-vision Offboard
  entry flow. Start state was `connected=true`, `armed=false`,
  `manual_input=true`, `mode=MANUAL`; `COM_RC_IN_MODE=3`,
  `EKF2_EV_CTRL=15`, `COM_ARM_WO_GPS=1`. Fake external vision seeded from local
  pose and showed `subs=vision:1 vision_cov:1 odom:1`, with local pose tracking
  the published fake pose. User armed by RC while still in MANUAL; the script
  then requested OFFBOARD, PX4 accepted (`mode=OFFBOARD armed=True`), the
  zero-velocity sequence completed, and the script disarmed successfully by
  external command. This validates the indoor local-pose injection + MANUAL arm
  + Offboard entry path. Remaining issues: repeated PX4 health warnings for
  strong magnetic interference plus height/yaw estimate errors, and the old
  wrapper cleanup let fake vision continue until its 120s duration expired.
  Current script cleanup has since been changed to kill the fake-vision process
  group.
- 2026-06-20: Added
  `scripts/run_real_rover_mavros_indoor_fake_vision_offboard_motion_task.sh`
  for the next wheels-lifted test: fake external vision, manual arm in MANUAL,
  then script-requested OFFBOARD with small real setpoints
  (`LINEAR_SPEED_MPS=0.05`, `TURN_YAW_RATE_RADPS=0.12`, forward/backward 1s,
  turns 0.5s). The script refuses to start unless `/mavros/state` is `MANUAL`,
  `armed=false`, and `manual_input=true`; it also requires `COM_RC_IN_MODE=3`.
  Cleanup requests disarm, requests MANUAL, and kills the fake-vision process
  group. `bash -n` passed and the script is executable. This has not yet been
  run against hardware.
- 2026-06-20 02:05 CST: User ran the fake-vision Offboard motion task with
  wheels lifted. Start state was valid (`MANUAL`, `armed=false`,
  `manual_input=true`, `COM_RC_IN_MODE=3`). Fake external vision seeded and
  showed `subs=vision:1 vision_cov:1 odom:1`. User armed by RC; script requested
  OFFBOARD, PX4 accepted, and the motion sequence ran through forward/backward
  and left/right turn setpoints, then disarmed successfully and requested
  MANUAL. User reported the wheels never moved. Updated the motion script to
  start an integrated `mavros_rc_io_watch.py` process so the next run logs
  `/mavros/rc/out` changes in the same log. Also updated
  `src/mavros_fake_external_vision.py` to handle `ExternalShutdownException`
  cleanly when the wrapper stops the process group. Both `bash -n` and
  `py_compile` passed.
- 2026-06-20: User emphasized not bypassing PX4 because this rover is a
  transition state before aircraft work. Current diagnostic conclusion: manual
  mode proves the lower chain is good (`RC -> Pixhawk -> Arduino -> D24A ->
  motors`), and fake-vision Offboard entry proves `Orin/MAVROS -> PX4 Offboard`
  works. The missing segment is PX4 rover Offboard velocity setpoint conversion
  into actuator/PWM output. Updated `src/real_rover_mavros_offboard_smoke.py`
  so Offboard steps can publish `vx` and `vy`, not only `vx` plus `yaw_rate`.
  Updated `scripts/run_real_rover_mavros_offboard_smoke.sh` to pass
  `TURN_LINEAR_SPEED_MPS` and `TURN_LATERAL_SPEED_MPS`. Updated
  `scripts/run_real_rover_mavros_indoor_fake_vision_offboard_motion_task.sh`
  to set MAVROS `setpoint_velocity` to `BODY_NED` before the test and to use
  turn vectors (`TURN_LINEAR_SPEED_MPS=0.05`, `TURN_LATERAL_SPEED_MPS=0.03`,
  `TURN_YAW_RATE_RADPS=0.0`) because PX4 rover MAVLink velocity control ignores
  yaw/yaw-rate and derives yaw from velocity direction. The same wrapper still
  logs `/mavros/rc/out`; on the next run, if `rc/out` remains neutral, the
  issue is PX4 rover controller/setpoint interpretation rather than hardware.
  `py_compile` and `bash -n` passed.
- 2026-06-20 22:22 CST: Debugged the "left turn command drives right turn"
  hardware behavior on the real rover. Confirmed right-turn smoke with
  `BODY_NED`, `TURN_SIGN=-1.0`, `TURN_RIGHT_SEC=5.0`,
  `TURN_LINEAR_SPEED_MPS=0.05`, `TURN_YAW_RATE_RADPS=0.25`; user verified the
  motor behavior was right turn. The old left-turn path sent
  `turn_left vx=0.050 yaw_rate=-0.250` and later `vx=0.050 vy=+/-0.050
  yaw_rate=0`, but the vehicle still turned right; watcher evidence showed
  MAIN2 high (`~2000us`) for right-turn output. PX4 rejected MAVLink frames
  `BODY_FRD` (12) and `BODY_OFFSET_NED` (9) as unsupported. The successful real
  left-turn candidate was `SETPOINT_VELOCITY_MAV_FRAME=BODY_NED`,
  `TURN_LINEAR_SPEED_MPS=0.05`, `TURN_LINEAR_DIRECTION_SIGN=-1.0`,
  `TURN_LATERAL_SPEED_MPS=0.05`, `TURN_YAW_RATE_RADPS=0.0`,
  `TURN_SIGN=1.0`; this sends `turn_left vx=-0.050 vy=+0.050 yaw_rate=0`.
  User confirmed it physically turned left, with the right wheel forward.
  A follow-up 5s run also succeeded; `/mavros/rc/out` showed MAIN2 low
  (`~1010us`) during the left-turn segment and the script disarmed afterward.
  Final safety state was restored to `MANUAL`, `armed=false`, `mav_frame=BODY_NED`.
  Updated `src/real_rover_mavros_offboard_smoke.py` and
  `scripts/run_real_rover_mavros_offboard_smoke.sh` to support/pass
  `TURN_LINEAR_DIRECTION_SIGN`; also passed it through
  `scripts/run_real_rover_mavros_indoor_fake_vision_offboard_motion_task.sh`.
  `bash -n` and `py_compile` passed. Right-turn behavior with
  `TURN_LINEAR_DIRECTION_SIGN=-1.0` has not been separately validated, so keep
  using the previously confirmed right-turn command for right-turn-only tests.
- 2026-06-20 22:35 CST: Final wheels-lifted validation completed for real
  rover motion before wheel installation. First combined run revealed that the
  straight-line sign was reversed: the scripted `forward` step moved backward
  and scripted `backward` moved forward, while the right-turn segment was
  correct. Re-ran with `LINEAR_DIRECTION_SIGN=-1.0`; user confirmed the physical
  order was then forward, backward, right turn. Left-turn single-step validation
  using `BODY_NED`, `TURN_LINEAR_SPEED_MPS=0.05`,
  `TURN_LINEAR_DIRECTION_SIGN=-1.0`, `TURN_LATERAL_SPEED_MPS=0.05`,
  `TURN_YAW_RATE_RADPS=0.0`, `TURN_SIGN=1.0` was also confirmed correct by the
  user. Updated defaults in the real rover smoke/fake-vision/forward scripts
  and the smoke node parameter default so `linear_direction_sign` is now `-1.0`.
  Final confirmed safe state after the runs: `MANUAL`, `armed=false`,
  `manual_input=true`, setpoint velocity frame `BODY_NED`.
- 2026-06-20 22:55 CST: After installing wheels, user found RC could still
  arm/kill but could not command forward/back/steering. Diagnosis confirmed
  MAVROS state had recovered to `MANUAL` only after explicit mode request, but
  PX4 output mapping was still the Offboard rover-controller mapping:
  `PWM_MAIN_FUNC1=101`, `PWM_MAIN_FUNC2=201`, `PWM_MAIN_FUNC6=0`,
  `PWM_MAIN_FUNC7=0`. Restored the RC passthrough baseline while wheels were
  lifted: `PWM_MAIN_FUNC1=403`, `PWM_MAIN_FUNC2=405`,
  `PWM_MAIN_FUNC6=101`, `PWM_MAIN_FUNC7=101`,
  `PWM_MAIN_FAIL1=-1`, `PWM_MAIN_FAIL2=-1`; final checked state was
  `MANUAL`, `armed=false`, `manual_input=true`. Updated the MAVROS Offboard
  smoke wrapper and fake-vision Offboard wrappers so cleanup now requests
  disarm, requests MANUAL, and restores the RC passthrough output mapping by
  default (`AUTO_RESTORE_OUTPUT_MAPPING=true`). Future Offboard runs must pass
  `CONFIRM_QGC_DISARM_READY=true` and
  `CONFIRM_PHYSICAL_POWER_CUTOFF_READY=true`; do not leave the vehicle on
  `PWM_MAIN_FUNC1/2=101/201` after Offboard testing.
- 2026-06-20 23:05 CST: User still reported RC forward/steering did not drive
  the wheels after restoring output mapping. MAVROS/PX4 diagnostics showed the
  RC chain into PX4 and PX4 output chain were actually restored: while armed in
  MANUAL, `/mavros/rc/in` channel 2 and channel 4 followed the transmitter, and
  `/mavros/rc/out` channel 1 and channel 2 moved over large ranges
  (`rc/out1` roughly 1090-1900us, `rc/out2` roughly 1020-1918us). Arduino serial
  from `d24a_pixhawk_pwm_bridge.ino` then showed the real blocker:
  `thr_us=0` continuously while `steer_us` was valid around 1490us. Since the
  Arduino bridge stops all motors if either throttle or steering PWM input is
  invalid, the missing throttle input makes both forward/back and steering appear
  dead. Next hardware check: Pixhawk output 1 signal to Arduino D2 and the
  Pixhawk PWM ground to Arduino ground. `steer_us` being valid means output 2 to
  Arduino D13 is still connected. Final state after diagnosis:
  `MANUAL`, `armed=false`, `manual_input=true`.
- 2026-06-20 later: User reported the wire/check recovered and RC manual control
  is now working again. Treat the previous `thr_us=0` as a transient contact,
  Arduino reset/read, or wiring-seat issue rather than a persistent PX4 parameter
  problem. Final verified parameters remained the RC passthrough baseline
  (`COM_RC_IN_MODE=3`, `PWM_MAIN_FUNC1=403`, `PWM_MAIN_FUNC2=405`,
  `PWM_MAIN_FUNC6=101`, `PWM_MAIN_FUNC7=101`). After the user returned the RC
  arm switch to disarm, final checked state was `MANUAL`, `armed=false`,
  `manual_input=true`.
- 2026-06-20 23:25 CST: Outdoor GPS, wheels-on, ground Offboard crawl attempt
  was aborted. Commanded sequence was intended to be straight-only:
  `BODY_NED`, `LINEAR_SPEED_MPS=0.10`, `LINEAR_DIRECTION_SIGN=-1.0`,
  `FORWARD_SEC=20.0`, all backward/turn durations zero, estimated 2 m crawl.
  PX4 accepted ARM from RC and switched to OFFBOARD, but user observed the rover
  turning in circles / left-right steering instead of a clean straight crawl and
  hit kill. The script detected disarm and aborted. Do not run further
  wheels-on Offboard motion tests until controller/setpoint mapping is
  diagnosed on a stand or with a much safer restraint. Cleanup restored
  `MANUAL`, `armed=false`, `manual_input=true`, and the RC passthrough baseline:
  `PWM_MAIN_FUNC1=403`, `PWM_MAIN_FUNC2=405`, `PWM_MAIN_FUNC6=101`,
  `PWM_MAIN_FUNC7=101`, `PWM_MAIN_FAIL1=-1`, `PWM_MAIN_FAIL2=-1`.
  Offboard log:
  `results/offboard/20260620_232015/offboard.log`; RC watcher log:
  `results/rc_watch/20260620_232001/rc_watch.log`. The watcher saw Offboard-era
  `/mavros/rc/out` steering excursions including `rc/out2` up to about 2000us,
  matching the observed unintended turning.
- 2026-06-20 23:35 CST diagnosis: root cause is likely controller/vehicle-model
  mismatch, not action sequencing. MAVROS Offboard uses
  `/mavros/setpoint_velocity/cmd_vel` and PX4's rover controller outputs
  `Motor 1` plus `Servo 1`. Local QGC metadata confirms `GND_MAX_ANG` is for
  Ackermann steering, while the real vehicle uses Arduino throttle/steering PWM
  as a four-wheel differential mixer. Therefore PX4 `Servo 1` steering
  excursions become aggressive differential turning. Additional contributors:
  the test used `vx=-0.10` to get physical forward motion, which is reverse
  velocity semantics inside PX4, and logs repeatedly show `Strong magnetic
  interference` / prior `Yaw estimate error`, so heading-based steering can
  oscillate. Added `apply-limited` to
  `scripts/set_px4_rover_output_mapping.sh`: it maps MAIN1/MAIN2 to PX4
  Motor1/Servo1 but narrows PWM limits (`MAIN1 1400-1600`, `MAIN2 1450-1550`)
  for wheels-lifted diagnostics. `restore-baseline` now also restores MAIN1/2
  min/max to `1000-2000`. Next live diagnostic must be wheels-lifted: apply
  `apply-limited`, run a 2s `vx=+0.05` Offboard test, watch `/mavros/rc/out`,
  and confirm whether positive PX4 velocity produces physical forward/backward
  while steering remains constrained. Do not run another wheels-on Offboard
  crawl before that.
- 2026-06-20 23:55 CST: Wheels-lifted `apply-limited` diagnostic with
  `vx=+0.05` did not spin the wheels. Logs show PX4 accepted ARM/OFFBOARD and
  executed the sequence, but `/mavros/rc/out` summary was
  `out1 min=max 1500` and `out2 1480-1520`; therefore PX4 produced steering
  output but no throttle for that low positive velocity setpoint. This explains
  the stand test: it was not a motor/wheel wiring failure in that run. The
  earlier wheels-on crawl log also shows the unsafe behavior directly:
  `out1` only reached about `1551us` while `out2` swept `1000-2000us`, matching
  user-observed circling/left-right steering. Treat the current root cause as
  PX4 Ackermann rover controller output being mismatched to the four-wheel
  differential Arduino mixer, amplified by heading/magnetic estimator errors
  and by using negative velocity to mean physical forward.
- 2026-06-20 23:58 CST: MAVROS repeatedly became unstable around parameter
  refreshes. The crash log ended with `std::future_error: Promise already
  satisfied`; the local symptom is `mavros_node` aborting during or after
  repeated `ParamPull(force_pull=true)` / parameter service calls on a high-RTT
  link. Updated `scripts/set_px4_rover_output_mapping.sh`,
  `scripts/set_px4_com_rc_in_mode.sh`, and
  `scripts/set_px4_rover_controller_offboard_mapping.sh` so forced full
  post-change parameter pulls are no longer the default; they now rely on
  successful set responses and key cached reads unless an explicit environment
  override is set. Final verified safe baseline after the interrupted test:
  `MANUAL`, `armed=false`, `manual_input=true`, `COM_RC_IN_MODE=3`,
  `PWM_MAIN_FUNC1=403`, `PWM_MAIN_FUNC2=405`, `PWM_MAIN_FUNC6=101`,
  `PWM_MAIN_FUNC7=101`, `PWM_MAIN_MIN1/2=1000`, `PWM_MAIN_MAX1/2=2000`,
  `PWM_MAIN_FAIL1=-1`, `PWM_MAIN_FAIL2=-1`.
- 2026-06-21 01:40 CST: After setting valid `SYS_AUTOSTART=50000`, PX4 applied
  Generic Ground Vehicle Ackermann (`MAV_TYPE=10`, `CA_AIRFRAME=5`). This
  changed `PWM_MAIN_FUNC1/2=201`, `PWM_MAIN_FUNC6/7=101`, and the default
  `PWM_MAIN_DIS1/2/6/7=1000`. On this Arduino differential-drive bridge,
  `1500us` is neutral, so disarmed `1000us` on MAIN6/7 made the right front and
  right rear wheels spin while the vehicle was disarmed. Restored live safe
  values via PX4 NSH shell, not MAVLink `PARAM_SET`: `MANUAL`, disarmed,
  `COM_RC_IN_MODE=3`, `PWM_MAIN_FUNC1=403`, `PWM_MAIN_FUNC2=405`,
  `PWM_MAIN_FUNC6=101`, `PWM_MAIN_FUNC7=101`, and
  `PWM_MAIN_DIS1/2/6/7=1500`, `PWM_MAIN_FAIL1/2/6/7=1500`. Also updated
  `scripts/set_px4_rover_output_mapping.sh` so `restore-baseline` keeps
  MAIN6/7 disarmed/failsafe values at `1500`, avoiding the same right-wheel
  spin on future restores. Do not use QGC's listed Aion airframe on this
  firmware: `/etc/init.d/airframes` does not contain
  `50003_aion_robotics_r1_rover`; the correct long-term path is a PX4 rover
  custom firmware with differential rover support.
- 2026-06-21 01:58 CST: User explicitly requested full PX4 v1.16 firmware, not
  the separate rover build. Flashed official PX4 v1.16.1
  `px4_fmu-v6c_default.px4` to the Pixhawk 6C with PX4's v1.16.1
  `Tools/px_uploader.py`. Upload succeeded with erase/program/verify/reboot.
  Verified via NSH `ver all`: `PX4 version: 1.16.1 c0`, git hash
  `94cb2012792b2ae89f0b147cfee53ee31ae550be`, `Build variant: default`,
  hardware `PX4_FMU_V6C`. MAVROS also reports flight software `011001c0`.
  Final state after flashing: MAVROS connected, `MANUAL`, `armed=false`,
  `manual_input=true`; `SYS_AUTOSTART=0`, `MAV_TYPE=0`, `CA_AIRFRAME` unset;
  safe outputs retained with `COM_RC_IN_MODE=3`,
  `PWM_MAIN_FUNC1=403`, `PWM_MAIN_FUNC2=405`, `PWM_MAIN_FUNC6=0`,
  `PWM_MAIN_FUNC7=0`, and `PWM_MAIN_DIS1/2/6/7=1500`,
  `PWM_MAIN_FAIL1/2/6/7=1500`. Full/default v1.16.1 ROMFS contains rover
  airframes `59000_generic_ground_vehicle` and `59001_nxpcup_car_dfrobot_gpx`,
  but no Aion airframe; `59000` is still Ackermann (`CA_AIRFRAME=5`).
  Do not set `SYS_AUTOSTART=59000` with autoconfig unless ready to immediately
  re-apply neutral disarmed/failsafe outputs.
- 2026-06-21 02:15 CST: User asked to flash full/default PX4 v1.17.0 after
  checking whether v1.17 default includes differential rover. Flashed official
  `px4_fmu-v6c_default.px4` from PX4 v1.17.0 with the PX4 uploader. Upload
  succeeded with bootloader erase/program/verify/reboot. Verified on the
  vehicle via NSH `ver all`: `PX4 version: Release 1.17.0`, git hash
  `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`, `Build variant: default`,
  hardware `PX4_FMU_V6C`; MAVROS reports flight software `011100ff`. Important
  correction: the flashed `px4_fmu-v6c_default.px4` ROMFS does **not** contain
  rover airframes despite the upstream source tree listing them. On-device
  `/etc/init.d/airframes` has no `50000_generic_rover_differential`, and
  `/etc/init.d/rc.rover_differential_defaults` is also absent. Therefore full
  v1.17.0 default still does not provide a usable differential rover airframe on
  this board target. Final verified safe state: MAVROS connected, `MANUAL`,
  `armed=false`, `manual_input=true`; `SYS_AUTOSTART=0`, `MAV_TYPE=0`,
  `CA_AIRFRAME` unset; safe outputs retained with `COM_RC_IN_MODE=3`,
  `PWM_MAIN_FUNC1=403`, `PWM_MAIN_FUNC2=405`, `PWM_MAIN_FUNC6=0`,
  `PWM_MAIN_FUNC7=0`, `PWM_MAIN_DIS1/2/6/7=1500`, and
  `PWM_MAIN_FAIL1/2/6/7=1500`. To get differential rover, the next firmware to
  flash is the dedicated `px4_fmu-v6c_rover.px4`, not default.
- 2026-06-21 02:32 CST: Flashed official PX4 v1.17.0
  `px4_fmu-v6c_rover.px4` to the Pixhawk 6C. Upload succeeded
  erase/program/verify/reboot. After reboot the Linux USB controller initially
  did not re-enumerate the board until the user rebooted Linux; the board was
  not bricked. Verified on-device via NSH: `PX4 version: Release 1.17.0`, git
  hash `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`, `Build variant: rover`,
  hardware `PX4_FMU_V6C`. The rover ROMFS contains
  `50000_generic_rover_differential`, `50001_aion_robotics_r1_rover`,
  `51000_generic_rover_ackermann`, `51001_axial_scx10_2_trail_honcho`,
  `51002_nxp_b3rb`, and `52000_generic_rover_mecanum`. Set and saved the
  differential rover airframe with PX4 NSH shell, not MAVROS ParamSet:
  `SYS_AUTOSTART=50000`, `SYS_AUTOCONFIG=0`, `MAV_TYPE=10`,
  `CA_AIRFRAME=6`, `CA_R_REV=3`, `COM_RC_IN_MODE=3`. Rebooted and re-verified
  via MAVROS/NSH. Final state: MAVROS connected, `MANUAL`, `armed=false`,
  `manual_input=true`, `/mavros/rc/out` channels start at `[1500, 1500, 0...]`,
  `PWM_MAIN_FUNC1=403`, `PWM_MAIN_FUNC2=405`, `PWM_MAIN_FUNC6=0`,
  `PWM_MAIN_FUNC7=0`, and `PWM_MAIN_DIS1/2/6/7=1500`,
  `PWM_MAIN_FAIL1/2/6/7=1500`. Next step before any wheel motion: decide the
  correct actuator output mapping for differential rover to the Arduino bridge,
  keep wheels lifted, and verify tiny manual/offboard outputs.
- 2026-06-21 02:40 CST: User confirmed wiring stays unchanged and asked to
  restore the tested `RC -> PX4 -> motors` chain. Configured the live Pixhawk
  through PX4 NSH shell and saved the safe two-output RC passthrough baseline:
  `SYS_AUTOSTART=50000`, `SYS_AUTOCONFIG=0`, `MAV_TYPE=10`,
  `CA_AIRFRAME=6`, `CA_R_REV=3`, `COM_RC_IN_MODE=3`,
  `PWM_MAIN_FUNC1=403`, `PWM_MAIN_FUNC2=405`, `PWM_MAIN_FUNC6=0`,
  `PWM_MAIN_FUNC7=0`, `PWM_MAIN_MIN1/2=1000`, `PWM_MAIN_MAX1/2=2000`,
  and `PWM_MAIN_DIS1/2/6/7=1500`, `PWM_MAIN_FAIL1/2/6/7=1500`. MAVROS was
  restarted with `scripts/run_mavros_px4_usb_to_qgc_logged.sh`; verified
  `/mavros/state` as `connected=true`, `armed=false`, `manual_input=true`,
  `mode=MANUAL`, and `/mavros/rc/out` as `[1500, 1500, 0, ...]` at stick
  neutral. Updated `scripts/set_px4_rover_output_mapping.sh` so
  `restore-baseline` now disables MAIN6/7 and sets all disarmed/failsafe
  values to 1500, and documented the current PX4 output baseline in
  `docs/current_rover_success_baseline_2026_06_16.md`.
- 2026-06-21 02:50 CST correction: User clarified the current transmitter
  channels are `CH2=forward/backward` and `CH4=steering`. Restored live
  `PWM_MAIN_FUNC2=405` (`RC Yaw`) after a mistaken temporary test of
  `RC Roll`; saved parameters and verified `PWM_MAIN_FUNC1=403`,
  `PWM_MAIN_FUNC2=405`, `RC_MAP_PITCH=2`, `RC_MAP_YAW=4`, `MANUAL`,
  `armed=false`, and `/mavros/rc/out=[1500,1500,0,...]`. Treat this as the
  current recoverable manual RC baseline. Deleted
  `docs/at9s_pro_r9ds_d24a_rc_wiring.md` and
  `arduino/d24a_r9ds_rc_drive/d24a_r9ds_rc_drive.ino`, the old direct
  R9DS-to-Arduino CH1 steering artifacts, and updated
  `docs/current_rover_success_baseline_2026_06_16.md` to make CH2/CH4 explicit.
- 2026-06-21 later correction: User tested the `403/405` passthrough mapping
  and reported it was still wrong: CH2 controlled steering (up=left, down=right)
  and CH4 controlled throttle (left=forward, right=backward). Swapped the live
  PX4 passthrough functions and saved parameters:
  `PWM_MAIN_FUNC1=405` and `PWM_MAIN_FUNC2=403`, with `PWM_MAIN_FUNC6=0` and
  `PWM_MAIN_FUNC7=0`. Verified after saving: `MANUAL`, `armed=false`,
  `/mavros/rc/out=[1500,1500,0,...]`. Updated
  `scripts/set_px4_rover_output_mapping.sh`,
  `scripts/set_px4_rover_controller_offboard_mapping.sh`, and
  `docs/current_rover_success_baseline_2026_06_16.md` so future baseline
  restores use the observed working cross-mapping.
- 2026-06-21 night: User asked for PX4 v1.17 differential-rover Offboard test
  scripts for (1) forward/back/left/right each 5 seconds and (2) a true
  out-and-back mission: forward 5 m, turn 180 degrees, forward 5 m. Added
  `apply-differential` and `apply-differential-limited` actions to
  `scripts/set_px4_rover_output_mapping.sh`; these set `PWM_MAIN_FUNC1=101`,
  `PWM_MAIN_FUNC2=102`, `PWM_MAIN_FUNC6=0`, `PWM_MAIN_FUNC7=0`, and
  `CA_R_REV=3`, then rely on cleanup restore-baseline to return to
  `405/403`. Added Arduino sketch
  `arduino/d24a_pixhawk_differential_pwm_bridge/d24a_pixhawk_differential_pwm_bridge.ino`
  for left/right PWM command semantics. Added wheels-lifted script
  `scripts/run_real_rover_mavros_differential_fake_vision_offboard_5s_sequence.sh`
  and true local-pose script
  `scripts/run_real_rover_mavros_differential_offboard_out_and_back_5m.sh`
  plus Python node `src/real_rover_mavros_offboard_out_and_back.py`. Important:
  the 5-second script uses fake vision only for Offboard entry and does not
  prove distance; the 5 m script requires real `/mavros/local_position/pose`
  and must not be run with fake vision. All new motion scripts require explicit
  confirmations including `CONFIRM_ARDUINO_DIFFERENTIAL_PWM_BRIDGE=true`.
- 2026-06-21 13:45 CST: First wheels-lifted differential Offboard forward-5s
  test entered `OFFBOARD`, armed by RC, ran `step -> forward` for 5 seconds,
  and disarmed, but the user reported the motors did not spin. The test used
  `apply-differential-limited` (`PWM_MAIN_FUNC1=101`, `PWM_MAIN_FUNC2=102`,
  `PWM_MAIN_MIN/MAX1/2=1400/1600`) and the initial Arduino differential bridge
  deadband was `DEAD_BAND_US=80`, which can swallow small PX4 output movement.
  Changed
  `arduino/d24a_pixhawk_differential_pwm_bridge/d24a_pixhawk_differential_pwm_bridge.ino`
  to `DEAD_BAND_US=35`, rebuilt manually with `avr-gcc`/`avr-g++`, and flashed
  the UNO successfully with `avrdude`. Serial verification after flashing:
  `D24A Pixhawk differential PWM bridge ready` and neutral inputs around
  `1488-1495us` still produce `left=0 right=0`.
- 2026-06-21 13:45 CST: A second monitored forward-5s attempt did not reach
  motion. It started Arduino serial plus `/mavros/rc/out` monitors, then the
  wrapper aborted during the `apply-differential-limited` parameter phase after
  a MAVROS parameter-service timeout near `PWM_MAIN_MIN2`. MAVROS then exited
  and was restarted with `scripts/run_mavros_px4_usb_to_qgc_logged.sh`. Monitor
  logs showed only neutral Arduino inputs and `/mavros/rc/out=[1500,1500,...]`;
  the vehicle was never armed for that second attempt. Recovered final safe
  baseline through direct `/mavros/param/set` calls and verified
  `MANUAL`, `armed=false`, `manual_input=true`,
  `PWM_MAIN_FUNC1=405`, `PWM_MAIN_FUNC2=403`, `PWM_MAIN_FUNC6=0`,
  `PWM_MAIN_FUNC7=0`, `PWM_MAIN_MIN1/2=1000`, `PWM_MAIN_MAX1/2=2000`,
  `PWM_MAIN_DIS1/2=1500`, and `PWM_MAIN_FAIL1/2=1500`. The next retry should
  avoid the full wrapper: directly set differential output parameters with long
  timeouts, run `run_real_rover_mavros_offboard_smoke.sh` with monitors, then
  restore the baseline through direct parameter sets.
- 2026-06-21 14:00 CST: User asked to stop and fix MAVROS crashing. Logs show
  the crash is not a normal serial disconnect: `mavros_node` aborts with
  `std::future_error: Promise already satisfied` after `/mavros/param` service
  timeouts/failed responses on a high-RTT link. Installed MAVROS is
  `ros-humble-mavros 2.14.0`, so apt has no newer candidate. Added
  `config/mavros_px4_pluginlists_no_param.yaml` and changed
  `scripts/run_mavros_px4_usb_to_qgc.sh` to launch `mavros node.launch`
  directly with that pluginlist by default (`MAVROS_DISABLE_PARAM_PLUGIN=true`).
  The log now shows `Plugin param ignored`; `/mavros/param/*` services are
  intentionally absent, while `/mavros/state` is connected and QGC UDP routing
  remains active. For future work, do not use `/mavros/param/get|set|pull` in
  motion scripts. Change PX4 params through QGC, PX4 shell, or a separate
  MAVLink/serial tool while MAVROS is stopped or while using a non-conflicting
  MAVLink endpoint. Temporary override: start MAVROS with
  `MAVROS_DISABLE_PARAM_PLUGIN=false` only for short parameter sessions, then
  restart no-param before Offboard work.
- 2026-06-21 14:15 CST: Diagnosed the no-motion Offboard forward-5s attempt.
  The script did enter `OFFBOARD`, waited for RC arming, ran the `forward`
  step for 5 s, then disarmed, but `/mavros/rc/out` and Arduino serial stayed
  neutral (`1500` / `left=0 right=0`). PX4 v1.17 source shows the QGC error
  `Invalid configuration for rate control: Neither feed forward nor feedback is
  setup` comes from
  `src/modules/rover_differential/DifferentialRateControl`: it refuses rate
  control if `RD_WHEEL_TRACK <= 0` or `RO_MAX_THR_SPEED <= 0` and
  `RO_YAW_RATE_P <= 0`. Direct MAVLink parameter reads confirmed the controller
  defaults were still unset: `RD_WHEEL_TRACK=0`, `RO_MAX_THR_SPEED=0`,
  `RO_SPEED_LIM=-1`, `RO_YAW_RATE_LIM=0`, `RO_YAW_P=0`, and
  `RO_YAW_RATE_P=0`. Therefore the no-motion root cause was PX4 rover
  controller sanity checks blocking actuator output, not fake vision, GPS, or
  Arduino deadband. Added `scripts/px4_mavlink_param.py` for direct MAVLink
  parameter reads/writes while MAVROS param plugin is disabled.
- 2026-06-21 14:18 CST: With vehicle disarmed and outputs neutral, stopped
  MAVROS, wrote a wheels-lifted differential rover Offboard config directly
  over Pixhawk USB, saved params via `MAV_CMD_PREFLIGHT_STORAGE`, and restarted
  no-param MAVROS. Verified final PX4 params:
  `SYS_AUTOSTART=50000`, `CA_AIRFRAME=6`, `CA_R_REV=3`,
  `PWM_MAIN_FUNC1=101`, `PWM_MAIN_FUNC2=102`, `PWM_MAIN_FUNC6/7=0`,
  `PWM_MAIN_MIN/MAX1/2=1400/1600`, `PWM_MAIN_DIS1/2/6/7=1500`,
  `PWM_MAIN_FAIL1/2/6/7=1500`, `RD_WHEEL_TRACK=0.30`,
  `RO_MAX_THR_SPEED=0.50`, `RO_SPEED_LIM=0.35`, `RO_YAW_RATE_LIM=90`,
  `RO_YAW_P=1.0`, `RO_SPEED_P/I=0`, `RO_YAW_RATE_P/I=0`,
  `RO_YAW_RATE_CORR=1`. The deliberately low `RO_MAX_THR_SPEED=0.50` is for
  the current 1400-1600 us limited stand test so a `0.25 m/s` setpoint maps to
  about `1550 us`, above the Arduino `35 us` deadband. Current MAVROS:
  no-param plugin active, `/mavros/state connected=true armed=false
  mode=MANUAL manual_input=true`, `/mavros/rc/out=[1500,1500,0,...]`, and only
  `/mavros/cmd/arming` + `/mavros/set_mode` are available, no
  `/mavros/param/*`. Also updated
  `scripts/run_real_rover_mavros_offboard_smoke.sh` cleanup to skip
  output-mapping restore when MAVROS param services are unavailable.
- 2026-06-21 14:20 CST: Ran the confirmed wheels-lifted MAVROS Offboard
  forward-5s test after the rover controller parameter fix. Log directory:
  `results/differential_offboard_5s/forward5_paramfix_20260621_141916/`.
  The script held stop until RC arming, requested Offboard successfully
  (`mode_sent=True`), entered `OFFBOARD`, ran `forward` for 5 seconds at
  `vx=0.25`, then disarmed by external command. PX4 no longer emitted the
  previous `Invalid configuration for rate control` message. Arduino serial
  confirmed PX4 output is now reaching MAIN1/MAIN2: during the forward segment
  inputs rose to about `in1_us=1538-1545`, `in2_us=1538-1544`, producing
  `left/right` commands around `+11` to `+14`; stop/disarm returned to neutral
  `1488-1495` and `left=0 right=0`. Therefore the original no-motion bug is
  fixed at the PX4-controller-output level. If the wheels still do not visibly
  spin, the remaining issue is output magnitude too small for the motor/driver
  load; increase test authority conservatively by either lowering
  `RO_MAX_THR_SPEED` for stand testing or widening `PWM_MAIN_MIN/MAX1/2`, while
  keeping wheels lifted and disarm/failsafe at `1500`. Current final state after
  test: `MANUAL`, `armed=false`, `manual_input=true`, `/mavros/rc/out` publishing
  `[1500,1500,0,...]`.
- 2026-06-21 14:32 CST: User reported the forward-5s stand test spun slowly and
  seemed to go forward/back/forward. Log analysis found the moving Arduino
  samples were split into negative one-wheel corrections before/after the real
  positive forward segment. PX4 v1.17 differential rover velocity Offboard maps
  every velocity setpoint to both speed and yaw; a zero stop setpoint becomes
  `yaw_setpoint=atan2(0,0)=0`, so stop phases can command heading correction
  toward yaw zero instead of being a pure neutral stop. Also the smoke sequence
  kept zero-duration backward/turn steps plus their stop phases, making the
  test look much more chaotic than the intended forward-only run. Fixed
  `src/real_rover_mavros_offboard_smoke.py` so zero-duration actions and their
  stop phases are omitted, and added `INITIAL_STOP_SEC` support in
  `scripts/run_real_rover_mavros_offboard_smoke.sh`. For the next wheels-lifted
  forward-only retest, use `INITIAL_STOP_SEC=0.0`, `STOP_SEC=0.2`,
  `FINAL_STOP_SEC=0.2`, `BACKWARD_SEC=0`, `TURN_LEFT_SEC=0`,
  `TURN_RIGHT_SEC=0`. To make the stand output visibly stronger while retaining
  1400-1600 us PWM limits, changed and saved PX4 params
  `RO_MAX_THR_SPEED=0.25` and `RO_YAW_P=0.25` via direct MAVLink param tool;
  restarted no-param MAVROS. Current state after the change: `MANUAL`,
  `armed=false`, `/mavros/rc/out=[1500,1500,0,...]`.
- 2026-06-21 14:39 CST: User requested more actuator authority because
  `1400-1600` was still too weak for the small rover wheels. With the vehicle
  disarmed, stopped MAVROS, changed and saved differential test output limits to
  `PWM_MAIN_MIN1=1300`, `PWM_MAIN_MAX1=1700`, `PWM_MAIN_MIN2=1300`,
  `PWM_MAIN_MAX2=1700`; verified `PWM_MAIN_DIS1/2=1500` and
  `PWM_MAIN_FAIL1/2=1500` remain unchanged. Restarted no-param MAVROS and
  verified final state `MANUAL`, `armed=false`, `/mavros/rc/out=[1500,1500,...]`.
  Next forward-only retest should use the cleaned sequence:
  `INITIAL_STOP_SEC=0`, `STOP_SEC=0.2`, `FINAL_STOP_SEC=0.2`,
  `FORWARD_SEC=5`, `BACKWARD_SEC=0`, `TURN_LEFT_SEC=0`, `TURN_RIGHT_SEC=0`.
- 2026-06-21 14:42 CST: Ran the cleaned wheels-lifted forward-only 5 s retest
  with `PWM_MAIN_MIN/MAX1/2=1300/1700`. Log directory:
  `results/differential_offboard_5s/forward5_clean_1300_1700_20260621_143918/`.
  Sequence contained only `forward 5.00s`, `stop_after_forward 0.20s`, and
  `final_stop 0.50s`; no backward/turn steps. PX4 entered OFFBOARD, ran the
  sequence, and disarmed successfully. Arduino serial showed a strong stable
  forward segment: `in1_us/in2_us` mostly `1688-1695`, Arduino `left/right`
  commands `+58` to `+60`. Only 3 samples had a negative correction during
  transition/stop; the 5 s forward body was both wheels positive. Final state
  verified: `MANUAL`, `armed=false`, repeated `/mavros/rc/out=[1500,1500,0,...]`.
- 2026-06-21 14:46 CST: User observed that the cleaned "forward 5 s" physical
  wheel motion was actually backward for 5 s. Diagnosis: PX4 differential
  offboard and MAIN1/MAIN2 mapping are working, but the physical motor direction
  sign is inverted. With MAVROS stopped and the vehicle disarmed, changed and
  saved PX4 `PWM_MAIN_REV=3` to reverse MAIN1 and MAIN2 together while leaving
  `PWM_MAIN_FUNC1=101`, `PWM_MAIN_FUNC2=102`, and `PWM_MAIN_MIN/MAX1/2=1300/1700`
  unchanged. Restarted no-param MAVROS and verified `connected=true`,
  `mode=MANUAL`, `armed=false`, `/mavros/rc/out=[1500,1500,0,...]`. Next test:
  rerun the same wheels-lifted forward-only 5 s offboard smoke. After this
  reversal, Arduino command signs may appear negative during physical forward;
  trust physical wheel direction for the final direction check.
- 2026-06-21 14:49 CST: Reran the wheels-lifted forward-only 5 s offboard smoke
  after setting `PWM_MAIN_REV=3`. Run directory:
  `results/differential_offboard_5s/forward5_pwm_main_rev3_20260621_144809/`.
  PX4 entered OFFBOARD, armed, executed only `forward 5.00s` plus short stop
  phases, disarmed, and returned to `MANUAL`, `armed=false`. User confirmed the
  rover physically moved forward for 5 s correctly. Next requested test is a
  wheels-lifted backward-only 5 s offboard smoke using the same mapping and
  speed, with `FORWARD_SEC=0`, `BACKWARD_SEC=5`, `LINEAR_DIRECTION_SIGN=1.0`.
- 2026-06-21 14:53 CST: Ran the wheels-lifted backward-only 5 s offboard smoke
  with the corrected `PWM_MAIN_REV=3` mapping. Run directory:
  `results/differential_offboard_5s/backward5_pwm_main_rev3_20260621_145223/`.
  Sequence contained only `backward 5.00s` (`vx=-0.250`) plus short stop phases.
  PX4 entered OFFBOARD, armed, executed the sequence, disarmed, and subsequent
  state check showed `mode=MANUAL`, `armed=false`, `/mavros/rc/out=[1500,1500,0,...]`.
  Log scan showed no rate-control configuration errors and no ARM/mode reject.
  Awaiting user's physical confirmation that wheel direction was backward.
- 2026-06-21 15:02 CST: User reported the backward-only test still looked like
  forward motion and was chaotic. Root cause found in PX4 v1.17 source, not in
  wiring or Arduino. MAVROS `/mavros/setpoint_velocity/cmd_vel` sends
  `SET_POSITION_TARGET_LOCAL_NED`; PX4 converts BODY_NED velocity into local NED
  in `mavlink_receiver.cpp`, then differential rover offboard velocity mode in
  `DifferentialOffboardMode.cpp` does
  `rover_speed_setpoint.speed_body_x = velocity_ned.norm()` and
  `yaw_setpoint = atan2f(velocity_ned(1), velocity_ned(0))`. This discards the
  sign of body `vx`. Therefore a commanded `vx=-0.25` becomes positive speed
  with a 180-degree yaw target, so the rover tries to drive forward while turning
  around instead of reversing. The same MAVROS velocity path also ignores
  `cmd_vel.angular.z` in the velocity branch, so it is not suitable for true
  reverse or in-place left/right spin tests on stock PX4 v1.17 differential
  rover. `PWM_MAIN_REV=3` remains correct because user confirmed the forward-only
  5 s test physically moved forward. Do not use the current velocity offboard
  smoke script for backward/turn validation without changing approach.
- 2026-06-21 15:15 CST: Added a conservative ground L-turn Offboard test for the
  small-rover phase:
  `src/real_rover_mavros_offboard_l_turn.py` and
  `scripts/run_real_rover_mavros_differential_offboard_l_turn.sh`. This does
  not attempt in-place turning. It sets MAVROS `setpoint_velocity` to
  `LOCAL_NED`, captures current local pose/yaw, drives a short first leg along
  the current heading, commands a local velocity vector 90 degrees to the right,
  then drives a short second leg along that new heading. Defaults are
  `FIRST_DISTANCE_M=0.5`, `SECOND_DISTANCE_M=0.5`, `LINEAR_SPEED_MPS=0.12`,
  `TURN_DIRECTION_SIGN=-1`, `TURN_MAX_SEC=6`, and auto OFFBOARD/ARM enabled.
  The wrapper requires explicit ground-test, local-position, RC, QGC disarm,
  physical cutoff, current differential mapping, and wheels-installed
  confirmations before it will run; cleanup always requests disarm and MANUAL.
- 2026-06-21 15:24 CST: Ran the first ground L-turn test after user confirmed
  ready. Run directory:
  `results/differential_offboard_l_turn/l_turn_right_20260621_152259/`.
  Vehicle entered OFFBOARD, armed, and disarmed at the end. Post-run state
  verified `mode=MANUAL`, `armed=false`. Log showed first forward leg worked:
  `leg1 progress=0.35m elapsed=1.1s`, then it entered the right-turn-arc stage.
  During the turn stage the yaw error did not decrease (`73.2deg -> 98.6deg`)
  and the stage exited by `TURN_MAX_SEC=6.0`; second-leg progress stayed `0.00m`
  until timeout. This indicates the L-turn local velocity/yaw sign or coordinate
  transform is wrong for the real setup, or the local heading estimate is not
  aligned with the commanded local velocity. Do not rerun unchanged. Ask user
  what the physical path looked like; if it physically turned the opposite way,
  the next test should likely use `TURN_DIRECTION_SIGN=+1.0` with shorter
  distances/timeouts.
- 2026-06-21 15:39 CST: Ran the second ground L-turn test after fixing the
  wrapper's `ros2 topic list | grep -q` pipefail false-negative. Run directory:
  `results/differential_offboard_l_turn/l_turn_right_sign_plus_retry_20260621_153844/`.
  Settings: `FIRST_DISTANCE_M=0.3`, `SECOND_DISTANCE_M=0.3`,
  `LINEAR_SPEED_MPS=0.10`, `TURN_DIRECTION_SIGN=+1.0`, `TURN_MAX_SEC=4.0`.
  It entered OFFBOARD, armed, completed, disarmed, and cleanup requested MANUAL.
  Latest post-run check showed `mode=MANUAL`, `armed=false`,
  `/mavros/rc/out=[1500,1500,0,...]`; no L-turn script process remained. Log
  showed the turn still did not converge (`yaw_error=122deg -> 179deg`) and
  second-leg progress only reached `0.13m` before timeout. Need user's physical
  observation before deciding the next direction/coordinate fix; do not run more
  ground motion unchanged.
- 2026-06-21 16:xx CST: User observed the old L-turn behavior was wrong: the
  rover first corrected yaw, then moved forward, then turned. Root cause is the
  LOCAL_NED L-turn approach, which commanded a global velocity vector and caused
  PX4 differential rover Offboard to align yaw before driving. Replaced
  `src/real_rover_mavros_offboard_l_turn.py` with a body-frame L-turn task and
  updated `scripts/run_real_rover_mavros_differential_offboard_l_turn.sh` to
  default `SETPOINT_VELOCITY_MAV_FRAME=BODY_NED`. New defaults are
  `FIRST_DISTANCE_M=3.0`, `SECOND_DISTANCE_M=3.0`, `LINEAR_SPEED_MPS=0.12`,
  `TURN_DIRECTION_SIGN=-1.0`, `TURN_LATERAL_SPEED_MPS=0.10`,
  `TURN_FORWARD_SPEED_MPS=0.0`, `TURN_ANGLE_DEG=90`, and `TURN_MAX_SEC=12`.
  Straight legs publish body velocity `(x=forward, y=0)` so PX4 should not
  pre-correct yaw; the turn stage publishes body lateral velocity and uses the
  real local yaw only to decide when the 90-degree heading change is reached.
  Verification passed: `python3 -m py_compile src/real_rover_mavros_offboard_l_turn.py`
  and `bash -n scripts/run_real_rover_mavros_differential_offboard_l_turn.sh`.
  First ground run after user confirmation:
  `results/differential_offboard_l_turn/body_l_turn_left_20260621_161820/`.
  The body-frame fix worked for the straight legs: the rover did not pre-correct
  yaw before driving; leg1 reached about `2.73m` before stop tolerance and leg2
  reached about `2.84m`. However `TURN_DIRECTION_SIGN=+1.0` produced local yaw
  delta about `-90deg`; user confirmed the physical path was forward, right
  turn, then forward. Therefore on this PX4/MAVROS BODY_NED setup `+1` is
  right-turn direction, and the old yaw-progress sign caused the turn stage to
  exit by timeout instead of by angle. Fixed the script after this run: default
  `TURN_DIRECTION_SIGN=-1.0` for left turn, display text says `-1 means left`,
  and `_signed_turn_delta_rad()` now uses `-turn_direction_sign * yaw_delta` so
  a 90-degree turn can stop by angle. Final safety check after the run showed
  `/mavros/state`: `connected=true`, `armed=false`, `manual_input=true`,
  `mode=MANUAL`, `system_status=3`. Require fresh user confirmation before any
  second ground run. Immediately after the user reported the physical path,
  `/mavros/state` showed `MANUAL` but `armed=true` from an RC switch arm event;
  sent `/mavros/cmd/arming value=false`, which succeeded. Verified final state
  again as `connected=true`, `armed=false`, `manual_input=true`, `mode=MANUAL`,
  `system_status=3`.
- 2026-06-21 16:29 CST: Ran the corrected body-frame left L-turn after fresh
  user confirmation. Run directory:
  `results/differential_offboard_l_turn/body_l_turn_left_sign_minus_20260621_162904/`.
  Settings: `BODY_NED`, `FIRST_DISTANCE_M=3.0`, `SECOND_DISTANCE_M=3.0`,
  `LINEAR_SPEED_MPS=0.12`, `TURN_DIRECTION_SIGN=-1.0`,
  `TURN_LATERAL_SPEED_MPS=0.10`, `TURN_FORWARD_SPEED_MPS=0.0`,
  `TURN_ANGLE_DEG=90`, `YAW_TOLERANCE_DEG=12`, `TURN_MAX_SEC=12`.
  Pre-run state was clean: `connected=true`, `armed=false`, `manual_input=true`,
  `mode=MANUAL`, with a fresh local pose. PX4 entered OFFBOARD, armed by
  external command, and completed the sequence. Leg1 progressed to about
  `2.64m` at the last 1 Hz log sample and then reached the 3m-distance stop
  threshold. The turn stage no longer exited by timeout: corrected yaw progress
  reached `78.7deg`, satisfying the `90deg - 12deg` threshold, then it entered
  leg2. Leg2 had a mid-run local-position/progress plateau around `1.6m` for
  several seconds, then continued and reached about `2.81m` before final stop.
  Script requested DISARM successfully and cleanup requested MANUAL. Post-run
  safety check confirmed `/mavros/state`: `connected=true`, `armed=false`,
  `manual_input=true`, `mode=MANUAL`, `system_status=3`. User later confirmed
  the plateau was physical wheel slip caused by a small stone in front of the
  rover, not necessarily a controller/local-position fault. User also observed
  the previous `+1` right-turn run felt closer to an in-place turn, while this
  corrected `-1` left-turn run was clearly turning while moving forward.
- 2026-06-21 17:45 CST: User requested another right-turn ground test:
  forward 3m, right turn 90deg, forward 3m. MAVROS was not running, so started
  `scripts/run_mavros_px4_usb_to_qgc_logged.sh` again; log directory:
  `results/mavros/20260621_174537/`. Do not count this as a completed right-turn
  run. PX4/MAVROS came up but repeatedly reported
  `Preflight Fail: Strong magnetic interference`. Initial state after reconnect
  showed `mode=OFFBOARD`, `armed=false`, `manual_input=false`; sent disarm and
  MANUAL requests successfully. Final safety state before pausing:
  `connected=true`, `armed=false`, `manual_input=true`, `mode=MANUAL`,
  `system_status=3`. Because this 90-degree test depends on yaw, do not run the
  right-turn test until the user moves the rover away from magnetic interference
  and gives fresh safety confirmation. Right-turn command should use
  `TURN_DIRECTION_SIGN=+1.0`.
- 2026-06-21 17:5x CST: User said to start the right-turn test, but live
  `/mavros/statustext/recv` still reported
  `Preflight Fail: Strong magnetic interference`, and the MAVROS log continued
  to show the same warning. Did not run the motion script because this test uses
  yaw to stop the 90-degree turn. Safety state after refusing to start:
  `/mavros/state` `connected=true`, `armed=false`, `manual_input=true`,
  `mode=MANUAL`, `system_status=3`. Wait for magnetic warning to clear before
  running the right-turn test.
- 2026-06-21 18:xx CST: User said the vehicle was ready to retest. Read-only
  checks were clean for state and local pose (`MANUAL`, `armed=false`,
  `manual_input=true`, fresh `/mavros/local_position/pose`), but live
  `/mavros/statustext/recv` again emitted
  `Preflight Fail: Strong magnetic interference`. Did not run the right-turn
  motion script. Final safety state remained `connected=true`, `armed=false`,
  `manual_input=true`, `mode=MANUAL`, `system_status=3`.
- 2026-06-21 18:26 CST: User moved the GPS/compass position and asked to check
  again. Read-only checks showed `/mavros/state` clean (`connected=true`,
  `armed=false`, `manual_input=true`, `mode=MANUAL`, `system_status=3`) and a
  fresh `/mavros/local_position/pose`. A 10-second statustext listen and a
  follow-up 20-second statustext listen produced no new
  `Strong magnetic interference` message. Three repeated state samples over the
  same period stayed `MANUAL`, `armed=false`, `manual_input=true`. Treat the GPS
  reposition as likely effective, but require fresh ground-test/RC/QGC/physical
  cutoff confirmation before running the pending right-turn test.
- 2026-06-21 18:28 CST: Ran the corrected body-frame right L-turn after user
  confirmation and after magnetic-interference messages stopped. Run directory:
  `results/differential_offboard_l_turn/body_l_turn_right_sign_plus_20260621_182803/`.
  Settings: `BODY_NED`, `FIRST_DISTANCE_M=3.0`, `SECOND_DISTANCE_M=3.0`,
  `LINEAR_SPEED_MPS=0.12`, `TURN_DIRECTION_SIGN=+1.0`,
  `TURN_LATERAL_SPEED_MPS=0.10`, `TURN_FORWARD_SPEED_MPS=0.0`,
  `TURN_ANGLE_DEG=90`, `YAW_TOLERANCE_DEG=12`, `TURN_MAX_SEC=12`.
  Pre-run state was clean: `connected=true`, `armed=false`, `manual_input=true`,
  `mode=MANUAL`, with a fresh local pose. PX4 entered OFFBOARD, armed by
  external command, and completed the sequence. Leg1 reached the 3m-distance
  stop threshold after the last 1Hz log sample of `2.53m`. The right-turn stage
  exited by angle threshold, not timeout: yaw progress reached `76.6deg`, which
  satisfies `90deg - 12deg`, then leg2 started. Leg2 reached the distance stop
  threshold after the last 1Hz log sample of `2.64m`. Script requested DISARM
  successfully and cleanup requested MANUAL. Post-run safety check confirmed
  `/mavros/state`: `connected=true`, `armed=false`, `manual_input=true`,
  `mode=MANUAL`, `system_status=3`.
