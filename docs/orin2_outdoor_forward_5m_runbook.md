# Orin2 outdoor forward 5 m Offboard runbook

This is the first wheels-down outdoor gate for the current Orin Nano Mini.
It runs one forward leg only. It does not turn, reverse, use fake vision, use
fake GPS, start Pair B, or run the docking planner.

## Indoor-to-outdoor switch

Required persistent baseline:

```text
EKF2_EV_CTRL=0
EKF2_GPS_CTRL=7
SYS_AUTOSTART=50000
CA_AIRFRAME=6
MAV_SYS_ID=2
CA_R_REV=3
PWM_MAIN_FUNC1=101
PWM_MAIN_FUNC2=102
PWM_MAIN_FUNC6=0
PWM_MAIN_FUNC7=0
PWM_MAIN_DIS1/2=1500
PWM_MAIN_FAIL1/2=1500
COM_RC_IN_MODE=3
COM_OF_LOSS_T=1.0
COM_OBL_RC_ACT=0
NAV_RCL_ACT=2
```

The launcher reads and asserts these values but never writes them. It refuses
to start if fake EV/fake GPS/MAVROS/another mission is already running. The
Pixhawk must resolve through its fixed PX4 by-id path and must not resolve to
the Arduino by-id target. The numeric ttyACM index may change after a reboot;
direct ttyACM fallback is forbidden.

## Mission

```text
real GNSS/local-pose preflight
2 s zero setpoint prestream in MANUAL/disarmed
operator Arms once using RC while still in MANUAL
program requests and verifies OFFBOARD
0.2 s zero handoff
BODY_NED vx=+0.12 m/s until along-track progress reaches 4.85 m
1 s zero burst
Disarm
MANUAL
independent final verification
```

Default abort limits:

- local pose age: 1.0 s
- MAVROS state age: 2.0 s
- GPS age: 2.0 s; `NavSatFix.status` must not be `NO_FIX`
- GPSRAW age: 2.0 s; `fix_type >= 3` and at least 6 satellites visible
- cross-track error: 0.75 m
- heading change from initial heading: 35 degrees
- motion duration: 75 s
- progress stall: less than 0.08 m in 8 s
- any disarm, mode exit, connection loss, GPS loss, stale/non-finite local pose

Every abort publishes zero and runs bounded Disarm/MANUAL recovery. The
launcher keeps MAVROS alive for a separate final-state verification before it
stops its own processes.

## Preparation-only command

```bash
./scripts/run_orin2_outdoor_forward_5m.sh
```

This is the only command authorized during configuration preparation. It is a
dry run and cannot import ROS or publish a setpoint.

## Field GO gate

Do not construct the live command until all items are true for the current run:

- open straight corridor at least 8 m long and 3 m wide;
- wheels installed, loose HDMI/USB/power cables removed or secured;
- RC link and Kill/Disarm physically tested immediately before the run;
- a person remains beside the vehicle with physical cutoff access;
- QGC shows the expected Mini system and no red arming/EKF/GPS warning;
- real 3D GPS fix and stable local position, with no fake publisher;
- vehicle initially MANUAL and disarmed;
- current differential mapping and forward wheel direction are reconfirmed;
- the user gives a fresh explicit start command for this exact run.

`准备好` means preparation only. It is not a motion authorization. The live
launcher additionally requires twelve `CONFIRM_*` environment variables and
the exact one-time phrase printed by its dry run.
