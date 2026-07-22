# Mock Docking Hardware Execution Design - 2026-07-22

This is the rover hardware execution design for using two ground vehicles to
mock the two-aircraft docking scheme.

## Roles

```text
Orin1 / Carrier rover
  Represents the carrier aircraft.
  Runs the high-level docking planner / CorridorPlan leader.
  Tracks its own arc-to-corridor and terminal-corridor path locally.
  Sends compact plans and phase commands to Mini over PairB.

Orin2 / Mini rover
  Represents the faster fixed-wing child aircraft.
  First performs stable orbit around the planned circle.
  Sends timestamped state to Carrier over PairB.
  Receives CorridorPlan and phase commands, then exits orbit at the planned
  tangent trigger phase and tracks the terminal tangent line.

Ground station
  Runs QGC and monitoring/logging.
  Does not run the primary docking planner.
```

Current LR24 topology:

```text
PairB / ADDR=1102: Carrier Orin1 <-> Mini Orin2/Pixhawk side
  Compact MiniState uplink, CorridorPlan/PlanCommand/abort downlink.

PairC / ADDR=1103: Ground station <-> Carrier Pixhawk
  QGC sees Carrier, MAV_SYS_ID=1.

PairA / ADDR=1101: Ground station <-> Mini Pixhawk
  QGC sees Mini, MAV_SYS_ID=2.
```

## Planner Contract From easydocking

The aerial controller computes a tangent-intercept CorridorPlan:

1. Mini is already orbiting.
2. Carrier is outside the Mini orbit circle.
3. Carrier computes the tangent point `T` on the Mini orbit that aligns with
   Mini's orbit direction.
4. Carrier computes Mini arrival delay from Mini's current orbit phase to `T`.
5. Carrier follows a smooth arc to `T`, timed to arrive ahead of Mini.
6. Mini stays in orbit until the trigger phase, then exits along the tangent.
7. Both vehicles continue along the same straight terminal corridor.
8. Carrier must remain ahead of Mini in the tangent frame.

The rover preview uses the scaled route:

```text
Mini orbit center O:      (0.0, 0.0)
Mini orbit radius:        4.5 m
Mini speed target:        0.9 m/s
Mini yaw rate magnitude:  0.2 rad/s
Mini required stable lap: 1
Carrier start:            (-7.0, -6.0)
Carrier max speed:        0.7 m/s
Carrier max accel:        0.30 m/s^2
Tangent point T:          (-1.553, -4.224)
Tangent direction:        (0.939, -0.345)
Trigger phase:            about 249.8 deg
Terminal path target:     about 8 m
```

The expected plot shape is not negotiable: Mini is circle then tangent line;
Carrier is smooth arc into `T`, then the same straight tangent corridor. A
visible hook, S-turn, or lateral chase after `T` is a planner failure.

## Low-Level Interface

Both rovers should execute bounded body-frame primitives:

```text
v_mps      forward speed
omega_radps yaw-rate command
duration_ms or distance_m
valid_until_ms
phase      HOLD / ORBIT / ARC_TO_CORRIDOR / TERMINAL / STOP / ABORT
```

Do not command global `vx/vy` directly into PX4 rover Offboard for the field
route. We already saw the rover can pre-correct yaw before driving when a
global velocity vector is used. Use local path following to convert route
geometry into body-frame `v, omega`.

Each vehicle needs its own calibrated yaw sign:

```text
omega_ccw_vehicle = TURN_SIGN_CCW * abs(v / radius)
```

On the Carrier rover's earlier BODY_NED setup, positive turn sign produced a
physical right turn. Do not assume the same sign on Mini; calibrate it before
the docking run.

## Software Blocks To Run

Carrier / Orin1:

```text
carrier_state_source
  Reads Carrier PX4 position/yaw/velocity from local MAVROS or MAVLink.

pairb_bridge_carrier
  Receives MiniState at 5-10 Hz over PairB.
  Sends CorridorPlan at event rate or low rate.
  Sends PlanCommand at 1-5 Hz.
  Sends ABORT immediately on stale state, operator abort, RC/QGC stop, or
  planner violation.

mock_docking_leader
  Waits for Mini stable-orbit evidence.
  Computes CorridorPlan using the same geometry as easydocking.
  Starts Carrier arc tracking.
  Logs plan, state, command, distance, front_gap, lateral_gap, and phase.

carrier_primitive_executor
  Converts planned arc/terminal path into body-frame v/omega.
  Commands Carrier PX4 Offboard only after explicit field approval.
```

Mini / Orin2:

```text
mini_state_tx
  Publishes timestamped MiniState over PairB at 5-10 Hz.

mini_plan_rx
  Receives CorridorPlan and PlanCommand from Carrier.
  Rejects stale plan/commands.

mini_primitive_executor
  ORBIT: follow R=4.5 m circle at v=0.9 m/s after low-speed validation.
  TERMINAL: exit at trigger phase and drive along tangent direction.
  STOP/ABORT: stop local command stream and disarm/hold as configured.
```

Ground station:

```text
QGC serial link PairC -> Carrier sysid 1
QGC serial link PairA -> Mini sysid 2
Optional SSH/log/video over WiFi/4G
```

## Field Execution Ladder

1. Static setup:
   - Mark Mini orbit center `O=(0,0)`.
   - Mark the 4.5 m circle.
   - Mark Carrier start `(-7,-6)`.
   - Mark tangent point `T=(-1.553,-4.224)`.
   - Mark a tangent line through `T` along `(0.939,-0.345)`.

2. No-motion comms:
   - PairB ping/echo.
   - MiniState uplink.
   - CorridorPlan downlink.
   - HOLD PlanCommand downlink.
   - Confirm stale MiniState causes Carrier HOLD/ABORT behavior.

3. Wheels-lifted:
   - Mini ORBIT primitive at low speed, then target `v=0.9`, `|omega|=0.2`.
   - Carrier arc/terminal primitives at low speed, then target max `0.7`.
   - Confirm yaw sign, RC stop, QGC stop, Arduino timeout brake.

4. Single-rover ground:
   - Mini alone: one circle at low speed, then circle plus tangent exit.
   - Carrier alone: arc-to-`T`, then straight tangent corridor.

5. Two-rover mock docking, low speed:
   - Mini starts ORBIT and sends real MiniState.
   - Carrier waits until Mini completes one stable lap.
   - Carrier publishes CorridorPlan.
   - Carrier starts arc tracking.
   - Mini exits at trigger phase.
   - Both enter terminal tangent corridor.
   - Abort if link stale, front-gap rule fails, lateral error grows, or operator
     calls stop.

6. Two-rover target-speed demo:
   - Only after the low-speed run shape matches the preview.

## Pass / Fail

Pass:

- Mini completes at least one stable orbit before tangent exit.
- Mini remains visually faster than Carrier.
- Carrier follows a smooth arc into `T`.
- Terminal segment is a shared straight tangent corridor.
- Carrier remains ahead in the tangent frame until first pass.
- First mock pass reaches less than `0.5 m` separation.
- Ground QGC sees both rovers.
- RC/QGC/manual stop works on both rovers.

Fail:

- Any unexpected motor motion before explicit command.
- Any Pixhawk reboot or LR24 instability during no-motion checks.
- Mini exits orbit early.
- Carrier makes a visible hook/S-turn after `T`.
- Carrier falls behind Mini in the terminal corridor.
- Either rover spins in place unexpectedly or reverses unexpectedly.
- Any stop path fails.

## Immediate Implementation Gap

The current repo already has the compact LR24 frames and no-motion dry-run.
The missing execution code is the live bridge from:

```text
PX4/MAVROS state -> MiniState
CorridorPlan/PlanCommand -> local primitive executor -> PX4 Offboard v/omega
```

Build that bridge in `mock_vehicle_test` first, with a hard no-motion mode by
default. Only enable Offboard output behind explicit environment confirmations.
