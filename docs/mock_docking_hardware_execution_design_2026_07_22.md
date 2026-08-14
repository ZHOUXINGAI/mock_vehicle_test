# Mock Docking Hardware Execution Design - 2026-07-22

> Superseded role map, 2026-08-07: Orin2/system2 is now Carrier leader and
> Orin1/system1 is Mini. Pair B wiring and system IDs did not change. See
> `docs/orin2_carrier_two_rover_plan_2026_08_07.md`. The original role labels
> below are retained only as historical design context.

This is the rover hardware execution design for using two ground vehicles to
mock the two-aircraft docking scheme.

## Roles

```text
Orin1 / Carrier rover
  Represents the carrier aircraft.
  Runs the high-level docking planner / CorridorPlan leader.
  Tracks its own approach-to-corridor and terminal-corridor path locally.
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
5. Carrier follows the planned approach to `T`, timed to arrive ahead of Mini.
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
Tangent point T:          (0.888, -4.411)
Tangent direction:        (0.980, 0.197)
Trigger phase:            about 281.4 deg
Terminal path target:     about 8 m
```

The expected plot shape is not negotiable: Mini is circle then tangent line;
Carrier reaches `T`, then uses the same straight tangent corridor. With the
current planner contract and no measured Carrier start heading, `C->T` is the
exact straight tangent and is represented honestly as a zero-curvature
`straight_tangent` approach. A future finite-radius arc requires an explicit
start-heading and Dubins/biarc/clothoid contract. A visible hook, S-turn, or
lateral chase after `T` remains a planner failure.

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
  Starts Carrier approach tracking (`ARC` phase name is retained on the wire).
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
  STOP/ABORT: stop locally. The current no-motion slice does not disarm or
  issue any flight-controller command.
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
   - Mark tangent point `T=(0.888,-4.411)`.
   - Mark a tangent line through `T` along `(0.980,0.197)`.

2. No-motion comms:
   - PairB ping/echo.
   - MiniState uplink.
   - CorridorPlan downlink.
   - HOLD PlanCommand downlink.
   - Confirm stale MiniState causes Carrier HOLD/ABORT behavior.

3. Wheels-lifted:
   - Mini ORBIT primitive at low speed, then target `v=0.9`, `|omega|=0.2`.
   - Carrier approach/terminal primitives at low speed, then target max `0.7`.
   - Confirm yaw sign, RC stop, QGC stop, Arduino timeout brake.

4. Single-rover ground:
   - Mini alone: one circle at low speed, then circle plus tangent exit.
   - Carrier alone: arc-to-`T`, then straight tangent corridor.

5. Two-rover mock docking, low speed:
   - Mini starts ORBIT and sends real MiniState.
   - Carrier waits until Mini completes one stable lap.
   - Carrier publishes CorridorPlan.
   - Carrier starts approach tracking.
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
- Carrier follows the planned approach into `T` without a hook or S-turn.
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

## Current No-Motion Execution Boundary
The repo now has a strict structural adapter from EasyDocking plans and
commands to Pair B, plus guarded Mini and Carrier-local followers. The only
executor backend in this stage is `no_motion`:

```text
EasyDocking object -> strict adapter -> Pair B type
Mini Pair B frame -> MiniCommandGate -> MiniLiveFollower -> NoMotionExecutor
Carrier local HOLD -> CarrierLocalFollower -> NoMotionExecutor
```

`NoMotionExecutor` always returns exactly `v=0` and `omega=0`. HOLD, STOP,
ABORT, TTL expiry, and watchdog fallback remain zero. A motion phase or
nonzero request is reported as blocked rather than disguised as a successful
zero command. The dry-run reports executor decisions, zero outputs, blocked
motion, and nonzero outputs, and exits nonzero if any nonzero output is ever
observed.

The next separately authorized execution stage remains the hardware bridge:

```text
PX4/MAVROS state -> MiniState
CorridorPlan/PlanCommand -> local primitive executor -> PX4 Offboard v/omega
```

That backend is not present in this slice. Passing pure-software or Pair B
no-motion tests does not authorize wheels-lifted, Offboard, or motor output.
