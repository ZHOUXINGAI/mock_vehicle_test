# Ground result: Orin1 two-rover EasyDocking read-only audit

Time: 2026-07-27 23:45 CST

Task lineage:

- Ground task: `f0514326-3a0b-47c8-aa1e-0029f0d7e3a8`
- Agent: `orin1-carrier`
- Codex session: `019e3b9d-8417-76d2-bf88-4ec59aba48c4`
- Result: completed

## Current stage

The current physical platform is two ground rovers:

- Orin1 controls the Carrier rover.
- Orin2 controls the Mini rover.
- Each rover has passed an independent PX4/MAVROS Offboard test.
- The pair has not yet completed a coordinated two-rover motion run.
- Aircraft deployment remains a later goal and is not the current execution
  stage.

This was a read-only audit. No QGC, MAVROS, PX4, Offboard, Arduino, serial,
MAVLink, LR24, actuator, motor, RC, docking runtime, or other vehicle/hardware
process was started.

## Reviewed baselines

- `/home/jetson/easydocking`: HEAD `b2e8e01`
- `/home/jetson/mock_vehicle_test`: HEAD `17aeaf4`

Both Orin1 working trees contain pre-existing uncommitted changes. The audit
did not modify or clean either tree, so its conclusions refer to the current
working trees and not only the committed baselines.

## Orin1 conclusion

The repositories already contain:

- the 2D tangent and Carrier-arc geometry;
- the finite-field Carrier-leader route;
- LR24 Pair B compact messages and MAVLink TUNNEL adaptation;
- the Mini command safety gate;
- individual rover Offboard test experience.

They do not yet contain one complete runtime connecting:

`GroundDockingLeader -> Pair B -> two path followers -> two local safety executors`

Therefore an end-to-end two-rover motion run is not ready to start.

## Existing ground geometry

- Mini orbit center: `(0, 0)` in shared field ENU.
- Mini orbit: radius `4.5 m`, CCW, target speed `0.9 m/s`.
- Mini must complete at least one stable orbit.
- Tangent trigger phase: approximately `249.817 deg`.
- Tangent point: `T=(-1.553, -4.224)`.
- Terminal tangent unit vector: `(0.939, -0.345)`.
- Carrier start: `(-7, -6)`.
- Carrier limits: `0.7 m/s`, `0.30 m/s^2`.
- Carrier follows a smooth arc into `T`, then both rovers use the same straight
  terminal corridor.
- Carrier must remain ahead of Mini in the tangent frame.

The recommended ground state machine is:

`HOLD -> WAIT_MINI_STABLE_ORBIT -> PLAN_VALIDATED ->`
`CARRIER_ARC/MINI_ORBIT -> MINI_TANGENT_EXIT -> SHARED_TERMINAL ->`
`COMPLETE_HOLD`

Any safety failure enters locally latched `ABORT_LATCHED`.

## One coherent Carrier plan

Orin1 must be the only leader planner. It consumes local Carrier state and
Pair B MiniState, converts both to the same field ENU frame, and creates one
immutable plan with one `plan_id`, one `origin_id`, one tangent point and
direction, one phase trigger, and one set of speed, acceleration, and expiry
limits.

- Local branch: the same plan goes to the Carrier follower and then a local
  safety executor before MAVROS.
- Remote branch: the same plan is encoded as `CorridorPlanCompact` plus
  `PlanCommand` and sent over Pair B from `1.242` to `2.242`.

Pair B carries only low-rate state, plan, phase commands, and Abort. Pair A/C
remain the QGC links. NATS remains Codex coordination only and is not part of
the vehicle runtime.

## Pair B safety contract

- Coordinate frame: shared field ENU.
- MiniState: about `10 Hz`, stale after `300 ms`.
- PlanCommand: about `5 Hz`, TTL `500 ms`.
- Mini local watchdog: `750 ms`.
- Abort: highest priority and locally latched.
- Sender `CLOCK_BOOTTIME` timestamps are for sender-side ordering. Receivers
  must not compare absolute monotonic time between the two computers.

## Offboard adaptation requirement

The current L-turn and Smoke scripts cannot be used unchanged behind the
planner because they can own mode changes, arm, and disarm.

First extract a local primitive executor that:

- has no authority to arm or change mode;
- accepts only expiring zero-speed or bounded BODY_NED primitives;
- independently stops on stale state, stale pose, wrong mode, expired command,
  watchdog timeout, or local Abort;
- preserves the already tested stop-pulse and freshness logic.

Carrier steering is known to use `linear.y` with sign `-1`. Mini steering and
wheel direction must be reverified with wheels lifted before motion.

## Recommended implementation and validation order

1. Pure software geometry and log replay.
2. Linked-PTY/fake transport using targeted `1.242 <-> 2.242`.
3. Real Pair B with no-motion HOLD only.
4. Local executor with output disabled.
5. Wheels lifted: zero command, watchdog, and Abort tests.
6. Each rover independently follows its low-speed route.
7. Outdoor low-speed coordinated two-rover run.

Failure at any stage returns to HOLD and blocks advancement.

## Smallest first software slice

Add:

- `src/ground_corridor_geometry.py`
- `src/ground_docking_leader.py`
- `scripts/run_ground_docking_replay.py`
- `tests/test_ground_corridor_geometry.py`
- `tests/test_ground_docking_leader.py`
- `tests/test_ground_pairb_plan_adapter.py`

This slice must implement geometry, stable-orbit qualification, one immutable
plan, the ground phase machine, Pair B adaptation, and failure injection. It
must not modify or invoke the hardware scripts.

## Physical blockers before any two-rover motion

Reverify:

- actual firmware on both rovers and Arduino timeout braking;
- wheel direction and steering sign on both rovers;
- low-speed calibration;
- shared ENU origin, GPS, and yaw consistency;
- Pair B targeted full-duplex behavior;
- simultaneous Pair A/C stability;
- RC stop, QGC stop, physical power cutoff, field clearance, and personnel
  exclusion.

