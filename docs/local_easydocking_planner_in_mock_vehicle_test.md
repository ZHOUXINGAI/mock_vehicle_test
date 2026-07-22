# Local Easydocking Planner Copy In mock_vehicle_test

This repo is now the execution workspace for the outdoor rover experiment.

`/home/jetson/easydocking` remains the upstream planning repository, but field
tests should start from:

```bash
cd /home/jetson/mock_vehicle_test
```

## Local Copies

Copied from `easydocking` for local execution/reference:

```text
scripts/run_ground_2d_corridor_sim.py
docs/easydocking_ground_vehicle_2d_corridor_plan_handoff.md
docs/easydocking_msgs/CorridorPlan.msg
```

Local wrappers and rover transport code:

```text
scripts/run_ground_2d_corridor_preview.sh
src/lr24_compact_protocol.py
scripts/lr24_link_benchmark.py
scripts/lr24_pairb_dry_run.py
scripts/run_lr24_pairb_dry_run.sh
```

## What The Planner Does

The imported 2D planner mirrors the current `easydocking` CorridorPlan idea:

1. Mini moves on a circle.
2. Carrier starts outside the Mini circle.
3. The planner computes tangent candidates from Carrier to Mini orbit.
4. It chooses the tangent aligned with Mini orbit direction.
5. Carrier follows a smooth arc to the tangent point.
6. Mini completes the required orbit phase, then exits along the tangent.
7. Both vehicles enter the same corridor direction.
8. Carrier must stay ahead; Mini must not overtake Carrier.

This is high-level route geometry, not motor control.

The expected plot shape is part of the contract:

- Mini should look like circle first, then tangent-line exit.
- Carrier should look like smooth arc into tangent point `T`, then straight
  along the same tangent corridor.
- Carrier terminal motion should not show a hook, S-turn, or visible lateral
  chase after `T`.
- `front_violations == 0` is necessary but not sufficient; also check
  `terminal_corridor_shape_ok=True`.

The default preview uses:

```text
--terminal-lateral-mode straight-corridor
```

The old lateral-PD behavior is only available as an explicit comparison mode:

```bash
./scripts/run_ground_2d_corridor_preview.sh --terminal-lateral-mode chase
```

## Offline Preview

Run from this repo:

```bash
./scripts/run_ground_2d_corridor_preview.sh
```

Default scaled field parameters:

```text
Mini orbit center: (0, 0)
Mini orbit radius: 4.5m
Mini speed: 0.9m/s
Mini required laps: 1
Mini planning/start phase: 315 deg
Tangent trigger phase: about 249.8 deg
Mini arrival delay after plan: about 25.724s
Carrier start: (-7, -6)
Carrier max speed: 0.70m/s
Carrier max accel: 0.30m/s^2
Pass distance: 0.50m
```

Output:

```text
results/ground_2d_corridor_preview/latest/
```

Expected plots:

```text
trajectory_xy.png
trajectory_xy_full.png
speed_profile.png
distance_convergence.png
front_gap.png
lateral_gap.png
carrier_corridor_lateral_error.png
phase_timeline.png
speed_distance_front.png
```

No hardware is touched by this preview.

## LR24 Transport Boundary

The rover-side LR24 protocol now has two planning levels:

```text
CORRIDOR_PLAN:
  high-level geometry copied from easydocking CorridorPlan
  tangent point, tangent direction, trigger phase, arrival delay, speeds
  current compact frame size: 49 bytes

PLAN_COMMAND:
  short bounded primitive or phase command
  phase, v, omega, duration/distance, valid-until
  current compact frame size: 37 bytes
```

Morning no-motion mixed-traffic test uses Pair B (`ADDR=1102`), the
Carrier-Mini link.

Mini side:

```bash
CONFIRM_NO_MOTION=true ./scripts/run_lr24_pairb_dry_run.sh mini \
  --port /dev/serial/by-id/<B-MINI-LR24> \
  --duration-sec 120 \
  --state-rate-hz 10 \
  --simulate-orbit \
  --radius-m 4.5 \
  --speed-mps 0.9
```

Carrier side:

```bash
CONFIRM_NO_MOTION=true ./scripts/run_lr24_pairb_dry_run.sh carrier \
  --port /dev/serial/by-id/<B-CARRIER-LR24> \
  --duration-sec 120 \
  --command-rate-hz 2 \
  --phase hold \
  --stale-ms 300 \
  --send-corridor-plan \
  --corridor-plan-rate-hz 0.2
```

This validates:

- MiniState uplink;
- CorridorPlan downlink;
- HOLD PlanCommand downlink;
- stale-state fallback to HOLD.

It still does not execute motor commands.

## What Is Still Not Connected

The real bridge is still a later step:

```text
Carrier onboard Nano/Orin
  real Carrier state + Mini state
  -> CorridorPlan / route phase
  -> local Carrier primitive tracking
  -> LR24 compact CorridorPlan/PlanCommand to Mini

Mini onboard side
  receives CorridorPlan/PlanCommand
  -> local primitive tracker
  -> PX4/Arduino/motor execution
```

Do not skip directly from offline preview to full two-rover motion.
