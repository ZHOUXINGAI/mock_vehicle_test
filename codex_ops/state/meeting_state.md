# Shared Meeting State

Last updated: 2026-07-22 CST

This is the operational shared whiteboard for the rover/docking Codex pair.
Project-specific docs in each repo still contain details, but this file is the
first place both agents read for current decisions.

## Current Boss Decisions

- Carrier/Orin1 is the onboard docking leader.
- Mini/Orin2 is the faster fixed-wing mock.
- Current `/home/seeed` coordination host is Orin2 / Mini, the fixed-wing
  child-aircraft simulation side. Its peer is Orin1 / Carrier, the quadrotor
  mother-aircraft simulation side.
- The immediate purpose of the two-Codex coordination channel is to keep both
  sides synchronized while bringing the end-to-end mockdocking workflow up.
- Ground station monitors both vehicles; it is not the primary docking planner.
- Codex agents coordinate through `mock_vehicle_test/codex_ops` plus each code
  repo's handoff/state docs.
- The two Codex homes remain separate.

## Active Repositories

```text
codex_ops:
  /home/jetson/mock_vehicle_test/codex_ops
  tracked inside: /home/jetson/mock_vehicle_test
  parent remote: git@github.com:ZHOUXINGAI/mock_vehicle_test.git

rover:
  /home/jetson/mock_vehicle_test
  remote: git@github.com:ZHOUXINGAI/mock_vehicle_test.git

docking:
  /home/jetson/easydocking
  remote: git@github.com:ZHOUXINGAI/easydocking.git
```

## Agent Roles

```text
rover Codex:
  real rover, PX4 rover, MAVROS, QGC, RC safety, Arduino/D24A, LR24 field tests

docking Codex:
  easydocking planner, CorridorPlan, simulation, metrics, reports, algorithm contracts
```

## Current LR24 Topology

```text
PairB / ADDR=1102:
  Carrier <-> Mini
  compact MiniState / CorridorPlan / PlanCommand / Abort

PairC / ADDR=1103:
  Ground station <-> Carrier Pixhawk
  QGC monitors Carrier, MAV_SYS_ID=1

PairA / ADDR=1101:
  Ground station <-> Mini Pixhawk
  QGC monitors Mini, MAV_SYS_ID=2
```

## Mock Docking Execution Contract

- Mini first completes one stable orbit.
- Carrier computes CorridorPlan after Mini orbit is stable.
- Mini exits only at the planned tangent trigger phase.
- Carrier follows a smooth arc into tangent point `T`.
- Both vehicles then use the same straight terminal tangent corridor.
- Carrier must remain ahead in the tangent frame before first pass.
- Rover low-level execution uses body-frame bounded `(v_mps, omega_radps)`
  primitives, not raw global `vx/vy` Offboard commands.

Detailed rover design:

```text
/home/jetson/mock_vehicle_test/docs/mock_docking_hardware_execution_design_2026_07_22.md
```

## Communication Rule

If a change needs the peer Codex to know or act, it must produce:

1. an event in `events/YYYY/YYYY-MM-DD.jsonl`;
2. if action is needed, an inbox note under `inbox/<peer>/`;
3. a commit that includes `codex_ops/`.

The final user reply should include a short "give this to the other Codex"
message only as a convenience. The durable source is `codex_ops/`.
