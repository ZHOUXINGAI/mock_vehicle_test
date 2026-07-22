# Ownership And Interfaces

## Repo Ownership

```text
mock_vehicle_test:
  owner: rover Codex
  role: hardware execution, real rover tests, MAVROS/PX4/Arduino/LR24 field work

easydocking:
  owner: docking Codex
  role: docking planner, CorridorPlan, simulation, reports

codex_ops:
  owner: both
  role: coordination state, inboxes, events, interface contracts
```

## Stable Interface Boundary

The initial rover/docking interface is:

```text
MiniState:
  Mini -> Carrier
  low-rate timestamped state

CorridorPlan:
  Carrier -> Mini
  high-level tangent corridor plan

PlanCommand:
  Carrier -> Mini
  bounded primitive / phase command

Abort:
  either direction, highest priority
```

Rover execution consumes bounded primitives:

```text
phase
v_mps
omega_radps
duration_ms
distance_m
valid_until_ms
```

Planner code may produce global geometry, but rover low-level execution must
convert it locally to body-frame `v, omega`.

## Do Not Cross These Boundaries Accidentally

- Do not send ROS 2 DDS over LR24.
- Do not stream images/video over LR24.
- Do not make QGC/ground station the primary terminal docking controller.
- Do not change rover motion scripts from docking side without rover review.
- Do not change CorridorPlan semantics from rover side without docking review.
