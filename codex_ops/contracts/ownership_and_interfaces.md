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
  required semantics: seq, timestamp, position/local pose, yaw, speed,
  mode, arm state, health, GPS state, RC-stop state

CorridorPlan:
  Carrier -> Mini
  high-level tangent corridor plan

PlanCommand:
  Carrier -> Mini
  bounded primitive / phase command
  required envelope: seq, timestamp, valid_until

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

## Mini Command Acceptance And Safety

- Mini rejects duplicate, out-of-order, malformed, stale, or expired commands.
- `Abort` has higher priority than every phase or motion command.
- Loss of PairB or command freshness causes a local freeze/stop/abort; the
  vehicle must not continue on the last nonzero command indefinitely.
- The LR24 packet encoding, version, CRC, timestamp epoch, update rate, UART,
  and baud remain pending shared decisions. These semantics do not imply a
  specific wire format.

## Mini Phase Gate

- Mini first completes one stable orbit and must not cut out early.
- Mini orbit speed is greater than Carrier speed for the current mock.
- Mini exits the orbit only after a fresh Carrier cut-out phase command.
- The orbit exit follows the local tangent and then enters the shared terminal
  tangent corridor.

## Do Not Cross These Boundaries Accidentally

- Do not send ROS 2 DDS over LR24.
- Do not stream images/video over LR24.
- Do not make QGC/ground station the primary terminal docking controller.
- Do not change rover motion scripts from docking side without rover review.
- Do not change CorridorPlan semantics from rover side without docking review.
