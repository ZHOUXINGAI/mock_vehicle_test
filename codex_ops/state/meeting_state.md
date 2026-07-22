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
- Orin2 / Mini Codex owns the Mini rover execution layer and the
  `MiniState`/`PlanCommand` LR24 endpoint. It does not own or run the primary
  `easydocking` planner.
- Orin1 / Carrier is the docking leader: it receives Mini state, runs the
  planner, and emits phase, primitive, corridor, or abort commands.

## Active Repositories

```text
codex_ops:
  Orin1: /home/jetson/mock_vehicle_test/codex_ops
  Orin2: /home/seeed/mock_vehicle_test/codex_ops
  tracked inside: mock_vehicle_test on each host
  parent remote: git@github.com:ZHOUXINGAI/mock_vehicle_test.git

rover:
  Orin1: /home/jetson/mock_vehicle_test
  Orin2: /home/seeed/mock_vehicle_test
  remote: git@github.com:ZHOUXINGAI/mock_vehicle_test.git

docking:
  Orin1: /home/jetson/easydocking
  remote: git@github.com:ZHOUXINGAI/easydocking.git
```

## Agent Roles

```text
rover Codex:
  real rover, PX4 rover, MAVROS, QGC, RC safety, Arduino/D24A, LR24 field tests

Orin2 / Mini rover Codex (current /home/seeed host):
  Mini execution, timestamped MiniState sender, validated PlanCommand receiver,
  body-frame primitive adapter, local timeout/stop/abort; no primary planner

docking Codex:
  Orin1 / Carrier leader, easydocking planner, CorridorPlan, simulation,
  metrics, reports, algorithm contracts
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

Orin2 audit status on 2026-07-22:

- Mini Pixhawk `MAV_SYS_ID=2` was confirmed by a direct read-only MAVLink
  parameter query.
- PairA QGC transport has not yet been verified from the ground station.
- PairB physical UART/radio mapping and Carrier-to-Mini packet exchange have
  not yet been verified.
- The repository does not yet contain an executable MiniState sender or a
  PlanCommand receiver with sequence, expiry, timeout, and Abort enforcement.

Update from Orin1 at 2026-07-22 20:50 CST:

- Boss reports Pair A, Pair B, and Pair C are all connected and their binding
  LEDs remain steadily lit. This confirms RF binding only; packet loss and
  latency are still awaiting the Pair B no-motion benchmark.
- Pair B physical contract is finalized as `ADDR=1102`, full duplex, LR24 low
  rate 2.4 KB/s, 500 mW, `57600 8N1`, Orin1 FHSS-ground and Orin2
  FHSS-vehicle.
- Orin1 Pair B is the CP2102 by-id device. The CH340 by-id device is the
  Carrier Arduino and must not be opened by the Pair B program.
- Wire contract and safety gate are now implemented in the shared repository.
  Canonical document: `docs/lr24_pairb_wire_contract_v1.md`.
- The shared frame is field ENU: `+x East`, `+y North`, yaw CCW from East.
  Carrier sends `FIELD_ORIGIN`; plans and Mini state carry a matching
  `origin_id`.
- Sender timestamps are uint32 `CLOCK_BOOTTIME` milliseconds. Receivers derive
  relative TTL from `(valid_until - timestamp)` and run expiry/watchdog from
  their own monotonic clock; cross-computer clock equality is not assumed.
- Implemented safety behavior includes CRC/length/version checks, wrapping
  sequence checks, duplicate/old rejection, local speed/yaw/acceleration
  limits, zero-only HOLD/STOP, command expiry, 750 ms watchdog, and local-only
  Abort-latch clearing.
- Virtual serial integration passed: Mini transmitted 40 states, Carrier saw
  no sequence gap after startup, Mini accepted 20 HOLD commands, 8
  CorridorPlans, and 8 FieldOrigins with zero gate rejections.
- All current Pair B tests remain no-motion and are not connected to MAVROS,
  PX4 command output, Arduino, or motors.

## Mock Docking Execution Contract

- Mini first completes one stable orbit.
- Carrier computes CorridorPlan after Mini orbit is stable.
- Mini exits only at the planned tangent trigger phase.
- Carrier follows a smooth arc into tangent point `T`.
- Both vehicles then use the same straight terminal tangent corridor.
- Carrier must remain ahead in the tangent frame before first pass.
- Rover low-level execution uses body-frame bounded `(v_mps, omega_radps)`
  primitives, not raw global `vx/vy` Offboard commands.

The detailed Mini hardware-execution design document is not present in the
current shared clone. Do not treat its former path as an implemented contract.

## Communication Rule

If a change needs the peer Codex to know or act, it must produce:

1. an event in `events/YYYY/YYYY-MM-DD.jsonl`;
2. if action is needed, an inbox note under `inbox/<peer>/`;
3. a commit that includes `codex_ops/`.

The final user reply should include a short "give this to the other Codex"
message only as a convenience. The durable source is `codex_ops/`.
