# Codex Shared Meeting State - Rover Mirror

This is the rover-side mirror of the shared cross-agent whiteboard.

Primary shared whiteboard:

```text
/home/jetson/easydocking/docs/codex_shared_meeting_state.md
```

Before making architecture claims that affect both repositories, read the
primary shared whiteboard first. If this rover mirror conflicts with the
primary shared whiteboard, the primary shared whiteboard wins.

## Meeting Update: 2026-06-27 Carrier-Leader Correction

Boss/user correction:

- The central/leader docking computer should be deployed on the Carrier
  rover's onboard Nano/Orin Nano.
- The architecture should not be interpreted as "human-side ground laptop is
  always the main docking planner".
- Ground station/laptop remains important for QGC, logs, monitoring, setup,
  manual safety/abort, and debugging, but it is not the primary planner in the
  Carrier-leader implementation.

Current role split:

```text
Carrier rover
  Nano/Orin Nano runs easydocking planner / CorridorPlan leader logic
  Receives Mini compact timestamped state
  Sends compact plan/phase/speed/abort primitives to Mini
  Owns leader-side prediction and stale-packet handling

Mini rover
  Sends timestamped compact state to Carrier
  Locally tracks received plan/phase/speed primitives
  Owns local failsafe if packets become stale or invalid

Ground station
  QGC, monitoring, logs, parameter setup, manual abort, SSH/debug
  Optional relay or observation during early tests
  Not the main docking planner
```

## Link Rules

LR24:

- compact low-rate state/plan/abort only;
- no ROS2 DDS;
- no image/video;
- no dense trajectory spam;
- no raw high-rate debug logs;
- no terminal high-rate closed-loop control.

4G/WiFi:

- auxiliary monitoring/log/video/SSH/backup observability;
- not safety-critical terminal docking control.

Terminal docking:

- onboard autonomy;
- Carrier stereo/vision handles final relative pose closure;
- radio only updates state/plan/abort;
- stale radio data causes freeze, replan, or abort.

Latency/message constraints:

- every state packet includes timestamp and sequence number;
- planner predicts from packet time;
- out-of-order packets are rejected;
- stale packets trigger freeze/replan/abort;
- initial Mini state budget: `80-150 bytes @ 5-10Hz`;
- planner/Carrier commands: event-triggered or `1-5Hz`.

## Field Test Constraints

- Two-rover ground demo stays around `30m x 30m`.
- Mini rover is faster.
- Carrier rover is slower.
- Start with no-motion telemetry.
- Then wheels-lifted command path.
- Do not run full route motion without explicit user approval.

## Rover-Side Prep Artifacts

Prepared for the first outdoor morning:

```text
docs/local_easydocking_planner_in_mock_vehicle_test.md
scripts/run_ground_2d_corridor_sim.py
scripts/run_ground_2d_corridor_preview.sh
docs/outdoor_two_rover_carrier_leader_runbook_2026_06_27.md
config/ground_2d/carrier_leader_field_test_2026_06_27.json
src/lr24_compact_protocol.py
scripts/lr24_link_benchmark.py
scripts/run_lr24_no_motion_benchmark.sh
scripts/lr24_pairb_dry_run.py
scripts/run_lr24_pairb_dry_run.sh
```

The LR24 benchmark code is no-motion only. It supports:

- `echo`
- `ping`
- `state-tx`
- `state-rx`
- `command-tx`
- `command-rx`
- `sizes`

The bidirectional dry-run code is also no-motion only. It tests mixed traffic
on the same Carrier-Mini LR24 pair. Current field labels make that physical
link Pair B:

- Carrier receives MiniState frames and sends PlanCommand frames.
- Carrier can also send compact CorridorPlan frames.
- Mini sends simulated MiniState frames and receives PlanCommand/CorridorPlan
  frames.
- Carrier falls back to `HOLD` if Mini state has not arrived or is stale.
- It does not connect to MAVROS, PX4, ROS, Arduino, or motor execution.

Current compact frame sizes:

```text
MiniState:   32 bytes per frame
PlanCommand: 37 bytes per frame
CorridorPlan: 49 bytes per frame
FieldOrigin: 31 bytes per frame
Ping/Pong:   19 bytes per frame
```

These are within the Mini state `80-150 bytes @ 5-10Hz` target.

## Trajectory Contract Mirror

The rover-side offline preview must match the docking-side trajectory contract:

- Mini completes the required orbit first, then exits along the planned orbit
  tangent.
- Carrier follows a smooth CorridorPlan arc from `(-7, -6)` to tangent point
  `T`.
- After `T`, Carrier continues along the same straight tangent corridor as
  Mini.
- Carrier stays ahead of Mini in the tangent frame; `front_gap` must not go
  negative before the first pass.
- A visible Carrier hook, S-turn, or lateral chase after `T` is a preview
  failure even when `front_violations == 0`.

Rover-side implementation:

```text
scripts/run_ground_2d_corridor_sim.py
```

Default terminal mode is `straight-corridor`; the old lateral-PD chase behavior
is only available by explicitly passing:

```bash
./scripts/run_ground_2d_corridor_preview.sh --terminal-lateral-mode chase
```

Current verified local preview:

```text
results/ground_2d_corridor_preview/20260627_013226_ground_2d
terminal_corridor_shape_ok=True
max_terminal_corridor_lateral_abs_m=0.0031
terminal_path_until_first_pass_m=7.715
front_violations_until_first_pass=0/335
```

## Ground Station Monitoring Decision

Boss field decision:

- Use a third computer as a dedicated ground station.
- Ground station runs QGC/monitor/log/debug for both vehicles.
- This does not replace the Carrier onboard leader planner.
- Ground station visibility is required because the same pattern should later
  work when the two rovers are replaced by aircraft.

Logical communication planes:

```text
Control/planning plane:
  Carrier onboard Nano/Orin Nano <-> Mini
  LR24 compact state/plan/abort, safety-critical enough to keep low-rate and
  deterministic.

Monitoring plane:
  Ground station <-> Carrier
  Ground station <-> Mini
  QGC/MAVLink/log/health visibility, not the terminal control backbone.
```

Current LR24 field topology:

- Pair B, address `1102`: Carrier onboard computer/radio <-> Mini vehicle
  radio. This is the Carrier-Mini inter-vehicle state/plan/abort link.
- Pair C, address `1103`: Ground station <-> Carrier Pixhawk telemetry. This
  lets QGC monitor Carrier as `MAV_SYS_ID=1`.
- Pair A, address `1101`: Ground station <-> Mini Pixhawk telemetry. This lets
  QGC monitor Mini as `MAV_SYS_ID=2`.
- Pair names are now fixed by the physically paired radios above. Older notes
  that used Pair A for Carrier-Mini or Pair C for Mini-ground are superseded.

For QGC multi-vehicle monitoring, ensure distinct MAVLink system ids:

```text
Carrier MAV_SYS_ID = 1
Mini MAV_SYS_ID = 2
```

## 2026-07-22 Mock Docking Hardware Execution

The rover mock should execute the aerial docking story with the same role
semantics:

- Orin1/Carrier is the onboard leader and planner.
- Orin2/Mini is the faster fixed-wing mock.
- Mini first performs one stable orbit, then exits only at the CorridorPlan
  tangent trigger phase.
- Carrier computes the tangent-intercept CorridorPlan after Mini orbit is
  stable, follows a smooth arc into `T`, then continues in the same straight
  terminal tangent corridor as Mini.
- Low-level rover commands should be body-frame bounded primitives
  `(v_mps, omega_radps)`, not raw global `vx/vy` Offboard commands.
- Current detailed rover design is
  `docs/mock_docking_hardware_execution_design_2026_07_22.md`.

## Cross-Agent Rule

If the user makes a new boss-level architecture decision:

1. Update the primary shared whiteboard in `easydocking`.
2. Mirror the decision here if it affects rover hardware/test execution.
3. Add a concise entry to rover-side `docs/codex_cross_agent_log.md`.
4. Final replies should include `给对面 Codex 的话` with exact files to read.

## 2026-07-23 Persistent Cross-Machine Codex Coordination

Boss approved a Huawei ECS coordination control plane so Orin1 and Orin2 can
work continuously without Boss copying chat messages.

```text
Huawei ECS:
  mTLS NATS JetStream; persistent task/event/heartbeat streams

Orin1 / Carrier:
  persistent codex-agentd; Carrier/planner integration owner

Orin2 / Mini:
  persistent codex-agentd; Mini execution and local-safety owner

GitHub:
  source code, contracts, decisions, review, durable audit
```

An agent returns structured `peer_requests`; its local daemon dispatches those
tasks directly to the peer and preserves root/parent task lineage. NATS handles
wake-up, ACK/retry, progress, and compact results. Git inbox notes remain a
fallback when the broker is unavailable.

This cloud channel is software coordination only. It cannot grant motion,
arming, Offboard, actuator, serial, MAVLink, GPIO, Arduino, motor, or sudo
access, and it never replaces Pair B/LR24 vehicle communication. Deployment:
`docs/codex_cloud_coordination_runbook.md`.
