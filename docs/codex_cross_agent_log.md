# Codex Cross-Agent Log - Rover Side

This is the rover/hardware Codex outbound log. Write here when the docking
Codex needs a fact, test result, or request from the real rover side.

Protocol:

- Workflow: `docs/codex_cross_agent_workflow.md`
- Local state: `AGENT_STATE.md`
- Peer state: `/home/jetson/easydocking/HANDOFF.md`
- Peer log: `/home/jetson/easydocking/docs/codex_cross_agent_log.md`

## 2026-06-25 00:00 CST - Protocol Installed

From: rover
To: docking
Status: done

Context:
- The user wants two Codex profiles to work as a closed loop from simulation to
  hardware.
- The profiles must remain isolated: `/home/jetson/.codex` for rover and
  `/home/jetson/.codex_docking` for docking.

Rover baseline for docking:
- PX4 v1.17 differential rover firmware is the active real-rover baseline.
- MAVROS param plugin is disabled for stability in the no-param run script.
- Manual RC mapping is `CH2` forward/backward and `CH4` steering.
- Verified Offboard primitives include forward 3 m, left/right 90 deg turn,
  then forward 3 m.

Need from docking:
- When CorridorPlan is ready for a small-car field experiment, provide a
  reduced 2D test with expected inputs, outputs, speed limits, turn behavior,
  and pass/fail criteria.

## 2026-06-25 00:55 CST - Result for finite-field slow-carrier route review

From: rover
To: docking
Status: needs-followup before any real ground route

What was reviewed:
- Latest docking request:
  `/home/jetson/easydocking/docs/codex_cross_agent_log.md`
  entry `2026-06-25 00:32 CST - Revision to rover: finite-field slow-carrier route`.
- Route config captured locally:
  `config/ground_2d/finite_field_slow_carrier_route_2026_06_25.json`.
- Rover-side preflight document:
  `docs/ground_2d_corridor_rover_preflight_2026_06_25.md`.
- Offline no-hardware review output:
  `results/ground_2d_corridor_preflight/latest/route_summary.md`.

Field feasibility:
- Sim route envelope is `12.23m x 11.22m`.
- It fits inside `30m x 30m`.
- It also fits inside the recommended `20m x 20m` safety area, but that leaves
  only about `3.88m` x-margin and `4.39m` y-margin around the simulated
  envelope. Prefer `30m x 30m` for the first outdoor run.

Recommended rover control interface:
- Immediate bench stage: short primitives.
- First field implementation: bounded continuous `v, omega` primitive follower.
- Not recommended as first interface: raw waypoint/pose-only control.

Reason:
- The current proven real rover path is MAVROS `setpoint_velocity` in
  `BODY_NED` with short primitives, not a complete dual-rover route follower.
- The successful `3m -> 90deg -> 3m` tests used body-frame velocity primitives
  and local pose/yaw checks.
- Direct yaw-rate / reverse behavior was previously not reliable enough to
  assume for a full route.

Can rover run this now?
- No full ground route yet.
- Yes for offline prep and next wheels-lifted bench checks.
- The route asks for Mini `0.9m/s` and Carrier `0.7m/s`; current proven safety
  caps were around `0.20-0.35m/s`, so target speed must be bench-calibrated
  before outdoor use.

Missing before first real ground route:
- PX4/QGC parameter export for both rovers.
- Confirm both rovers use the differential Arduino bridge and PX4 output
  mapping expected by the Offboard scripts.
- Confirm local frame alignment: physical `(0,0)`, x/y axes, Mini circle
  markers, Carrier start marker.
- Confirm synchronized start procedure for Mini and Carrier.
- Verify RC/QGC/manual stop and physical power cutoff on both vehicles.
- Verify Arduino timeout brake on both vehicles.
- Verify yaw/turn sign on bench; current BODY_NED left/ccw baseline used
  `TURN_DIRECTION_SIGN=-1.0`.

Next rover-side preparation:
- Do not run on the ground first.
- Run wheels-lifted/low-power bench checks only after fresh user safety
  confirmation.
- Bench sequence should verify Mini low-speed -> target `0.9m/s`, Carrier
  low-speed -> target `0.7m/s`, left/right yaw signs, stop behavior, and
  Arduino timeout brake.

## 2026-06-26 00:00 CST - Request to docking: central two-rover LR24 bridge

From: rover
To: docking
Related file: `docs/two_rover_lr24_comms_architecture_2026_06_26.md`
Status: superseded by `2026-06-27 00:55 CST - Correction applied: Carrier onboard Nano is central leader`

Superseded note:
- The ground-computer primary-planner wording below is historical and should
  not be used as the active architecture.
- Active architecture is Carrier onboard Nano/Orin Nano as central/leader
  docking computer; ground station is QGC/logging/monitoring/setup/manual
  safety/debug.

Context:
- User bought two sets of MicoAir LR24 telemetry radios.
- User asked how the two rovers should communicate and where the docking
  planner should run.
- Rover side reviewed the LR24 manual and PX4 telemetry usage.

Rover-side recommendation:
- First outdoor two-rover experiment should use an independent ground computer
  as the mission coordinator/planner.
- Do not deploy the first planner on the moving Carrier Orin Nano.
- Use two independent LR24 pairs:
  - Pair A: ground computer USB serial <-> Carrier Pixhawk TELEM.
  - Pair B: ground computer USB serial <-> Mini Pixhawk TELEM.
- Configure the two LR24 pairs with different module addresses, for example
  `ADDR=1101` for Carrier and `ADDR=1102` for Mini.
- Keep LR24 in duplex mode for PX4 telemetry/control.

Need from docking:
- Implement the first ground bridge as a central coordinator that can talk to
  two vehicles, e.g. `/carrier/mavros/...` and `/mini/mavros/...`.
- Keep the command protocol compact and bounded: role, phase, `v_mps`,
  `omega_radps`, duration/distance, speed/accel limits, stop conditions.
- Do not assume ROS2 DDS over LR24.
- Do not assume QGC owns the serial port directly; prefer a router such as
  mavlink-router/MAVProxy so QGC and the planner can both observe MAVLink.

Safety/scope limits:
- LR24 is for telemetry/control, not the only stop path.
- RC/QGC/manual stop and vehicle-local failsafe remain rover-side requirements.
- First tests are no-motion telemetry and wheels-lifted command path only.

## 2026-06-27 00:15 CST - Sync from docking memory: architecture constraints

From: rover
To: docking
Related file:
`/home/jetson/.codex_docking/memories/cross_agent_reply_rule.md`
Status: superseded by `2026-06-27 00:55 CST - Correction applied: Carrier onboard Nano is central leader`

What changed:
- Docking profile added a durable cross-agent reply rule that preserves
  scheme-level communication/deployment conclusions.
- Rover side reviewed it and updated
  `docs/two_rover_lr24_comms_architecture_2026_06_26.md` to match.

Shared architecture constraints now treated as active:
- Historical note: this entry still carried an interim "central ground
  computer" phrase. The active boss decision is Carrier onboard Nano/Orin Nano
  as central/leader docking computer.
- Carrier is docking leader; Carrier Orin runs docking planner; Mini sends
  timestamped compact state and locally tracks received CorridorPlan/phase
  commands.
- LR24: low-rate compact state/plan/abort only; no ROS2 DDS, no image/video,
  no dense trajectory spam, no high-rate terminal closed loop.
- 4G/WiFi: auxiliary monitoring/log/video/backup observability only; not
  safety-critical terminal docking control.
- Terminal docking: onboard autonomy; Carrier stereo/vision handles final
  relative pose closure.
- Latency rule: all state packets need timestamp and sequence number; planner
  predicts forward and freezes/replans/aborts on stale packets.
- Rover field constraint: about `30m x 30m`; Mini faster, Carrier slower; start
  with no-motion telemetry and wheels-lifted tests.
- Message budget target: Mini state `80-150 bytes @ 5-10Hz`; planner command
  event-triggered or `1-5Hz`.

Need from docking:
- When implementing the central ground bridge, keep the message schema aligned
  with timestamp/sequence/stale-data handling from the start.
- Keep the later Carrier-leader onboard architecture in mind so the central
  bridge can be migrated instead of rewritten.

## 2026-06-27 00:55 CST - Correction applied: Carrier onboard Nano is central leader

From: rover
To: docking
Related files:
- `/home/jetson/easydocking/docs/codex_shared_meeting_state.md`
- `docs/codex_shared_meeting_state.md`
- `docs/two_rover_lr24_comms_architecture_2026_06_26.md`
Status: synced / supersedes earlier rover-side ground-laptop wording

Correction:
- Rover side previously still had text that made the human-side ground laptop
  sound like the first primary planner. That is now corrected.
- Active boss decision: Carrier rover onboard Nano/Orin Nano is the
  central/leader docking computer.
- Ground station remains QGC/logging/monitoring/setup/manual safety/debug, not
  the main planner.

Current rover-side architecture:
- Carrier Nano/Orin runs `easydocking` planner / CorridorPlan leader.
- Carrier receives Mini compact timestamped state over LR24 or equivalent
  low-rate link.
- Carrier sends Mini compact plan/phase/speed/abort primitives.
- Mini locally tracks received primitives and owns local failsafe.
- LR24 stays low-rate compact state/plan/abort only.
- Full route motion still requires explicit user approval.

Need from docking:
- Treat any older "ground computer central planner" text as superseded unless
  explicitly framed as a temporary shadow/monitor/debug role.
- Implement bridge and message schema for Carrier onboard leader topology.

## 2026-06-27 01:20 CST - Rover prep ready: no-motion LR24 benchmark and field runbook

From: rover
To: docking
Related files:
- `docs/outdoor_two_rover_carrier_leader_runbook_2026_06_27.md`
- `config/ground_2d/carrier_leader_field_test_2026_06_27.json`
- `src/lr24_compact_protocol.py`
- `scripts/lr24_link_benchmark.py`
- `scripts/run_lr24_no_motion_benchmark.sh`
Status: ready for no-motion communication testing only

What changed:
- Added dependency-free compact LR24 frame protocol for no-motion benchmark.
- Added benchmark modes:
  - `sizes`
  - `echo`
  - `ping`
  - `state-tx`
  - `state-rx`
  - `command-tx`
  - `command-rx`
- Added morning outdoor runbook for Carrier-leader topology.

Compact frame sizes:
- MiniState frame: `30 bytes`
- PlanCommand frame: `37 bytes`
- Ping/Pong frame: `19 bytes`

Carrier-leader morning sequence:
- Pair A: Carrier Nano/Orin <-> Mini side compact LR24 link.
- Pair B: Ground station <-> Carrier monitor/router endpoint, optional for QGC
  and logs.
- First tests are no-motion: ping/echo, MiniState stream, PlanCommand stream.
- Then wheels-lifted checks only after fresh user safety confirmation.
- Full route motion still requires explicit user approval.

Need from docking:
- Align the `easydocking` Carrier-leader bridge message schema with these
  fields: sequence, timestamp, role, phase, `v_mps`, `omega_radps`,
  duration/distance, valid-until, max speed/accel, abort/hold.
- Keep bridge code separate from ROS2 DDS over LR24; use compact link or an
  equivalent low-rate transport boundary.

## 2026-06-27 01:55 CST - Pair A bidirectional dry-run added

From: rover
To: docking
Related files:
- `docs/outdoor_two_rover_carrier_leader_runbook_2026_06_27.md`
- `docs/codex_shared_meeting_state.md`
- `scripts/lr24_pair_a_dry_run.py`
- `scripts/run_lr24_pair_a_dry_run.sh`
Status: ready for no-motion mixed-traffic Pair A testing only

What changed:
- Added a Carrier/Mini dry-run that uses one LR24 pair bidirectionally:
  - Mini side sends simulated compact `MiniState` frames at `5-10Hz`.
  - Carrier side receives state and sends compact `PlanCommand` frames at
    `1-5Hz`.
  - Carrier falls back to `HOLD` when Mini state is missing or stale.
- The dry-run is serial-only. It does not connect to MAVROS, PX4, ROS,
  Arduino, or motor execution.
- Logs/CSV are saved under `results/lr24_pair_a_dry_run/latest/`.

Need from docking:
- Use this as the transport boundary for the first Carrier-leader bridge:
  replace simulated Mini state with real Mini state, and replace Carrier HOLD
  source with CorridorPlan/phase decisions.
- Keep default first field command at `HOLD`; non-HOLD route commands should
  only be enabled after no-motion link, wheels-lifted checks, and explicit
  user approval.

## 2026-06-27 02:10 CST - Easydocking ground planner copied into mock execution repo

From: rover
To: docking
Related files:
- `scripts/run_ground_2d_corridor_sim.py`
- `scripts/run_ground_2d_corridor_preview.sh`
- `docs/local_easydocking_planner_in_mock_vehicle_test.md`
- `docs/easydocking_ground_vehicle_2d_corridor_plan_handoff.md`
- `docs/easydocking_msgs/CorridorPlan.msg`
- `src/lr24_compact_protocol.py`
- `scripts/lr24_link_benchmark.py`
- `scripts/lr24_pair_a_dry_run.py`
Status: ready for local no-motion planner preview and LR24 transport dry-run

What changed:
- User approved copying easydocking planning/control scripts into
  `mock_vehicle_test` so outdoor execution can happen from one repo.
- Copied easydocking `scripts/run_ground_2d_corridor_sim.py` into this repo.
- Added local wrapper `scripts/run_ground_2d_corridor_preview.sh`.
- Added compact LR24 `CORRIDOR_PLAN` frame:
  - MiniState frame: `30 bytes`
  - PlanCommand frame: `37 bytes`
  - CorridorPlan frame: `47 bytes`
  - Ping/Pong frame: `19 bytes`
- Pair A dry-run can now test:
  - MiniState uplink;
  - CorridorPlan downlink;
  - HOLD PlanCommand downlink.

Local preview result:
- completed: `true`
- route bbox until first pass: `12.23m x 11.22m`
- front violations until first pass: `0/334`
- tangent point: `(-1.553, -4.224)`
- tangent direction: `(0.939, -0.345)`
- tangent trigger phase: about `249.8 deg`
- Mini arrival delay after plan: about `25.724s`

Need from docking:
- Treat `mock_vehicle_test` as the outdoor execution repo.
- Use the copied 2D planner and compact `CORRIDOR_PLAN` transport boundary
  when discussing rover field tests.
- Do not assume dense trajectory or ROS2 DDS over LR24.

## 2026-06-27 01:34 CST - Rover preview terminal corridor shape fixed

From: rover
To: docking
Related files:
- `scripts/run_ground_2d_corridor_sim.py`
- `docs/codex_shared_meeting_state.md`
- `docs/local_easydocking_planner_in_mock_vehicle_test.md`
- `docs/outdoor_two_rover_carrier_leader_runbook_2026_06_27.md`
Status: offline preview corrected; no hardware motion

Problem:
- User noticed the Carrier/green track in `trajectory_xy_full.png` bent after
  tangent point `T`.
- That was caused by the terminal lateral-PD chase term pulling Carrier toward
  Mini instead of preserving the straight tangent corridor.

Fix:
- Default terminal mode is now `straight-corridor`.
- Carrier still adjusts along-track speed/front gap, but terminal lateral
  speed defaults to `0`.
- The old behavior can only be requested explicitly:
  `--terminal-lateral-mode chase`.
- The plots now include the planned terminal tangent corridor and a
  `carrier_corridor_lateral_error.png` diagnostic.
- Summary now reports:
  - `terminal_corridor_shape_ok`
  - `max_terminal_corridor_lateral_abs_m`
  - `max_terminal_heading_error_deg`
  - `terminal_lateral_mode`

Verified local result:
- Output:
  `results/ground_2d_corridor_preview/20260627_013226_ground_2d`
- `completed=True`
- `terminal_lateral_mode=straight-corridor`
- `terminal_corridor_shape_ok=True`
- `max_terminal_corridor_lateral_abs_m=0.0031`
- `max_terminal_heading_error_deg=2.62`
- `terminal_path_until_first_pass_m=7.715`
- `front_violations_until_first_pass=0/335`
- `trajectory_xy_full.png` visually checked: Carrier arc reaches `T`, then
  follows the same straight tangent corridor as Mini. No visible hook/S-turn.

Need from docking:
- Treat rover-side preview default as the corrected high-level CorridorPlan
  contract.
- If comparing old plots, explicitly label them as lateral chase mode.
- For field execution discussion, use the straight terminal corridor geometry;
  do not ask rover-side execution to chase Mini laterally at the macro-planner
  layer.

## 2026-06-27 17:25 CST - Safety blocker: Carrier left wheels twitch after Mini LR24 setup

From: rover
To: docking
Related file:
- `docs/field_incident_left_wheel_twitch_after_mini_lr24_2026_06_27.md`
Status: blocks any real dual-rover ground motion

Field incident:
- During outdoor setup, user observed the Carrier rover left-front and
  left-rear wheels occasionally twitching/spinning even with RC kill engaged.
- User correctly noted the symptom appeared after the Mini LR24 communication
  setup was changed.
- Rover side stopped field motion testing.

What rover side did:
- Killed QGC, MAVROS, Offboard, manual-control, RC override, and fake-vision
  processes.
- Verified no process was holding `/dev/ttyUSB0`, `/dev/ttyUSB1`, or
  `/dev/ttyACM0`.
- Hardened and flashed the Arduino D24A PWM bridge with input pullups,
  consecutive-valid-frame gating, and `STBY` low on zero command.
- Arduino serial then showed neutral Pixhawk PWM and `left=0 right=0`.
- User still observed occasional twitch after D24A motor power was reconnected.

Current interpretation:
- Treat the Mini LR24 change as a real possible trigger.
- The most likely path is not a confirmed high-level Carrier MAVLink drive
  command. It is more likely LR24/USB/RF/cable/ground/power interaction
  exposing a bottom-layer motor-driver safety weakness:
  Arduino reset/floating pins, D24A `STBY` without hardware pulldown, noisy
  motor power, or D24A channel/hardware behavior.

Next rover-side bench test:
- Upload `standby` Arduino firmware, which ignores Pixhawk inputs and forces
  D24A `STBY=LOW`, all PWM pins low, all direction pins low.
- With wheels lifted, connect D24A motor power and observe.
- If twitch remains in standby firmware, the root cause is downstream hardware:
  D24A/wiring/ground/power/noise.

Need from docking:
- Do not request real Carrier/Mini route motion until this blocker is cleared.
- Planner/LR24 no-motion dry-runs are still fine if motor power is physically
  disconnected.
- Any next field plan must include a real motor-power cutoff, not only PX4 RC
  kill.

## 2026-07-22 CST - Rover hardware execution design for mock docking

From: rover
To: docking
Related files:
- `docs/mock_docking_hardware_execution_design_2026_07_22.md`
- `docs/two_rover_lr24_comms_architecture_2026_06_26.md`
- `config/ground_2d/carrier_leader_field_test_2026_06_27.json`

User hardware mapping:
- Orin1 is the Carrier rover / carrier aircraft mock.
- Orin2 is the Mini rover / fixed-wing child aircraft mock.
- PairB (`ADDR=1102`) is the Carrier-Mini compact coordination link.
- PairC (`ADDR=1103`) is Ground-QGC <-> Carrier Pixhawk.
- PairA (`ADDR=1101`) is Ground-QGC <-> Mini Pixhawk.

Rover-side interpretation after reading easydocking:
- Carrier remains the leader and computes CorridorPlan after Mini has completed
  at least one stable orbit.
- Mini executes stable orbit first, sends timestamped state to Carrier, then
  exits at the planned tangent trigger phase.
- Carrier follows the tangent-compatible arc into `T`, then both vehicles use
  the same straight terminal tangent corridor.
- Terminal macro geometry must remain straight corridor; no hook/S-turn or
  lateral chase after `T`.

Hardware execution constraint:
- Rover execution should use local bounded body-frame primitives
  `(v_mps, omega_radps, duration/distance, valid_until)` and local path
  following.
- Do not feed global `vx/vy` directly into rover Offboard for the field route;
  earlier rover testing showed it can cause yaw pre-correction before motion.

Immediate gap for rover side:
- Implement the live bridge:
  `PX4/MAVROS state -> MiniState` and
  `CorridorPlan/PlanCommand -> primitive executor -> PX4 Offboard v/omega`.
- Keep no-motion mode as the default. Motion requires fresh user confirmation.
