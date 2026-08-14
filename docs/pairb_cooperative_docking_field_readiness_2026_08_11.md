# Pair B Cooperative Docking Field Readiness

Date: 2026-08-11

## Scope and role map

- Orin2 / MAV_SYS_ID 2 is Carrier, leader, planner, and task publisher.
- Orin1 / MAV_SYS_ID 1 is Mini and executes validated Mini commands.
- Pair B transports only `MiniState`, `CorridorPlan`, `PlanCommand`,
  `MissionStatus`, and `Abort` during a mission.
- Pair B does not transport wheel PWM, high-rate body commands, ROS 2 DDS, or
  files. Each rover tracks its own finite ENU trajectory with its local generic
  trajectory controller.

## Implemented software path

`src/pairb_cooperative_docking.py` is hardware-independent and adds:

1. A hard gate requiring at least one qualified Mini orbit before planning.
2. A search over future Mini cutout phases. Each candidate is ranked by extra
   laps first, then path length, heading travel, and curvature.
3. A heading-constrained quintic Bezier Carrier approach with zero curvature
   at both endpoints. It starts at the observed Carrier yaw and ends tangent to
   the shared corridor. Candidates that reverse,
   self-intersect, enter the Mini orbit, or exceed minimum turn radius are
   rejected.
4. A common terminal tangent with different finite endpoints so Carrier stops
   ahead by the configured front gap.
5. An ETA and along-track-gap coordinator. Short Mini delays reduce the Carrier
   speed envelope. Persistent low Mini speed or Carrier inability enters HOLD
   and then ABORT. Stale state and Carrier-ahead violation abort immediately.
6. Conversion to compact `CorridorPlan` and low-rate `PlanCommand` messages.
   Commands contain phase, TTL, velocity ceiling, acceleration ceiling, and
   remaining distance; they are not wheel commands.
7. Separate Mini and Carrier speed envelopes. The rover configuration uses
   Mini `0.12--0.20 m/s`, Carrier `0--0.16 m/s`, and rejects a plan if the two
   terminal ranges do not overlap. The current common terminal window is
   `0.12--0.16 m/s`; the selected rendezvous speed is `0.14 m/s`.
8. Sustained terminal capture qualification. Success requires the Carrier-ahead
   gap, relative speed, lateral error, heading error, and yaw-rate error to stay
   inside their limits for 2 seconds. A one-sample position coincidence is not
   accepted.

`src/orin2_trajectory_tracker.py` now accepts an optional
`speed_ceiling_mps`. Its default is `None`, so existing single-rover behavior
is unchanged. A validated, fresh PlanCommand can reduce this ceiling down to
zero while the 50 Hz local tracker continues to own path projection, curvature,
and BODY_NED conversion. Invalid or over-limit ceilings are rejected.

The existing EasyDocking `StableOrbitQualifier` and Pair B ingress remain the
source of the real one-lap proof. The new planner consumes that qualified lap
count. Dense trajectories are not sent over Pair B: Mini reconstructs its
pre-installed orbit plus the transmitted tangent/corridor, while Carrier owns
its locally generated heading-constrained approach. Both sides must use the
same mission configuration and `origin_id`.

## State flow

```text
MiniState --> full-orbit qualifier --> future tangent search
                                      |              |
                                      |              +--> Mini orbit until trigger
                                      +--> Carrier smooth approach
                                                     |
                                shared terminal tangent corridor
                                                     |
MiniState + Carrier state --> ETA/gap coordinator --> speed envelopes
                                      |              |
                                      +--> HOLD -----+--> ABORT when persistent
```

Required live phase order:

```text
WAIT_MINI_STABLE_ORBIT
  -> PLAN_VALIDATED
  -> CARRIER_APPROACH_MINI_ORBIT
  -> MINI_TANGENT_EXIT
  -> SHARED_TERMINAL
  -> COMPLETE_HOLD
```

No command may skip the full-orbit proof or trigger Mini cutout early.

## Offline acceptance

Run:

```bash
python3 scripts/run_pairb_cooperative_docking_offline.py --scenario all
```

The run writes `timeline.csv`, `plan.json`, `summary.json`, `trajectory.png`,
and `speed_and_gap.png` under
`results/pairb_cooperative_docking_offline/<timestamp>/`.

Current accepted scenarios:

| Scenario | Expected result | Required evidence |
|---|---|---|
| nominal | COMPLETE | full lap first, no Carrier-behind sample |
| mini_lag | COMPLETE | Carrier speed envelope is reduced |
| persistent_mini_lag | ABORT | HOLD precedes persistent-sync ABORT |
| carrier_lag | ABORT | no blind pursuit after synchronization fails |
| link_loss | ABORT | stale MiniState immediately produces zero envelope |

## Read-only HIL and RViz

The HIL mode uses the real Carrier Pixhawk only as the initial pose/yaw source.
It never Arms, changes mode, publishes a setpoint, opens Pair B, or accesses the
Arduino. Carrier and Mini motion are shadow simulations anchored to the real
Carrier pose. This mode is intentionally separate from all outdoor executors.

```bash
CONFIRM_READ_ONLY_HIL=READ_ONLY_HIL \
HIL_ALLOWED_DISARMED_MODE=MANUAL \
./scripts/run_pairb_virtual_mini_hil_rviz.sh
```

RViz colors:

- cyan: Carrier cooperative plan
- yellow: Carrier shadow execution
- magenta: virtual Mini orbit, cutout, and terminal plan
- green: virtual Mini execution
- red: real disarmed Carrier sensor trace

The RViz text marker shows the current mission phase, coordination mode,
both full speed envelopes, measured and commanded shadow speeds, common
rendezvous speed, relative speed, front gap, terminal capture progress, and
coordination reason. `VIRTUAL_MINI` remains visible so replay data cannot be
mistaken for a real Pair B peer.

The HIL process stops its replay if MAVROS disconnects, pose/state becomes
stale, the vehicle Arms, or the observed mode leaves the explicit allow-list.

### Dual-Pixhawk read-only HITL

When both rover computers are present, run each Pixhawk through a separate
MAVROS namespace (`/carrier/mavros`, system 2; `/mini/mavros`, system 1). The
Carrier display gates the shadow replay on both real FCUs being connected,
disarmed, fresh, and in an explicitly allowed disarmed mode. It still never
Arms, changes mode, or publishes a vehicle setpoint.

The 2026-08-14 phone-hotspot test exposed asymmetric ROS 2 endpoint discovery:
both hosts discovered the peer nodes and multicast worked, but the Carrier did
not reliably discover the Mini MAVROS topic endpoints. For HITL visualization
only, `scripts/run_pairb_hitl_state_relay.py` sends strict, send-only Mini
`State`/pose JSON over unicast UDP. It has no receiver or command interface.
This relay is not a production vehicle link and must never replace Pair B for
outdoor `MiniState`, `CorridorPlan`, `PlanCommand`, `MissionStatus`, or `Abort`.

Carrier-side display command after both namespaced MAVROS instances and the
Mini relay are running:

```bash
CONFIRM_DUAL_READ_ONLY_HITL=DUAL_READ_ONLY_HITL \
ROS_DOMAIN_ID=99 \
MINI_HITL_RELAY_HOST=<current-mini-lan-ip> \
./scripts/run_pairb_dual_read_only_hitl_rviz.sh
```

The Mini IP is deliberately mandatory because field networks change. The
2026-08-14 run used Carrier `192.168.8.122`, Mini `192.168.8.244`, received the
Mini relay at 10 Hz, entered `HIL_READY dual_real_health=True`, completed the
full replay, and reported `HIL_REPLAY_COMPLETE; vehicle remained disarmed`.
Both FCUs were still connected and disarmed after completion.

After each replay or field run that produces `timeline.csv` and `plan.json`,
generate a time-correct XY animation directly from those logs:

```bash
python3 scripts/render_pairb_cooperative_xy_gif.py \
  --replay-dir results/pairb_cooperative_docking_offline/<run>/nominal \
  --speedup 4
```

The GIF keeps the full plans visible while growing both actual traces and
annotating phase, coordination mode, vehicle speeds, relative speed, front
gap, rendezvous speed, and terminal capture progress. GIF delay quantization
is included when deriving frame count; the 2026-08-14 result contains 498
frames at 80 ms and replays 158.9 seconds of log time in 39.76 seconds
(`3.996x`).

## Outdoor sequence for the next session

Every motion stage needs a new operator confirmation. Do not combine stages.

1. **No-motion baseline**: both systems disarmed; verify MAV_SYS_ID 2/1,
   FieldOrigin/origin_id, GPS/EKF, yaw, RC Kill, QGC stop, Pair B RTT/loss, and
   command TTL. FAIL stops the day.
2. **Mini-only orbit**: low speed, one complete circle, MiniState at 50 Hz.
   PASS requires radius, tangent heading, speed, state age, and accumulated
   phase to remain inside the qualifier limits. Carrier remains disarmed.
3. **Plan-only review**: Carrier computes the plan but both remain stopped.
   Inspect the chosen tangent, extra-orbit count, curve radius, ETA, validity,
   and RViz geometry. Reject any reverse, fold-back, extra loop, or field-boundary
   violation.
4. **Carrier approach with Mini still orbiting**: run only the approach phase.
   Mini must not cut out. Verify Carrier reaches the corridor ahead and both
   local trackers stay within cross-track limits.
5. **Bounded full mission**: permit Mini cutout, shared terminal, and finite
   stop. Start at conservative speed ceilings. PASS requires Carrier ahead for
   every terminal sample and the target gap at stop.
6. **Cooperation injection**: deliberately reduce Mini speed for a short window.
   PASS requires visible Carrier slowdown and recovery. Repeat with persistent
   slowdown; PASS requires HOLD then ABORT.
7. **Link-loss injection**: stop MiniState. PASS requires TTL/watchdog stop and
   Abort without wheel-level commands over Pair B.

Stop immediately on unexpected Arm/mode, invalid origin, stale state, RC loss,
cross-track envelope violation, Carrier behind, wall/obstacle risk, or any plan
that differs from RViz/logged geometry.

## Remaining live integration gate

The pure planner, wire objects, temporal coordinator, offline scenarios, and
read-only HIL visualization are ready. The production wheel executors are not
connected to this new cooperative path yet. Before outdoor motion, bind the
validated `CorridorPlan` to each local generic trajectory worker and bind only
the low-rate `PlanCommand.max_speed_mps` envelope to the worker's speed ceiling.
That binding must be tested first with both wheels lifted and then as separate
single-rover stages; it must not bypass existing Arm, Kill, stale-state, mode,
or zero-output protections.
