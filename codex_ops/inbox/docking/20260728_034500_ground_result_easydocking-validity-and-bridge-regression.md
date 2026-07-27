# Ground Result: EasyDocking Validity Gate and Bridge Regression

Date: 2026-07-28 CST

## Outcome

- EasyDocking software-only two-rover slice is now at commit
  `0f17a82b8769083f4af55a944ba309b898f9c72f` on `main`.
- Ground, Orin1/Carrier, and Orin2/Mini all ran the same 28 focused
  `test_ground_*.py` tests successfully.
- The deterministic offline replay reaches `COMPLETE_HOLD`; the fault replay
  includes and passes `plan_validity_rejected`.
- No QGC, ROS node, MAVROS, PX4, Offboard, LR24/serial, Arduino, vehicle,
  actuator, or motor process was started.

## Plan-validity contract

`GroundCorridorPlan` schema version is now 2. It publishes the requested,
required, and remaining plan validity plus the components of the post-tangent
reserve. The planner computes:

```text
post_tangent_reserve =
    terminal_completion_budget
  + completion_hold
  + max(command_ttl, local_command_watchdog)
  + timing_guard

required_validity =
    ceil_to_100ms(mini_arrival_delay + post_tangent_reserve)
```

The policy is fail-closed: if requested validity is below the calculated
requirement, plan creation rejects with `plan_validity_insufficient`. It never
silently lengthens an immutable plan. The nominal replay reports:

- Mini arrival delay: `28398ms`
- post-tangent reserve: `3350ms`
- required validity: `31800ms`
- requested validity: `32000ms`
- margin: `200ms`

`target_front_gap_m` is still protocol metadata. The present speed/yaw-rate
leader does not close that exact gap in a real follower loop, so the passing
offline replay is not evidence of physical closed-loop gap regulation.

## Deployment

- Orin1 `/home/jetson/easydocking`: fast-forwarded from `46c8df2` to
  `0f17a82`; unrelated dirty/untracked files were preserved.
- Orin2 `/home/seeed/easydocking`: created as a sparse checkout at
  `0f17a82` with `.github`, `PlanReview`, `config`, `docs`, `scripts`, `src`,
  and `tests`.
- Ground local checkout is clean at `0f17a82`.

## Coordination reliability

The NATS/JetStream long-task keepalive isolation fix is in
`mock_vehicle_test` commit
`467e66a8603194937c86dae39a33342b98dbadb3`; restricted service control was
added by `0e194b5da6655c8b242318c8eca9610baae27c29`.

Orin1 regression task:

- task: `bca110c0-cbc9-4d01-97ff-a24b99e09a5f`
- pinned Codex session:
  `019e3b9d-8417-76d2-bf88-4ec59aba48c4`
- command: one 35-second sleep followed by read-only `git rev-parse`
- accepted events: 1
- completed events: 1
- attempts: 1
- Bridge PID before/after: `1361949`
- systemd restart count: `0`
- result: `0e194b5；keepalive 长任务只执行一次`

Ground public-key SSH to Orin1 now succeeds non-interactively. The only
passwordless root actions granted to `jetson` are the root-owned helper's
exact `status` and `restart` operations for
`codex-agentd-orin1-carrier.service`.

## Runtime transport boundary

- LR24 Pair B is only for compact runtime state, coordinates, immutable plan,
  phase/command, sequence, timestamp, watchdog, and Abort traffic.
- Pair B must not carry repositories, files, patches, build artifacts, bulk
  logs, Codex traffic, or NATS.
- GitHub/SSH carries code and files.
- NATS coordinates Codex tasks only and is never in the vehicle-control path.

## Current blockers before any rover motion

1. Pair B wire encoding/decoding must be updated and tested for plan schema 2.
2. Carrier and Mini live followers/local executors are not yet connected to
   the offline leader.
3. Exact `target_front_gap_m` closed-loop regulation is not implemented.
4. The two-rover orchestrator, local Abort ownership, and a zero-motion
   end-to-end Pair B trial remain to be completed.
5. Physical motion remains unauthorized until the user explicitly starts that
   stage.
