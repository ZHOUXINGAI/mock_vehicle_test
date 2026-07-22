# Codex Cross-Agent Workflow

This document defines how the rover/hardware Codex and the docking/simulation
Codex coordinate work without mixing their memories or ownership.

Active profiles:

- Rover Codex:
  - Workspace: `/home/jetson/mock_vehicle_test`
  - Codex home: `/home/jetson/.codex`
  - Role: real rover, PX4 rover firmware, MAVROS, QGC, RC safety, motors,
    field tests, hardware logs.
- Docking Codex:
  - Workspace: `/home/jetson/easydocking`
  - Codex home: `/home/jetson/.codex_docking`
  - Role: two-vehicle docking simulation, CorridorPlan, planners,
    controller logic, metrics, reports.

Launch commands:

```bash
# Rover/hardware side
cd /home/jetson/mock_vehicle_test
codex

# Docking/simulation side, new or resumed session
/home/jetson/bin/codex-docking
/home/jetson/bin/codex-docking-resume
```

## Primary Coordination Workspace

Use `/home/jetson/mock_vehicle_test/codex_ops` as the first shared office for both Codex agents.
This directory is tracked by the existing `mock_vehicle_test` GitHub repo:

```text
/home/jetson/mock_vehicle_test/codex_ops
git@github.com:ZHOUXINGAI/mock_vehicle_test.git
```

The preferred live path is the Huawei ECS NATS JetStream service plus one
persistent `codex-agentd` on each Orin. It supports direct Orin1-to-Orin2
structured tasks, ACK/retry, progress, and heartbeat. The Git sequence below is
still mandatory for source synchronization and is the broker-outage fallback.

Mandatory Git sequence for cross-agent work:

```bash
cd /home/jetson/mock_vehicle_test/codex_ops
git pull --rebase --autostash
./scripts/codex_ops.py doctor --agent rover
./scripts/codex_ops.py checkin --agent rover --repo /home/jetson/mock_vehicle_test --status working --task "<task>"
```

Write peer requests to `codex_ops/inbox/docking/` using
`./scripts/codex_ops.py note`. The old repo-local
`docs/codex_cross_agent_log.md` files remain useful for detailed project
history, but `codex_ops` is now the durable cross-machine coordination surface.

## Ownership

Rover Codex owns:

- `mock_vehicle_test` code, docs, scripts, Arduino bridge code, PX4 rover
  parameter procedures, MAVROS run scripts, real rover test reports.
- Ground truth about what the physical rover actually did.
- Hardware safety decisions. Any motion test requires fresh user confirmation.

Docking Codex owns:

- `easydocking` code, CorridorPlan, PX4 SIH/SITL simulation, controller
  parameters, planner metrics, report generation.
- Algorithmic expectations that should be tested on the rover.
- Simulation model changes when hardware observations show a bad assumption.

Do not edit the peer repo unless the user explicitly asks for a cross-repo
change. Normal cross-agent communication should be written as a `codex_ops`
event or peer inbox note; repo-local `docs/codex_cross_agent_log.md` files are
secondary history.

## Shared Truth

Use these files as the shared memory surface. Read `codex_ops` first:

- Operational shared office:
  `/home/jetson/mock_vehicle_test/codex_ops`
- Operational meeting state:
  `/home/jetson/mock_vehicle_test/codex_ops/state/meeting_state.md`
- Peer inboxes:
  `/home/jetson/mock_vehicle_test/codex_ops/inbox/rover/` and
  `/home/jetson/mock_vehicle_test/codex_ops/inbox/docking/`

- Primary shared meeting whiteboard:
  `/home/jetson/easydocking/docs/codex_shared_meeting_state.md`
- Rover-side shared meeting mirror:
  `/home/jetson/mock_vehicle_test/docs/codex_shared_meeting_state.md`
- Rover state: `/home/jetson/mock_vehicle_test/AGENT_STATE.md`
- Rover cross-agent log:
  `/home/jetson/mock_vehicle_test/docs/codex_cross_agent_log.md`
- Docking state: `/home/jetson/easydocking/HANDOFF.md`
- Docking ground-vehicle plan:
  `/home/jetson/easydocking/docs/ground_vehicle_2d_corridor_plan_handoff.md`
- Docking cross-agent log:
  `/home/jetson/easydocking/docs/codex_cross_agent_log.md`

For architecture or deployment decisions, the latest boss-decision section in
the primary shared meeting whiteboard wins over older cross-agent log entries.

Keep `.codex` and `.codex_docking` separate. Do not merge SQLite state,
memory, goals, or logs.

## Work Loop

1. The active Codex receives a versioned task from its local persistent worker,
   or pulls the Git inbox when the broker is unavailable.
2. The active Codex reads its local state file and this workflow.
3. For architecture, communication, planner-deployment, or field-test protocol
   questions, read the primary shared meeting whiteboard before answering.
4. If the task came from the peer, read the peer inbox note in `codex_ops`.
5. Make changes only in the owned repo unless the user explicitly widens scope.
6. Run the smallest useful verification.
7. Return a structured `peer_requests` entry when the peer needs to act. The
   local worker dispatches it directly; use a Git inbox note as durable context
   or fallback.
8. Commit/push only when the user asks, or when the active repo's standing
   instructions explicitly allow it.

Full cloud deployment procedure:

```text
/home/jetson/mock_vehicle_test/docs/codex_cloud_coordination_runbook.md
```

## Request Template

Use this when asking the peer Codex to act:

```text
## YYYY-MM-DD HH:MM CST - Request to <rover|docking>

From:
To:
Related commit or file:
Status: open

Context:
- ...

What changed:
- ...

Need from peer:
- ...

Expected validation:
- ...

Safety or scope limits:
- ...
```

## Result Template

Use this when reporting back to the peer:

```text
## YYYY-MM-DD HH:MM CST - Result for <request id or topic>

From:
To:
Status: done | blocked | needs-followup

What was tested:
- ...

Observed result:
- ...

Evidence:
- Commit:
- Log/result path:
- User field observation:

Implication for peer:
- ...

Next suggested action:
- ...
```

## Hardware Safety

The docking Codex may propose rover experiments, but the rover Codex controls
how they are made safe. Before any real vehicle motion:

- confirm vehicle location and clear area;
- confirm RC/QGC/manual stop path;
- confirm whether wheels are lifted or on the ground;
- use conservative speed/distance first;
- record what actually happened, including slips, magnetic interference,
  GPS problems, and unexpected arming behavior.

## Current Bridge Topic

The first shared technical bridge is:

```text
CorridorPlan / 2D docking planner in easydocking
        -> rover offboard primitive tests in mock_vehicle_test
        -> hardware observations back into the simulation model
```

The rover side has already validated basic differential-rover Offboard motion:

- forward 3 m, right turn about 90 deg, forward 3 m;
- forward 3 m, left turn about 90 deg, forward 3 m;
- turn direction sign must be selected per test;
- magnetic/GPS placement can affect heading behavior.

The docking side should treat those as the current hardware baseline before
requesting more complex 2D CorridorPlan field tests.
