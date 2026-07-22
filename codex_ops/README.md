# Codex Ops

Shared coordination workspace for the rover/hardware Codex and the
docking/planner Codex.

This directory lives inside the existing `mock_vehicle_test` repo. It is not a
separate repository and it is not part of the robot control loop. Treat it as
the durable shared office:

- current boss decisions;
- agent status and heartbeat;
- cross-agent requests/results;
- event log;
- interface contracts;
- run coordination notes.

Keep the two Codex homes separate:

```text
Rover Codex:   /home/jetson/.codex
Docking Codex: /home/jetson/.codex_docking
```

Use this workspace so the agents do not depend on copied chat snippets.

## Quick Start

At the start of every Codex session:

```bash
cd /home/jetson/mock_vehicle_test/codex_ops
git pull --rebase --autostash
./scripts/codex_ops.py doctor
./scripts/codex_ops.py checkin --agent rover --repo /home/jetson/mock_vehicle_test --status working --task "short task summary"
```

For the docking Codex:

```bash
cd /home/jetson/mock_vehicle_test/codex_ops
git pull --rebase --autostash
./scripts/codex_ops.py doctor
./scripts/codex_ops.py checkin --agent docking --repo /home/jetson/easydocking --status working --task "short task summary"
```

When one Codex needs the other to act:

```bash
./scripts/codex_ops.py note \
  --from rover \
  --to docking \
  --type request \
  --title "Update CorridorPlan rover primitive contract" \
  --summary "Rover execution uses body-frame v/omega, not global vx/vy." \
  --need "Update easydocking handoff and planner bridge assumptions." \
  --file /home/jetson/mock_vehicle_test/docs/mock_docking_hardware_execution_design_2026_07_22.md
```

After updating this workspace:

```bash
cd /home/jetson/mock_vehicle_test
git add codex_ops
git commit -m "ops: update rover/docking coordination" -- codex_ops
git push
```

Or use the wrapper:

```bash
codex-ops-commit "ops: update rover/docking coordination"
```

## GitHub Sync

`codex_ops/` is tracked by the existing `mock_vehicle_test` GitHub repository:

```text
/home/jetson/mock_vehicle_test
git@github.com:ZHOUXINGAI/mock_vehicle_test.git
```

If there is no internet in the field, push/pull through a ground-station bare
repo first, then sync the ground station to GitHub later.

## Main Files

```text
codex_ops/AGENTS.md                         mandatory operating rules
codex_ops/state/meeting_state.md            current shared truth
codex_ops/agents/rover.json                 rover Codex heartbeat/status
codex_ops/agents/docking.json               docking Codex heartbeat/status
codex_ops/inbox/rover/                      notes docking -> rover
codex_ops/inbox/docking/                    notes rover -> docking
codex_ops/events/YYYY/YYYY-MM-DD.jsonl      append-only event stream
codex_ops/contracts/                        stable cross-repo contracts
codex_ops/rules/                            long-term operating rules
codex_ops/scripts/codex_ops.py              helper CLI
```

## Rule Of Thumb

Use GitHub for coordination facts, not robot runtime control.

Runtime robot communication still belongs in MAVLink, LR24 compact packets,
ROS 2 bridges, logs, and local safety controllers.
