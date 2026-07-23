# Codex Ops

Shared coordination workspace for Orin1/Carrier, Orin2/Mini, and the local
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

Use this workspace so the agents do not depend on copied chat snippets. The
preferred live path is the mTLS NATS JetStream service under `realtime/`; GitHub
remains the source-code and audit path.

## Live Coordination

```text
Huawei ECS NATS JetStream
  codex.task.orin1-carrier -> persistent Orin1 worker
  codex.task.orin2-mini    -> persistent Orin2 worker
  codex.event.*            -> ACK/progress/result stream
  codex.heartbeat.*        -> online status
```

The owner can emit a structured peer request and wake the other Orin directly.
Boss no longer needs to copy the first agent's response into the second chat.

Deployment and commissioning:

```text
docs/codex_cloud_coordination_runbook.md
```

## Visible Agent Consoles

The persistent worker remains a systemd service, while an operator can watch
the complete task lifecycle in a normal terminal or a VS Code integrated
terminal. This avoids competing consumers and does not inject external tasks
into an unrelated interactive Codex chat.

On Ground:

```bash
cd /home/ai/mock_vehicle_test
./codex_ops/scripts/watch_ground_events.sh
```

On Orin1:

```bash
cd /home/jetson/mock_vehicle_test
./codex_ops/scripts/watch_agent_console.sh orin1-carrier
```

On Orin2, only after it has been separately commissioned:

```bash
cd /home/seeed/mock_vehicle_test
./codex_ops/scripts/watch_agent_console.sh orin2-mini
```

The console renders `accepted`, `progress`, `completed`, `blocked`, and failure
events as readable work updates. When `codex.enabled=true`, it also renders
observable `codex exec --json` activity such as commands, file changes, tool
calls, agent messages, peer handoffs, and results. Raw JSONL is retained only
under `codex_ops/runs/<agent>/<task-id>/`; private model reasoning is not
displayed.

Opening or closing the console does not start or stop the worker. Keep
`policy.mode=observe` for the first visible Codex gate, and never enable vehicle
hardware capabilities through this console.

For an explicitly approved Orin1 observe-only Gate B, use the audited wrapper
with local interactive sudo:

```bash
sudo ./codex_ops/scripts/enable_agent_codex_observe.sh orin1-carrier
```

The wrapper verifies the absolute native Codex binary recorded during agentd
installation, refuses an unexpected agent or policy mode, keeps a timestamped
configuration backup, and restarts only agentd. It does not change any vehicle
or hardware service.

## Git Fallback

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

## GitHub Sync And Audit

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

Use NATS for live software-work coordination and GitHub for code, contracts,
decisions, and recovery after outages. Neither channel is robot runtime control.

Runtime robot communication still belongs in MAVLink, LR24 compact packets,
ROS 2 bridges, logs, and local safety controllers.
