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

## Visible Interactive Codex (Recommended)

Run one normal interactive Codex on each commissioned computer. This is the
default operator-facing mode: the user sees the native Codex UI, commands,
edits, and replies directly.

On Orin1:

```bash
cd /home/jetson/mock_vehicle_test
./codex_ops/scripts/launch_visible_codex.sh orin1-carrier
```

On Orin2, only after it has been separately commissioned:

```bash
cd /home/seeed/mock_vehicle_test
./codex_ops/scripts/launch_visible_codex.sh orin2-mini
```

Keep agentd `codex.enabled=false` in this mode. The launcher refuses to create a
second Codex when background execution is enabled. NATS may report online
status and notify the operator that Git inbox work exists, but it does not and
cannot inject a prompt into the running interactive Codex. The operator tells
the visible Codex to pull Git and process its inbox.

Ground can retain the readable notification console:

```bash
cd /home/ai/mock_vehicle_test
./codex_ops/scripts/watch_ground_events.sh
```

Automated `codex exec` through agentd is experimental and is not the default
long-running setup. Do not enable it concurrently with the interactive Codex.
No mode grants vehicle, serial, MAVLink, actuator, motor, or sudo capability.

## Automatic Visible App-Server Bridge (Pilot)

For direct Ground-to-Orin work without user message relay, the pilot Bridge
uses the official local `codex app-server` stdio protocol. NATS remains the
cross-machine transport; app-server is never exposed on a network port.

Only one consumer may run. Stop the installed coordination service with local
interactive sudo, then start the Bridge in a visible terminal:

```bash
sudo systemctl stop codex-agentd-orin1-carrier.service
cd /home/jetson/mock_vehicle_test
./codex_ops/scripts/launch_visible_app_bridge.sh orin1-carrier
```

The launcher refuses to start while the system service is active, validates
the native Codex binary and observe policy, loads the user's login environment,
and creates a private local Bridge config. It does not alter the installed
`/etc` config or start any vehicle service. Do not run the native interactive
Codex against the Bridge thread at the same time.

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
