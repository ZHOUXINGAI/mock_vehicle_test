# PASS: Ground directly drove the visible Orin1 Codex Bridge

Time: 2026-07-23 13:16:30 CST (+08:00)

## Outcome

Ground -> NATS -> Orin1 -> official Codex app-server -> result is now working
without Boss copying task text between Codex windows.

- Ground NATS endpoint: `tls://192.168.43.13:4222`
- Agent: `orin1-carrier` on Orin1 / Carrier (`192.168.43.15`)
- Orin1 Bridge process at verification time: PID `153413`
- Orin1 systemd consumer: `codex-agentd-orin1-carrier.service` is inactive
- Bridge policy: `observe`, read-only sandbox, approvals `never`
- Cross-machine transport: mTLS NATS JetStream
- Local Codex transport: official `codex app-server --listen stdio://`
- Orin2: uncommissioned, unconfigured, and untouched

The normal VS Code Codex extension app-server remains allowed because it is not
a NATS task consumer. A file lock prevents two visible Bridges, and native
interactive/exec Codex processes are still rejected.

## Successful Task

```text
task_id: 7699453d-80b2-4bd0-813a-59b9281554c0
status: completed
thread: 019f8d64-2486-7a73-86a8-4583aba7a1ba
HEAD: da50149903724c2c6e0d9fbed68b412da76c7916
```

Ground command:

```bash
./codex_ops/scripts/coordctl.sh \
  --config codex_ops/local/boss.json send \
  --to orin1-carrier \
  --task-type analysis \
  --objective-file codex_ops/local/orin1-first-visible-bridge-task.md \
  --base-commit da50149 \
  --wait 0
```

Orin1 automatically read the required coordination documents and ran read-only
commands including:

```text
git rev-parse HEAD
git status --short
find codex_ops/inbox/docking ...
cat / wc -l on the requested coordination files
```

Final Orin1 summary:

```text
Read-only coordination verification completed as orin1-carrier.
HEAD is da50149903724c2c6e0d9fbed68b412da76c7916.
No files were modified, no peer was contacted, and no vehicle or hardware
process was started or accessed.
```

The Orin1 worktree was already dirty before this task. It contains five
modified paths and three untracked paths:

```text
 M AGENT_STATE.md
 M arduino/d24a_pixhawk_differential_pwm_bridge/d24a_pixhawk_differential_pwm_bridge.ino
 M arduino/d24a_pixhawk_pwm_bridge/d24a_pixhawk_pwm_bridge.ino
 M docs/current_rover_success_baseline_2026_06_16.md
 M scripts/configure_qgc_udp_only.sh
?? arduino/d24a_forced_standby/
?? scripts/run_mavros_mini_lr24_to_qgc_logged.sh
?? scripts/upload_d24a_bridge_safety_firmware.sh
```

Ground preserved all of these user changes and did not clean, reset, stage, or
commit them.

## Real Protocol Corrections

The live Orin1 app-server exposed three integration details that the fake
protocol test did not initially enforce:

1. `thread/start.sandbox` uses `read-only`.
2. `turn/start.sandboxPolicy.type` uses `readOnly`.
3. A saved thread may exist before a rollout is durable. The Bridge now detects
   `no rollout found for thread id`, removes only its stale local session
   pointer, and automatically falls back to `thread/start`.

Relevant commits:

```text
cb9d9db  ops: distinguish VS Code app server from native Codex
74c91c5  fix: use app server sandbox enum
eaf24db  fix: use app server sandbox enum
da50149  fix: recover stale app server threads
```

The Windows/WSL command-boundary failure is also persisted in
`codex_ops/state/meeting_state.md`: do not pass compound quoted Bash through
PowerShell to `wt.exe` or `wsl.exe ... bash -lc`; use a WSL script as the
single argument or reuse an OpenSSH ControlMaster socket.

## Verification

```text
python3 -W error::ResourceWarning -m unittest \
  tests.test_codex_realtime_coordination
Ran 22 tests ... OK

agentd.sqlite3:
task 7699453d-80b2-4bd0-813a-59b9281554c0
status completed
attempts 1

app_server_stderr.log:
0 bytes
```

No QGC, MAVROS, PX4, Offboard, Arduino, serial, MAVLink, GPIO, actuator,
motor, RC, docking, or vehicle program was started.

## Current Blocker

Orin1 read-only automatic work is ready and remains visible in the
`Orin1 Automatic Codex Bridge v4` terminal. Repository-write mode is not
enabled. Orin2 has not been touched and requires a separate Boss commissioning
decision, its own credentials, and its own visible Bridge before peer task
execution can be validated.

## Orin1 ACK

At 2026-07-23 13:20 CST, Ground sent task
`ad4afa75-a630-4a42-b296-ffe986fde697` through NATS. The existing Orin1 Codex
thread automatically resumed, read this result file, and returned:

```text
status: completed
summary: Acknowledged the successful Orin1 visible Bridge result.
successful task: 7699453d-80b2-4bd0-813a-59b9281554c0
mode: observe / read-only / approvals never / repository-write disabled
blocker: Orin2 remains uncommissioned, unconfigured, and untouched
peer rule: use structured peer_requests; Boss must not relay normal messages
```

No peer request or hardware/vehicle process was created by the ACK task.
