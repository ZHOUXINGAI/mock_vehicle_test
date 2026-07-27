# Ground result: Orin2 commissioned and direct Codex round trip passed

Date: 2026-07-27 22:31 CST

## Result

Both requested software-coordination milestones passed:

1. Ground Codex continued the existing Orin2 VS Code Codex conversation and
   received a visible structured reply.
2. Orin1 Codex sent a structured peer task directly to Orin2 Codex; Orin2 sent
   a structured return peer task directly to Orin1; Orin1 completed the return
   leg without Ground relaying either peer message.

No QGC, MAVROS, PX4, Offboard, Arduino, serial, MAVLink, LR24, actuator, motor,
RC, docking runtime, or other vehicle/hardware process was started.

## Identities And Network

- Ground Wi-Fi IPv4: `192.168.1.35`
- NATS endpoint: `tls://192.168.1.35:4222`
- Orin1 / Carrier IPv4: `192.168.1.34`
- Orin2 / Mini IPv4: `192.168.1.36`
- Orin1 Codex session:
  `019e3b9d-8417-76d2-bf88-4ec59aba48c4`
- Orin2 Codex session:
  `019f853a-eed1-71a0-80ce-176ac16e3150`

Orin2 uses its existing dirty vehicle worktree at
`/home/seeed/mock_vehicle_test` as the Codex task workspace. Its uncommitted
vehicle work was preserved. The coordination runtime uses the separate clean
checkout `/home/seeed/codex_ground_bridge`.

## Orin2 Commissioning

- Service: `codex-agentd-orin2-mini.service`
- State: `enabled / active`
- Policy: `observe`
- Sandbox behavior: read-only
- Backend: local Codex app-server over stdio
- mTLS identity: `CN=orin2-mini`
- Client private-key mode: `0600`
- Temporary certificate staging copies: removed after installation
- CA private key: remained on Ground and was not transferred
- Codex CLI: `/home/seeed/.local/bin/codex`, `codex-cli 0.144.6`
- Authentication: existing ChatGPT login

Orin2's login shell uses the localhost proxy at `127.0.0.1:7897`. A systemd
drop-in now supplies the same proxy variables to the Bridge. The incompatible
old `models_cache.json` was preserved as a timestamped backup.

A scoped sudo helper permits the `seeed` user to run only:

```text
/usr/local/sbin/codex-agentd-orin2-control status
/usr/local/sbin/codex-agentd-orin2-control restart
```

It does not grant passwordless access to arbitrary commands or vehicle
services.

## Gate A: transport-only

Task:

```text
37c728f9-e5cc-4dfd-af98-7510908c69d6
```

Observed lineage:

```text
Ground dispatched
Orin2 accepted
Orin2 completed: app-server bridge transport test completed with Codex disabled
```

The completion details explicitly reported that no Codex or hardware process
was started.

## Ground Codex To Orin2 Codex

The first attempt, task `79a002f4-c0b7-44c7-b06c-adc0eb1ca0d5`, was recorded
as failed because the systemd service did not inherit Orin2's localhost proxy.
It was not counted as a pass.

After the proxy and cache repair, task
`ec3e1329-dbbc-4ac7-bda5-e7557bcd5e2a` passed. Orin2 continued session
`019f853a-eed1-71a0-80ce-176ac16e3150`, ran only:

```bash
git rev-parse --short HEAD
```

and returned:

```text
Orin2 Codex 已收到 Ground Codex 指令
current short commit: a4f39f9
existing Orin2 conversation context is active
role: fixed-wing Mini child aircraft
Orin1: quadrotor Carrier, Docking planner and command publisher
current mode: read-only observe; no vehicle, flight-control, radio or hardware program started
waiting for Orin1 Codex direct handshake
```

## Direct Orin1 Codex To Orin2 Codex Round Trip

Root task:

```text
c10d0043-22ef-4651-8343-aa0e78fe649f
```

JetStream lineage:

```text
Ground -> Orin1 trigger:
  c10d0043-22ef-4651-8343-aa0e78fe649f

Orin1 -> Orin2 direct peer task:
  1a472a27-844f-46b7-ab18-3325baeedaca

Orin2 -> Orin1 direct return peer task:
  a5aa96a1-7de4-4f70-aab8-efce42c87a7d
```

Results:

- Orin1: `Orin1 Codex 已直接发起对 Orin2 Codex 的握手`
- Orin2: `Orin2 Codex 已收到 Orin1 Codex 直接握手`
- Orin1: `Orin1 Codex 已收到 Orin2 Codex 直接回执`
- The final Orin1 result contained `peer_requests=[]`, proving that the test
  terminated without a task loop.
- Both peers confirmed that Carrier publishes the docking corridor and
  commands.

Ground observed the common `root_task_id` in JetStream but did not copy either
peer message.

## End Goal And Next Step

The end goal is physical docking between:

- Orin1: quadrotor Carrier / mother aircraft;
- Orin2: fixed-wing Mini / child aircraft.

Orin1 owns `easydocking`, the Docking planner, and active command publication.
Carrier publishes the docking corridor and commands to both its own execution
side and Orin2.

The next safe task is a read-only Orin1 Codex review of
`/home/jetson/easydocking` that produces the exact state, corridor and command
interface needed by both aircraft, reconciled against the existing three-radio
design. Aircraft motion and docking remain separately authorized later stages.
