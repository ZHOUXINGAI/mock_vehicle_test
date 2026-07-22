# Orin2 Handoff: Install Persistent Cloud Agent

From: Orin1 / Carrier
To: Orin2 / Mini
Status: open
Created: 2026-07-23 00:27 CST

## Context

Boss approved replacing manual chat relay with a persistent Huawei ECS
coordination channel. The shared implementation is now in `codex_ops/realtime`,
`codex_ops/cloud`, and `codex_ops/deploy`.

The design uses mTLS NATS JetStream for task wake-up, ACK/retry, progress,
heartbeat, and direct peer requests. GitHub remains canonical for source code
and formal decisions. This is not a vehicle runtime link.

## Orin2 Action

1. Pull the latest `mock_vehicle_test` commit containing this handoff.
2. Read `docs/codex_cloud_coordination_runbook.md` and
   `codex_ops/state/meeting_state.md`.
3. Upgrade Orin2 to `codex-cli 0.145.0` and configure `gpt-5.6-sol` directly.
4. Remove any legacy `thirdparty`, `default_profile`, or old profile blocks from
   `/home/seeed/.codex/config.toml`; back up the file before editing.
5. Run `sudo ./codex_ops/deploy/install_agentd.sh orin2-mini`.
6. Do not enable Codex execution until Orin2 receives its mTLS certificate and
   the transport-only Gate A test passes.
7. First commissioning values must remain:

```json
"policy": { "mode": "observe" },
"codex": { "enabled": false }
```

8. After transport passes, enable read-only Codex for Gate B. Enter `code` mode
   only after a separate software-only review.

## Required Verification

```bash
codex --version
rg -n -i 'thirdparty|default_profile|\[profiles\.' /home/seeed/.codex/config.toml
systemctl status codex-agentd-orin2-mini.service --no-pager
journalctl -u codex-agentd-orin2-mini.service -n 100 --no-pager
```

Expected residue search result is empty. ACK through NATS after the cloud broker
is live; until then, commit a Git inbox ACK that references this file.

## Safety Boundary

This task must not start QGC, MAVROS, PX4, Offboard, Arduino, serial, actuator,
or motor processes and must not change flight-controller parameters.
