# Pull 4e700de and bring up Mini Pair B MAVLink tunnel no-motion

Status: open
Type: handoff
From: docking
To: rover
Created: 2026-07-22T22:59:23+08:00

## Summary

Orin1 implemented the Pixhawk-compatible Pair B adapter: one compact L2 frame per MAVLink 2 TUNNEL message, component routing 1.242 <-> 2.242. Carrier uses Pair B CP2102 directly; Mini uses a dynamic MAVROS Router endpoint over Pixhawk USB. The implementation includes source/target checks, MAVROS QoS compatibility, heartbeat route learning, guarded TELEM2 parameter writes, and a full runbook.

## Related Files Or Commits

- `commit 4e700de`
- `docs/lr24_pairb_mavlink_tunnel_runbook.md`
- `scripts/configure_px4_pairb_telem2.sh`
- `src/lr24_mavlink_tunnel.py`

## Need From Peer

On Orin2: pull 4e700de; run the 19 LR24 tests; stop MAVROS; run configure_px4_pairb_telem2.sh check, then apply only with all three confirmations; controlled-reboot Mini Pixhawk and check again; restart Mini MAVROS; start the Mini no-motion endpoint. Report all parameter values and the 120-second Mini summary. Do not attach received commands to the executor.

## Expected Validation

First reproduce 19 passing tests. Hardware pass then requires Orin1 receiving simulated MINI_STATE near 10 Hz and Orin2 accepting zero-speed HOLD near 2 Hz plus FIELD_ORIGIN/CORRIDOR_PLAN, with no persistent sequence gaps, no gate rejects, and Pair A/C QGC telemetry still available.

## Safety Or Scope Limits

No arming, Offboard transition, MAVROS setpoint, Arduino output, or wheel motion is authorized. Both vehicles disarmed; wheels lifted or motor power disconnected; RC stop ready. The config script identifies Mini as system 2 component 1 and never reboots or sends setpoints.

## Response Rule

Respond with a `result` or `ack` note that references this file path:

```text
/home/jetson/mock_vehicle_test/codex_ops/inbox/rover/20260722_225923_docking_handoff_pull-4e700de-and-bring-up-mini-pair-b-mavlink-tunnel-no-motion.md
```
