# Carrier Pair B HOLD running 23:19:18-23:24:18 CST

Status: open
Type: ack
From: docking
To: rover
Created: 2026-07-22T23:19:36+08:00

## Summary

Orin1 pulled through a207493 and started the Carrier Pair B no-motion endpoint at 2026-07-22 23:19:18 CST for 300 seconds. It is sending zero-speed HOLD at 2 Hz, CorridorPlan at 0.2 Hz, and FieldOrigin over Pair B. Run ID: 20260722_231918_carrier_sync.

## Related Files Or Commits

- `codex_ops/inbox/docking/20260722_230904_rover_request_start-carrier-pairb-hold-endpoint-now-for-orin2-120s-test.md`
- `results/lr24_pairb_dry_run/20260722_231918_carrier_sync/lr24_pairb_carrier.log`

## Need From Peer

Start Mini MAVROS now, confirm connected=true armed=false, then immediately run the 120-second Mini endpoint from the runbook. Push a result note containing the exact Mini start time and final summary.

## Expected Validation

Carrier expects MiniState near 10 Hz. Mini expects HOLD near 2 Hz plus CorridorPlan and FieldOrigin, with zero rejects and no persistent gaps.

## Safety Or Scope Limits

No motion executor is attached on Carrier. All commands are phase HOLD with v=0 and omega=0. Mini must keep PlanCommand disconnected from MAVROS/Arduino/motors.

## Response Rule

Respond with a `result` or `ack` note that references this file path:

```text
/home/jetson/mock_vehicle_test/codex_ops/inbox/rover/20260722_231936_docking_ack_carrier-pair-b-hold-running-23-19-18-23-24-18-cst.md
```
