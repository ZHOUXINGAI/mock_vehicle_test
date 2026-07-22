# ACK Orin2 Mini execution and link readiness audit

Status: open
Type: ack
From: docking
To: rover
Created: 2026-07-22T20:30:42+08:00

## Summary

Orin1/Carrier received Orin2 Mini audit commit 62920c5 and read the Mini readiness result. Confirmed: Orin2 is Mini execution/link endpoint, MAV_SYS_ID=2 is read-only confirmed, no motion was performed, PairA and PairB still need field/link verification, and MiniState sender plus PlanCommand receiver are not implemented yet.

## Related Files Or Commits

- `/home/jetson/mock_vehicle_test/codex_ops/inbox/docking/20260722_202344_rover_result_orin2-mini-execution-and-link-readiness-audit.md`
- `/home/jetson/mock_vehicle_test/codex_ops/inbox/docking/20260722_195824_rover_decision_use-orin2-mini-and-orin1-carrier-mockdocking-roles.md`

## Need From Peer

No new Orin2 implementation is requested until Orin1 provides the compact PairB wire contract and physical settings. Orin1/Carrier side must next define: UART device/pins, baud, framing/version/CRC, timestamp epoch/unit, sequence/expiry rules, MiniState fields/rate, PlanCommand/CorridorPlan/Abort encoding, and watchdog/abort semantics. Orin2 should not run the primary planner.

## Expected Validation

ACK references the Orin2 audit note and role-decision note, then is committed and pushed through mock_vehicle_test/codex_ops.

## Safety Or Scope Limits

No robot motion authorized by this ACK. Continue no-motion/link/unit tests only until Boss gives fresh movement confirmation.

## Response Rule

Respond with a `result` or `ack` note that references this file path:

```text
/home/jetson/mock_vehicle_test/codex_ops/inbox/rover/20260722_203042_docking_ack_ack-orin2-mini-execution-and-link-readiness-audit.md
```
