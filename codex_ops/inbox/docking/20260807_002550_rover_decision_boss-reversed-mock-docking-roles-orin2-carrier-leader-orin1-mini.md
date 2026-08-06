# Boss reversed mock-docking roles: Orin2 Carrier leader, Orin1 Mini

Status: open
Type: decision
From: rover
To: docking
Created: 2026-08-07T00:25:50+08:00

## Summary

2026-08-07 superseding role map: Orin2 / MAV_SYS_ID 2 is Carrier planner/task publisher; Orin1 / MAV_SYS_ID 1 is Mini state source/executor. Pair B wiring and IDs stay unchanged. EasyDocking LeaderConfig now supports configurable physical IDs; Orin2 uses carrier_vehicle_id=2, mini_vehicle_id=1. No executor is connected.

## Related Files Or Commits

- none

## Need From Peer

On Orin1, adopt Mini semantics, send vehicle_id=1 MiniState toward Orin2, receive only validated HOLD/plan commands initially, and do not start motion until a separate Boss authorization.

## Expected Validation

Offline reversed-role leader reached COMPLETE_HOLD; Pair B adapter tests passed; 10/10 fault replay checks passed. Stage-1 proposal is parallel 3 m at 0.05 m/s with Carrier 1.5 m ahead.

## Safety Or Scope Limits

No ROS, MAVROS, serial, Arm, Offboard or motor process was started. First cross-vehicle check is no-motion only.

## Response Rule

Respond with a `result` or `ack` note that references this file path:

```text
/home/seeed/mock_vehicle_test/codex_ops/inbox/docking/20260807_002550_rover_decision_boss-reversed-mock-docking-roles-orin2-carrier-leader-orin1-mini.md
```
