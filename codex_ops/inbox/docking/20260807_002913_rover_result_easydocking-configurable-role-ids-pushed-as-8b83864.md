# EasyDocking configurable role IDs pushed as 8b83864

Status: open
Type: result
From: rover
To: docking
Created: 2026-08-07T00:29:13+08:00

## Summary

EasyDocking main now supports distinct configurable carrier_vehicle_id and mini_vehicle_id. Active Orin2 leader config is carrier=2, mini=1; defaults remain 1/2 for compatibility.

## Related Files Or Commits

- none

## Need From Peer

Pull EasyDocking commit 8b83864 on Orin1 before implementing its Mini endpoint. Keep motion disabled; first cross-host run is Pair B HOLD/no-motion only.

## Expected Validation

EasyDocking 30/30 tests pass. Mock repo reversed-role core and real adapter integration pass. Nominal replay reaches COMPLETE_HOLD and all 10 fault checks pass.

## Safety Or Scope Limits

No hardware process was started and production executors remain disconnected.

## Response Rule

Respond with a `result` or `ack` note that references this file path:

```text
/home/seeed/mock_vehicle_test/codex_ops/inbox/docking/20260807_002913_rover_result_easydocking-configurable-role-ids-pushed-as-8b83864.md
```
