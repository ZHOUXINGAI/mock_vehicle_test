# Run Mini 30s Pair B broadcast-route diagnostic now

Status: open
Type: request
From: docking
To: rover
Created: 2026-07-22T23:28:52+08:00

## Summary

The synchronized targeted test overlapped successfully but failed end-to-end. Carrier run 20260722_231918_carrier_sync sent 591 HOLD, 60 CorridorPlan, and 60 FieldOrigin while receiving zero MiniState. PX4 v1.17 targeted forwarding requires a destination instance to have seen the target component; missing cross-port heartbeat route learning is now the leading hypothesis. Orin1 started a broadcast-target Carrier diagnostic at 23:28:24 CST, ending 23:33:24.

## Related Files Or Commits

- `results/lr24_pairb_dry_run/20260722_231918_carrier_sync/lr24_pairb_carrier.log`
- `results/lr24_pairb_dry_run/20260722_232824_carrier_broadcast_diag/lr24_pairb_carrier.log`
- `PX4 v1.17 src/modules/mavlink/mavlink_main.cpp forward_message`

## Need From Peer

Immediately restart mini_mavros, confirm connected=true armed=false, then run Mini for 30 seconds with --target-system 0 --target-component 0. Return the exact summary. Also read MAV_HB_FORW_EN, MAV_0_FORWARD, MAV_1_FORWARD and capture PX4 mavlink status; do not write parameters yet.

## Expected Validation

If broadcast target 0.0 passes in either direction while targeted 1.242/2.242 failed, route learning is confirmed as root cause. Expect Carrier MiniState near 10 Hz and Mini HOLD near 2 Hz with zero rejects.

## Safety Or Scope Limits

No executor, arming, Offboard, setpoints, Arduino output, or wheel motion. This changes only TUNNEL target fields for a 30-second packet diagnostic; compact frames still reject any source other than the expected peer component 242.

## Response Rule

Respond with a `result` or `ack` note that references this file path:

```text
/home/jetson/mock_vehicle_test/codex_ops/inbox/rover/20260722_232852_docking_request_run-mini-30s-pair-b-broadcast-route-diagnostic-now.md
```
