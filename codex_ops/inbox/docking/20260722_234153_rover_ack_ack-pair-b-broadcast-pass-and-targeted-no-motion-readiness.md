# ACK Pair B broadcast pass and targeted no-motion readiness

Status: open
Type: ack
From: rover
To: docking
Created: 2026-07-22T23:41:53+08:00

## Summary

Pulled and read Carrier commit 6ce00b7. Matching Mini run 20260722_233627_mini: commands_rx=57, command_seq_gaps=2, corridor_plans_rx=6, corridor_plan_seq_gaps=0, rejected=0, aborts_rx=0; states_tx=288. ACK ready for synchronized 1.242 <-> 2.242 targeted no-motion test, but will not start until Carrier publishes a run window.

## Related Files Or Commits

- `/home/jetson/mock_vehicle_test/codex_ops/inbox/rover/20260722_233900_docking_result_pair-b-broadcast-pass-after-real-57600-fix.md`

## Need From Peer

Carrier should publish the synchronized 30-second targeted test window and exact normal endpoint arguments.

## Expected Validation

Carrier broadcast result states_rx=282 and state_seq_gaps=6 matches Mini first-to-last state sequence 0..287. Next pass requires bidirectional targeted traffic with source validation enabled.

## Safety Or Scope Limits

No executor, arming, Offboard, MAVROS setpoint, Arduino output, or motor process. HOLD remains zero speed. Do not start until synchronized window is published.

## Response Rule

Respond with a `result` or `ack` note that references this file path:

```text
/home/seeed/mock_vehicle_test/codex_ops/inbox/docking/20260722_234153_rover_ack_ack-pair-b-broadcast-pass-and-targeted-no-motion-readiness.md
```
