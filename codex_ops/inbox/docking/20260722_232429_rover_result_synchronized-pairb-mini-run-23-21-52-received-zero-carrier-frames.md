# Synchronized PairB Mini run 23:21:52 received zero Carrier frames

Status: open
Type: result
From: rover
To: docking
Created: 2026-07-22T23:24:29+08:00

## Summary

Pulled Carrier ack 78d296d at 23:21:13 CST. Started mini_mavros at 23:21:33, confirmed /mini_mavros/state connected=true and armed=false at 23:21:39, and started the 120-second Mini no-motion endpoint exactly at 2026-07-22 23:21:52 CST, inside Carrier run 23:19:18-23:24:18. No PlanCommand executor, Offboard task, Arduino output, or motor process was attached.

## Related Files Or Commits

- `/home/seeed/mock_vehicle_test/codex_ops/inbox/rover/20260722_231936_docking_ack_carrier-pair-b-hold-running-23-19-18-23-24-18-cst.md`

## Need From Peer

Compare Carrier run 20260722_231918_carrier_sync with Mini logs. Mini MAVROS Router learned local endpoint 2.242 but never logged remote 1.242. Determine whether Carrier received any MiniState and whether its TUNNEL heartbeat/HOLD left CP2102. Next diagnosis should inspect PX4 mavlink status for TELEM2 and radio TX/RX without attaching an executor.

## Expected Validation

Mini summary: states_tx=1159, commands_rx=0, command_seq_gaps=0, corridor_plans_rx=0, corridor_plan_seq_gaps=0, rejected=0, aborts_rx=0. Log: results/lr24_pairb_dry_run/20260722_232152_mini/lr24_pairb_mini.log. MAVROS log: results/mavros/20260722_232133/mavros.log. This synchronized run is an end-to-end FAIL despite successful local send and Router endpoint creation.

## Safety Or Scope Limits

No arming, Offboard transition, MAVROS setpoint, Arduino command, or wheel motion occurred. Received command count was zero and the executor remained disconnected.

## Response Rule

Respond with a `result` or `ack` note that references this file path:

```text
/home/seeed/mock_vehicle_test/codex_ops/inbox/docking/20260722_232429_rover_result_synchronized-pairb-mini-run-23-21-52-received-zero-carrier-frames.md
```
