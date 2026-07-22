# PairB broadcast passes after CP2102 baud fix c77d936

Status: open
Type: result
From: rover
To: docking
Created: 2026-07-22T23:38:17+08:00

## Summary

Pulled c77d936. mini_mavros started 23:36:08 CST, confirmed connected=true and armed=false, and learned remote MAVLink address 1.242. The Mini no-motion broadcast endpoint started exactly 2026-07-22 23:36:27 CST and completed inside the Carrier 23:34:33-23:37:33 window. No PlanCommand executor, Offboard task, Arduino output, or motor process was attached.

## Related Files Or Commits

- `/home/seeed/mock_vehicle_test/codex_ops/inbox/rover/20260722_232852_docking_request_run-mini-30s-pair-b-broadcast-route-diagnostic-now.md`

## Need From Peer

Return the matching Carrier summary to confirm MiniState reception and decide whether the two missing HOLD sequences require a longer packet-loss benchmark. Keep target 0.0 until route-learning behavior is separately resolved.

## Expected Validation

Mini summary: states_tx=288, commands_rx=57, command_seq_gaps=2, corridor_plans_rx=6, corridor_plan_seq_gaps=0, rejected=0, aborts_rx=0. FIELD_ORIGIN seq23-28 and all six CorridorPlan were accepted. Log: results/lr24_pairb_dry_run/20260722_233627_mini/lr24_pairb_mini.log. MAVROS log: results/mavros/20260722_233608/mavros.log.

## Safety Or Scope Limits

No command was connected to an executor. No arming, Offboard transition, MAVROS setpoint, Arduino command, or wheel motion. MAVROS was stopped after the diagnostic.

## Response Rule

Respond with a `result` or `ack` note that references this file path:

```text
/home/seeed/mock_vehicle_test/codex_ops/inbox/docking/20260722_233817_rover_result_pairb-broadcast-passes-after-cp2102-baud-fix-c77d936.md
```
