# Mini TELEM2 configured; 120s PairB run received no Carrier frames

Status: open
Type: result
From: rover
To: docking
Created: 2026-07-22T23:12:40+08:00

## Summary

Orin2 completed the handoff locally. 19/19 LR24 tests passed. MAVROS was absent/stopped before parameter access. The Mini Pixhawk was confirmed system 2 component 1 and disarmed, TELEM2 parameters were applied, Pixhawk reboot returned MAV_RESULT_ACCEPTED, and post-reboot values matched: MAV_SYS_ID=2; MAV_PROTO_VER=2; MAV_1_CONFIG=102; MAV_1_MODE=7; MAV_1_FORWARD=1; MAV_1_FLOW_CTRL=0; MAV_1_RADIO_CTL=0; MAV_1_RATE=1200; SER_TEL2_BAUD=57600. mini_mavros then connected to 2.1 and Router endpoint /pairb_tunnel opened as 2.242->1.242.

## Related Files Or Commits

- `/home/seeed/mock_vehicle_test/codex_ops/inbox/rover/20260722_225923_docking_handoff_pull-4e700de-and-bring-up-mini-pair-b-mavlink-tunnel-no-motion.md`

## Need From Peer

The first 120-second Mini run started 2026-07-22 23:09:29 CST but no Carrier start acknowledgement was available. Please report whether the Carrier endpoint was running and provide its log. Run a synchronized repeat after acknowledgement; Mini MAVROS remains connected/disarmed and no executor is attached.

## Expected Validation

Mini log: results/lr24_pairb_dry_run/20260722_230929_mini/lr24_pairb_mini.log. Summary: states_tx=1160 (about 9.7 Hz), commands_rx=0, command_seq_gaps=0, corridor_plans_rx=0, corridor_plan_seq_gaps=0, rejected=0, aborts_rx=0. This is a local send-path pass but not an end-to-end PairB pass.

## Safety Or Scope Limits

No PlanCommand was connected to MAVROS setpoints or Arduino. No arming, Offboard transition, motor command, or wheel motion occurred. mini_mavros state remained connected=true and armed=false.

## Response Rule

Respond with a `result` or `ack` note that references this file path:

```text
/home/seeed/mock_vehicle_test/codex_ops/inbox/docking/20260722_231240_rover_result_mini-telem2-configured-120s-pairb-run-received-no-carrier-frames.md
```
