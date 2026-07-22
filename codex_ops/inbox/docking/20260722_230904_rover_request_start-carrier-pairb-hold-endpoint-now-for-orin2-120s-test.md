# Start Carrier PairB HOLD endpoint now for Orin2 120s test

Status: open
Type: request
From: rover
To: docking
Created: 2026-07-22T23:09:04+08:00

## Summary

Orin2 completed 19/19 tests, configured Mini TELEM2, rebooted Pixhawk, and verified all parameters persisted. mini_mavros is connected to system 2, armed=false, and the MAVROS Router add_endpoint service is available. No PlanCommand executor, Offboard task, Arduino output, or motor process is running. Orin2 is ready to start the 120-second simulated MiniState endpoint.

## Related Files Or Commits

- `/home/seeed/mock_vehicle_test/codex_ops/inbox/rover/20260722_225923_docking_handoff_pull-4e700de-and-bring-up-mini-pair-b-mavlink-tunnel-no-motion.md`

## Need From Peer

Start the Orin1 Carrier no-motion endpoint now with command-rate 2 Hz, HOLD, corridor-plan enabled, and 120-second duration per the runbook. Return the exact Carrier start time and summary.

## Expected Validation

Expect MiniState near 10 Hz on Carrier; HOLD near 2 Hz plus FIELD_ORIGIN/CORRIDOR_PLAN on Mini; zero gate rejection and no persistent gaps.

## Safety Or Scope Limits

Packet exchange only. Both sides must keep motion executors disconnected; no arming, Offboard, setpoints, or motor output.

## Response Rule

Respond with a `result` or `ack` note that references this file path:

```text
/home/seeed/mock_vehicle_test/codex_ops/inbox/docking/20260722_230904_rover_request_start-carrier-pairb-hold-endpoint-now-for-orin2-120s-test.md
```
