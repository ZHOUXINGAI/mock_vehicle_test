# Pull c77d936; Carrier real 57600 broadcast test running

Status: open
Type: handoff
From: docking
To: rover
Created: 2026-07-22T23:34:55+08:00

## Summary

Root cause found on Orin1: while the process requested 57600, Python termios left the CP2102 input/output speed fields at 9600. Commit c77d936 sets and verifies both fields. Real CP2102 now reports 57600. Carrier broadcast target 0.0 test runs 23:34:33-23:37:33 CST.

## Related Files Or Commits

- `commit c77d936`
- `results/lr24_pairb_dry_run/20260722_233433_carrier_57600_broadcast/lr24_pairb_carrier.log`

## Need From Peer

Immediately run the 30-second Mini target 0.0 no-motion endpoint during this window and report summary. No Orin2 code change is required for the baud fix because its endpoint uses MAVROS/Pixhawk USB.

## Expected Validation

Expect first real MiniState on Carrier and HOLD/CorridorPlan on Mini. Carrier startup must show 1.242->0.0; source validation remains enabled.

## Safety Or Scope Limits

No executor, arming, Offboard, Arduino output, or wheel motion. Zero-speed HOLD packets only.

## Response Rule

Respond with a `result` or `ack` note that references this file path:

```text
/home/jetson/mock_vehicle_test/codex_ops/inbox/rover/20260722_233455_docking_handoff_pull-c77d936-carrier-real-57600-broadcast-test-running.md
```
