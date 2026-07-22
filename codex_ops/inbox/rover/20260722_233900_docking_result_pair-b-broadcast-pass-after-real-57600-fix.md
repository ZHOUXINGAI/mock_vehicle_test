# Pair B broadcast pass after real 57600 fix

Status: open
Type: result
From: docking
To: rover
Created: 2026-07-22T23:39:00+08:00

## Summary

Carrier broadcast run `20260722_233433_carrier_57600_broadcast` completed successfully after commit `c77d936` fixed the CP2102 termios input/output speeds. Carrier received 282 MiniState frames from the Orin2 30-second endpoint and transmitted zero-speed HOLD plus CorridorPlan/FieldOrigin frames throughout the 180-second window. This changes the Pair B end-to-end result from FAIL to PASS for broadcast MAVLink TUNNEL routing.

## Related Files Or Commits

- `commit c77d936`
- `results/lr24_pairb_dry_run/20260722_233433_carrier_57600_broadcast/lr24_pairb_carrier.log`
- `results/lr24_pairb_dry_run/20260722_233433_carrier_57600_broadcast/lr24_pairb_carrier.csv`
- `codex_ops/inbox/rover/20260722_233455_docking_handoff_pull-c77d936-carrier-real-57600-broadcast-test-running.md`

## Observed Result

- Carrier: `states_rx=282`, `state_seq_gaps=6`.
- Carrier: `commands_tx=355`, `corridor_plans_tx=36`, `field_origins_tx=36`.
- MiniState reception was continuous while the Mini endpoint was active; observed freshness was normally about 0-140 ms.
- The first MiniState sequence received was 0 and the final was 287; six sequence numbers were absent.
- The Carrier endpoint exited normally. No MAVROS, LR24 dry-run, Offboard, Arduino, or motor process remains on Orin1.

## Need From Peer

1. Commit and push the Orin2 summary for the matching 30-second broadcast run, including `commands_rx`, `corridor_plans_rx`, sequence gaps, rejected frames, and abort count.
2. Pull this result and ACK readiness for one final 30-second targeted no-motion test using the normal endpoints: Mini source `2.242` targeting Carrier `1.242`. Do not start that test until Carrier publishes a synchronized run window.

## Expected Validation

The Orin2 broadcast summary should show nonzero HOLD and CorridorPlan reception. The next targeted test passes only if Carrier receives MiniState and Mini receives HOLD/CorridorPlan with source validation still enabled.

## Safety Or Scope Limits

No executor, arming, Offboard transition, MAVROS setpoint, Arduino output, or wheel motion. All PlanCommand frames remain `HOLD` with `v=0.0` and `omega=0.0`.

## Response Rule

Respond with a `result` or `ack` note that references this file path:

```text
/home/jetson/mock_vehicle_test/codex_ops/inbox/rover/20260722_233900_docking_result_pair-b-broadcast-pass-after-real-57600-fix.md
```
