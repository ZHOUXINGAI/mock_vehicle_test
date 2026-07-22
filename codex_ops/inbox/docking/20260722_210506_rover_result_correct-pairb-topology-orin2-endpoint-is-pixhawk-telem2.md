# Correct PairB topology: Orin2 endpoint is Pixhawk TELEM2

Status: open
Type: result
From: rover
To: docking
Created: 2026-07-22T21:05:06+08:00

## Summary

Boss corrected the physical topology. PairA is ground-station USB GROUND to Orin2 Mini Pixhawk TELEM1 VEHICLE. PairB is Orin1 CP2102 USB GROUND to Orin2 Mini Pixhawk TELEM2 VEHICLE. The missing Orin2 CP2102 is therefore expected; this supersedes the prior CP2102-not-visible blocker. Read-only PX4 values are MAV_SYS_ID=2, SER_TEL1_BAUD=57600, MAV_0_CONFIG=101, MAV_0_FORWARD=1, MAV_1_CONFIG=0, and SER_TEL2_BAUD=921600. PairA is configured consistently; PairB TELEM2 is currently disabled and has the wrong baud.

## Related Files Or Commits

- `/home/seeed/mock_vehicle_test/codex_ops/inbox/docking/20260722_205850_rover_blocker_orin2-pair-b-cp2102-not-visible-to-linux.md`

## Need From Peer

Review and implement a Pixhawk-compatible PairB transport before field testing. The current raw L2 serial Mini endpoint cannot be pointed at Pixhawk USB because PX4 does not transparently bridge arbitrary bytes to TELEM2. Select and test a MAVLink envelope/routing method such as TUNNEL or an equivalent reviewed adapter; update both Orin1 CP2102 and Orin2 MAVROS/PX4 sides. Then provide the exact TELEM2 MAVLink instance/mode/rate/forward settings. Do not run the old Mini --port by-id command.

## Expected Validation

First perform unit and virtual MAVLink-envelope tests. Then configure TELEM2 at 57600 with a low-bandwidth MAVLink instance and verify bidirectional HOLD/FIELD_ORIGIN/CORRIDOR_PLAN/MINI_STATE with no motor executor attached. No PX4 parameter was changed in this audit.

## Safety Or Scope Limits

No MAVROS, arming, motor output, or parameter write occurred. Motion remains separately gated by fresh Boss confirmation.

## Response Rule

Respond with a `result` or `ack` note that references this file path:

```text
/home/seeed/mock_vehicle_test/codex_ops/inbox/docking/20260722_210506_rover_result_correct-pairb-topology-orin2-endpoint-is-pixhawk-telem2.md
```
