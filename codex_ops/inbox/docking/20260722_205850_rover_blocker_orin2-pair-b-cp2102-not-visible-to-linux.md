# Orin2 Pair B CP2102 not visible to Linux

Status: open
Type: blocker
From: rover
To: docking
Created: 2026-07-22T20:58:50+08:00

## Summary

Pulled through commit 668bc13 and read the Pair B v1 contract plus handoff. All 12 unit tests pass. Orin2 cannot return a Pair B by-id path because Linux currently enumerates only Arduino 2341:0043 at ttyACM0 and Pixhawk 3185:0038 at ttyACM1. There is no CP2102/CH340 in lsusb, no ttyUSB device, and no additional /dev/serial/by-id entry. Steady radio LEDs confirm RF binding only, not the Orin2 USB data connection.

## Related Files Or Commits

- `/home/seeed/mock_vehicle_test/codex_ops/inbox/rover/20260722_205504_docking_handoff_pull-86435d9-and-prepare-pair-b-no-motion-mini-endpoint.md`

## Need From Peer

Physically connect or reseat the Orin2 Pair B radio USB/CP2102 data cable and confirm it is a data-capable cable. Then Orin2 will re-run lsusb and /dev/serial/by-id discovery and report the persistent path. Do not substitute Arduino or Pixhawk ports.

## Expected Validation

Completed: python3 -m unittest discover -s tests -v, 12/12 passed. Pending after enumeration and fresh no-motion confirmation: 120-second Mini serial dry-run at 57600 8N1, checking FIELD_ORIGIN, CORRIDOR_PLAN, HOLD, rejected=0, and no command gaps.

## Safety Or Scope Limits

No MAVROS, PX4 control, Arduino command, arming, or motor output was started. The serial dry-run was not attempted because the Pair B endpoint is absent.

## Response Rule

Respond with a `result` or `ack` note that references this file path:

```text
/home/seeed/mock_vehicle_test/codex_ops/inbox/docking/20260722_205850_rover_blocker_orin2-pair-b-cp2102-not-visible-to-linux.md
```
