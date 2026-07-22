# Orin2 Mini execution and link readiness audit

Status: open
Type: result
From: rover
To: docking
Created: 2026-07-22T20:23:44+08:00

## Summary

我是 Orin2 / Mini Codex，负责 Mini 小车执行层和 MiniState/PlanCommand 链路，不负责 easydocking planner。 Mini hardware baseline exists: PX4 v1.17 differential rover, MAVROS/offboard scripts, Arduino D24A differential bridge, and prior forward/left/right motor-only checks passed. Direct read-only MAVLink query confirmed MAV_SYS_ID=2, SYS_AUTOSTART=50000, CA_AIRFRAME=6, and left/right actuator functions 101/102. No control runtime is currently active. PairA QGC is not yet field-verified. PairB is not verified: no dedicated USB LR24 endpoint enumerates, candidate onboard UART is not identified, and baud/pins/device path are undocumented. No executable MiniState sender or validated PlanCommand receiver exists in this clone.

## Related Files Or Commits

- none

## Need From Peer

Orin1 must provide/finalize the compact wire contract and PairB physical settings: UART device/pins, baud, framing/version/CRC, timestamp epoch/unit, sequence rules, valid_until interpretation, MiniState rate/fields, PlanCommand/CorridorPlan/Abort encoding. Then Orin2 can implement sender, receiver, watchdog, body-frame primitive adapter, one-orbit gate, and logs without changing easydocking planner ownership.

## Expected Validation

No-motion first: identify PairB UART without transmitting; verify PairA from ground QGC; read MAVROS state while disarmed; unit-test packet golden vectors, duplicate/out-of-order/stale/expired commands, Abort priority, watchdog stop, and local loopback transport. Wheels-lifted only after fresh Boss confirmation: low-speed v/omega forward-turn-stop, expired-command stop, Abort, PairB disconnect stop, RC takeover, and QGC disarm.

## Safety Or Scope Limits

No arming or motor movement occurred during this audit. Every real movement still requires a fresh exact Boss confirmation plus RC/QGC/physical stop readiness, cable clearance, low initial limits, and logging.

## Response Rule

Respond with a `result` or `ack` note that references this file path:

```text
/home/seeed/mock_vehicle_test/codex_ops/inbox/docking/20260722_202344_rover_result_orin2-mini-execution-and-link-readiness-audit.md
```
