# Pull 86435d9 and prepare Pair B no-motion Mini endpoint

Status: open
Type: handoff
From: docking
To: rover
Created: 2026-07-22T20:55:04+08:00

## Summary

Orin1 finalized Pair B v1 at commit 86435d9. Boss reports Pair A/B/C radios bound with steady LEDs. Pair B is ADDR 1102, full duplex, low 2.4KB/s, 500mW, 57600 8N1. Shared field frame is ENU with FIELD_ORIGIN/origin_id. Protocol now includes CRC, seq wrap checks, relative TTL, command watchdog, local limits, and latched ABORT. Virtual serial mixed traffic passed with zero sequence gaps after startup and zero Mini gate rejections.

## Related Files Or Commits

- `/home/jetson/mock_vehicle_test/docs/lr24_pairb_wire_contract_v1.md`

## Need From Peer

Pull origin/master; read docs/lr24_pairb_wire_contract_v1.md; identify Orin2 Pair B /dev/serial/by-id path; run python3 -m unittest discover -s tests -v. Then report the path and test result. Do not start the real Pair B endpoint until Boss freshly confirms motors disabled or wheels lifted. After confirmation, run: CONFIRM_NO_MOTION=true ./scripts/run_lr24_pairb_dry_run.sh mini --port <B-MINI-BY-ID> --duration-sec 120 --state-rate-hz 10 --simulate-orbit --radius-m 4.5 --speed-mps 0.9 --origin-id 1

## Expected Validation

Unit tests must pass. Mini endpoint is serial-only and must show FIELD_ORIGIN accepted, CORRIDOR_PLAN accepted, HOLD accepted, rejected=0, no command gaps. Return log summary in inbox/docking.

## Safety Or Scope Limits

No MAVROS/PX4/Arduino/motor output is connected by this dry-run. Do not arm or move. Real movement remains blocked pending a separate exact Boss confirmation and stop-path checks.

## Response Rule

Respond with a `result` or `ack` note that references this file path:

```text
/home/jetson/mock_vehicle_test/codex_ops/inbox/rover/20260722_205504_docking_handoff_pull-86435d9-and-prepare-pair-b-no-motion-mini-endpoint.md
```
