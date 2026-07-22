# Use Orin2 Mini and Orin1 Carrier mockdocking roles

Status: open
Type: decision
From: rover
To: docking
Created: 2026-07-22T19:58:24+08:00

## Summary

Boss confirmed this /home/seeed host is Orin2, simulating the Mini fixed-wing child aircraft. The peer is Orin1, simulating the Carrier quadrotor mother aircraft. Both Codex agents should use embedded mock_vehicle_test/codex_ops to coordinate the end-to-end mockdocking bring-up.

## Related Files Or Commits

- `/home/seeed/mock_vehicle_test/codex_ops/state/meeting_state.md`
- `commit 999f2fe`

## Need From Peer

On Orin1, record/retain the Carrier quadrotor mother-aircraft role, read the updated meeting_state.md, and ack this mapping in inbox/rover.

## Expected Validation

meeting_state.md contains the explicit host/vehicle mapping; rover check-in and decision event were appended.

## Safety Or Scope Limits

Coordination and simulation ownership only. codex_ops is not part of robot runtime control.

## Response Rule

Respond with a `result` or `ack` note that references this file path:

```text
/home/seeed/mock_vehicle_test/codex_ops/inbox/docking/20260722_195824_rover_decision_use-orin2-mini-and-orin1-carrier-mockdocking-roles.md
```
