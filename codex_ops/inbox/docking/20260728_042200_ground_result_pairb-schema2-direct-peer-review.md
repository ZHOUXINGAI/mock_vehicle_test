# Ground Result: Pair B Schema 2 and Direct Peer Review

Date: 2026-07-28 CST

## Implemented commit

Orin1/Carrier implemented and pushed:

```text
a9aded8fb76daeea72df5858ce0a3a7cc1bba59e
feat: carry docking plan timing over Pair B
```

The commit contains exactly six files:

- `src/lr24_compact_protocol.py`
- `src/lr24_command_guard.py`
- `tests/test_lr24_compact_protocol.py`
- `scripts/lr24_pairb_dry_run.py`
- `scripts/lr24_link_benchmark.py`
- `docs/lr24_pairb_wire_contract_v1.md`

Existing Orin1 hardware/configuration worktree changes were not staged,
committed, reset, or otherwise modified.

## Wire result

- Global LR24 compact frame envelope remains `VERSION=1`.
- `CORRIDOR_PLAN` payload schema is now `2`.
- Compact payload is `59 bytes`; complete compact frame is `66 bytes`.
- MAVLink TUNNEL link-budget size is `83 bytes`.
- Added wire fields:
  - `plan_schema_version` (`uint8`)
  - `required_validity_ms` (`uint32`)
  - `post_tangent_reserve_ms` (`uint16`)
  - `terminal_completion_budget_ms` (`uint16`)
  - `completion_hold_ms` (`uint16`)
  - `plan_timing_guard_ms` (`uint16`)
  - `command_ttl_ms` (`uint16`)
  - `local_command_watchdog_ms` (`uint16`)
- Sender rejects out-of-range safety timing fields instead of clamping,
  wrapping, or silently extending an immutable plan.
- Mini independently rejects unsupported schema, reserve mismatch,
  required-validity mismatch, sender validity below required, and invalid zero
  timing values.

## Verification

Ground and Orin1 independently ran:

- compact protocol and Mini gate: `17/17`
- MAVLink TUNNEL: `7/7`
- Pair B run analysis: `11/11`
- Codex coordination regression: `36/36`

Orin2 independently reviewed the commit and ran the first three groups from
its clean checkout: `35/35` passed. Its separate dirty rover checkout remained
at `a4f39f9` with the same dirty paths before and after review.

## Direct Orin1 -> Orin2 -> Orin1 lineage

Ground dispatched only the root instruction that Orin1 should request peer
review. The review content and return verdict were exchanged as structured
peer requests:

```text
Ground -> Orin1 root:
fcaff474-1eab-42c2-922e-853a09cb4110

Orin1 -> Orin2 review:
59ce579e-59ba-4249-8b31-c9c2ff0e5e98

Orin2 -> Orin1 verdict:
0cf7c94a-adb9-4c54-bb64-1c550382372c
```

Orin2 verdict: commit `a9aded8` has no blocking review issue. Orin1
acknowledged and accepted the verdict, emitted no further peer request, and
started no hardware process.

## Transport boundary

Pair B carries only compact runtime state, plans, commands, origins, aborts,
and link probes. It never carries files, Git repositories or patches, bulk
logs, Codex traffic, or NATS. GitHub/SSH remains the code/file path; NATS is
Codex coordination only and never part of the vehicle-control loop.

## Remaining gates before motion

1. The dirty Orin2 rover checkout has not been merged to `a9aded8`; do not
   force-update or overwrite it. Runtime deployment needs a scoped integration
   plan that preserves its local rover work.
2. Live Carrier/Mini followers and local primitive executors are still not
   connected to the offline leader.
3. Exact `target_front_gap_m` closed-loop regulation is still not implemented.
4. A zero-motion end-to-end Pair B trial must pass before any motion trial.
5. No motion or hardware stage is authorized by this result.
