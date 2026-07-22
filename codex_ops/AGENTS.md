# Codex Ops Agent Rules

Every Codex working on rover/docking coordination must treat
`mock_vehicle_test/codex_ops` as the shared office.

## Mandatory Start Sequence

1. `cd /home/jetson/mock_vehicle_test/codex_ops`
2. `git pull --rebase --autostash`
3. Read:
   - `state/meeting_state.md`
   - `agents/rover.json`
   - `agents/docking.json`
   - your inbox directory
   - `contracts/codex_sync_protocol.md`
4. Run:
   - rover: `./scripts/codex_ops.py checkin --agent rover --repo /home/jetson/mock_vehicle_test --status working --task "..."`
   - docking: `./scripts/codex_ops.py checkin --agent docking --repo /home/jetson/easydocking --status working --task "..."`

If `git pull` fails, say so and continue locally, but record that the shared
state may be stale.

## Ownership

Rover Codex owns:

- `/home/jetson/mock_vehicle_test`
- rover hardware, PX4 rover firmware, MAVROS, QGC, RC safety, Arduino/D24A
- real field observations and safety decisions

Docking Codex owns:

- `/home/jetson/easydocking`
- CorridorPlan, docking planner, simulation metrics, reports
- simulation updates caused by rover observations

Do not edit the peer repo unless the user explicitly asks.

## When To Notify The Peer

Write a note in the peer inbox when any of these happen:

- boss/user makes an architecture decision;
- an interface contract changes;
- a planner assumption changes;
- a hardware observation invalidates a simulation assumption;
- a simulation change requires rover code/test changes;
- a safety blocker appears or is cleared;
- a code commit should be mirrored or reviewed by the other repo;
- a run result changes pass/fail expectations.

Use `scripts/codex_ops.py note` instead of hand-making inbox files when
possible.

## End Sequence

Before ending a meaningful work session:

1. Update local project state/HANDOFF if needed.
2. Add an event or note in `codex_ops/` if the peer must know.
3. Run `./scripts/codex_ops.py checkin --agent <agent> --repo <repo> --status idle --task "done: ..."`
4. Commit and push `codex_ops/` through the existing `mock_vehicle_test` repo
   when network is available.

## Hard Boundaries

- Do not merge `/home/jetson/.codex` and `/home/jetson/.codex_docking`.
- Do not use `codex_ops/` for secrets, API keys, private tokens, or huge logs.
- Do not rely on chat memory alone for cross-agent state.
- Robot runtime safety does not depend on `codex_ops/` being current.
