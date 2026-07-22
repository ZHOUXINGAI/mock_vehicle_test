# Codex Sync Protocol

This contract defines how rover and docking Codex sessions coordinate through
Git. The coordination workspace is `codex_ops/` inside the existing
`mock_vehicle_test` repository, not a separate repo.

## Required Sync Points

Start of work:

```bash
cd /home/jetson/mock_vehicle_test/codex_ops
git pull --rebase --autostash
./scripts/codex_ops.py doctor
./scripts/codex_ops.py checkin --agent <rover|docking> --repo <owned_repo> --status working --task "<task>"
```

After meaningful work:

```bash
./scripts/codex_ops.py checkin --agent <rover|docking> --repo <owned_repo> --status idle --task "<summary>"
git add .
git commit -m "ops: <summary>" -- .
git push
```

When running these commands from `/home/jetson/mock_vehicle_test/codex_ops`,
`git add .` stages only the coordination directory inside the parent
`mock_vehicle_test` repo. From the parent repo, use:

```bash
git add codex_ops
git commit -m "ops: <summary>" -- codex_ops
git push
```

If peer action is needed:

```bash
./scripts/codex_ops.py note --from <agent> --to <peer> --type request --title "..." --summary "..." --need "..."
git add .
git commit -m "ops: request <peer> <topic>"
git push
```

## Inbox Semantics

- `inbox/rover/`: notes for rover Codex to read.
- `inbox/docking/`: notes for docking Codex to read.
- Notes are append-friendly artifacts. Do not delete notes unless both sides
  agree they are archived.
- A peer response should be another note or event that references the original
  note path.

## Event Types

Use these types:

```text
checkin
decision
request
result
blocker
observation
commit
handoff
ack
```

## Conflict Rule

If two files disagree:

1. Latest explicit boss decision wins.
2. `state/meeting_state.md` wins for architecture.
3. The owning repo's state file wins for implementation details:
   - rover: `/home/jetson/mock_vehicle_test/AGENT_STATE.md`
   - docking: `/home/jetson/easydocking/HANDOFF.md`
4. Append a correction event instead of silently rewriting history.

## Robot Runtime Boundary

`codex_ops/` is not part of the vehicle control loop. It can be stale or
offline without making the robots unsafe.
