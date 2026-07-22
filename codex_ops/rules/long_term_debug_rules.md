# Long-Term Debug Rules

These rules are intentionally boring. Boring rules survive long field
debugging sessions.

## Keep Facts Small And Durable

- Put decisions in `state/meeting_state.md`.
- Put requests in the peer inbox.
- Put raw large logs in the owning repo or external storage, then link paths.
- Put summaries in events.

## Every Field Result Needs Evidence

Record:

- date/time;
- vehicle;
- command/script;
- software commit or dirty-state note;
- hardware configuration;
- observed behavior;
- pass/fail;
- result/log path;
- implication for the peer Codex.

## Never Hide Safety Blockers

Safety blockers are not "noise". Use event type `blocker` and put the blocker
in the peer inbox if it affects planner assumptions or field schedule.

## Prefer Contracts Over Chat

If two agents disagree repeatedly, write the contract. The contract beats
memory and natural-language summaries.

## Keep Runtime And Coordination Separate

Runtime:

- MAVLink
- LR24 compact packets
- ROS 2 bridges
- local controller state

Coordination:

- Git
- inbox notes
- event logs
- commits and docs
