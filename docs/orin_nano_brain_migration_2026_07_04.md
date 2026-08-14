# Orin Nano Brain Migration 2026-07-04

The Lubancat companion-computer path is being abandoned. The project is moving
back to an Orin Nano companion computer while preserving the successful PX4
v1.17 differential-rover baseline, Codex session context, and QGC local
configuration.

## Source State

Source machine/user:

```text
/home/cat
```

Active project source:

```text
/home/cat/mock_vehicle_test
```

Active Codex source:

```text
/home/cat/.codex
```

QGC settings source:

```text
/home/cat/.config/QGroundControl.org
```

## Migration Archive

Created archive:

```text
/home/cat/orin_nano_brain_migration_20260704_011107.tar.gz
/home/cat/orin_nano_brain_migration_20260704_011107.tar.gz.sha256
```

The archive contains:

```text
payload/home/cat/mock_vehicle_test/
payload/home/cat/.codex/
payload/home/cat/.config/QGroundControl.org/
RESTORE_ORIN_NANO.md
MANIFEST/
```

The archive includes Codex private state and `~/.codex/auth.json`. Treat it as
a credential backup. It intentionally does not include `~/.ssh`; configure a
fresh GitHub SSH key on the target Orin Nano if needed.

Excluded from Codex payload:

```text
.codex/cache/
.codex/.tmp/
.codex/tmp/
old codex_context_migration_20260622_144310 archive/directory
```

## Restore Rule

On the new Orin Nano, restore the contents of `payload/home/cat/*` into the new
user's `$HOME`. If the new user is `jetson`, copy:

```text
payload/home/cat/mock_vehicle_test/              -> /home/jetson/mock_vehicle_test/
payload/home/cat/.codex/                         -> /home/jetson/.codex/
payload/home/cat/.config/QGroundControl.org/     -> /home/jetson/.config/QGroundControl.org/
```

After restore, start Codex from:

```bash
cd ~/mock_vehicle_test
```

Then ask Codex to read:

```text
AGENT_STATE.md
docs/lubancat_outdoor_offboard_checkpoint_2026_06_22.md
docs/orin_nano_brain_migration_2026_07_04.md
```

## Current Vehicle Baseline To Preserve

The accepted rover state is the PX4 v1.17 differential-rover Offboard baseline
validated outdoors on 2026-06-24:

- Pixhawk 6C with `px4_fmu-v6c_rover.px4`.
- Arduino UNO running `d24a_pixhawk_differential_pwm_bridge.ino`.
- D24A four-wheel mapping preserved in the project docs.
- PX4 MAIN1/MAIN2 differential motor outputs preserved.
- Offboard forward, high-authority differential turn, yaw-wrap handling,
  cleanup, and safety recovery validated.

## Safety Carryover

The fresh-start safety rule carries forward to Orin Nano:

`准备好了` only authorizes setup and non-motion checks. Every real motor or
wheels-down motion test requires a fresh current-run `确认开始` after checking
HDMI/display/USB/power/loose cables, field clearance, RC kill/disarm, QGC
disarm, physical cutoff, PX4 safe state, and neutral outputs.
