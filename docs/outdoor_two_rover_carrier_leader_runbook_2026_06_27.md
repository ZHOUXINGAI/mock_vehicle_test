# Outdoor Two-Rover Carrier-Leader Runbook - 2026-06-27

This is the morning field checklist for the first two-rover docking-prep day.

Do not start route motion from this document alone. Motion still requires fresh
user confirmation at the field.

## Active Architecture

Boss decision:

```text
Carrier onboard Nano/Orin Nano = central/leader docking computer
Mini rover = sends timestamped compact state, tracks received primitives
Ground station = QGC/logs/monitoring/setup/manual safety/debug
```

Read first:

```text
docs/codex_shared_meeting_state.md
docs/local_easydocking_planner_in_mock_vehicle_test.md
docs/two_rover_lr24_comms_architecture_2026_06_26.md
config/ground_2d/carrier_leader_field_test_2026_06_27.json
```

## Hardware Roles

Carrier rover:

- Carrier onboard Nano/Orin Nano runs `easydocking` planner / CorridorPlan
  leader.
- Carrier Nano talks to its own Carrier Pixhawk locally over USB or local
  serial.
- Carrier Nano receives Mini state over LR24 Pair B.
- Carrier Nano sends Mini plan/phase/speed/abort over LR24 Pair B.

Mini rover:

- Sends compact timestamped state to Carrier.
- Receives compact plan/phase/speed/abort from Carrier.
- Locally tracks received primitives.
- Keeps local failsafe if link becomes stale.

Ground station:

- Runs QGC.
- Collects logs.
- Monitors both vehicles.
- Handles setup, parameter checks, and manual safety/abort.
- Does not run the primary docking planner.

## LR24 Pair Plan

Use three separate LR24 pairs:

```text
Pair B: Carrier Nano/Orin Nano <-> Mini side
  purpose: Carrier-Mini state/plan/abort
  suggested address: ADDR=1102
  current status: this is the active Mini observation/leader link

Pair C: Ground station <-> Carrier Pixhawk telemetry
  purpose: QGC/log/monitor/debug for Carrier
  suggested address: ADDR=1103

Pair A: Ground station <-> Mini Pixhawk telemetry
  purpose: QGC/log/monitor/debug for Mini
  suggested address: ADDR=1101
```

Keep LR24 at default `57600` baud for the first benchmark unless measured
results force a change.

For Pixhawk TELEM wiring:

```text
LR24 TX -> Pixhawk RX
LR24 RX -> Pixhawk TX
LR24 GND -> Pixhawk GND
```

Do not connect power in a way that back-powers the wrong board. Use the cable
intended for the LR24/Pixhawk combination and verify the pinout before power.

Do not connect two LR24 radios to the same Pixhawk UART. Pair A needs a second
Mini Pixhawk telemetry port, for example TELEM2, unless the Mini side uses a
router/companion computer to share one MAVLink stream.

## Before Leaving

Pack:

- Carrier rover, Mini rover.
- Carrier Nano/Orin power and keyboard/display/SSH path.
- Ground station laptop.
- Three complete LR24 pairs, physically labeled:
  - `A-GROUND`
  - `A-MINI`
  - `B-CARRIER`
  - `B-MINI`
  - `C-GROUND`
  - `C-CARRIER`
- USB cables for LR24 ground/computer-side radios.
- Pixhawk TELEM cables for air/rover-side radios.
- RC transmitter(s), charged.
- Main batteries, charged.
- Physical power cutoff method.
- Tape/markers for a `30m x 30m` field and `R=4.5m` Mini circle.
- Tools to lift wheels safely.

Software on Carrier Nano:

```bash
cd /home/jetson/mock_vehicle_test
python3 scripts/lr24_link_benchmark.py sizes
python3 scripts/lr24_link_benchmark.py --help
python3 scripts/prepare_ground_2d_corridor_route.py --no-write-results
./scripts/run_ground_2d_corridor_preview.sh
```

Software on Mini side:

- It needs the same LR24 benchmark script or an equivalent echo/state/command
  test endpoint.
- If Mini uses another computer, copy:

```text
src/lr24_compact_protocol.py
scripts/lr24_link_benchmark.py
scripts/run_lr24_no_motion_benchmark.sh
src/lr24_command_guard.py
src/lr24_field_frame.py
scripts/lr24_pairb_dry_run.py
scripts/run_lr24_pairb_dry_run.sh
```

Expected compact frame sizes:

```text
MiniState frame:    32 bytes
PlanCommand frame:  37 bytes
Ping/Pong frame:    19 bytes
CorridorPlan frame: 49 bytes
FieldOrigin frame:  31 bytes
```

These are below the current message budget and leave room for later fields.

## Step 0 - Local CorridorPlan Preview

Use `mock_vehicle_test` as the field execution workspace:

```bash
cd /home/jetson/mock_vehicle_test
./scripts/run_ground_2d_corridor_preview.sh
```

Pass target:

- route completes in the offline preview;
- first-pass field box stays inside about `30m x 30m`;
- Mini speed is greater than Carrier speed;
- front violations are zero before first pass;
- Carrier reaches tangent point `T`, then follows the same straight tangent
  corridor as Mini;
- terminal Carrier path must not show a hook, S-turn, or lateral chase;
- `terminal_corridor_shape_ok=True`.

Output:

```text
results/ground_2d_corridor_preview/latest/
```

Expected images in that directory:

```text
trajectory_xy.png
trajectory_xy_full.png
speed_profile.png
distance_convergence.png
front_gap.png
lateral_gap.png
carrier_corridor_lateral_error.png
phase_timeline.png
speed_distance_front.png
```

## Step 1 - Identify Serial Devices

On Carrier Nano:

```bash
ls -l /dev/serial/by-id/
dmesg | tail -n 50
```

Pick the LR24 Pair B serial device path. Prefer `/dev/serial/by-id/...` over
`/dev/ttyUSB0`.

On Mini side, do the same for its Pair B radio.

On ground station, identify Pair A and Pair C if used.

## Step 2 - Pair B Ping/Echo Benchmark

No motion. Motors disabled or wheels lifted.

Mini side:

```bash
cd /home/jetson/mock_vehicle_test
CONFIRM_NO_MOTION=true ./scripts/run_lr24_no_motion_benchmark.sh echo \
  --port /dev/serial/by-id/<B-MINI-LR24> \
  --duration-sec 120
```

Carrier Nano:

```bash
cd /home/jetson/mock_vehicle_test
CONFIRM_NO_MOTION=true ./scripts/run_lr24_no_motion_benchmark.sh ping \
  --port /dev/serial/by-id/<B-CARRIER-LR24> \
  --duration-sec 60 \
  --rate-hz 10 \
  --csv results/lr24_benchmark/latest/ping_rtt.csv
```

Pass target before any motion:

- `lost=0` preferred.
- `p95_rtt_ms < 150ms` for rover demo.
- `max_rtt_ms < 300ms`.
- If worse, do not run route motion. Move antennas, reduce rate, shorten range,
  or debug power/baud/address first.

## Step 3 - Mini State Stream Benchmark

Mini side, no motion:

```bash
CONFIRM_NO_MOTION=true ./scripts/run_lr24_no_motion_benchmark.sh state-tx \
  --port /dev/serial/by-id/<B-MINI-LR24> \
  --duration-sec 60 \
  --rate-hz 10 \
  --simulate-orbit \
  --radius-m 4.5 \
  --speed-mps 0.9
```

Carrier Nano:

```bash
CONFIRM_NO_MOTION=true ./scripts/run_lr24_no_motion_benchmark.sh state-rx \
  --port /dev/serial/by-id/<B-CARRIER-LR24> \
  --duration-sec 60
```

Pass target:

- no large sequence gaps;
- mean interval near `100ms` at `10Hz`;
- max interval below the stale threshold target, initially `300ms`.

## Step 4 - Carrier Command Stream Benchmark

Mini side, no motion:

```bash
CONFIRM_NO_MOTION=true ./scripts/run_lr24_no_motion_benchmark.sh command-rx \
  --port /dev/serial/by-id/<B-MINI-LR24> \
  --duration-sec 60
```

Carrier Nano:

```bash
CONFIRM_NO_MOTION=true ./scripts/run_lr24_no_motion_benchmark.sh command-tx \
  --port /dev/serial/by-id/<B-CARRIER-LR24> \
  --duration-sec 30 \
  --rate-hz 2 \
  --role mini \
  --phase hold \
  --v-mps 0.0 \
  --omega-radps 0.0 \
  --valid-for-ms 500
```

This only proves command delivery. It must not be wired to motor execution yet.

## Step 5 - Pair B Bidirectional Dry-Run

This is the first mixed-traffic test on Pair B:

```text
Mini -> Carrier: compact MiniState at 10Hz
Carrier -> Mini: compact PlanCommand at 2Hz
```

It is still no-motion. Do not wire the Mini receiver to motor execution.

Mini side:

```bash
CONFIRM_NO_MOTION=true ./scripts/run_lr24_pairb_dry_run.sh mini \
  --port /dev/serial/by-id/<B-MINI-LR24> \
  --duration-sec 120 \
  --state-rate-hz 10 \
  --simulate-orbit \
  --radius-m 4.5 \
  --speed-mps 0.9
```

Carrier Nano:

```bash
CONFIRM_NO_MOTION=true ./scripts/run_lr24_pairb_dry_run.sh carrier \
  --port /dev/serial/by-id/<B-CARRIER-LR24> \
  --duration-sec 120 \
  --command-rate-hz 2 \
  --phase hold \
  --stale-ms 300 \
  --send-corridor-plan \
  --corridor-plan-rate-hz 0.2
```

Pass target:

- Carrier summary has no large `state_seq_gaps`.
- Mini summary has no large `command_seq_gaps`.
- Mini receives `CORRIDOR_PLAN` frames.
- Carrier sends `HOLD` if no state has arrived or state is stale.
- CSV/log files are saved under:

```text
results/lr24_pairb_dry_run/latest/
```

Only after this passes should docking-side code replace the simulated Mini
state source or HOLD command source.

## Step 6 - Ground Monitors

Pair C and Pair A are optional for first inter-vehicle link tests, but
recommended for outdoor work.

Use them for QGC/log/monitor/debug only. Do not make either ground-monitor link
the primary docking planner path.

Check:

- Ground station can see Carrier telemetry over Pair C.
- Ground station can see Mini telemetry over Pair A.
- QGC and any router do not fight for the same serial port.
- If QGC opens the serial port directly, no other process can use that same
  serial device. Prefer a router when shared access is needed.

## Step 7 - Rover Safety Checks Before Wheels-Lifted Commands

For each rover separately:

- RC transmitter on.
- QGC visible.
- Vehicle starts disarmed.
- Manual stop path known.
- Physical power cutoff ready.
- Wheels lifted.
- Arduino differential PWM bridge loaded if using PX4 differential Offboard.
- PX4 output mapping confirmed.
- Arduino timeout brake tested by invalidating/removing PWM input.

No route motion until all of these pass.

## Step 8 - Wheels-Lifted Primitive Checks

Use existing rover scripts only after explicit confirmation.

For the currently proven differential Offboard path:

```bash
CONFIRM_WHEELS_LIFTED=true \
CONFIRM_VEHICLE_DISARMED=true \
CONFIRM_RC_READY=true \
CONFIRM_PARAM_BACKUP=true \
CONFIRM_QGC_DISARM_READY=true \
CONFIRM_PHYSICAL_POWER_CUTOFF_READY=true \
CONFIRM_FAKE_LOCAL_POSITION_ONLY=true \
CONFIRM_LOW_SPEED_WHEELS_TEST=true \
CONFIRM_ARDUINO_DIFFERENTIAL_PWM_BRIDGE=true \
  ./scripts/run_real_rover_mavros_differential_fake_vision_offboard_5s_sequence.sh
```

Do this one rover at a time.

Target checks:

- forward wheels direction correct;
- left/ccw sign correct;
- right sign correct;
- no unexpected reverse;
- stop commands stop all motors;
- RC/QGC/manual stop remains available.

## Step 9 - Field Layout

Only after no-motion and wheels-lifted checks pass:

```text
Field box: about 30m x 30m
Mini orbit center: (0, 0)
Mini orbit radius: 4.5m
Mini direction: ccw
Carrier start: (-7, -6)
Mini target speed: 0.9m/s
Carrier max speed: 0.7m/s
```

Physically mark:

- origin `(0,0)`;
- x-axis direction;
- `R=4.5m` circle;
- Carrier start point;
- safe spectator/operator zone outside the field.

## Step 10 - First Ground Motion Ramp

Do not start with two-rover docking.

Ramp:

1. Carrier alone, low-speed forward `2-3m`, stop.
2. Carrier alone, low-speed left/right arc signs.
3. Mini alone, low-speed forward `2-3m`, stop.
4. Mini alone, partial `R=4.5m` circle at reduced speed.
5. Mini alone, full `R=4.5m` circle at reduced speed.
6. Carrier alone, arc/straight shape at reduced speed.
7. Two rovers, reduced speed, widened separation.
8. Full shape demo only after fresh user approval.

## Abort Rules

Abort immediately if:

- QGC/manual stop unavailable;
- RC link not behaving;
- LR24 state stale beyond threshold;
- sequence gaps are large;
- unexpected arming;
- wrong wheel direction;
- unexpected reverse;
- in-place spin;
- Mini exits orbit early;
- Carrier is not clearly controllable;
- people enter field box.

On abort:

```text
1. Stop command / hold phase.
2. QGC disarm.
3. RC/manual stop.
4. Physical power cutoff if needed.
5. Save logs before changing setup.
```

## Log Locations

LR24 benchmark:

```text
results/lr24_benchmark/latest/
results/lr24_pairb_dry_run/latest/
```

Route preflight:

```text
results/ground_2d_corridor_preflight/latest/
```

MAVROS:

```text
results/mavros/latest/
```

After each run, copy the relevant result directory name into
`docs/codex_cross_agent_log.md`.
