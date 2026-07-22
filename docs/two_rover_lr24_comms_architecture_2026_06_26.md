# Two-Rover LR24 Communication Architecture - 2026-06-26

This is the rover-side communication and planner-deployment recommendation for
the first two-rover outdoor CorridorPlan experiment.

No hardware motion is requested by this document.

## Decision

Boss decision, updated 2026-06-27:

- The central/leader docking computer is the Carrier rover's onboard
  Nano/Orin Nano.
- The Carrier onboard computer runs `easydocking` planner / CorridorPlan leader
  logic.
- The Mini rover sends compact timestamped state to Carrier and locally tracks
  received plan/phase/speed primitives.
- The human-side ground station is for QGC, logging, monitoring, parameter
  setup, manual safety/abort, and debugging. It is not the primary planner.

```text
Mini vehicle
  -> sends compact timestamped state
  -> locally tracks received CorridorPlan / phase commands

Carrier vehicle
  -> acts as docking leader
  -> Carrier Orin runs docking planner
  -> Carrier onboard vision/stereo closes terminal relative pose loop
```

This supersedes the earlier wording that placed the first central planner on a
human-side ground laptop.

Recommended first-field architecture with the user's three LR24 pairs:

```text
Carrier rover
  Onboard Nano/Orin Nano
    -> runs easydocking planner / CorridorPlan leader
    -> talks to Carrier Pixhawk locally
    -> LR24 Pair B
       -> Mini rover Pixhawk or Mini onboard computer

Ground station / laptop
  QGC / logs / monitoring / setup / manual abort / SSH debug
    -> LR24 Pair C
       -> Carrier Pixhawk telemetry
    -> LR24 Pair A
       -> Mini Pixhawk telemetry
```

Each rover still keeps its own local safety chain:

```text
RC transmitter -> receiver -> Pixhawk manual/stop path -> Arduino/D24A -> motors
```

The Carrier sends only bounded mission commands or low-rate plan/phase
primitives to Mini. Radio must not be the only stop mechanism.

## Why Carrier Onboard First

Carrier onboard leader is the current target because:

- It matches the final aerial docking concept: Carrier is the cooperative
  docking leader.
- Terminal docking must not depend on a ground laptop, 4G, WiFi, or
  high-latency external link.
- Carrier has the compute that will later host stereo/vision and terminal
  relative-pose closure.
- Ground tests should exercise onboard leader autonomy early, not only a
  centralized lab-control setup.

The ground station remains important for human supervision:

- QGC;
- parameter setup;
- logging and post-run analysis;
- manual abort / operator stop workflow;
- SSH/debug tools;
- optional shadow planner for comparison.

But the ground station should not be treated as the main docking planner in
the Carrier-leader implementation.

## LR24 Use

The LR24 should be treated as a transparent serial telemetry radio.

Use three independent pairs:

```text
Pair B: Carrier Nano/Orin Nano <-> Mini Pixhawk or Mini onboard computer
  purpose: inter-vehicle leader link for Mini state and compact plan/abort

Pair C: Ground station <-> Carrier Pixhawk telemetry port
  purpose: QGC/log/parameter monitor for Carrier, sysid 1

Pair A: Ground station <-> Mini Pixhawk telemetry port
  purpose: QGC/log/parameter monitor for Mini, sysid 2
```

Required setup notes:

- Configure the three pairs with different module addresses.
- Keep each pair in duplex mode for PX4 telemetry/control.
- Start with the default 57600 baud unless a measured bandwidth problem forces
  a higher rate.
- If using higher LR24 transfer rates, update the serial baud consistently on
  both the LR24 module and PX4 serial port.
- Use the Pixhawk TELEM port, usually TELEM1 or TELEM2.
- Cross serial lines: LR24 Tx -> Pixhawk Rx, LR24 Rx -> Pixhawk Tx.
- Label every USB radio and every air-side radio physically.
- Do not connect two LR24 radios to the same Pixhawk UART. If Mini already uses
  one TELEM port for Pair B, Pair A must use another TELEM port such as TELEM2,
  or a Mini-side router/companion-computer bridge.

Suggested address plan:

```text
Pair A Ground <-> Mini:    ADDR=1101
Pair B Carrier <-> Mini:   ADDR=1102
Pair C Ground <-> Carrier: ADDR=1103
```

Suggested PX4 starting point per rover:

```text
MAV_0_CONFIG = TELEM1        # or the actual TELEM port used
MAV_0_MODE   = Normal
MAV_0_RATE   = 1200B/s
SER_TEL1_BAUD = 57600 8N1    # if TELEM1 is used
```

Use the actual MAV instance and serial port numbers for the second telemetry
port if TELEM2 is used.

## Do Not Let QGC Steal the Serial Port

For two rovers, avoid having two processes open the same LR24 serial port. The
ground station may directly open Pair A and Pair C from QGC if no router also
uses those serial ports. If logging, routing, or companion software also needs
the same serial stream, put a router in front of QGC.

Preferred structure:

```text
Ground station LR24 serial ports
  -> QGC direct serial links, or mavlink-router/MAVProxy
      -> QGC UDP endpoint
      -> optional log/monitor tools
```

This prevents serial-port contention and keeps QGC as monitor/safety UI while
the planner also receives MAVLink.

## Planner/Bridge Contract for easydocking

Ask the docking Codex to make the bridge Carrier-leader first:

```text
Carrier onboard easydocking planner
  subscribes:
    Carrier local state
    Mini timestamped compact state received over LR24

  publishes:
    Carrier local bounded primitive or velocity setpoint
    Mini compact plan/phase/speed primitive over LR24
```

First implementation target:

```text
Carrier Nano/Orin:
  /carrier/mavros/... or local MAVLink to Carrier Pixhawk
  mini_state_rx over LR24
  mini_plan_tx over LR24
```

The bridge should not require full ROS2 networking over LR24. Ground station
may observe or relay during early tests, but Carrier onboard planner remains
the architecture target.

## Link Roles

LR24 role:

- compact low-rate vehicle state;
- compact plan/phase command;
- explicit abort/freeze/replan events;
- no ROS2 DDS;
- no image/video;
- no dense trajectory spam;
- no high-rate terminal closed-loop control.

4G or outdoor WiFi role, if available:

- auxiliary monitoring;
- logs;
- video preview;
- backup human observability.

4G/WiFi must not be required for safety-critical terminal docking control.
Treat it as helpful visibility, not the control backbone.

## Latency and Stale Data Rule

Every inter-vehicle or planner-facing state packet must include:

```text
vehicle_id
sequence_number
timestamp_monotonic_or_gps_time
position
velocity
yaw
health/status flags
```

Planner behavior:

- predict the peer vehicle forward to planner time before computing a command;
- reject out-of-order sequence numbers;
- mark stale state if age exceeds the configured threshold;
- freeze, replan, or abort when state is stale;
- never keep terminal docking active on old radio data.

Initial message budget target:

```text
Mini state:        80-150 bytes @ 5-10 Hz
Planner command:   event-triggered or 1-5 Hz
Abort/freeze:      event-triggered, highest priority
```

The current rover-side compact test protocol is below this budget:

```text
MiniState frame:    32 bytes
PlanCommand frame:  37 bytes
CorridorPlan frame: 49 bytes
FieldOrigin frame:  31 bytes
Ping/Pong frame:    19 bytes
```

The first rover field tests should stay inside this budget even if any local
computer has higher bandwidth.

## Command Level

For the first real route, do not send dense high-rate global trajectories over
LR24.

Use compact, bounded commands:

```text
plan_id
role = mini | carrier
phase = hold | orbit | arc_to_corridor | terminal | stop
sequence_number
timestamp
v_mps
omega_radps
duration_or_distance
max_speed
max_accel
stop_on_link_loss
valid_until_timestamp
```

The low-level rover side converts these into the currently proven PX4/MAVROS
velocity or primitive commands.

## Test Ladder

1. One LR24 pair only:
   - Carrier Nano/Orin connects to Mini-side radio endpoint;
   - exchange timestamped no-motion state packets;
   - run the bidirectional Pair B dry-run with MiniState uplink and HOLD
     PlanCommand downlink;
   - log packet age, sequence gaps, and drop count;
   - no motion.
2. Both LR24 pairs connected:
   - verify Carrier-Mini coordination pair;
   - verify ground-monitor pair;
   - verify different module addresses;
   - verify serial devices and routers do not fight.
3. Two-rover no-motion telemetry:
   - Carrier planner logs Carrier local state and Mini received state;
   - ground station observes summary/QGC/logs;
   - no arming.
4. Wheels-lifted command path:
   - bounded low-speed primitive to one rover at a time;
   - verify RC/QGC/manual stop and Arduino timeout brake.
5. Low-speed single-rover ground test.
6. Low-speed two-rover route demo.

Rover field constraint:

- Keep the first two-rover route inside about `30m x 30m`.
- Mini should be faster than Carrier for the current scaled route.
- Carrier should remain slower in the first ground demo.
- Start with no-motion telemetry and wheels-lifted tests before real route
  motion.

## Terminal Docking Principle

Radio can coordinate approach, state, plan updates, and aborts. It should not
own the final high-rate terminal loop.

Final terminal docking must be onboard autonomy:

- Carrier onboard computer runs the final planner/control logic;
- Carrier stereo/vision estimates relative pose during terminal closure;
- Mini broadcasts compact state and executes its own received plan/phase;
- LR24 only updates low-rate state/plan/abort;
- stale or missing radio data causes freeze, replan, or abort, not blind
  continuation.

## Current Recommendation to Docking Codex

Implement the first bridge as a Carrier onboard leader coordinator.

Do not assume:

- ROS2 DDS works over LR24;
- the LR24 link can carry high-rate logs or dense trajectory topics;
- QGC can directly own the serial port while MAVROS also uses it.

Do assume:

- Carrier Nano/Orin is the leader planner computer;
- Mini state is compact, timestamped, and rate-limited;
- bounded primitive or `v, omega` commands;
- timestamped messages with sequence numbers;
- stale-data freeze/replan/abort behavior;
- hard stop/failsafe owned by each rover and PX4/RC chain;
- ground station is QGC/logging/monitoring/manual safety/debug, not the main
  planner.
