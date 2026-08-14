# Orin2 Carrier Two-Rover Plan - 2026-08-07

## Active Roles

```text
Orin2 / system 2: Carrier leader, owns the plan and publishes both tasks
Orin1 / system 1: Mini executor, sends MiniState and validates commands
Ground: QGC, logs and operator safety; not the runtime planner
```

System IDs and Pair B wiring do not change. Pair B remains the compact runtime
link; Git/NATS remain outside vehicle control.

## RViz Offboard Trajectory

The guarded Orin2 trajectory controller now publishes:

```text
/orin2/offboard/planned_path
/orin2/offboard/actual_path
/orin2/offboard/vehicle_pose
/orin2/offboard/lookahead_target
```

Start RViz in a separate terminal before or during an authorized Offboard run:

```bash
cd /home/seeed/mock_vehicle_test
./scripts/run_orin2_trajectory_rviz.sh
```

The planned path is transient-local, so RViz can start late. The actual path is
bounded and republished as it grows. Visualization is fail-soft: an RViz
publisher error disables visualization but cannot alter the motion controller,
setpoint bounds, fault gates, or recovery.

## First Coordinated Motion

Do not begin with orbit docking. The first useful two-rover test is:

1. Survey one shared `field_enu` x-axis and one `origin_id`.
2. Place Orin2 Carrier 1.5 m ahead of Orin1 Mini on the same line.
3. Leader sends HOLD while both state streams and stop paths are qualified.
4. One fresh operator authorization starts both 3 m paths at 0.05 m/s.
5. Both use their local trajectory tracker; Pair B carries the task and state,
   not a high-rate steering loop.
6. Either stale state, command timeout, RC stop, local fault, cross-track limit,
   or operator abort commands both sides to STOP/ABORT.
7. PASS requires both stop, Carrier remains ahead, no reversal/spin, and each
   actual RViz path stays close to its planned line.

Generate the offline plan and run the full leader replay without hardware:

```bash
python3 scripts/run_orin2_carrier_coordination_offline.py --scenario all
```

This writes the Stage-1 paths plus the EasyDocking nominal/fault replay under
`results/orin2_carrier_coordination/`.

## Pair B Role-Reversal Gate

The first physical test remains no-motion. No production executor is connected.

Orin2 Carrier side (MAVROS Router through its Pixhawk/TELEM2):

```bash
CONFIRM_NO_MOTION=true DURATION_SEC=30 \
  ./scripts/run_orin2_carrier_pairb_hold_no_motion.sh
```

Orin1 Mini side (its Pair B CP2102 by-id path):

```bash
CONFIRM_NO_MOTION=true DURATION_SEC=30 \
PAIRB_PORT=/dev/serial/by-id/<orin1-pairb-cp2102> \
  ./scripts/run_orin1_mini_pairb_hold_no_motion.sh
```

Expected direction is now Orin1 MiniState -> Orin2 Carrier, and Orin2 HOLD /
CorridorPlan -> Orin1 Mini. The wrappers explicitly set system IDs so the old
CLI defaults cannot silently restore the previous role assignment.

## Incremental Ladder

1. Offline replay and RViz path review.
2. Pair B role-reversal no-motion HOLD and stale-link Abort tests.
3. One rover moves while the other remains HOLD; repeat with roles swapped.
4. Stage-1 parallel 3 m straight, then repeat with longer distance.
5. Mini alone: one complete stable circle, then circle plus tangent exit.
6. Carrier alone: heading-constrained smooth approach plus shared tangent.
7. Two-rover orbit/tangent run at low speed.
8. Gap closing only after the terminal corridor and front guard pass reliably.

## Full EasyDocking State Machine

The pure leader retains:

```text
WAIT_MINI_STABLE_ORBIT
  -> PLAN_VALIDATED
  -> CARRIER_ARC/MINI_ORBIT
  -> MINI_TANGENT_EXIT
  -> SHARED_TERMINAL
  -> COMPLETE_HOLD
```

The new role-aware configuration is `carrier_vehicle_id=2` and
`mini_vehicle_id=1`. Pair B adaptation is tested end to end. Mini must complete
one continuous qualified orbit before the plan carries `ONE_ORBIT_COMPLETE`.

## Remaining Planner Requirement

The current corrected EasyDocking geometry uses an exact external tangent.
Without a measured Carrier start heading, its Carrier approach is honestly a
zero-curvature straight segment. A real differential rover cannot assume it is
already aligned with that segment.

Before Stage 6, extend the plan contract with measured Carrier start yaw and a
tested Dubins/biarc/clothoid path subject to the rover minimum turn radius. Do
not restore the historical ad-hoc circular arc: that version used a non-general
tangent formula. Until this contract exists, full two-rover motion remains
blocked at the independent single-rover stages.

## Current Boundary

Implemented now:

- Orin2/system2 Carrier role configuration.
- Orin1/system1 MiniState acceptance.
- EasyDocking leader to local Carrier + Pair B Mini command adaptation.
- One-orbit proof and immutable CorridorPlan adaptation.
- Stage-1 parallel plan generator.
- no-motion role-reversal launchers.
- RViz planned/actual path publication for Orin2 Offboard trajectories.

Not connected now:

- production wheel executor;
- automatic dual-rover Arm/Offboard;
- live Pair B motion commands;
- heading-constrained Carrier approach planner.

## Orin1 Software Adaptation

Orin1 was reached over the indoor LAN as `jetson@192.168.8.244` and updated
with fast-forward-only merges on 2026-08-07. Existing modified and untracked
files on Orin1 were preserved.

```text
/home/jetson/mock_vehicle_test: 4bae768
/home/jetson/easydocking:       8b83864
mock_vehicle_test tests:        186 passed, 2 ROS-message tests skipped
easydocking tests:              30 passed
```

Orin1 now has `scripts/run_orin1_outdoor_forward_5m.sh`, which reuses the
current guarded trajectory controller while requiring physical
`MAV_SYS_ID=1`. The common launcher still defaults to system 2, so Orin2
behavior is unchanged. The Orin1 Mini no-motion Pair B launcher uses this
stable endpoint:

```text
/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
```

Only dry-run and pure-software tests were run during adaptation. MAVROS, PX4
services, Offboard, Arduino, LR24 serial traffic and motor executors were not
started. The next physical communication gate is a separately authorized
Pair B HOLD/MiniState exchange with the production executor disconnected.

## Pair B Role-Reversal No-Motion Result

The first real-radio role-reversal exchange passed on 2026-08-07 at about
10:46 CST. Orin1 used physical system 1 as semantic Mini over its CP2102;
Orin2 used physical system 2 as semantic Carrier through the Pixhawk TELEM2
MAVLink TUNNEL route.

```text
Orin2 Carrier: states_rx=275, state_seq_gaps=0
Orin2 Carrier: commands_tx=59, field_origins_tx=6
Orin1 Mini:    states_tx=415, commands_rx=59, command_seq_gaps=0
Orin1 Mini:    rejected=0, aborts_rx=0
Orin1 executor decisions=815, zero_output_count=815
blocked_motion_count=0, nonzero_output_count=0
```

After Carrier stopped transmitting, Mini transitioned through
`command_expired` to `no_command`, with zero output throughout. Production
wheel executors were not connected, and neither vehicle was armed or switched
to Offboard. Both endpoints and the temporary Orin2 MAVROS Router were stopped
after the test.

Artifacts copied to Orin2:

```text
results/pairb_role_reversal_20260807/orin2_carrier.csv
results/pairb_role_reversal_20260807/orin1_mini.csv
```

A second 20-second run added schema-v2 `CorridorPlan` traffic while retaining
HOLD-only commands:

```text
Orin2 Carrier: states_rx=176, state_seq_gaps=0
Orin2 Carrier: commands_tx=40, corridor_plans_tx=10, field_origins_tx=4
Orin1 Mini:    commands_rx=40, command_seq_gaps=0
Orin1 Mini:    corridor_plans_rx=10, corridor_plan_seq_gaps=0
Orin1 Mini:    rejected=0, aborts_rx=0
Orin1 executor decisions=638, zero_output_count=638
blocked_motion_count=0, nonzero_output_count=0
```

The additional artifacts are:

```text
results/pairb_role_reversal_20260807/orin2_carrier_with_plan.csv
results/pairb_role_reversal_20260807/orin1_mini_with_plan.csv
```

## Live Orin1 MiniState Result

The first live-MAVROS MiniState attempt exposed a cross-host ROS 2 discovery
fault: both computers used `ROS_DOMAIN_ID=99` and the same `/mavros/*` topic
names, so Orin1's state source alternated between Orin1 and Orin2 poses over
the LAN. That attempt is invalid and must not be used as navigation evidence.

Runtime ROS is now host-local by default:

```text
ROS_LOCALHOST_ONLY=1
```

Cross-vehicle runtime data continues to use Pair B; DDS is not a vehicle link.
After this isolation, the real Orin1 MAVROS source remained stable near
`(-15.42, -17.34)` in its local PX4 ENU frame and the radio exchange passed:

```text
Orin2 Carrier: states_rx=136, state_seq_gaps=0, commands_tx=30
Orin1 Mini:    states_tx=226, commands_rx=30, command_seq_gaps=0
Orin1 Mini:    rejected=0, aborts_rx=0
Orin1 executor decisions=450, zero_output_count=450, nonzero_output_count=0
```

The live source reported `health=0x000f` and `origin_id=0` intentionally. It
can prove fresh local PX4 position, velocity, yaw, and link state, but it must
not claim a shared field origin or production-executor readiness. The isolated
artifacts are:

```text
results/pairb_role_reversal_20260807/orin2_carrier_live_mavros_isolated.csv
results/pairb_role_reversal_20260807/orin1_live_mavros_isolated.csv
```

## Indoor Readiness Check

### Fake-position and outdoor GPS isolation

Indoor fake external vision is a temporary, explicit profile and must never be
treated as the outdoor baseline. Run an indoor command only through:

```bash
CONFIRM_WHEELS_LIFTED=true \
CONFIRM_VEHICLE_DISARMED=true \
CONFIRM_RC_KILL_READY=true \
CONFIRM_RESTORE_GPS_BASELINE=true \
EXPECTED_SYSTEM_ID=1 \
PIXHAWK_DEVICE=/dev/serial/by-id/usb-Auterion_PX4_FMU_v6C.x_0-if00 \
./scripts/run_indoor_fake_ev_profile.sh \
  --execute \
  --confirm INDOOR_FAKE_EV_WHEELS_LIFTED_RESTORE_GPS_ON_EXIT \
  -- <indoor command>
```

The wrapper requires the outdoor baseline (`EKF2_EV_CTRL=0`,
`EKF2_GPS_CTRL=7`), writes `results/runtime_state/indoor_fake_ev.active`, sets
`EKF2_EV_CTRL=15` only for the child process, and restores plus reads back
`EKF2_EV_CTRL=0` on every normal or signalled exit. If restoration cannot be
verified, the marker remains and outdoor launchers fail closed. Never remove
the marker manually as a substitute for a direct PX4 parameter readback.

The outdoor launcher independently requires `EKF2_EV_CTRL=0`,
`EKF2_GPS_CTRL=7`, real GPS/local-position confirmations, no fake-position
process, and no indoor marker. Indoor logs live under
`results/indoor_fake_ev/`; outdoor logs retain their separate outdoor path.

On 2026-08-07 both Pixhawks were read through their stable USB by-id paths.
Both were `SYS_AUTOSTART=50000`, `CA_AIRFRAME=6`, `CA_R_REV=3`, with MAIN1/2
assigned to motor functions `101/102`. Orin2 remained system 2 and Orin1
system 1. Both were disarmed and landed, but indoors had no GPS fix, no valid
horizontal velocity/position estimate, and no current RC input
(`AUTO.LOITER`, `manual_input=false`). Therefore no Arm, Offboard, or motor
test was attempted. The next live gate requires each RC link to report a safe
`MANUAL`, disarmed prestate with Kill available before any separately
authorized zero-only test.
