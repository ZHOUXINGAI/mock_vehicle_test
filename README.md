# mock_vehicle_test

Standalone PX4 rover/offboard training repo.

This repo is intentionally separate from `/home/hw/easydocking` so rover
training does not affect the docking project while another agent or Claude is
working there.

## Goal

Short-term task:

1. Start PX4 rover SITL.
2. Open QGC for monitoring.
3. Run a ROS2 offboard node.
4. Move the rover forward 3m.
5. Return to the start point.
6. If the rover goes outside a 10m software fence, return home.

## Quick Start

From this repo:

```bash
cd /home/hw/mock_vehicle_test
./start.sh
```

Chinese alias:

```bash
./启动.sh
```

Logs are written to `results/<timestamp>_rover_sitl/`.

The script refuses to start if an existing PX4 or MicroXRCEAgent process is
detected. This is deliberate so it does not interfere with `easydocking`.

## Run Only The Offboard Node

Use this when PX4, MicroXRCEAgent, and QGC are already running:

```bash
cd /home/hw/mock_vehicle_test
./scripts/run_offboard_only.sh
```

## Common Settings

```bash
MISSION_MODE=position ./start.sh
MISSION_MODE=velocity ./start.sh
TRAVEL_DISTANCE_M=3.0 ./start.sh
FENCE_RADIUS_M=10.0 ./start.sh
PX4_NAMESPACE=/px4_1 ./scripts/run_offboard_only.sh
```

`position` is the default and preferred first test. PX4 gets a local position
target 3m in front of home, then home again.

`velocity` is for later experiments. It sends forward and return velocity
setpoints. Validate it in SITL or with wheels lifted before using hardware.

## Real Rover Hardware Path

For Pixhawk 6C hardware, start with manual rover control first:

[docs/current_rover_success_baseline_2026_06_16.md](docs/current_rover_success_baseline_2026_06_16.md)

[docs/offboard_minimal_task_design_2026_06_16.md](docs/offboard_minimal_task_design_2026_06_16.md)

[docs/mavros_px4_usb_to_qgc_plan_2026_06_16.md](docs/mavros_px4_usb_to_qgc_plan_2026_06_16.md)

[docs/beginner_rc_rover_step_by_step.md](docs/beginner_rc_rover_step_by_step.md)

[docs/pixhawk6c_manual_rover.md](docs/pixhawk6c_manual_rover.md)

The intended learning sequence is:

1. RC manual rover through PX4 and QGC.
2. Companion computer reads PX4 state only.
3. Companion computer sends offboard setpoints.

The RC/manual path must remain independent of ROS/offboard.

## Files

- `src/mock_rover_offboard.py`: ROS2 offboard mission node.
- `scripts/start_sitl_rover.sh`: starts PX4 rover SITL, MicroXRCEAgent, and mission.
- `scripts/run_offboard_only.sh`: mission node only.
- `config/default.env`: editable defaults.
- `docs/pixhawk6c_manual_rover.md`: hardware first-step guide.

## Dependencies

Expected on this machine:

- ROS 2 Humble
- `px4_msgs` built and sourceable
- PX4-Autopilot at `/home/hw/PX4-Autopilot`
- MicroXRCEAgent at `/home/hw/uxrce_agent_ws/install/microxrcedds_agent/bin/MicroXRCEAgent`

The scripts source `/opt/ros/humble/setup.bash` and, if present,
`/home/hw/easydocking/install/setup.bash` only to reuse the existing local
`px4_msgs` build. They do not modify `easydocking`.
