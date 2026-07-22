# LR24 Pair B MAVLink Tunnel Runbook

Status: no-motion hardware bring-up for Orin1/Carrier and Orin2/Mini.

This runbook validates the Carrier-to-Mini control-plane link without attaching
received commands to MAVROS setpoints, Arduino output, or motors.

## Topology

```text
Orin1 Carrier
  lr24_pairb_dry_run.py carrier
  MAVLink 2 source 1.242
        |
        | CP2102 USB, 57600 8N1
        v
  Pair B GROUND (ADDR 1102)
        <======== LR24 RF ========>
  Pair B VEHICLE (ADDR 1102)
        |
        | Mini Pixhawk TELEM2, 57600 8N1
        v
  Mini PX4 MAVLink forwarding
        |
        | Pixhawk USB
        v
  Orin2 MAVROS Router /pairb_tunnel
  lr24_pairb_dry_run.py mini
  MAVLink 2 source 2.242
```

Pair A remains Mini-to-ground-station QGC telemetry. Pair C remains
Carrier-to-ground-station QGC telemetry. Pair B is the onboard docking
control-plane link and must not carry ROS 2 DDS or video.

## Safety Gate

Before every command in this runbook:

```text
- both vehicles disarmed
- RC kill/stop available
- wheels lifted or motor power physically disconnected
- no Mini executor process running
- no Offboard smoke test running
- Pair A/B/C binding LEDs steady
```

`CONFIRM_NO_MOTION=true` only authorizes packet exchange. It does not authorize
arming or wheel movement.

## 1. Synchronize Both Computers

On Orin1 and Orin2:

```bash
cd ~/mock_vehicle_test
git pull --ff-only
python3 -m unittest discover -s tests -p 'test_lr24*.py' -v
```

All compact-protocol and MAVLink tunnel tests must pass.

## 2. Configure Mini Pixhawk TELEM2

Run these commands only on Orin2. Stop MAVROS first so the parameter tool has
exclusive Pixhawk USB access.

Read-only check:

```bash
cd ~/mock_vehicle_test
./scripts/configure_px4_pairb_telem2.sh check
```

Apply the reviewed values:

```bash
CONFIRM_NO_MOTION=true \
CONFIRM_MINI_SYSID_2=true \
CONFIRM_PX4_PARAM_WRITE=true \
  ./scripts/configure_px4_pairb_telem2.sh apply
```

The script refuses to write unless the USB heartbeat source is Pixhawk
`MAV_SYS_ID=2`, component 1. It stores these values:

```text
MAV_PROTO_VER=2
MAV_1_CONFIG=102
MAV_1_MODE=7
MAV_1_FORWARD=1
MAV_1_FLOW_CTRL=0
MAV_1_RADIO_CTL=0
MAV_1_RATE=1200
SER_TEL2_BAUD=57600
```

Reboot Mini Pixhawk with QGC or a controlled power cycle. Run the read-only
check again after reboot. Do not continue if any value differs.

## 3. Start Mini MAVROS

On Orin2:

```bash
cd ~/mock_vehicle_test
MAVROS_NS=mini_mavros \
TARGET_SYSTEM=2 \
TARGET_COMPONENT=1 \
MAVLINK_DEVICE=/dev/serial/by-id/usb-Auterion_PX4_FMU_v6C.x_0-if00 \
MAVLINK_BAUD=115200 \
QGC_UDP_URL=udp://:14556@127.0.0.1:14550 \
  ./scripts/run_mavros_px4_usb_to_qgc_logged.sh
```

Wait until `/mini_mavros/state` reports `connected: true`. The Mini tunnel
process auto-discovers the MAVROS Router `add_endpoint` service, so its exact
namespace may differ. The dry-run wrapper also sources `scripts/env.sh`, so a
fresh SSH shell does not need a manual ROS setup command first.

## 4. Start Mini No-Motion Endpoint

In a second Orin2 terminal:

```bash
cd ~/mock_vehicle_test
CONFIRM_NO_MOTION=true \
  ./scripts/run_lr24_pairb_dry_run.sh mini \
    --duration-sec 120 \
    --state-rate-hz 10 \
    --simulate-orbit
```

Expected startup text includes:

```text
transport=mavros-router-tunnel:/pairb_tunnel 2.242->1.242
```

This publishes simulated `MINI_STATE`; it does not read GPS for control and
does not publish a setpoint.

## 5. Start Carrier HOLD Endpoint

On Orin1 after the Mini endpoint is running:

```bash
cd ~/mock_vehicle_test
CONFIRM_NO_MOTION=true \
  ./scripts/run_lr24_pairb_dry_run.sh carrier \
    --duration-sec 120 \
    --command-rate-hz 2 \
    --phase hold \
    --send-corridor-plan \
    --corridor-plan-rate-hz 0.2
```

The built-in Carrier device is:

```text
/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
```

Do not substitute the CH340 Arduino path.

## 6. Pass/Fail

Pass requires all of the following:

```text
- Orin1 receives MINI_STATE continuously at approximately 10 Hz
- Orin2 receives HOLD approximately 2 Hz
- FIELD_ORIGIN and CORRIDOR_PLAN are accepted with zero gate rejection
- no persistent sequence gaps
- no stale-state HOLD caused by Pair B loss after startup
- QGC telemetry on Pair A and Pair C remains available
- no wheel movement, arming, Offboard transition, or Arduino motor output
```

Any wheel motion is an immediate fail. Stop both endpoint processes and
physically remove motor power before diagnosis.

## 7. Routing Diagnosis

If RF LEDs are steady but no compact frames arrive:

1. Re-run `configure_px4_pairb_telem2.sh check` on Orin2.
2. Confirm Mini MAVROS is connected to system 2 over Pixhawk USB.
3. Confirm `/pairb_tunnel/mavlink_source` and `mavlink_sink` exist.
4. In QGC MAVLink Inspector, look for heartbeats from components `1.242` and
   `2.242`, plus `TUNNEL` message ID 385.
5. Check PX4 `mavlink status` for a 57600 TELEM2 instance with forwarding.
6. Verify Pair B radio address 1102 and opposite GROUND/VEHICLE modes.

Do not increase rates until the 120-second HOLD run passes. If TUNNEL packets
are dropped while RF remains healthy, collect both CSV/log directories before
changing `MAV_1_RATE`; the first reviewed increase is 1200 to 1600 B/s.

## 8. Next Integration Boundary

After this run passes, replace simulated Mini state with timestamped MAVROS
position/velocity/yaw. Keep the command executor disabled. Motion integration
is a separate stage requiring a new local confirmation and a wheels-lifted
HOLD/watchdog/abort test.
