# Field Incident - Carrier Left Wheels Twitch After Mini LR24 Setup

Date: 2026-06-27 CST

Status: outdoor motion testing stopped. Do not continue ground motion until the
root cause is cleared.

## Symptom

- Carrier rover left-front and left-rear wheels occasionally twitched or spun.
- The RC kill switch was engaged.
- The symptom appeared during the outdoor LR24/Mini telemetry setup sequence.
- After flashing the safer Arduino PWM bridge, Arduino serial showed neutral
  input and zero commands, but the user still observed occasional twitching
  after D24A motor power was reconnected.

## What Changed Before the Symptom

The user's important observation is valid and must be preserved:

- The symptom was first noticed after changing the setup to communicate with
  the Mini rover over LR24.

Concrete changes made around that time:

- One LR24 pair was used as:
  - LR24 A1 USB -> Carrier Orin Nano.
  - LR24 A2 TELEM1 -> Mini Pixhawk.
- Carrier Orin started Mini MAVROS through:
  `scripts/run_mavros_mini_lr24_to_qgc_logged.sh`.
- That MAVROS instance used:
  - namespace `/mini_mavros`
  - FCU URL on the CP2102 LR24 USB serial device
  - target system `2`
  - QGC UDP forwarding to local QGC.
- Local QGC was started to view the Mini through MAVROS UDP forwarding.
- QGC serial auto-connect was then configured off so QGC should use UDP only.

No rover motion script was intentionally started at that point.

## Evidence Collected On Site

Software state after the incident:

- QGC, MAVROS, Offboard, manual-control, RC override, and fake-vision related
  processes were killed.
- No process was holding `/dev/ttyUSB0`, `/dev/ttyUSB1`, or `/dev/ttyACM0`.
- The Mini MAVROS log showed Mini/PX4 messages and `Kill switch engaged`, but
  no evidence that a Carrier motion command was being streamed from Orin.

Arduino safety firmware was flashed:

- `arduino/d24a_pixhawk_pwm_bridge/d24a_pixhawk_pwm_bridge.ino`
  now uses `INPUT_PULLUP` on Pixhawk PWM inputs.
- It requires several consecutive valid PWM frames before enabling output.
- When both commands are zero, it forces all motors off and pulls D24A `STBY`
  low.

Post-flash Arduino serial check with motor power disconnected:

```text
thr_us=1488-1495
steer_us=1488-1495
thr=0
steer=0
left=0
right=0
valid_pair_count=3
```

That means the Arduino logic saw neutral Pixhawk input and was not commanding
left or right wheel motion at the time of the serial check.

One read-only Pixhawk parameter check returned:

```text
PWM_MAIN_FUNC1=0
PWM_MAIN_FUNC2=0
PWM_MAIN_DIS1=0
PWM_MAIN_DIS2=0
PWM_MAIN_FAIL1=0
PWM_MAIN_FAIL2=0
```

This is not the expected manual baseline and needs a proper QGC/MAVLink
parameter export back at the bench. Do not treat the vehicle as being in the
known-good baseline until this is resolved.

Correction on 2026-07-03:

- The field-side zero values were a decoding mistake. PX4 sent integer
  parameters as `MAV_PARAM_TYPE_INT32` bytewise encoded into the MAVLink
  `PARAM_VALUE.param_value` float field. Interpreting the float numerically
  rounded small bit patterns to zero.
- Correctly decoded Carrier key parameters are saved at
  `results/param_snapshot/carrier_key_params_latest.json`.
- The current Carrier Pixhawk output state is differential rover:

```text
SYS_AUTOSTART=50000
MAV_TYPE=10
CA_AIRFRAME=6
CA_R_REV=3
COM_RC_IN_MODE=3
RC_MAP_PITCH=2
RC_MAP_YAW=4
RC_MAP_THROTTLE=2
RC_MAP_ARM_SW=7
RC_MAP_KILL_SW=10
PWM_MAIN_FUNC1=101
PWM_MAIN_FUNC2=102
PWM_MAIN_MIN1=1300
PWM_MAIN_MAX1=1700
PWM_MAIN_MIN2=1300
PWM_MAIN_MAX2=1700
PWM_MAIN_DIS1=1500
PWM_MAIN_DIS2=1500
PWM_MAIN_FAIL1=1500
PWM_MAIN_FAIL2=1500
```

- Therefore, before any Carrier motion on the current PX4 parameter set,
  Arduino should use
  `arduino/d24a_pixhawk_differential_pwm_bridge/d24a_pixhawk_differential_pwm_bridge.ino`,
  not the throttle/steering `pwm` bridge.
- Manual full-throttle speed was later found too slow because PX4 outputs only
  `1300-1700us`, while the Arduino differential bridge still mapped `+/-450us`
  as full stick and capped `MAX_DRIVE_PWM=140`. On 2026-07-03 the differential
  bridge was updated to `MAX_STICK_US=200` and `MAX_DRIVE_PWM=255`, matching the
  current PX4 output range more closely.

## Current Best Explanation

The incident is strongly correlated with the Mini LR24 setup timing, but the
best current explanation is not "Mini sent a Carrier drive command".

More likely:

1. The Mini LR24/QGC/MAVROS setup changed the electrical environment on the
   Carrier:
   - added USB radio load on the Orin
   - added RF activity near flight-controller and motor-driver wiring
   - changed ground paths through USB, Pixhawk, Arduino, D24A, and LR24
   - changed cable placement near D24A/PWM signal lines
2. The Carrier motor bridge did not have a hard hardware-level stop:
   - Arduino reset makes pins high impedance for a short time.
   - D24A `STBY` had no confirmed external pulldown.
   - D24A direction/PWM inputs may float during reset/noise events.
   - RC kill affects PX4 state, but does not physically remove D24A motor
     power.
3. That marginal bottom-layer safety design was exposed once the LR24/USB/RF
   setup was added.

In short: the Mini LR24 change may still be the trigger, but through
power/ground/EMI/reset/floating-input effects, not necessarily through a
high-level MAVLink command.

## Hypotheses Ranked

### H1 - Most likely: D24A/Arduino electrical fail-safe weakness

If D24A motor power is connected and Arduino/Pixhawk signals reset, float, or
pick up noise, left-side channels may twitch even when the software command is
zero.

This fits:

- twitching with RC kill engaged
- persistence after QGC/MAVROS were stopped
- persistence after Arduino reported neutral commands
- left-side-only behavior, which can point to a specific D24A side/channel or
  wiring sensitivity

### H2 - Likely trigger: LR24/USB/RF changed noise or ground behavior

The user's timing observation fits this:

- LR24 USB radio on the Carrier Orin can add current draw and RF activity.
- USB ground can change the ground reference seen by Arduino/Pixhawk/D24A.
- Cable movement and antenna placement can couple noise into PWM or D24A input
  wiring.

### H3 - Possible: QGC serial auto-connect or serial reset side effect

Before QGC was forced to UDP-only, it may have probed serial devices. Opening
an Arduino USB serial port resets an UNO. If motor power is connected during
an UNO reset, D24A input pins can float unless hardware pulldowns hold them.

This needs confirmation from logs and a controlled bench reproduction.

### H4 - Lower probability: MAVLink command to Carrier

This is lower probability because:

- the Mini MAVROS process targeted system `2`
- the link was opened on the LR24 USB serial device for Mini
- no control process remained running after cleanup
- no serial device was held after cleanup

It is not fully closed until a full bench replay/export confirms the Carrier
Pixhawk state and QGC link behavior.

## Back-Home Test Plan

Do not start QGC, MAVROS, or Offboard for the first tests.

### Test A - Forced standby isolation

Upload:

```bash
CONFIRM_MOTOR_POWER_DISCONNECTED=true \
CONFIRM_WHEELS_LIFTED=true \
CONFIRM_PHYSICAL_POWER_CUTOFF_READY=true \
  ./scripts/upload_d24a_bridge_safety_firmware.sh standby
```

The standby firmware ignores Pixhawk inputs and continuously forces:

```text
STBY = LOW
all PWM pins = LOW
all direction pins = LOW
```

Then:

1. Keep wheels lifted.
2. Connect D24A motor power.
3. Observe for at least 60 seconds.
4. Plug/unplug or move the LR24 USB radio and antenna near the previous outdoor
   position, still with no QGC/MAVROS running.

Interpretation:

- If twitching still happens: root cause is downstream of normal control logic:
  D24A hardware, wiring, ground, motor power noise, or missing pulldowns.
- If twitching stops: the issue is upstream or at the Arduino input/control
  logic layer.

Result on 2026-07-03:

- `standby` firmware was uploaded and verified.
- Arduino serial repeatedly printed:

```text
standby=LOW pwm=0 dir=LOW
```

- With wheels lifted and D24A motor power connected, the user observed no
  twitching.

Interpretation:

- D24A and motors are stable when Arduino actively drives `STBY` low and all
  motor input pins low.
- The remaining root cause is now upstream of or inside the normal PWM bridge
  behavior: Pixhawk PWM output state, Arduino PWM input reading, Arduino reset
  timing, serial/QGC probing, or LR24/USB/RF/ground effects that perturb the
  normal bridge path.
- This result lowers the probability of a purely downstream D24A hardware fault,
  but does not eliminate missing external pulldowns as a safety issue during
  Arduino reset or high-impedance windows.

### Test B - Safe PWM bridge with Pixhawk neutral

Upload:

```bash
CONFIRM_MOTOR_POWER_DISCONNECTED=true \
CONFIRM_WHEELS_LIFTED=true \
CONFIRM_PHYSICAL_POWER_CUTOFF_READY=true \
  ./scripts/upload_d24a_bridge_safety_firmware.sh pwm
```

Watch Arduino serial:

```bash
python3 scripts/arduino_serial_watch.py \
  --port /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 \
  --duration 30
```

Expected while kill/disarmed/neutral:

```text
left=0 right=0
```

Then connect D24A motor power while wheels are lifted and observe.

Result on 2026-07-03:

- Hardened `pwm` bridge was uploaded and verified.
- Arduino serial was monitored for about 90 seconds.
- D24A motor power was connected while wheels were lifted.
- The user observed no twitching.
- Arduino serial stayed neutral for the full observation:

```text
thr=0
steer=0
left=0
right=0
valid_pair_count=3
```

Interpretation:

- With QGC/MAVROS/Offboard stopped and the hardened bridge active, the normal
  Pixhawk PWM -> Arduino -> D24A path is stable at neutral.
- The original outdoor twitch may have depended on the older Arduino bridge
  behavior, Arduino reset/serial-probe timing, QGC/MAVROS serial behavior, or
  LR24/USB/RF/cable conditions that still need staged reproduction.
- Do not resume ground motion yet. Continue by rebuilding the original trigger
  conditions one layer at a time while wheels remain lifted.

### Test C - Pixhawk parameter export

Before changing outputs again, export the Carrier Pixhawk parameters and
resolve why the field read showed `PWM_MAIN_FUNC1/2=0`.

Expected manual baseline for the current wiring is documented in
`docs/current_rover_success_baseline_2026_06_16.md`:

```text
PWM_MAIN_FUNC1=405
PWM_MAIN_FUNC2=403
PWM_MAIN_DIS1=1500
PWM_MAIN_DIS2=1500
PWM_MAIN_FAIL1=1500
PWM_MAIN_FAIL2=1500
```

Do not restore this blindly with motor power connected.

## Hardware Fixes To Add Before Next Field Run

Minimum recommended fixes:

- Add an external pulldown resistor on D24A `STBY` to GND, e.g. `10k`.
- Consider pulldowns on D24A PWM/direction inputs if the module does not have
  reliable input biasing.
- Add local decoupling near D24A logic and motor power:
  - `0.1uF` ceramic near logic supply/GND.
  - `470uF` to `1000uF` electrolytic across motor power/GND near D24A.
- Route LR24 antenna/USB cable away from D24A, motor wires, and Pixhawk PWM
  signal lines.
- Twist motor wires where practical and keep them away from Arduino input
  wires.
- Use a star-ground layout or otherwise verify Pixhawk, Arduino, D24A, and
  Orin grounds do not create noisy ground loops.
- Add a real motor-power cut path. RC kill must eventually cut D24A motor
  power through a relay/MOSFET/power switch, not only ask PX4 to stop.

## Operational Rule Until Cleared

- No dual-rover ground motion.
- No autonomous/offboard motion.
- No QGC/MAVROS link testing with motor power connected.
- Wheels lifted for all tests.
- D24A motor power disconnected whenever flashing Arduino or changing Pixhawk
  output parameters.
