# LR24 Pair B Wire Contract v1

Status: implementation contract for Orin1/Carrier and Orin2/Mini.

This is the compact, point-to-point control-plane protocol on Pair B. Pair A
and Pair C remain MAVLink/QGC telemetry links and must not carry these frames.

## Physical Link

```text
Pair B address:       1102
Orin1 radio mode:     FHSS ground
Orin2 radio mode:     FHSS vehicle
Work mode:            full duplex
Rate mode:            low, 2.4 KB/s
Radio power:          500 mW
UART/USB format:      57600 baud, 8 data bits, no parity, 1 stop bit
Flow control:         none
```

Orin1 currently identifies the Pair B USB radio as:

```text
/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
```

Pair B endpoint roles and wiring are:

```text
Orin1 / Carrier: GROUND radio -> CP2102 USB -> Orin1 Linux
Orin2 / Mini:    VEHICLE radio -> Pixhawk TELEM2
```

The CH340 device is the Carrier Arduino, not Pair B:

```text
/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0
```

Orin2 does not have, and must not search for, a Pair B Linux
`/dev/serial/by-id/...` path. Its persistent local computer endpoint is the
Mini Pixhawk USB device; Pair B itself terminates at Pixhawk `TELEM2`.

The current `lr24_pairb_dry_run.py` raw-serial Mini mode is valid only for a
temporary direct-USB Mini radio setup. It cannot be pointed at the Pixhawk USB
port: PX4 does not transparently relay arbitrary `L2` bytes between USB and
`TELEM2`. The compact `L2` frame must be carried by an agreed MAVLink routing
adapter (for example MAVLink `TUNNEL` or an equivalent reviewed envelope)
before the physical Pair B test. This adapter is not implemented or validated
yet; do not enable motion based on the existing raw-serial dry-run.

## Shared Field Frame

All plan and state positions use one explicit field frame:

```text
origin: surveyed WGS84 latitude, longitude, altitude
x: East, metres
y: North, metres
yaw: radians, counter-clockwise from +x East, wrapped to [-pi, pi)
omega: positive counter-clockwise
```

The Carrier sends `FIELD_ORIGIN` before any `CORRIDOR_PLAN`. Every
`MINI_STATE` and `CORRIDOR_PLAN` includes the same nonzero `origin_id`. A Mini
must reject a plan with a different origin ID. A change of origin invalidates
the current plan and command.

PX4 native NED yaw is clockwise from North. Convert it with:

```text
field_yaw_enu = wrap_pi(pi/2 - px4_yaw_ned)
```

`src/lr24_field_frame.py` contains the common WGS84-to-ENU conversion.

## Frame Envelope

All multibyte values are little-endian.

```text
offset  size  field
0       2     magic = ASCII "L2" (0x4c 0x32)
2       1     version = 1
3       1     message type
4       1     payload length, 0..255
5       N     payload
5+N     2     CRC16, little-endian
```

CRC is CRC-16/CCITT with polynomial `0x1021` and initial value `0xffff`, over
the complete header and payload but not the CRC bytes. Frames with a wrong
version, type, payload length, or CRC are dropped.

Message types:

```text
1   MINI_STATE
2   PLAN_COMMAND
3   ABORT
4   CORRIDOR_PLAN
5   FIELD_ORIGIN
10  PING
11  PONG
```

## Time And Sequence Rules

`timestamp_ms` is unsigned 32-bit `CLOCK_BOOTTIME` milliseconds on the sender.
It wraps about every 49.7 days. It is for source ordering and sample intervals;
the receiver must not directly compare it with its own clock.

For commands and plans:

```text
ttl_ms = uint32(valid_until_ms - timestamp_ms)
```

The receiver validates the TTL against its local policy and starts that TTL at
local receipt time. The Mini also has a local `750 ms` command watchdog. Thus
clock synchronization is not a prerequisite for stopping safely.

Sequence numbers are independent per message type and sender. A sequence is
newer when:

```text
delta = uint32(candidate - previous)
0 < delta < 0x80000000
```

Duplicates and old/out-of-order plans or commands are rejected. An `ABORT` is
idempotent and always takes priority; it is never ignored merely because it is
duplicated.

## Payloads

### FIELD_ORIGIN, 24-byte payload, 31-byte frame

```text
uint16 origin_id
uint32 seq
uint32 timestamp_ms
int32  latitude_e7
int32  longitude_e7
int32  altitude_mm
uint16 flags
```

`origin_id=0` is invalid for executable plans.

### MINI_STATE, 25-byte payload, 32-byte frame

```text
uint8  vehicle_id             Mini = 2
uint32 seq
uint32 timestamp_ms           state sample time
int16  x_cm                   field ENU
int16  y_cm                   field ENU
int16  vx_cm_s                field ENU
int16  vy_cm_s                field ENU
int16  yaw_cdeg               CCW from East
int16  omega_cdeg_s           CCW positive
uint16 health_flags
uint16 origin_id
```

Health bits:

```text
bit 0  POSITION_VALID
bit 1  VELOCITY_VALID
bit 2  YAW_VALID
bit 3  PX4_CONNECTED
bit 4  RC_STOP_READY
bit 5  EXECUTOR_READY
bit 6  ORIGIN_VALID
```

Carrier freshness is based on local receive time. Initial policy is stale at
`300 ms`. Planner execution requires valid position, velocity, yaw, PX4 link,
and matching origin. The readiness bits are reported separately and do not
authorize motion by themselves.

### CORRIDOR_PLAN, 42-byte payload, 49-byte frame

```text
uint16 plan_id
uint32 seq
uint32 timestamp_ms
uint32 valid_until_ms
int16  rendezvous_x_cm
int16  rendezvous_y_cm
int16  tangent_dir_x_x10000
int16  tangent_dir_y_x10000
uint16 corridor_length_cm
uint16 ahead_distance_cm
uint32 mini_arrival_delay_ms
uint16 trigger_phase_cdeg
uint16 mini_speed_cm_s
uint16 carrier_max_speed_cm_s
uint16 target_front_gap_cm
uint16 flags
uint16 origin_id
```

Plan flags:

```text
bit 0  CORRIDOR_VALID
bit 1  ONE_ORBIT_COMPLETE
```

The Mini requires `CORRIDOR_VALID`, a matching nonzero origin ID, a unit
tangent vector within two percent, a new sequence, and a bounded TTL. The
first full docking run also requires `ONE_ORBIT_COMPLETE` before tangent exit.

### PLAN_COMMAND, 30-byte payload, 37-byte frame

```text
uint16 plan_id
uint8  target_role             Mini = 1, Carrier = 2
uint8  phase                   HOLD=0, ORBIT=1, ARC=2, TERMINAL=3,
                              STOP=4, ABORT=5
uint32 seq
uint32 timestamp_ms
uint32 valid_until_ms
int16  v_cm_s                  body-forward speed
int16  omega_cdeg_s            positive CCW
uint16 duration_ms
uint16 distance_cm
uint16 max_speed_cm_s
uint16 max_accel_cm_s2
uint16 flags
```

Initial stream rate is `5 Hz`, command validity is `500 ms`, and Mini watchdog
is `750 ms`. `HOLD` and `STOP` must contain zero `v` and zero `omega`. Reverse
motion is not allowed in the docking route. `ARC_TO_CORRIDOR` and `TERMINAL`
must reference the active plan ID. Local limits override every transmitted
limit.

### ABORT, 14-byte payload, 21-byte frame

```text
uint8  source_role
uint8  reason
uint16 plan_id
uint32 seq
uint32 timestamp_ms
uint16 flags
```

Reasons currently include operator abort, link stale, invalid state, invalid
planner output, front-gap violation, lateral error, and local safety failure.
Carrier repeats abort at `10 Hz` for one second. Mini latches abort, commands a
local stop through its own executor, and refuses subsequent wireless motion
commands. Clearing the latch is local-only after the operator repeats safety
checks; there is no wireless clear-abort command.

## Rates And Budget

```text
MiniState:       32 bytes x 10 Hz = 320 B/s
PlanCommand:     37 bytes x  5 Hz = 185 B/s
CorridorPlan:    49 bytes x 0.2 Hz, plus event send
FieldOrigin:     31 bytes x 0.2 Hz during setup
ABORT:           21 bytes x 10 Hz for one second
```

Normal mixed traffic remains well below the LR24 low-rate 2.4 KB/s mode.

## Safety Boundary

Pair B software is split into three layers:

```text
binary frame parser -> command safety gate -> local rover executor
```

The first two layers are implemented and testable with no motion. The executor
must remain disabled by default and requires local, explicit motion approval.
The radio binding LED only proves RF pairing; it does not prove packet loss,
latency, common coordinates, watchdog behavior, or motor-stop behavior.

Implementation files:

```text
src/lr24_compact_protocol.py
src/lr24_command_guard.py
src/lr24_field_frame.py
scripts/lr24_pairb_dry_run.py
scripts/run_lr24_pairb_dry_run.sh
tests/test_lr24_compact_protocol.py
config/lr24/pairb_v1.json
```
