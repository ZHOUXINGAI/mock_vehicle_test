# Lubancat 4 Rover Migration

Goal: move the working Arduino + RF209S + rover setup toward Lubancat 4 control,
using the D153B/TB6612 motor driver and MG513X brushed DC motors.

This is still a safety training platform before aircraft offboard control. Do
not treat it as a toy: keep wheels lifted for first tests and keep power easy to
disconnect.

## Current Direction

The first Lubancat stage should not use ROS 2 or offboard yet.

Do this order:

1. Lubancat controls D153B only, no receiver input.
2. Low PWM motor smoke test, wheels lifted.
3. Confirm left/right motor channels and forward/backward polarity.
4. Add RF209S receiver input through level shifting.
5. Add encoders.
6. Add ROS 2/offboard-style rover tests.
7. Move to outdoor GPS/geofence tests.

## Critical Electrical Rules

- Lubancat 4 GPIO is 3.3V logic.
- Do not feed 5V receiver PWM directly into Lubancat GPIO.
- D153B/TB6612 `VM` is motor power. Use the 12V motor supply there.
- D153B/TB6612 `VCC` is logic power. Use Lubancat 3.3V for logic input level.
- Common ground is required: Lubancat GND, D153B GND, and 12V supply GND must
  be connected together.
- First motor tests must be wheels-up.
- Keep one hand near the power switch or battery plug.

## Power Wiring

D153B board:

- 12V supply positive -> D153B power input positive / `VM`
- 12V supply negative -> D153B power input negative / `GND`
- Lubancat pin 17 `3V3` -> D153B `VCC`
- Lubancat pin 20 or 25 or 30 or 34 or 39 `GND` -> D153B `GND`

Do not power Lubancat from the D153B 5V output for the first tests. Keep
Lubancat on its own Type-C 5V power.

## Motor Wiring

Assumption for first test:

- Left motor -> D153B A output: `AO1/AO2`
- Right motor -> D153B B output: `BO1/BO2`

If left/right are swapped, change the script mapping or swap motor plugs after
powering down.

If forward/backward is reversed on one motor, swap that motor's two output wires
or flip the corresponding direction constant in code.

## Lubancat 4 40Pin To D153B Control Wiring

These use the `Num` values from the Lubancat 4 hardware specification for sysfs
GPIO control.

| D153B pin | Lubancat physical pin | Lubancat GPIO Num | Purpose |
| --- | ---: | ---: | --- |
| `PWMA` | 12 | 62 | A/left speed PWM |
| `AIN2` | 16 | 113 | A/left direction |
| `AIN1` | 18 | 122 | A/left direction |
| `STBY` | 22 | 124 | driver enable |
| `BIN1` | 29 | 102 | B/right direction |
| `BIN2` | 31 | 111 | B/right direction |
| `PWMB` | 32 | 63 | B/right speed PWM |
| `VCC` | 17 | - | 3.3V logic power |
| `GND` | 20/25/30/34/39 | - | common ground |

The first script uses low-frequency software PWM through Linux sysfs GPIO. This
is only for smoke testing. Later closed-loop control should use proper PWM
hardware or a more suitable GPIO/PWM backend.

## First Smoke Test

Before running:

- Rover lifted, wheels not touching anything.
- D153B switch off.
- Lubancat Type-C powered and booted.
- 12V motor power connected to D153B input, but switch still off.
- Wiring checked twice.

Dry-run first:

```bash
cd /home/cat/mock_vehicle_test_repo
python3 scripts/lubancat_d153b_smoke_test.py
```

Real motor test:

```bash
cd /home/cat/mock_vehicle_test_repo
sudo python3 scripts/lubancat_d153b_smoke_test.py --armed --duty 0.25 --duration 1.0
```

The script sequence is:

1. left forward;
2. stop;
3. left backward;
4. stop;
5. right forward;
6. stop;
7. right backward;
8. stop and disable `STBY`.

If the motor does not move at `0.25`, stop and inspect wiring before increasing.
Do not jump straight to high duty on the ground.

## Later RF209S Receiver Input

Receiver PWM output is usually 5V. Lubancat GPIO is 3.3V. Use a level shifter or
proper divider before connecting RF209S signal lines.

Planned later input pins:

- RF209S throttle/elevator PWM -> level shift -> Lubancat pin 33
- RF209S steering/aileron PWM -> level shift -> Lubancat pin 35

Do not connect those directly until level shifting is in place.
