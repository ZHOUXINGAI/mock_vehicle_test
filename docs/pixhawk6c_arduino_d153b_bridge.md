# Pixhawk 6C + Arduino + D153B Rover Bridge

This is the practical transition path after validating the rover with
Lubancat -> Arduino -> D153B.

Architecture:

```text
RC transmitter / QGC / later offboard
        -> Pixhawk 6C
        -> two PWM outputs
        -> Arduino UNO translation layer
        -> D153B/TB6612 motor driver
        -> MG513X brushed DC motors
```

The Arduino exists only as a translation layer because Pixhawk outputs RC-style
servo PWM, while D153B needs `PWM + direction GPIO` signals.

## What Not To Do

- Do not connect Pixhawk 6C directly to D153B `PWMA/AIN/BIN/PWMB`.
- Do not use three-phase drone ESCs with MG513X. MG513X is a brushed DC geared
  motor with encoder.
- Do not power Arduino from both USB and another 5V regulator at the same time.

## Power Rules

- Arduino: power from USB during bench tests.
- D153B: 12V battery into `J3/VM`.
- Pixhawk: power by USB or its normal power module.
- Grounds must be common:
  - Arduino GND -> D153B GND directly.
  - Pixhawk PWM output GND -> Arduino GND.
- Do not connect Pixhawk servo `+5V` rail to Arduino `5V` for the first tests.

## Arduino To D153B Wiring

Use the D153B manual's Arduino wiring:

| D153B | Arduino UNO |
| --- | --- |
| `PWMA` | D3 |
| `AIN2` | D4 |
| `AIN1` | D5 |
| `STBY` | D7 |
| `BIN1` | D8 |
| `BIN2` | D9 |
| `PWMB` | D10 |
| `GND1` | GND |

## Pixhawk To Arduino Wiring

Initial left/right motor command mode:

| Pixhawk output | Arduino UNO | Meaning |
| --- | --- | --- |
| PWM output 1 signal | D2 | left wheel command |
| PWM output 2 signal | D6 | right wheel command |
| PWM output GND | GND | signal ground |

Only connect signal and ground for the first tests. Leave Pixhawk output `+`
unconnected unless there is a deliberate power plan.

## Arduino Firmware

Sketch:

```text
arduino/d153b_pixhawk_pwm_bridge/d153b_pixhawk_pwm_bridge.ino
```

It expects:

- `1000us` = reverse
- `1500us` = neutral
- `2000us` = forward

If either PWM input is missing or invalid, it stops both motors.

## First Test Procedure

1. Upload the Arduino sketch.
2. Keep D153B 12V motor power off.
3. Connect Pixhawk output signals and ground to Arduino.
4. Power Pixhawk and Arduino.
5. Confirm Arduino serial prints `D153B Pixhawk PWM bridge ready`.
6. Lift the rover so wheels are off the ground.
7. Turn on D153B 12V power.
8. In QGC/PX4, confirm output behavior at low risk.

Do not test on the ground until wheels-up output direction is confirmed.
