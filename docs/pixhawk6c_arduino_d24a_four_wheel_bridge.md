# Pixhawk 6C + Arduino + D24A Four-Wheel Bridge

Current live baseline is maintained in
`docs/current_rover_success_baseline_2026_06_16.md`. For the current vehicle,
do not infer throttle/steering from output numbering alone; use the observed
PX4 passthrough mapping `PWM_MAIN_FUNC1=405`, `PWM_MAIN_FUNC2=403`.

This is the four-wheel version after D24A raw motor calibration.

## Architecture

```text
AT9S PRO -> R9DS -> Pixhawk 6C -> two PWM outputs -> Arduino UNO -> D24A -> 4 motors
```

The Arduino is only a translator:

```text
Pixhawk servo PWM 1000-2000us
  -> Arduino
  -> D24A PWM + direction pins
```

## Wiring

Keep the existing Arduino -> D24A wiring:

```text
D24A PWMA -> Arduino D3
D24A AIN1 -> Arduino D4
D24A AIN2 -> Arduino D7

D24A PWMB -> Arduino D5
D24A BIN1 -> Arduino D8
D24A BIN2 -> Arduino D12

D24A PWMC -> Arduino D6
D24A CIN1 -> Arduino D10
D24A CIN2 -> Arduino D11

D24A PWMD -> Arduino D9
D24A DIN1 -> Arduino A0
D24A DIN2 -> Arduino A1

D24A STBY -> Arduino A2
D24A GND  -> Arduino GND
```

Pixhawk -> Arduino:

```text
Pixhawk PWM output 1 signal -> Arduino D2   // forward/back command
Pixhawk PWM output 2 signal -> Arduino D13  // left/right command
Pixhawk PWM output GND      -> Arduino GND
```

Do not connect Pixhawk PWM `+5V` to Arduino `5V` for this test.

## Arduino Firmware

Use:

```text
arduino/d24a_pixhawk_pwm_bridge/d24a_pixhawk_pwm_bridge.ino
```

It expects:

```text
1500us = stop/neutral
>1500us = positive command
<1500us = negative command
```

Current D24A physical mapping:

```text
forward  = A:-pwm  B:-pwm  C:+pwm  D:+pwm
backward = A:+pwm  B:+pwm  C:-pwm  D:-pwm
left     = A:-pwm  B:+pwm  C:-pwm  D:+pwm
right    = A:+pwm  B:-pwm  C:+pwm  D:-pwm
```

## Test Procedure

1. Keep D24A 12V motor power off.
2. Upload the Arduino firmware.
3. Power Pixhawk and connect QGC.
4. Confirm RC input in QGC:
   - channel 2 = forward/back stick
   - channel 1 = left/right stick
   - channel 7 can be arm/safety
5. Configure Pixhawk outputs so output 1 and output 2 produce neutral 1500us
   when sticks are centered.
6. Watch Arduino serial at 115200:

```bash
python3 scripts/arduino_serial_watch.py --port /dev/ttyUSB0 --duration 20
```

Expected neutral:

```text
thr_us=1500-ish thr=0 steer_us=1500-ish steer=0 left=0 right=0
```

7. Lift wheels.
8. Turn on D24A 12V.
9. Test low-risk stick movement.

Do not test on the ground until wheels-up direction is confirmed.
