# D24A Four-Wheel Current Motor Mapping

Last updated: 2026-06-11

This file records the raw D24A channel calibration for the Orin Nano four-wheel
rover. Keep wheels lifted during calibration.

## Raw Calibration Results

```text
A forward  = right-front wheel backward
A backward = right-front wheel forward
B forward  = left-front wheel backward
B backward = left-front wheel forward
C forward  = left-rear wheel forward
C backward = left-rear wheel backward
D forward  = right-rear wheel forward
D backward = right-rear wheel backward
```

## Derived Physical Commands

The Arduino serial bridge uses signed raw commands:

- positive command = raw `forward`
- negative command = raw `backward`

Physical wheel mapping:

```text
right-front forward = A backward = -A
right-front backward = A forward = +A

left-front forward = B backward = -B
left-front backward = B forward = +B

left-rear forward = C forward = +C
left-rear backward = C backward = -C

right-rear forward = D forward = +D
right-rear backward = D backward = -D
```

High-level rover commands:

```text
forward  = A:-pwm  B:-pwm  C:+pwm  D:+pwm
backward = A:+pwm  B:+pwm  C:-pwm  D:-pwm
left     = A:-pwm  B:+pwm  C:-pwm  D:+pwm
right    = A:+pwm  B:-pwm  C:+pwm  D:-pwm
```
