# D24A Four-Wheel Current Motor Mapping

Last updated: 2026-06-22

This file records the raw D24A channel calibration for the Lubancat/Orin Nano
four-wheel rover hardware. Keep wheels lifted during calibration.

## Raw Calibration Results

```text
A forward  = left-rear wheel forward
A backward = left-rear wheel backward
B forward  = right-front wheel forward
B backward = right-front wheel backward
C forward  = right-rear wheel backward
C backward = right-rear wheel forward
D forward  = left-front wheel backward
D backward = left-front wheel forward
```

## Derived Physical Commands

The Arduino serial bridge uses signed raw commands:

- positive command = raw `forward`
- negative command = raw `backward`

Physical wheel mapping:

```text
right-front forward = B forward = +B
right-front backward = B backward = -B

left-front forward = D backward = -D
left-front backward = D forward = +D

left-rear forward = A forward = +A
left-rear backward = A backward = -A

right-rear forward = C backward = -C
right-rear backward = C forward = +C
```

High-level rover commands:

```text
forward  = A:+pwm  B:+pwm  C:-pwm  D:-pwm
backward = A:-pwm  B:-pwm  C:+pwm  D:+pwm
left     = A:-pwm  B:+pwm  C:-pwm  D:+pwm
right    = A:+pwm  B:-pwm  C:+pwm  D:-pwm
```
