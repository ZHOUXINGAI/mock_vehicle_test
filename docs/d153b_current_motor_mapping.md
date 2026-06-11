# D153B Current Motor Mapping

Last calibrated on 2026-06-10.

Observed raw command mapping:

| Raw command | Raw meaning | Physical result |
| --- | --- | --- |
| `Q` | A channel forward | right motor backward |
| `A` | A channel backward | right motor forward |
| `E` | B channel forward | left motor backward |
| `D` | B channel backward | left motor forward |

Derived physical command mapping:

| Physical command | Raw command |
| --- | --- |
| left motor forward | `D` |
| left motor backward | `E` |
| right motor forward | `A` |
| right motor backward | `Q` |
| vehicle forward | Arduino `B` command |
| vehicle backward | Arduino `F` command |
| pivot left | Arduino `L` command |
| pivot right | Arduino `R` command |

Reason:

- D153B A output is physically wired to the right motor.
- D153B B output is physically wired to the left motor.
- Both motor directions are inverted relative to the Arduino raw command names.

## Current PWM Notes

Observed on 2026-06-10:

- Single-motor test can run at `PWM=80-120`.
- Two-motor sequence at `PWM=120` is not reliable: right motor can fail to
  start during physical backward/right-turn commands.
- Two-motor sequence at `PWM=160` works for all four actions.

Use this as the current safe bench default:

```bash
python3 scripts/d153b_arduino_sequence_test.py --pwm 160 --duration 1 --armed
```
