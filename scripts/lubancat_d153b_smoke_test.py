#!/usr/bin/env python3
"""Low-risk D153B/TB6612 motor smoke test for Lubancat 4.

This uses Linux sysfs GPIO and software PWM. It is intended only for the first
wheels-up wiring/direction test, not final rover control.
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from dataclasses import dataclass
from pathlib import Path


SYSFS_GPIO = Path("/sys/class/gpio")


@dataclass(frozen=True)
class MotorPins:
    pwm: int
    in1: int
    in2: int


PIN_STBY = 124

# Physical wiring plan:
#   A channel = left motor
#   B channel = right motor
LEFT = MotorPins(pwm=62, in1=122, in2=113)
RIGHT = MotorPins(pwm=63, in1=102, in2=111)

ALL_GPIO = [
    PIN_STBY,
    LEFT.pwm,
    LEFT.in1,
    LEFT.in2,
    RIGHT.pwm,
    RIGHT.in1,
    RIGHT.in2,
]


class SysfsGpio:
    def __init__(self, number: int) -> None:
        self.number = number
        self.path = SYSFS_GPIO / f"gpio{number}"

    def export(self) -> None:
        if not self.path.exists():
            (SYSFS_GPIO / "export").write_text(str(self.number), encoding="ascii")
            deadline = time.monotonic() + 1.0
            while not self.path.exists() and time.monotonic() < deadline:
                time.sleep(0.01)
        (self.path / "direction").write_text("out", encoding="ascii")
        self.write(0)

    def write(self, value: int) -> None:
        (self.path / "value").write_text("1" if value else "0", encoding="ascii")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Lubancat 4 D153B/TB6612 wheels-up smoke test."
    )
    parser.add_argument(
        "--armed",
        action="store_true",
        help="actually drive GPIO pins; without this, only print the sequence",
    )
    parser.add_argument(
        "--duty",
        type=float,
        default=0.25,
        help="software PWM duty cycle, 0.05 to 0.80 for this safety test",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=1.0,
        help="seconds per motor direction step",
    )
    parser.add_argument(
        "--pwm-hz",
        type=float,
        default=100.0,
        help="software PWM frequency for smoke test",
    )
    parser.add_argument(
        "--left-invert",
        action="store_true",
        help="invert left motor forward/backward direction",
    )
    parser.add_argument(
        "--right-invert",
        action="store_true",
        help="invert right motor forward/backward direction",
    )
    return parser.parse_args()


def validate_args(args: argparse.Namespace) -> None:
    if not 0.05 <= args.duty <= 0.80:
        raise SystemExit("Refusing unsafe duty. Use 0.05 <= --duty <= 0.80.")
    if not 0.1 <= args.duration <= 3.0:
        raise SystemExit("Refusing unsafe duration. Use 0.1 <= --duration <= 3.0.")
    if not 10.0 <= args.pwm_hz <= 500.0:
        raise SystemExit("Use 10 <= --pwm-hz <= 500 for this sysfs smoke test.")


def require_sysfs_gpio() -> None:
    if not SYSFS_GPIO.exists():
        raise SystemExit("/sys/class/gpio does not exist on this system.")
    if os.geteuid() != 0:
        raise SystemExit("GPIO sysfs writes usually need root. Re-run with sudo.")


def set_direction(
    pins: dict[int, SysfsGpio],
    motor: MotorPins,
    forward: bool,
    invert: bool,
) -> None:
    actual_forward = not forward if invert else forward
    if actual_forward:
        pins[motor.in1].write(1)
        pins[motor.in2].write(0)
    else:
        pins[motor.in1].write(0)
        pins[motor.in2].write(1)


def stop_motor(pins: dict[int, SysfsGpio], motor: MotorPins) -> None:
    pins[motor.pwm].write(0)
    pins[motor.in1].write(0)
    pins[motor.in2].write(0)


def stop_all(pins: dict[int, SysfsGpio]) -> None:
    stop_motor(pins, LEFT)
    stop_motor(pins, RIGHT)
    pins[PIN_STBY].write(0)


def run_pwm(
    pins: dict[int, SysfsGpio],
    motor: MotorPins,
    duty: float,
    duration: float,
    pwm_hz: float,
) -> None:
    period = 1.0 / pwm_hz
    high_time = period * duty
    low_time = period - high_time
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        pins[motor.pwm].write(1)
        time.sleep(high_time)
        pins[motor.pwm].write(0)
        time.sleep(low_time)


def dry_run(args: argparse.Namespace) -> None:
    print("DRY RUN ONLY. Add --armed to drive GPIO pins.")
    print("Wheels must be lifted before real test.")
    print("Pin map:")
    print(f"  STBY GPIO {PIN_STBY}")
    print(f"  left  PWM/IN1/IN2 = {LEFT.pwm}/{LEFT.in1}/{LEFT.in2}")
    print(f"  right PWM/IN1/IN2 = {RIGHT.pwm}/{RIGHT.in1}/{RIGHT.in2}")
    print("Sequence:")
    print(f"  left forward  {args.duration:.1f}s duty={args.duty:.2f}")
    print(f"  left backward {args.duration:.1f}s duty={args.duty:.2f}")
    print(f"  right forward {args.duration:.1f}s duty={args.duty:.2f}")
    print(f"  right backward {args.duration:.1f}s duty={args.duty:.2f}")


def main() -> int:
    args = parse_args()
    validate_args(args)

    if not args.armed:
        dry_run(args)
        return 0

    require_sysfs_gpio()

    pins = {number: SysfsGpio(number) for number in ALL_GPIO}
    for pin in pins.values():
        pin.export()

    print("ARMED: GPIO active. Keep wheels lifted. Ctrl-C stops and disables STBY.")

    try:
        stop_all(pins)
        time.sleep(0.5)
        pins[PIN_STBY].write(1)
        time.sleep(0.2)

        print("left forward")
        set_direction(pins, LEFT, forward=True, invert=args.left_invert)
        run_pwm(pins, LEFT, args.duty, args.duration, args.pwm_hz)
        stop_motor(pins, LEFT)
        time.sleep(0.7)

        print("left backward")
        set_direction(pins, LEFT, forward=False, invert=args.left_invert)
        run_pwm(pins, LEFT, args.duty, args.duration, args.pwm_hz)
        stop_motor(pins, LEFT)
        time.sleep(1.0)

        print("right forward")
        set_direction(pins, RIGHT, forward=True, invert=args.right_invert)
        run_pwm(pins, RIGHT, args.duty, args.duration, args.pwm_hz)
        stop_motor(pins, RIGHT)
        time.sleep(0.7)

        print("right backward")
        set_direction(pins, RIGHT, forward=False, invert=args.right_invert)
        run_pwm(pins, RIGHT, args.duty, args.duration, args.pwm_hz)
        stop_motor(pins, RIGHT)
        time.sleep(0.5)
    except KeyboardInterrupt:
        print("\nInterrupted, stopping motors.", file=sys.stderr)
    finally:
        stop_all(pins)
        print("Stopped. STBY disabled.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
