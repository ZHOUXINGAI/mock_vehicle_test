#!/usr/bin/env python3
"""Wheels-up paired motor drive test for Lubancat 4 + D153B/TB6612."""

from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path


SYSFS_GPIO = Path("/sys/class/gpio")

PIN_STBY = 124

LEFT_PWM = 62
LEFT_IN1 = 122
LEFT_IN2 = 113

RIGHT_PWM = 63
RIGHT_IN1 = 102
RIGHT_IN2 = 111

ALL_GPIO = [
    PIN_STBY,
    LEFT_PWM,
    LEFT_IN1,
    LEFT_IN2,
    RIGHT_PWM,
    RIGHT_IN1,
    RIGHT_IN2,
]


class GpioOut:
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
        description="Lubancat 4 D153B paired motor wheels-up drive test."
    )
    parser.add_argument("--armed", action="store_true", help="actually drive GPIO")
    parser.add_argument("--duty", type=float, default=0.20)
    parser.add_argument("--duration", type=float, default=0.8)
    parser.add_argument("--pwm-hz", type=float, default=100.0)
    parser.add_argument("--left-invert", action="store_true")
    parser.add_argument("--right-invert", action="store_true")
    return parser.parse_args()


def validate(args: argparse.Namespace) -> None:
    if not 0.05 <= args.duty <= 1.00:
        raise SystemExit("Refusing unsafe duty. Use 0.05 <= --duty <= 1.00.")
    if not 0.1 <= args.duration <= 2.0:
        raise SystemExit("Refusing unsafe duration. Use 0.1 <= --duration <= 2.0.")
    if not 10.0 <= args.pwm_hz <= 500.0:
        raise SystemExit("Use 10 <= --pwm-hz <= 500 for this sysfs test.")


def require_gpio_access() -> None:
    if not SYSFS_GPIO.exists():
        raise SystemExit("/sys/class/gpio does not exist.")
    if os.geteuid() != 0:
        raise SystemExit("GPIO sysfs writes usually need root. Re-run with sudo.")


def motor_direction(
    pins: dict[int, GpioOut],
    in1: int,
    in2: int,
    command: int,
    invert: bool,
) -> None:
    if command == 0:
        pins[in1].write(0)
        pins[in2].write(0)
        return

    forward = command > 0
    if invert:
        forward = not forward

    pins[in1].write(1 if forward else 0)
    pins[in2].write(0 if forward else 1)


def stop_all(pins: dict[int, GpioOut]) -> None:
    for number in [LEFT_PWM, RIGHT_PWM, LEFT_IN1, LEFT_IN2, RIGHT_IN1, RIGHT_IN2]:
        pins[number].write(0)
    pins[PIN_STBY].write(0)


def drive(
    pins: dict[int, GpioOut],
    left_cmd: int,
    right_cmd: int,
    args: argparse.Namespace,
) -> None:
    motor_direction(pins, LEFT_IN1, LEFT_IN2, left_cmd, args.left_invert)
    motor_direction(pins, RIGHT_IN1, RIGHT_IN2, right_cmd, args.right_invert)

    if args.duty >= 0.99:
        if left_cmd:
            pins[LEFT_PWM].write(1)
        if right_cmd:
            pins[RIGHT_PWM].write(1)
        time.sleep(args.duration)
        pins[LEFT_PWM].write(0)
        pins[RIGHT_PWM].write(0)
        return

    period = 1.0 / args.pwm_hz
    high_time = period * args.duty
    low_time = period - high_time
    deadline = time.monotonic() + args.duration

    while time.monotonic() < deadline:
        if left_cmd:
            pins[LEFT_PWM].write(1)
        if right_cmd:
            pins[RIGHT_PWM].write(1)
        time.sleep(high_time)
        pins[LEFT_PWM].write(0)
        pins[RIGHT_PWM].write(0)
        time.sleep(low_time)


def dry_run(args: argparse.Namespace) -> None:
    print("DRY RUN ONLY. Add --armed to drive GPIO pins.")
    print("Wheels must be lifted before real test.")
    print(f"duty={args.duty:.2f}, duration={args.duration:.1f}s")
    print("Sequence:")
    print("  both forward")
    print("  stop")
    print("  both backward")
    print("  stop")
    print("  pivot left")
    print("  stop")
    print("  pivot right")
    print("  stop")


def main() -> int:
    args = parse_args()
    validate(args)

    if not args.armed:
        dry_run(args)
        return 0

    require_gpio_access()
    pins = {number: GpioOut(number) for number in ALL_GPIO}
    for pin in pins.values():
        pin.export()

    print("ARMED: paired motor test. Keep wheels lifted. Ctrl-C stops motors.")

    try:
        stop_all(pins)
        time.sleep(0.5)
        pins[PIN_STBY].write(1)
        time.sleep(0.2)

        for label, left_cmd, right_cmd in [
            ("both forward", 1, 1),
            ("both backward", -1, -1),
            ("pivot left", -1, 1),
            ("pivot right", 1, -1),
        ]:
            print(label)
            drive(pins, left_cmd, right_cmd, args)
            pins[LEFT_PWM].write(0)
            pins[RIGHT_PWM].write(0)
            time.sleep(0.8)
    except KeyboardInterrupt:
        print("\nInterrupted, stopping motors.", file=sys.stderr)
    finally:
        stop_all(pins)
        print("Stopped. STBY disabled.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
