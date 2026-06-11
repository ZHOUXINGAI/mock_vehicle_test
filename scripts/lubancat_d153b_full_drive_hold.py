#!/usr/bin/env python3
"""Hold both D153B/D153B motor channels on together.

This is the closest sysfs GPIO equivalent to Arduino code setting both PWM pins
high continuously. Use only with the rover lifted or in a clear area.
"""

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
    parser = argparse.ArgumentParser(description="Hold both D153B motors together.")
    parser.add_argument("--armed", action="store_true", help="actually drive GPIO")
    parser.add_argument(
        "--mode",
        choices=["forward", "backward", "left", "right"],
        default="forward",
    )
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--left-invert", action="store_true")
    parser.add_argument("--right-invert", action="store_true")
    return parser.parse_args()


def validate(args: argparse.Namespace) -> None:
    if not 0.2 <= args.duration <= 10.0:
        raise SystemExit("Use 0.2 <= --duration <= 10.0.")


def require_gpio_access() -> None:
    if not SYSFS_GPIO.exists():
        raise SystemExit("/sys/class/gpio does not exist.")
    if os.geteuid() != 0:
        raise SystemExit("Re-run with sudo.")


def set_motor(
    pins: dict[int, GpioOut],
    pwm: int,
    in1: int,
    in2: int,
    command: int,
    invert: bool,
) -> None:
    if command == 0:
        pins[pwm].write(0)
        pins[in1].write(0)
        pins[in2].write(0)
        return

    forward = command > 0
    if invert:
        forward = not forward

    pins[in1].write(1 if forward else 0)
    pins[in2].write(0 if forward else 1)
    pins[pwm].write(1)


def stop_all(pins: dict[int, GpioOut]) -> None:
    for number in [LEFT_PWM, RIGHT_PWM, LEFT_IN1, LEFT_IN2, RIGHT_IN1, RIGHT_IN2]:
        pins[number].write(0)
    pins[PIN_STBY].write(0)


def mode_to_commands(mode: str) -> tuple[int, int]:
    if mode == "forward":
        return 1, 1
    if mode == "backward":
        return -1, -1
    if mode == "left":
        return -1, 1
    if mode == "right":
        return 1, -1
    raise ValueError(mode)


def dry_run(args: argparse.Namespace) -> None:
    left, right = mode_to_commands(args.mode)
    print("DRY RUN ONLY. Add --armed to drive GPIO pins.")
    print(f"mode={args.mode}, duration={args.duration:.1f}s")
    print(f"left_cmd={left}, right_cmd={right}")
    print("PWM pins will be held HIGH continuously.")


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

    left_cmd, right_cmd = mode_to_commands(args.mode)
    print(
        f"ARMED: {args.mode} for {args.duration:.1f}s at continuous full PWM. "
        "Ctrl-C stops."
    )

    try:
        stop_all(pins)
        time.sleep(0.3)
        pins[PIN_STBY].write(1)
        time.sleep(0.2)
        set_motor(
            pins,
            LEFT_PWM,
            LEFT_IN1,
            LEFT_IN2,
            left_cmd,
            args.left_invert,
        )
        set_motor(
            pins,
            RIGHT_PWM,
            RIGHT_IN1,
            RIGHT_IN2,
            right_cmd,
            args.right_invert,
        )
        time.sleep(args.duration)
    except KeyboardInterrupt:
        print("\nInterrupted.", file=sys.stderr)
    finally:
        stop_all(pins)
        print("Stopped.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
