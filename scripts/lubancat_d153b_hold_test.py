#!/usr/bin/env python3
"""Hold one D153B/D153B motor channel on for wiring diagnosis."""

from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path


SYSFS_GPIO = Path("/sys/class/gpio")

PIN_STBY = 124
CHANNELS = {
    "left": {"pwm": 62, "in1": 122, "in2": 113},
    "right": {"pwm": 63, "in1": 102, "in2": 111},
}


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
    parser = argparse.ArgumentParser(description="Hold one D153B/D153B motor on.")
    parser.add_argument("--armed", action="store_true", help="actually drive GPIO")
    parser.add_argument("--channel", choices=sorted(CHANNELS), required=True)
    parser.add_argument("--reverse", action="store_true")
    parser.add_argument("--duration", type=float, default=3.0)
    return parser.parse_args()


def validate(args: argparse.Namespace) -> None:
    if not 0.2 <= args.duration <= 8.0:
        raise SystemExit("Use 0.2 <= --duration <= 8.0.")


def require_gpio_access() -> None:
    if not SYSFS_GPIO.exists():
        raise SystemExit("/sys/class/gpio does not exist.")
    if os.geteuid() != 0:
        raise SystemExit("Re-run with sudo.")


def stop_all(pins: dict[int, GpioOut]) -> None:
    for pin in pins.values():
        pin.write(0)


def main() -> int:
    args = parse_args()
    validate(args)
    ch = CHANNELS[args.channel]
    numbers = [PIN_STBY, ch["pwm"], ch["in1"], ch["in2"]]

    if not args.armed:
        print("DRY RUN ONLY. Add --armed to drive GPIO pins.")
        print(f"channel={args.channel}, reverse={args.reverse}")
        print(f"pins: STBY={PIN_STBY}, PWM={ch['pwm']}, IN1={ch['in1']}, IN2={ch['in2']}")
        print(f"Would hold motor for {args.duration:.1f}s at 100% PWM.")
        return 0

    require_gpio_access()
    pins = {number: GpioOut(number) for number in numbers}
    for pin in pins.values():
        pin.export()

    print(
        f"ARMED: holding {args.channel} motor for {args.duration:.1f}s. "
        "Ctrl-C stops."
    )

    try:
        stop_all(pins)
        time.sleep(0.3)
        pins[PIN_STBY].write(1)
        if args.reverse:
            pins[ch["in1"]].write(0)
            pins[ch["in2"]].write(1)
        else:
            pins[ch["in1"]].write(1)
            pins[ch["in2"]].write(0)
        pins[ch["pwm"]].write(1)
        time.sleep(args.duration)
    except KeyboardInterrupt:
        print("\nInterrupted.", file=sys.stderr)
    finally:
        stop_all(pins)
        print("Stopped.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
