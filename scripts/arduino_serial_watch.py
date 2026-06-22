#!/usr/bin/env python3
"""Watch Arduino serial output for RC channel debugging."""

from __future__ import annotations

import argparse
import os
import select
import sys
import termios
import time


ARDUINO_BY_ID_PREFIXES = (
    "/dev/serial/by-id/usb-1a86_USB_Serial",
    "/dev/serial/by-id/usb-Arduino",
)
DEFAULT_PORTS = [
    "/dev/ttyUSB0",
    "/dev/ttyACM0",
    "/dev/ttyACM1",
    "/dev/ttyUSB1",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Print Arduino serial output.")
    parser.add_argument("--port", help="Arduino serial port; default auto-detect")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--duration", type=float, default=20.0)
    return parser.parse_args()


def choose_port(requested: str | None) -> str:
    if requested:
        return requested

    by_id_dir = "/dev/serial/by-id"
    if os.path.isdir(by_id_dir):
        for name in sorted(os.listdir(by_id_dir)):
            path = os.path.join(by_id_dir, name)
            if any(path.startswith(prefix) for prefix in ARDUINO_BY_ID_PREFIXES):
                return path

    for port in DEFAULT_PORTS:
        if os.path.exists(port):
            return port

    raise SystemExit("No Arduino serial port found.")


def open_serial(port: str, baud: int) -> int:
    baud_map = {
        9600: termios.B9600,
        19200: termios.B19200,
        38400: termios.B38400,
        57600: termios.B57600,
        115200: termios.B115200,
    }
    if baud not in baud_map:
        raise SystemExit(f"Unsupported baud: {baud}")

    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0
    attrs[1] = 0
    attrs[2] |= termios.CLOCAL | termios.CREAD
    attrs[2] &= ~termios.CSTOPB
    attrs[2] &= ~termios.PARENB
    attrs[3] = 0
    attrs[4] = baud_map[baud]
    attrs[5] = baud_map[baud]
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    return fd


def main() -> int:
    args = parse_args()
    port = choose_port(args.port)

    print(f"Watching Arduino serial: {port} @ {args.baud}")
    print("Opening serial resets Arduino UNO. Press Ctrl-C to stop.")

    fd = open_serial(port, args.baud)
    try:
        end_time = time.time() + args.duration
        while time.time() < end_time:
            readable, _, _ = select.select([fd], [], [], 0.2)
            if not readable:
                continue
            try:
                data = os.read(fd, 4096)
            except BlockingIOError:
                continue
            if data:
                sys.stdout.write(data.decode("ascii", errors="replace"))
                sys.stdout.flush()
    except KeyboardInterrupt:
        return 130
    finally:
        os.close(fd)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
