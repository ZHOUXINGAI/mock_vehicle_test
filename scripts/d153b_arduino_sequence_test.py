#!/usr/bin/env python3
"""Run a wheels-up D153B rover sequence through an Arduino serial bridge."""

from __future__ import annotations

import argparse
import os
import select
import sys
import termios
import time


ARDUINO_BY_ID_PREFIX = "/dev/serial/by-id/usb-Arduino"
DEFAULT_PORTS = [
    "/dev/ttyACM1",
    "/dev/ttyACM0",
    "/dev/ttyUSB0",
    "/dev/ttyUSB1",
]

# Current physical wiring calibration:
#   raw A forward  -> right motor backward
#   raw A backward -> right motor forward
#   raw B forward  -> left motor backward
#   raw B backward -> left motor forward
#
# The Arduino currently flashed on the board applies raw A/B mapping internally:
#   F = raw A forward  + raw B forward  -> physical backward
#   B = raw A backward + raw B backward -> physical forward
#   L/R already rotate in the expected physical direction.
SEQUENCE = [
    ("forward", "B"),
    ("backward", "F"),
    ("left", "L"),
    ("right", "R"),
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Forward/backward/left/right test via Arduino -> D153B."
    )
    parser.add_argument("--port", help="Arduino serial port; default auto-detect")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--pwm", type=int, default=160)
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--gap", type=float, default=0.7)
    parser.add_argument(
        "--armed",
        action="store_true",
        help="actually drive motors; without this the script only prints the plan",
    )
    return parser.parse_args()


def choose_port(requested: str | None) -> str:
    if requested:
        return requested

    by_id_dir = "/dev/serial/by-id"
    if os.path.isdir(by_id_dir):
        for name in sorted(os.listdir(by_id_dir)):
            path = os.path.join(by_id_dir, name)
            if path.startswith(ARDUINO_BY_ID_PREFIX):
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


def read_available(fd: int, timeout: float) -> str:
    chunks: list[bytes] = []
    deadline = time.time() + timeout
    while time.time() < deadline:
        wait = max(0.0, min(0.2, deadline - time.time()))
        readable, _, _ = select.select([fd], [], [], wait)
        if not readable:
            continue
        try:
            chunks.append(os.read(fd, 1024))
        except BlockingIOError:
            pass
    return b"".join(chunks).decode("ascii", errors="replace").strip()


def write_line(fd: int, line: str) -> str:
    os.write(fd, line.encode("ascii"))
    return read_available(fd, 0.8)


def main() -> int:
    args = parse_args()
    port = args.port or "(auto-detect when armed)"
    pwm = max(0, min(255, args.pwm))
    duration_ms = max(100, min(3000, int(args.duration * 1000)))

    print(f"Arduino port: {port}")
    sequence_text = ", ".join(
        f"{name}->{letter} {duration_ms}ms" for name, letter in SEQUENCE
    )
    print(f"Sequence: {sequence_text}")
    print(f"PWM: {pwm}")
    print("Safety: wheels must be lifted; D153B 12V should be enabled only when ready.")

    if not args.armed:
        print("DRY RUN ONLY. Add --armed to drive motors.")
        return 0

    port = choose_port(args.port)
    print(f"Using Arduino port: {port}")

    fd = open_serial(port, args.baud)
    try:
        # Opening Arduino UNO serial resets it; wait for bridge startup.
        time.sleep(2.2)
        boot = read_available(fd, 0.5)
        if boot:
            print(f"boot={boot}")

        for name, letter in SEQUENCE:
            cmd = f"{letter} {pwm} {duration_ms}\n"
            print(f"send {name}: {cmd.strip()}")
            reply = write_line(fd, cmd)
            print(f"reply={reply if reply else '<none>'}")
            time.sleep(args.duration + args.gap)

        stop_reply = write_line(fd, "S\n")
        print(f"stop={stop_reply if stop_reply else '<none>'}")
    except KeyboardInterrupt:
        try:
            os.write(fd, b"S\n")
        except Exception:
            pass
        print("\nInterrupted; sent stop.", file=sys.stderr)
        return 130
    finally:
        os.close(fd)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
