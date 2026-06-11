#!/usr/bin/env python3
"""Send simple drive commands from Lubancat to an Arduino serial bridge."""

from __future__ import annotations

import argparse
import os
import select
import sys
import termios
import time


DEFAULT_PORTS = [
    "/dev/ttyACM0",
    "/dev/ttyACM1",
    "/dev/ttyUSB0",
    "/dev/ttyUSB1",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Lubancat -> Arduino USB serial rover drive test."
    )
    parser.add_argument("--port", help="serial port, default auto-detect")
    parser.add_argument(
        "--mode",
        choices=[
            "forward",
            "backward",
            "left",
            "right",
            "left_forward",
            "left_backward",
            "right_forward",
            "right_backward",
            "stop",
        ],
        default="forward",
    )
    parser.add_argument("--pwm", type=int, default=220)
    parser.add_argument("--duration", type=float, default=1.0)
    parser.add_argument("--baud", type=int, default=115200)
    return parser.parse_args()


def choose_port(requested: str | None) -> str:
    if requested:
        return requested
    for port in DEFAULT_PORTS:
        if os.path.exists(port):
            return port
    raise SystemExit(
        "No Arduino serial port found. Try --port /dev/ttyACM0 or check USB."
    )


def command_for(mode: str, pwm: int, duration: float) -> str:
    letter = {
        "forward": "F",
        "backward": "B",
        "left": "L",
        "right": "R",
        "left_forward": "Q",
        "left_backward": "A",
        "right_forward": "E",
        "right_backward": "D",
        "stop": "S",
    }[mode]
    if letter == "S":
        return "S\n"
    pwm = max(0, min(255, pwm))
    duration_ms = max(100, min(3000, int(duration * 1000)))
    return f"{letter} {pwm} {duration_ms}\n"


def open_serial(port: str, baud: int) -> int:
    baud_map = {
        9600: termios.B9600,
        19200: termios.B19200,
        38400: termios.B38400,
        57600: termios.B57600,
        115200: termios.B115200,
    }
    if baud not in baud_map:
        raise SystemExit(f"Unsupported baud for termios path: {baud}")

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


def main() -> int:
    args = parse_args()
    port = choose_port(args.port)
    cmd = command_for(args.mode, args.pwm, args.duration)

    print(f"port={port}")
    print(f"send={cmd.strip()}")

    fd = open_serial(port, args.baud)
    try:
        # Opening the UNO serial port usually resets the board.
        time.sleep(2.2)
        boot = read_available(fd, 0.3)
        if boot:
            print(f"boot={boot}")

        os.write(fd, cmd.encode("ascii"))
        reply = read_available(fd, 0.8)
        print(f"reply={reply if reply else '<none>'}")

        if args.mode != "stop":
            time.sleep(args.duration + 0.3)
            os.write(fd, b"S\n")
            stop_reply = read_available(fd, 0.5)
            if stop_reply:
                print(f"stop_reply={stop_reply}")
    finally:
        os.close(fd)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
