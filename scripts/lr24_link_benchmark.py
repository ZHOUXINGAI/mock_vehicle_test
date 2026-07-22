#!/usr/bin/env python3

"""No-motion LR24 compact-link benchmark.

This tool only reads/writes the selected serial port. It does not arm PX4, does
not connect to MAVROS, and does not command motors.
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import select
import sys
import termios
import time
from pathlib import Path

REPO_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_DIR / "src"))

from lr24_compact_protocol import (  # noqa: E402
    Abort,
    AbortReason,
    CorridorPlanCompact,
    Frame,
    FrameReader,
    HealthFlag,
    MessageType,
    MiniState,
    Phase,
    Ping,
    PlanCommand,
    PlanFlag,
    Role,
    describe_frame,
    encode_frame,
    frame_sizes,
)


BAUD = {
    9600: termios.B9600,
    19200: termios.B19200,
    38400: termios.B38400,
    57600: termios.B57600,
    115200: termios.B115200,
    230400: termios.B230400,
    460800: termios.B460800,
    921600: termios.B921600,
}


def monotonic_ms() -> int:
    return int(time.monotonic() * 1000.0) & 0xFFFFFFFF


def monotonic_ns() -> int:
    return time.monotonic_ns()


def require_no_motion(args: argparse.Namespace) -> None:
    if not args.confirm_no_motion:
        raise SystemExit(
            "Refusing to run: pass --confirm-no-motion after motors are disabled "
            "or wheels are lifted. This tool is serial-only, but field workflow "
            "requires explicit no-motion confirmation."
        )


def open_serial(port: str, baud: int) -> int:
    if baud not in BAUD:
        raise SystemExit(f"Unsupported baud {baud}; supported: {sorted(BAUD)}")
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0
    attrs[1] = 0
    attrs[2] = BAUD[baud] | termios.CLOCAL | termios.CREAD | termios.CS8
    attrs[3] = 0
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 1
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    termios.tcflush(fd, termios.TCIOFLUSH)
    return fd


def read_frames(fd: int, reader: FrameReader, timeout_sec: float) -> list[Frame]:
    readable, _, _ = select.select([fd], [], [], timeout_sec)
    if not readable:
        return []
    try:
        data = os.read(fd, 4096)
    except BlockingIOError:
        return []
    if not data:
        return []
    return reader.feed(data)


def write_frame(fd: int, msg_type: MessageType, payload: bytes) -> None:
    os.write(fd, encode_frame(msg_type, payload))


def print_frame_sizes() -> None:
    print("Compact LR24 frame sizes:")
    for name, size in frame_sizes():
        print(f"  {name}: {size} bytes")


def role_echo(args: argparse.Namespace) -> int:
    require_no_motion(args)
    fd = open_serial(args.port, args.baud)
    reader = FrameReader()
    end = time.monotonic() + args.duration_sec if args.duration_sec > 0 else None
    count = 0
    print(f"echo listening on {args.port} baud={args.baud}")
    while end is None or time.monotonic() < end:
        for frame in read_frames(fd, reader, 0.2):
            count += 1
            print(f"rx {describe_frame(frame)}")
            if frame.msg_type == MessageType.PING:
                ping = Ping.decode(frame.payload)
                write_frame(fd, MessageType.PONG, ping.encode())
                print(f"tx PONG seq={ping.seq}")
    print(f"echo done frames_rx={count}")
    return 0


def role_ping(args: argparse.Namespace) -> int:
    require_no_motion(args)
    fd = open_serial(args.port, args.baud)
    reader = FrameReader()
    period = 1.0 / max(0.1, args.rate_hz)
    end = time.monotonic() + args.duration_sec
    next_tx = 0.0
    seq = 0
    sent: dict[int, int] = {}
    rows: list[dict[str, float | int]] = []
    print(f"ping on {args.port} baud={args.baud} rate={args.rate_hz}Hz duration={args.duration_sec}s")
    while time.monotonic() < end:
        now = time.monotonic()
        if now >= next_tx:
            pkt = Ping(seq=seq, timestamp_ns=monotonic_ns())
            write_frame(fd, MessageType.PING, pkt.encode())
            sent[seq] = pkt.timestamp_ns
            seq += 1
            next_tx = now + period
        for frame in read_frames(fd, reader, 0.02):
            if frame.msg_type != MessageType.PONG:
                print(f"rx {describe_frame(frame)}")
                continue
            pong = Ping.decode(frame.payload)
            tx_ns = sent.pop(pong.seq, None)
            if tx_ns is None:
                continue
            rtt_ms = (monotonic_ns() - tx_ns) / 1.0e6
            rows.append({"seq": pong.seq, "rtt_ms": rtt_ms})
            print(f"pong seq={pong.seq} rtt_ms={rtt_ms:.2f}")

    expected = seq
    received = len(rows)
    lost = max(0, expected - received)
    rtts = sorted(float(row["rtt_ms"]) for row in rows)
    p95 = rtts[int(0.95 * (len(rtts) - 1))] if rtts else float("nan")
    max_rtt = rtts[-1] if rtts else float("nan")
    mean = sum(rtts) / len(rtts) if rtts else float("nan")
    print(
        f"summary sent={expected} received={received} lost={lost} "
        f"loss_pct={(lost / expected * 100.0) if expected else 0.0:.1f} "
        f"mean_rtt_ms={mean:.2f} p95_rtt_ms={p95:.2f} max_rtt_ms={max_rtt:.2f}"
    )
    if args.csv:
        with open(args.csv, "w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=["seq", "rtt_ms"])
            writer.writeheader()
            writer.writerows(rows)
        print(f"wrote {args.csv}")
    return 0


def role_state_tx(args: argparse.Namespace) -> int:
    require_no_motion(args)
    fd = open_serial(args.port, args.baud)
    period = 1.0 / max(0.1, args.rate_hz)
    end = time.monotonic() + args.duration_sec if args.duration_sec > 0 else None
    seq = 0
    next_tx = 0.0
    print(f"state-tx on {args.port} rate={args.rate_hz}Hz simulate_orbit={args.simulate_orbit}")
    while end is None or time.monotonic() < end:
        now = time.monotonic()
        if now < next_tx:
            time.sleep(min(0.02, next_tx - now))
            continue
        t = seq * period
        if args.simulate_orbit:
            omega = args.speed_mps / max(0.01, args.radius_m)
            phase = args.phase_rad + omega * t
            x = args.radius_m * math.cos(phase)
            y = args.radius_m * math.sin(phase)
            vx = -args.speed_mps * math.sin(phase)
            vy = args.speed_mps * math.cos(phase)
            yaw = phase + math.pi / 2.0
        else:
            x, y, vx, vy, yaw, omega = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        state = MiniState(
            vehicle_id=args.vehicle_id,
            seq=seq,
            timestamp_ms=monotonic_ms(),
            x_m=x,
            y_m=y,
            vx_mps=vx,
            vy_mps=vy,
            yaw_rad=yaw,
            omega_radps=omega,
            health=int(
                HealthFlag.POSITION_VALID
                | HealthFlag.VELOCITY_VALID
                | HealthFlag.YAW_VALID
                | HealthFlag.ORIGIN_VALID
            ),
            origin_id=args.origin_id,
        )
        write_frame(fd, MessageType.MINI_STATE, state.encode())
        print(f"tx MINI_STATE seq={seq}")
        seq += 1
        next_tx = now + period
    print(f"state-tx done sent={seq}")
    return 0


def role_state_rx(args: argparse.Namespace) -> int:
    require_no_motion(args)
    fd = open_serial(args.port, args.baud)
    reader = FrameReader()
    end = time.monotonic() + args.duration_sec if args.duration_sec > 0 else None
    last_seq: int | None = None
    last_rx: float | None = None
    count = 0
    gaps = 0
    intervals: list[float] = []
    print(f"state-rx on {args.port}")
    while end is None or time.monotonic() < end:
        for frame in read_frames(fd, reader, 0.2):
            if frame.msg_type != MessageType.MINI_STATE:
                print(f"rx {describe_frame(frame)}")
                continue
            now = time.monotonic()
            msg = MiniState.decode(frame.payload)
            if last_seq is not None and msg.seq != last_seq + 1:
                gaps += max(0, msg.seq - last_seq - 1)
            if last_rx is not None:
                intervals.append((now - last_rx) * 1000.0)
            last_rx = now
            last_seq = msg.seq
            count += 1
            print(f"rx {describe_frame(frame)}")
    mean_interval = sum(intervals) / len(intervals) if intervals else float("nan")
    max_interval = max(intervals) if intervals else float("nan")
    print(
        f"summary received={count} seq_gaps={gaps} "
        f"mean_interval_ms={mean_interval:.2f} max_interval_ms={max_interval:.2f}"
    )
    return 0


def role_command_tx(args: argparse.Namespace) -> int:
    require_no_motion(args)
    fd = open_serial(args.port, args.baud)
    phase = Phase[args.phase.upper()]
    role = Role[args.role.upper()]
    period = 1.0 / max(0.1, args.rate_hz)
    end = time.monotonic() + args.duration_sec
    seq = 0
    next_tx = 0.0
    print(f"command-tx on {args.port} phase={phase.name} role={role.name}")
    while time.monotonic() < end:
        now = time.monotonic()
        if now < next_tx:
            time.sleep(min(0.02, next_tx - now))
            continue
        stamp = monotonic_ms()
        cmd = PlanCommand(
            plan_id=args.plan_id,
            role=role,
            phase=phase,
            seq=seq,
            timestamp_ms=stamp,
            valid_until_ms=(stamp + args.valid_for_ms) & 0xFFFFFFFF,
            v_mps=args.v_mps,
            omega_radps=args.omega_radps,
            duration_ms=args.command_duration_ms,
            distance_m=args.distance_m,
            max_speed_mps=args.max_speed_mps,
            max_accel_mps2=args.max_accel_mps2,
            flags=int(PlanFlag.CORRIDOR_VALID),
            origin_id=args.origin_id,
        )
        write_frame(fd, MessageType.PLAN_COMMAND, cmd.encode())
        print(f"tx PLAN_COMMAND seq={seq} phase={phase.name}")
        seq += 1
        next_tx = now + period
    print(f"command-tx done sent={seq}")
    return 0


def role_command_rx(args: argparse.Namespace) -> int:
    require_no_motion(args)
    fd = open_serial(args.port, args.baud)
    reader = FrameReader()
    end = time.monotonic() + args.duration_sec if args.duration_sec > 0 else None
    count = 0
    print(f"command-rx on {args.port}; this does not execute commands")
    while end is None or time.monotonic() < end:
        for frame in read_frames(fd, reader, 0.2):
            count += 1
            print(f"rx {describe_frame(frame)}")
    print(f"command-rx done frames_rx={count}")
    return 0


def role_corridor_plan_tx(args: argparse.Namespace) -> int:
    require_no_motion(args)
    fd = open_serial(args.port, args.baud)
    period = 1.0 / max(0.1, args.rate_hz)
    end = time.monotonic() + args.duration_sec
    seq = 0
    next_tx = 0.0
    print(
        f"corridor-plan-tx on {args.port} plan={args.plan_id} "
        f"T=({args.rendezvous_x_m:.2f},{args.rendezvous_y_m:.2f})"
    )
    while time.monotonic() < end:
        now = time.monotonic()
        if now < next_tx:
            time.sleep(min(0.02, next_tx - now))
            continue
        stamp = monotonic_ms()
        plan = CorridorPlanCompact(
            plan_id=args.plan_id,
            seq=seq,
            timestamp_ms=stamp,
            valid_until_ms=(stamp + args.valid_for_ms) & 0xFFFFFFFF,
            rendezvous_x_m=args.rendezvous_x_m,
            rendezvous_y_m=args.rendezvous_y_m,
            tangent_dir_x=args.tangent_dir_x,
            tangent_dir_y=args.tangent_dir_y,
            corridor_length_m=args.corridor_length_m,
            ahead_distance_m=args.ahead_distance_m,
            mini_arrival_delay_ms=args.mini_arrival_delay_ms,
            trigger_phase_rad=args.trigger_phase_rad,
            mini_speed_mps=args.mini_speed_mps,
            carrier_max_speed_mps=args.carrier_max_speed_mps,
            target_front_gap_m=args.target_front_gap_m,
            flags=int(PlanFlag.CORRIDOR_VALID),
            origin_id=args.origin_id,
        )
        write_frame(fd, MessageType.CORRIDOR_PLAN, plan.encode())
        print(f"tx CORRIDOR_PLAN seq={seq}")
        seq += 1
        next_tx = now + period
    print(f"corridor-plan-tx done sent={seq}")
    return 0


def role_corridor_plan_rx(args: argparse.Namespace) -> int:
    require_no_motion(args)
    fd = open_serial(args.port, args.baud)
    reader = FrameReader()
    end = time.monotonic() + args.duration_sec if args.duration_sec > 0 else None
    count = 0
    gaps = 0
    last_seq: int | None = None
    print(f"corridor-plan-rx on {args.port}; this does not execute commands")
    while end is None or time.monotonic() < end:
        for frame in read_frames(fd, reader, 0.2):
            if frame.msg_type != MessageType.CORRIDOR_PLAN:
                print(f"rx {describe_frame(frame)}")
                continue
            msg = CorridorPlanCompact.decode(frame.payload)
            if last_seq is not None and msg.seq != last_seq + 1:
                gaps += max(0, msg.seq - last_seq - 1)
            last_seq = msg.seq
            count += 1
            print(f"rx {describe_frame(frame)}")
    print(f"corridor-plan-rx done received={count} seq_gaps={gaps}")
    return 0


def role_abort_tx(args: argparse.Namespace) -> int:
    require_no_motion(args)
    fd = open_serial(args.port, args.baud)
    period = 1.0 / max(1.0, args.rate_hz)
    end = time.monotonic() + args.duration_sec
    seq = 0
    source_role = Role[args.source_role.upper()]
    reason = AbortReason[args.reason.upper()]
    print(
        f"abort-tx on {args.port} source={source_role.name} "
        f"reason={reason.name} rate={args.rate_hz}Hz"
    )
    while time.monotonic() < end:
        msg = Abort(
            source_role=source_role,
            reason=reason,
            plan_id=args.plan_id,
            seq=seq,
            timestamp_ms=monotonic_ms(),
        )
        write_frame(fd, MessageType.ABORT, msg.encode())
        print(f"tx ABORT seq={seq} reason={reason.name}")
        seq += 1
        time.sleep(period)
    print(f"abort-tx done sent={seq}")
    return 0


def role_abort_rx(args: argparse.Namespace) -> int:
    require_no_motion(args)
    fd = open_serial(args.port, args.baud)
    reader = FrameReader()
    end = time.monotonic() + args.duration_sec if args.duration_sec > 0 else None
    count = 0
    print(f"abort-rx on {args.port}; this does not execute commands")
    while end is None or time.monotonic() < end:
        for frame in read_frames(fd, reader, 0.2):
            if frame.msg_type != MessageType.ABORT:
                print(f"rx {describe_frame(frame)}")
                continue
            count += 1
            print(f"rx {describe_frame(frame)}")
    print(f"abort-rx done received={count}")
    return 0


def role_sizes(args: argparse.Namespace) -> int:
    del args
    print_frame_sizes()
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--print-frame-sizes", action="store_true")
    sub = parser.add_subparsers(dest="mode", required=True)

    p = sub.add_parser("sizes", help="Print compact frame sizes and exit.")
    p.set_defaults(func=role_sizes)

    def add_common(p: argparse.ArgumentParser) -> None:
        p.add_argument("--port", required=True)
        p.add_argument("--baud", type=int, default=57600)
        p.add_argument("--confirm-no-motion", action="store_true")
        p.add_argument("--duration-sec", type=float, default=30.0)

    p = sub.add_parser("echo", help="Reply to PING frames with PONG frames.")
    add_common(p)
    p.set_defaults(func=role_echo)

    p = sub.add_parser("ping", help="Send PING frames and measure round-trip time.")
    add_common(p)
    p.add_argument("--rate-hz", type=float, default=5.0)
    p.add_argument("--csv")
    p.set_defaults(func=role_ping)

    p = sub.add_parser("state-tx", help="Transmit compact MiniState frames.")
    add_common(p)
    p.add_argument("--rate-hz", type=float, default=5.0)
    p.add_argument("--vehicle-id", type=int, default=2)
    p.add_argument("--simulate-orbit", action="store_true")
    p.add_argument("--radius-m", type=float, default=4.5)
    p.add_argument("--speed-mps", type=float, default=0.9)
    p.add_argument("--phase-rad", type=float, default=0.0)
    p.add_argument("--origin-id", type=int, default=1)
    p.set_defaults(func=role_state_tx)

    p = sub.add_parser("state-rx", help="Receive compact MiniState frames.")
    add_common(p)
    p.set_defaults(func=role_state_rx)

    p = sub.add_parser("command-tx", help="Transmit compact PlanCommand frames.")
    add_common(p)
    p.add_argument("--rate-hz", type=float, default=1.0)
    p.add_argument("--plan-id", type=int, default=1)
    p.add_argument("--role", choices=["mini", "carrier"], default="mini")
    p.add_argument(
        "--phase",
        choices=["hold", "orbit", "arc_to_corridor", "terminal", "stop", "abort"],
        default="hold",
    )
    p.add_argument("--v-mps", type=float, default=0.0)
    p.add_argument("--omega-radps", type=float, default=0.0)
    p.add_argument("--command-duration-ms", type=int, default=1000)
    p.add_argument("--distance-m", type=float, default=0.0)
    p.add_argument("--max-speed-mps", type=float, default=0.0)
    p.add_argument("--max-accel-mps2", type=float, default=0.0)
    p.add_argument("--valid-for-ms", type=int, default=500)
    p.set_defaults(func=role_command_tx)

    p = sub.add_parser("command-rx", help="Receive compact PlanCommand frames.")
    add_common(p)
    p.set_defaults(func=role_command_rx)

    p = sub.add_parser("corridor-plan-tx", help="Transmit compact CorridorPlan frames.")
    add_common(p)
    p.add_argument("--rate-hz", type=float, default=1.0)
    p.add_argument("--plan-id", type=int, default=1)
    p.add_argument("--valid-for-ms", type=int, default=30000)
    p.add_argument("--rendezvous-x-m", type=float, default=-1.5526)
    p.add_argument("--rendezvous-y-m", type=float, default=-4.2237)
    p.add_argument("--tangent-dir-x", type=float, default=0.9386)
    p.add_argument("--tangent-dir-y", type=float, default=-0.3450)
    p.add_argument("--corridor-length-m", type=float, default=8.214)
    p.add_argument("--ahead-distance-m", type=float, default=0.35)
    p.add_argument("--mini-arrival-delay-ms", type=int, default=25724)
    p.add_argument("--trigger-phase-rad", type=float, default=4.360)
    p.add_argument("--mini-speed-mps", type=float, default=0.9)
    p.add_argument("--carrier-max-speed-mps", type=float, default=0.7)
    p.add_argument("--target-front-gap-m", type=float, default=0.35)
    p.add_argument("--origin-id", type=int, default=1)
    p.set_defaults(func=role_corridor_plan_tx)

    p = sub.add_parser("corridor-plan-rx", help="Receive compact CorridorPlan frames.")
    add_common(p)
    p.set_defaults(func=role_corridor_plan_rx)

    p = sub.add_parser("abort-tx", help="Transmit a repeated, idempotent Abort frame.")
    add_common(p)
    p.add_argument("--rate-hz", type=float, default=10.0)
    p.add_argument("--source-role", choices=["carrier", "mini"], default="carrier")
    p.add_argument(
        "--reason",
        choices=[
            "operator",
            "link_stale",
            "state_invalid",
            "planner_invalid",
            "front_gap_violation",
            "lateral_error",
            "local_safety",
            "unspecified",
        ],
        default="operator",
    )
    p.add_argument("--plan-id", type=int, default=0)
    p.set_defaults(func=role_abort_tx)

    p = sub.add_parser("abort-rx", help="Receive Abort frames without executing them.")
    add_common(p)
    p.set_defaults(func=role_abort_rx)

    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    if args.print_frame_sizes:
        print_frame_sizes()
    return int(args.func(args))


if __name__ == "__main__":
    raise SystemExit(main())
