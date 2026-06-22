#!/usr/bin/env python3

"""Read and write PX4 parameters over a direct MAVLink serial connection."""

from __future__ import annotations

import argparse
import struct
import sys
import time
from dataclasses import dataclass
from typing import Iterable

from pymavlink import mavutil


DEFAULT_DEVICE = "/dev/serial/by-id/usb-Auterion_PX4_FMU_v6C.x_0-if00"


INT_TYPES = {
    mavutil.mavlink.MAV_PARAM_TYPE_UINT8: (">xxxB", int),
    mavutil.mavlink.MAV_PARAM_TYPE_INT8: (">xxxb", int),
    mavutil.mavlink.MAV_PARAM_TYPE_UINT16: (">xxH", int),
    mavutil.mavlink.MAV_PARAM_TYPE_INT16: (">xxh", int),
    mavutil.mavlink.MAV_PARAM_TYPE_UINT32: (">I", int),
    mavutil.mavlink.MAV_PARAM_TYPE_INT32: (">i", int),
}


TYPE_NAMES = {
    mavutil.mavlink.MAV_PARAM_TYPE_UINT8: "uint8",
    mavutil.mavlink.MAV_PARAM_TYPE_INT8: "int8",
    mavutil.mavlink.MAV_PARAM_TYPE_UINT16: "uint16",
    mavutil.mavlink.MAV_PARAM_TYPE_INT16: "int16",
    mavutil.mavlink.MAV_PARAM_TYPE_UINT32: "uint32",
    mavutil.mavlink.MAV_PARAM_TYPE_INT32: "int32",
    mavutil.mavlink.MAV_PARAM_TYPE_REAL32: "float",
    mavutil.mavlink.MAV_PARAM_TYPE_REAL64: "double",
}


@dataclass(frozen=True)
class ParamValue:
    name: str
    value: float | int
    raw_value: float
    param_type: int
    index: int
    count: int


def param_name(raw: object) -> str:
    if isinstance(raw, bytes):
        return raw.decode("ascii", "ignore").rstrip("\x00").strip()
    return str(raw).rstrip("\x00").strip()


def decode_param_value(raw_value: float, param_type: int) -> float | int:
    if param_type in INT_TYPES:
        fmt, caster = INT_TYPES[param_type]
        packed = struct.pack(">f", float(raw_value))
        return caster(struct.unpack(fmt, packed)[0])
    return float(raw_value)


def encode_param_value(value: float | int, param_type: int) -> float:
    if param_type in INT_TYPES:
        fmt, _ = INT_TYPES[param_type]
        packed = struct.pack(fmt, int(value))
        return struct.unpack(">f", packed)[0]
    return float(value)


def parse_type(type_name: str) -> int:
    normalized = type_name.strip().lower()
    for param_type, name in TYPE_NAMES.items():
        if normalized == name:
            return param_type
    raise argparse.ArgumentTypeError(f"unsupported param type: {type_name}")


def wait_param(master: mavutil.mavfile, name: str, timeout: float) -> ParamValue | None:
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = master.recv_match(
            type="PARAM_VALUE",
            blocking=True,
            timeout=max(0.05, deadline - time.time()),
        )
        if msg is None:
            continue
        msg_name = param_name(msg.param_id)
        if msg_name == name:
            return ParamValue(
                name=msg_name,
                value=decode_param_value(msg.param_value, msg.param_type),
                raw_value=float(msg.param_value),
                param_type=int(msg.param_type),
                index=int(msg.param_index),
                count=int(msg.param_count),
            )
    return None


def fetch_param(
    master: mavutil.mavfile,
    name: str,
    retries: int,
    timeout: float,
) -> ParamValue:
    for _ in range(max(1, retries)):
        master.param_fetch_one(name)
        value = wait_param(master, name, timeout)
        if value is not None:
            return value
    raise TimeoutError(f"timeout reading {name}")


def set_param(
    master: mavutil.mavfile,
    name: str,
    value: float | int,
    param_type: int,
    retries: int,
    timeout: float,
) -> ParamValue:
    encoded = encode_param_value(value, param_type)
    for _ in range(max(1, retries)):
        master.mav.param_set_send(
            master.target_system,
            master.target_component,
            name.encode("ascii"),
            encoded,
            param_type,
        )
        ack = wait_param(master, name, timeout)
        if ack is None:
            continue
        if ack.value == value or (
            isinstance(ack.value, float) and abs(float(ack.value) - float(value)) < 1.0e-5
        ):
            return ack
    raise TimeoutError(f"timeout setting {name}={value}")


def print_values(values: Iterable[ParamValue]) -> None:
    for value in values:
        type_name = TYPE_NAMES.get(value.param_type, str(value.param_type))
        print(
            f"{value.name}={value.value} "
            f"type={type_name} index={value.index}/{value.count}"
        )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default=DEFAULT_DEVICE)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--source-system", type=int, default=246)
    parser.add_argument("--heartbeat-timeout", type=float, default=15.0)
    parser.add_argument("--timeout", type=float, default=2.5)
    parser.add_argument("--retries", type=int, default=3)
    subparsers = parser.add_subparsers(dest="command", required=True)

    read_parser = subparsers.add_parser("get", help="read one or more parameters")
    read_parser.add_argument("params", nargs="+")

    set_parser = subparsers.add_parser("set", help="set one parameter")
    set_parser.add_argument("param")
    set_parser.add_argument("value")
    set_parser.add_argument("--type", required=True, type=parse_type)

    return parser


def main() -> int:
    args = build_parser().parse_args()
    master = mavutil.mavlink_connection(
        args.device,
        baud=args.baud,
        source_system=args.source_system,
        autoreconnect=False,
    )
    heartbeat = master.wait_heartbeat(timeout=args.heartbeat_timeout)
    if heartbeat is None:
        raise TimeoutError("no Pixhawk heartbeat")

    if args.command == "get":
        values = [
            fetch_param(master, name, args.retries, args.timeout)
            for name in args.params
        ]
        print_values(values)
        return 0

    if args.command == "set":
        param_type = int(args.type)
        value: float | int
        if param_type in INT_TYPES:
            value = int(float(args.value))
        else:
            value = float(args.value)
        ack = set_param(master, args.param, value, param_type, args.retries, args.timeout)
        print_values([ack])
        return 0

    return 2


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        raise SystemExit(1)
