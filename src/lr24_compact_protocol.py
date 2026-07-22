#!/usr/bin/env python3

"""Compact framed messages for LR24 rover coordination tests.

The protocol is intentionally tiny and dependency-free. It is for no-motion
link benchmarks first, then for bounded state/plan exchange once the docking
bridge is ready.
"""

from __future__ import annotations

import binascii
import enum
import math
import struct
from dataclasses import dataclass
from typing import Iterable


MAGIC = b"L2"
VERSION = 1
HEADER = struct.Struct("<2sBBB")
CRC = struct.Struct("<H")
U32_MASK = 0xFFFFFFFF
U32_HALF_RANGE = 0x80000000


class MessageType(enum.IntEnum):
    MINI_STATE = 1
    PLAN_COMMAND = 2
    ABORT = 3
    CORRIDOR_PLAN = 4
    FIELD_ORIGIN = 5
    PING = 10
    PONG = 11


class Role(enum.IntEnum):
    MINI = 1
    CARRIER = 2


class Phase(enum.IntEnum):
    HOLD = 0
    ORBIT = 1
    ARC_TO_CORRIDOR = 2
    TERMINAL = 3
    STOP = 4
    ABORT = 5


class AbortReason(enum.IntEnum):
    OPERATOR = 1
    LINK_STALE = 2
    STATE_INVALID = 3
    PLANNER_INVALID = 4
    FRONT_GAP_VIOLATION = 5
    LATERAL_ERROR = 6
    LOCAL_SAFETY = 7
    UNSPECIFIED = 255


class HealthFlag(enum.IntFlag):
    POSITION_VALID = 1 << 0
    VELOCITY_VALID = 1 << 1
    YAW_VALID = 1 << 2
    PX4_CONNECTED = 1 << 3
    RC_STOP_READY = 1 << 4
    EXECUTOR_READY = 1 << 5
    ORIGIN_VALID = 1 << 6


class PlanFlag(enum.IntFlag):
    CORRIDOR_VALID = 1 << 0
    ONE_ORBIT_COMPLETE = 1 << 1


MINI_STATE = struct.Struct("<BIIhhhhhhHH")
PLAN_COMMAND = struct.Struct("<HBBIIIhhHHHHH")
CORRIDOR_PLAN = struct.Struct("<HIIIhhhhHHIHHHHHH")
ABORT_MESSAGE = struct.Struct("<BBHIIH")
FIELD_ORIGIN = struct.Struct("<HIIiiiH")
PING = struct.Struct("<IQ")


def clamp_int(value: float, low: int, high: int) -> int:
    rounded = value if isinstance(value, int) else round(value)
    return max(low, min(high, int(rounded)))


def u32_forward_delta(start: int, end: int) -> int:
    """Return the forward uint32 distance from start to end."""

    return (int(end) - int(start)) & U32_MASK


def sequence_is_newer(candidate: int, previous: int) -> bool:
    """Compare wrapping uint32 sequence numbers using serial-number arithmetic."""

    delta = u32_forward_delta(previous, candidate)
    return 0 < delta < U32_HALF_RANGE


def validity_window_ms(timestamp_ms: int, valid_until_ms: int) -> int:
    """Return sender-declared TTL; clocks on different vehicles are not compared."""

    return u32_forward_delta(timestamp_ms, valid_until_ms)


def crc16(data: bytes) -> int:
    return binascii.crc_hqx(data, 0xFFFF) & 0xFFFF


def encode_frame(msg_type: MessageType | int, payload: bytes) -> bytes:
    if len(payload) > 255:
        raise ValueError("payload too large for compact LR24 frame")
    header = HEADER.pack(MAGIC, VERSION, int(msg_type), len(payload))
    body = header + payload
    return body + CRC.pack(crc16(body))


@dataclass(frozen=True)
class Frame:
    msg_type: MessageType
    payload: bytes


class FrameReader:
    def __init__(self) -> None:
        self._buf = bytearray()
        self.crc_errors = 0
        self.version_errors = 0
        self.length_errors = 0
        self.unknown_type_errors = 0

    def feed(self, data: bytes) -> list[Frame]:
        self._buf.extend(data)
        frames: list[Frame] = []

        while True:
            start = self._buf.find(MAGIC)
            if start < 0:
                # Keep a trailing first magic byte in case the serial read split
                # the two-byte magic marker.
                if self._buf.endswith(MAGIC[:1]):
                    self._buf[:] = MAGIC[:1]
                else:
                    self._buf.clear()
                break
            if start > 0:
                del self._buf[:start]
            if len(self._buf) < HEADER.size:
                break

            magic, version, msg_type_raw, payload_len = HEADER.unpack(
                self._buf[: HEADER.size]
            )
            if magic != MAGIC:
                del self._buf[0]
                continue
            if version != VERSION:
                self.version_errors += 1
                del self._buf[:2]
                continue

            total_len = HEADER.size + payload_len + CRC.size
            if len(self._buf) < total_len:
                break

            raw = bytes(self._buf[:total_len])
            del self._buf[:total_len]
            expected_crc = CRC.unpack(raw[-CRC.size :])[0]
            actual_crc = crc16(raw[: -CRC.size])
            if expected_crc != actual_crc:
                self.crc_errors += 1
                continue

            try:
                msg_type = MessageType(msg_type_raw)
            except ValueError:
                self.unknown_type_errors += 1
                continue
            if payload_len != expected_payload_size(msg_type):
                self.length_errors += 1
                continue
            frames.append(Frame(msg_type=msg_type, payload=raw[HEADER.size : -CRC.size]))

        return frames


@dataclass(frozen=True)
class MiniState:
    vehicle_id: int
    seq: int
    timestamp_ms: int
    x_m: float
    y_m: float
    vx_mps: float
    vy_mps: float
    yaw_rad: float
    omega_radps: float
    health: int = 0
    origin_id: int = 0

    def encode(self) -> bytes:
        return MINI_STATE.pack(
            clamp_int(self.vehicle_id, 0, 255),
            clamp_int(self.seq, 0, 0xFFFFFFFF),
            clamp_int(self.timestamp_ms, 0, 0xFFFFFFFF),
            clamp_int(self.x_m * 100.0, -32768, 32767),
            clamp_int(self.y_m * 100.0, -32768, 32767),
            clamp_int(self.vx_mps * 100.0, -32768, 32767),
            clamp_int(self.vy_mps * 100.0, -32768, 32767),
            clamp_int(self.yaw_rad * 18000.0 / 3.141592653589793, -32768, 32767),
            clamp_int(self.omega_radps * 18000.0 / 3.141592653589793, -32768, 32767),
            clamp_int(self.health, 0, 0xFFFF),
            clamp_int(self.origin_id, 0, 0xFFFF),
        )

    @staticmethod
    def decode(payload: bytes) -> "MiniState":
        (
            vehicle_id,
            seq,
            timestamp_ms,
            x_cm,
            y_cm,
            vx_cms,
            vy_cms,
            yaw_cdeg,
            omega_cdeg_s,
            health,
            origin_id,
        ) = MINI_STATE.unpack(payload)
        return MiniState(
            vehicle_id=vehicle_id,
            seq=seq,
            timestamp_ms=timestamp_ms,
            x_m=x_cm / 100.0,
            y_m=y_cm / 100.0,
            vx_mps=vx_cms / 100.0,
            vy_mps=vy_cms / 100.0,
            yaw_rad=yaw_cdeg * 3.141592653589793 / 18000.0,
            omega_radps=omega_cdeg_s * 3.141592653589793 / 18000.0,
            health=health,
            origin_id=origin_id,
        )


@dataclass(frozen=True)
class PlanCommand:
    plan_id: int
    role: Role
    phase: Phase
    seq: int
    timestamp_ms: int
    valid_until_ms: int
    v_mps: float
    omega_radps: float
    duration_ms: int
    distance_m: float
    max_speed_mps: float
    max_accel_mps2: float
    flags: int = 0

    @property
    def validity_ms(self) -> int:
        return validity_window_ms(self.timestamp_ms, self.valid_until_ms)

    def encode(self) -> bytes:
        return PLAN_COMMAND.pack(
            clamp_int(self.plan_id, 0, 0xFFFF),
            int(self.role),
            int(self.phase),
            clamp_int(self.seq, 0, 0xFFFFFFFF),
            clamp_int(self.timestamp_ms, 0, 0xFFFFFFFF),
            clamp_int(self.valid_until_ms, 0, 0xFFFFFFFF),
            clamp_int(self.v_mps * 100.0, -32768, 32767),
            clamp_int(self.omega_radps * 18000.0 / 3.141592653589793, -32768, 32767),
            clamp_int(self.duration_ms, 0, 0xFFFF),
            clamp_int(self.distance_m * 100.0, 0, 0xFFFF),
            clamp_int(self.max_speed_mps * 100.0, 0, 0xFFFF),
            clamp_int(self.max_accel_mps2 * 100.0, 0, 0xFFFF),
            clamp_int(self.flags, 0, 0xFFFF),
        )

    @staticmethod
    def decode(payload: bytes) -> "PlanCommand":
        (
            plan_id,
            role,
            phase,
            seq,
            timestamp_ms,
            valid_until_ms,
            v_cms,
            omega_cdeg_s,
            duration_ms,
            distance_cm,
            max_speed_cms,
            max_accel_cmps2,
            flags,
        ) = PLAN_COMMAND.unpack(payload)
        return PlanCommand(
            plan_id=plan_id,
            role=Role(role),
            phase=Phase(phase),
            seq=seq,
            timestamp_ms=timestamp_ms,
            valid_until_ms=valid_until_ms,
            v_mps=v_cms / 100.0,
            omega_radps=omega_cdeg_s * 3.141592653589793 / 18000.0,
            duration_ms=duration_ms,
            distance_m=distance_cm / 100.0,
            max_speed_mps=max_speed_cms / 100.0,
            max_accel_mps2=max_accel_cmps2 / 100.0,
            flags=flags,
        )


@dataclass(frozen=True)
class CorridorPlanCompact:
    plan_id: int
    seq: int
    timestamp_ms: int
    valid_until_ms: int
    rendezvous_x_m: float
    rendezvous_y_m: float
    tangent_dir_x: float
    tangent_dir_y: float
    corridor_length_m: float
    ahead_distance_m: float
    mini_arrival_delay_ms: int
    trigger_phase_rad: float
    mini_speed_mps: float
    carrier_max_speed_mps: float
    target_front_gap_m: float
    flags: int = 0
    origin_id: int = 0

    @property
    def validity_ms(self) -> int:
        return validity_window_ms(self.timestamp_ms, self.valid_until_ms)

    def encode(self) -> bytes:
        trigger_phase = self.trigger_phase_rad % (2.0 * math.pi)
        return CORRIDOR_PLAN.pack(
            clamp_int(self.plan_id, 0, 0xFFFF),
            clamp_int(self.seq, 0, 0xFFFFFFFF),
            clamp_int(self.timestamp_ms, 0, 0xFFFFFFFF),
            clamp_int(self.valid_until_ms, 0, 0xFFFFFFFF),
            clamp_int(self.rendezvous_x_m * 100.0, -32768, 32767),
            clamp_int(self.rendezvous_y_m * 100.0, -32768, 32767),
            clamp_int(self.tangent_dir_x * 10000.0, -32768, 32767),
            clamp_int(self.tangent_dir_y * 10000.0, -32768, 32767),
            clamp_int(self.corridor_length_m * 100.0, 0, 0xFFFF),
            clamp_int(self.ahead_distance_m * 100.0, 0, 0xFFFF),
            clamp_int(self.mini_arrival_delay_ms, 0, 0xFFFFFFFF),
            clamp_int(trigger_phase * 18000.0 / math.pi, 0, 0xFFFF),
            clamp_int(self.mini_speed_mps * 100.0, 0, 0xFFFF),
            clamp_int(self.carrier_max_speed_mps * 100.0, 0, 0xFFFF),
            clamp_int(self.target_front_gap_m * 100.0, 0, 0xFFFF),
            clamp_int(self.flags, 0, 0xFFFF),
            clamp_int(self.origin_id, 0, 0xFFFF),
        )

    @staticmethod
    def decode(payload: bytes) -> "CorridorPlanCompact":
        (
            plan_id,
            seq,
            timestamp_ms,
            valid_until_ms,
            rendezvous_x_cm,
            rendezvous_y_cm,
            tangent_dir_x_scaled,
            tangent_dir_y_scaled,
            corridor_length_cm,
            ahead_distance_cm,
            mini_arrival_delay_ms,
            trigger_phase_cdeg,
            mini_speed_cms,
            carrier_max_speed_cms,
            target_front_gap_cm,
            flags,
            origin_id,
        ) = CORRIDOR_PLAN.unpack(payload)
        return CorridorPlanCompact(
            plan_id=plan_id,
            seq=seq,
            timestamp_ms=timestamp_ms,
            valid_until_ms=valid_until_ms,
            rendezvous_x_m=rendezvous_x_cm / 100.0,
            rendezvous_y_m=rendezvous_y_cm / 100.0,
            tangent_dir_x=tangent_dir_x_scaled / 10000.0,
            tangent_dir_y=tangent_dir_y_scaled / 10000.0,
            corridor_length_m=corridor_length_cm / 100.0,
            ahead_distance_m=ahead_distance_cm / 100.0,
            mini_arrival_delay_ms=mini_arrival_delay_ms,
            trigger_phase_rad=trigger_phase_cdeg * math.pi / 18000.0,
            mini_speed_mps=mini_speed_cms / 100.0,
            carrier_max_speed_mps=carrier_max_speed_cms / 100.0,
            target_front_gap_m=target_front_gap_cm / 100.0,
            flags=flags,
            origin_id=origin_id,
        )


@dataclass(frozen=True)
class Abort:
    source_role: Role
    reason: AbortReason
    plan_id: int
    seq: int
    timestamp_ms: int
    flags: int = 0

    def encode(self) -> bytes:
        return ABORT_MESSAGE.pack(
            int(self.source_role),
            int(self.reason),
            clamp_int(self.plan_id, 0, 0xFFFF),
            clamp_int(self.seq, 0, U32_MASK),
            clamp_int(self.timestamp_ms, 0, U32_MASK),
            clamp_int(self.flags, 0, 0xFFFF),
        )

    @staticmethod
    def decode(payload: bytes) -> "Abort":
        source_role, reason, plan_id, seq, timestamp_ms, flags = ABORT_MESSAGE.unpack(
            payload
        )
        return Abort(
            source_role=Role(source_role),
            reason=AbortReason(reason),
            plan_id=plan_id,
            seq=seq,
            timestamp_ms=timestamp_ms,
            flags=flags,
        )


@dataclass(frozen=True)
class FieldOrigin:
    origin_id: int
    seq: int
    timestamp_ms: int
    latitude_deg: float
    longitude_deg: float
    altitude_m: float
    flags: int = 0

    def encode(self) -> bytes:
        return FIELD_ORIGIN.pack(
            clamp_int(self.origin_id, 0, 0xFFFF),
            clamp_int(self.seq, 0, U32_MASK),
            clamp_int(self.timestamp_ms, 0, U32_MASK),
            clamp_int(self.latitude_deg * 1.0e7, -900000000, 900000000),
            clamp_int(self.longitude_deg * 1.0e7, -1800000000, 1800000000),
            clamp_int(self.altitude_m * 1000.0, -2147483648, 2147483647),
            clamp_int(self.flags, 0, 0xFFFF),
        )

    @staticmethod
    def decode(payload: bytes) -> "FieldOrigin":
        origin_id, seq, timestamp_ms, lat_e7, lon_e7, alt_mm, flags = FIELD_ORIGIN.unpack(
            payload
        )
        return FieldOrigin(
            origin_id=origin_id,
            seq=seq,
            timestamp_ms=timestamp_ms,
            latitude_deg=lat_e7 / 1.0e7,
            longitude_deg=lon_e7 / 1.0e7,
            altitude_m=alt_mm / 1000.0,
            flags=flags,
        )


@dataclass(frozen=True)
class Ping:
    seq: int
    timestamp_ns: int

    def encode(self) -> bytes:
        return PING.pack(clamp_int(self.seq, 0, 0xFFFFFFFF), int(self.timestamp_ns))

    @staticmethod
    def decode(payload: bytes) -> "Ping":
        seq, timestamp_ns = PING.unpack(payload)
        return Ping(seq=seq, timestamp_ns=timestamp_ns)


def describe_frame(frame: Frame) -> str:
    if frame.msg_type == MessageType.MINI_STATE:
        msg = MiniState.decode(frame.payload)
        return (
            f"MINI_STATE seq={msg.seq} t_ms={msg.timestamp_ms} "
            f"pos=({msg.x_m:.2f},{msg.y_m:.2f}) "
            f"vel=({msg.vx_mps:.2f},{msg.vy_mps:.2f}) "
            f"yaw={msg.yaw_rad:.3f} omega={msg.omega_radps:.3f} health={msg.health}"
        )
    if frame.msg_type == MessageType.PLAN_COMMAND:
        msg = PlanCommand.decode(frame.payload)
        return (
            f"PLAN_COMMAND seq={msg.seq} plan={msg.plan_id} role={msg.role.name} "
            f"phase={msg.phase.name} v={msg.v_mps:.2f} omega={msg.omega_radps:.3f} "
            f"duration_ms={msg.duration_ms} distance={msg.distance_m:.2f} "
            f"valid_until={msg.valid_until_ms}"
        )
    if frame.msg_type == MessageType.CORRIDOR_PLAN:
        msg = CorridorPlanCompact.decode(frame.payload)
        return (
            f"CORRIDOR_PLAN seq={msg.seq} plan={msg.plan_id} "
            f"T=({msg.rendezvous_x_m:.2f},{msg.rendezvous_y_m:.2f}) "
            f"dir=({msg.tangent_dir_x:.3f},{msg.tangent_dir_y:.3f}) "
            f"arrival_delay_ms={msg.mini_arrival_delay_ms} "
            f"trigger={msg.trigger_phase_rad:.3f} "
            f"mini_v={msg.mini_speed_mps:.2f} carrier_max={msg.carrier_max_speed_mps:.2f} "
            f"origin={msg.origin_id}"
        )
    if frame.msg_type == MessageType.ABORT:
        msg = Abort.decode(frame.payload)
        return (
            f"ABORT seq={msg.seq} plan={msg.plan_id} "
            f"source={msg.source_role.name} reason={msg.reason.name}"
        )
    if frame.msg_type == MessageType.FIELD_ORIGIN:
        msg = FieldOrigin.decode(frame.payload)
        return (
            f"FIELD_ORIGIN seq={msg.seq} id={msg.origin_id} "
            f"lat={msg.latitude_deg:.7f} lon={msg.longitude_deg:.7f} "
            f"alt={msg.altitude_m:.3f}"
        )
    if frame.msg_type in (MessageType.PING, MessageType.PONG):
        msg = Ping.decode(frame.payload)
        return f"{frame.msg_type.name} seq={msg.seq} timestamp_ns={msg.timestamp_ns}"
    return f"{frame.msg_type.name} payload_len={len(frame.payload)}"


def frame_sizes() -> Iterable[tuple[str, int]]:
    yield "mini_state_payload", MINI_STATE.size
    yield "mini_state_frame", HEADER.size + MINI_STATE.size + CRC.size
    yield "plan_command_payload", PLAN_COMMAND.size
    yield "plan_command_frame", HEADER.size + PLAN_COMMAND.size + CRC.size
    yield "corridor_plan_payload", CORRIDOR_PLAN.size
    yield "corridor_plan_frame", HEADER.size + CORRIDOR_PLAN.size + CRC.size
    yield "abort_payload", ABORT_MESSAGE.size
    yield "abort_frame", HEADER.size + ABORT_MESSAGE.size + CRC.size
    yield "field_origin_payload", FIELD_ORIGIN.size
    yield "field_origin_frame", HEADER.size + FIELD_ORIGIN.size + CRC.size
    yield "ping_payload", PING.size
    yield "ping_frame", HEADER.size + PING.size + CRC.size


def expected_payload_size(msg_type: MessageType) -> int:
    sizes = {
        MessageType.MINI_STATE: MINI_STATE.size,
        MessageType.PLAN_COMMAND: PLAN_COMMAND.size,
        MessageType.ABORT: ABORT_MESSAGE.size,
        MessageType.CORRIDOR_PLAN: CORRIDOR_PLAN.size,
        MessageType.FIELD_ORIGIN: FIELD_ORIGIN.size,
        MessageType.PING: PING.size,
        MessageType.PONG: PING.size,
    }
    return sizes[msg_type]
