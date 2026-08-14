#!/usr/bin/env python3

"""Strict structural adapters from EasyDocking ground plans to Pair B.

This module deliberately does not import EasyDocking, ROS, MAVLink, or a
vehicle executor. Callers may pass the real EasyDocking dataclasses or a
schema-shaped mapping with the same fields.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import Any, Mapping

try:
    from src.lr24_compact_protocol import (
        PLAN_SCHEMA_VERSION,
        U32_MASK,
        CorridorPlanCompact,
        Phase,
        PlanCommand,
        PlanFlag,
        Role,
        validity_window_ms,
    )
except ModuleNotFoundError:  # Direct execution with src/ on PYTHONPATH.
    from lr24_compact_protocol import (
        PLAN_SCHEMA_VERSION,
        U32_MASK,
        CorridorPlanCompact,
        Phase,
        PlanCommand,
        PlanFlag,
        Role,
        validity_window_ms,
    )


GROUND_COMMAND_SCHEMA_VERSION = 1
DEFAULT_MAX_PLAN_TTL_MS = 120_000
DEFAULT_MAX_COMMAND_TTL_MS = 2_000


class PairBAdapterError(ValueError):
    """Raised when source data cannot be represented without changing it."""


@dataclass(frozen=True)
class QuantizationTolerance:
    linear_m: float = 0.005000001
    direction: float = 0.000050001
    angle_rad: float = math.pi / 36_000.0 + 1.0e-12
    speed_mps: float = 0.005000001
    accel_mps2: float = 0.005000001

    def __post_init__(self) -> None:
        values = (
            self.linear_m,
            self.direction,
            self.angle_rad,
            self.speed_mps,
            self.accel_mps2,
        )
        if any(not math.isfinite(value) or value < 0.0 for value in values):
            raise ValueError("quantization tolerances must be finite and nonnegative")


DEFAULT_QUANTIZATION_TOLERANCE = QuantizationTolerance()


PHASE_MAP = {
    "HOLD": Phase.HOLD,
    "ORBIT": Phase.ORBIT,
    "ARC": Phase.ARC_TO_CORRIDOR,
    "TERMINAL": Phase.TERMINAL,
    "STOP": Phase.STOP,
    "ABORT": Phase.ABORT,
}

ROLE_MAP = {
    "mini": Role.MINI,
    "carrier": Role.CARRIER,
}


def _field(source: object, name: str) -> Any:
    if isinstance(source, Mapping):
        if name not in source:
            raise PairBAdapterError(f"missing_field:{name}")
        return source[name]
    try:
        return getattr(source, name)
    except AttributeError as exc:
        raise PairBAdapterError(f"missing_field:{name}") from exc


def _integer(source: object, name: str) -> int:
    value = _field(source, name)
    if isinstance(value, bool) or not isinstance(value, int):
        raise PairBAdapterError(f"invalid_integer:{name}")
    return value


def _uint(source: object, name: str, maximum: int, *, positive: bool = False) -> int:
    value = _integer(source, name)
    minimum = 1 if positive else 0
    if not minimum <= value <= maximum:
        raise PairBAdapterError(f"wire_range:{name}")
    return value


def _finite(source: object, name: str) -> float:
    value = _field(source, name)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise PairBAdapterError(f"invalid_number:{name}")
    result = float(value)
    if not math.isfinite(result):
        raise PairBAdapterError(f"nonfinite:{name}")
    return result


def _vec2(source: object, name: str) -> tuple[float, float]:
    value = _field(source, name)
    if not isinstance(value, (tuple, list)) or len(value) != 2:
        raise PairBAdapterError(f"invalid_vec2:{name}")
    result: list[float] = []
    for index, component in enumerate(value):
        if isinstance(component, bool) or not isinstance(component, (int, float)):
            raise PairBAdapterError(f"invalid_number:{name}[{index}]")
        number = float(component)
        if not math.isfinite(number):
            raise PairBAdapterError(f"nonfinite:{name}[{index}]")
        result.append(number)
    return (result[0], result[1])


def _enum_text(value: object) -> str:
    if isinstance(value, Enum):
        value = value.value
    if not isinstance(value, str):
        raise PairBAdapterError("invalid_enum_value")
    return value


def _check_quantized(
    name: str,
    value: float,
    *,
    scale: float,
    minimum: int,
    maximum: int,
    tolerance: float,
) -> None:
    scaled = round(value * scale)
    if not minimum <= scaled <= maximum:
        raise PairBAdapterError(f"wire_range:{name}")
    decoded = scaled / scale
    if abs(decoded - value) > tolerance:
        raise PairBAdapterError(f"quantization_tolerance:{name}")


def _check_identity(
    source: object,
    *,
    expected_frame_id: str,
    expected_origin_id: int,
) -> int:
    if not expected_frame_id:
        raise PairBAdapterError("invalid_expected_frame")
    if not 0 < expected_origin_id <= 0xFFFF:
        raise PairBAdapterError("invalid_expected_origin")
    frame_id = _field(source, "frame_id")
    if frame_id != expected_frame_id:
        raise PairBAdapterError("frame_id_mismatch")
    origin_id = _uint(source, "origin_id", 0xFFFF, positive=True)
    if origin_id != expected_origin_id:
        raise PairBAdapterError("origin_id_mismatch")
    return origin_id


def adapt_ground_corridor_plan(
    plan: object,
    *,
    expected_frame_id: str,
    expected_origin_id: int,
    one_orbit_complete: bool = False,
    tolerance: QuantizationTolerance = DEFAULT_QUANTIZATION_TOLERANCE,
    max_plan_ttl_ms: int = DEFAULT_MAX_PLAN_TTL_MS,
) -> CorridorPlanCompact:
    """Map one immutable GroundCorridorPlan to schema-2 Pair B bytes.

    The legacy wire ``ahead_distance_m`` field is populated from the same
    ``target_front_gap_m`` metadata because GroundCorridorPlan has one signed
    terminal front-gap target, not a second ahead-distance contract.
    """

    if not isinstance(one_orbit_complete, bool):
        raise PairBAdapterError("invalid_one_orbit_complete")
    if _integer(plan, "schema_version") != PLAN_SCHEMA_VERSION:
        raise PairBAdapterError("unsupported_plan_schema")
    origin_id = _check_identity(
        plan,
        expected_frame_id=expected_frame_id,
        expected_origin_id=expected_origin_id,
    )
    if _field(plan, "validity_policy") != "reject":
        raise PairBAdapterError("unsupported_validity_policy")
    if _field(plan, "validity_extended") is not False:
        raise PairBAdapterError("silently_extended_plan")

    plan_id = _uint(plan, "plan_id", 0xFFFF, positive=True)
    sequence = _uint(plan, "sequence", U32_MASK, positive=True)
    timestamp_ms = _uint(plan, "sender_monotonic_ms", U32_MASK)
    valid_until_ms = _uint(
        plan, "valid_until_sender_monotonic_ms", U32_MASK
    )
    validity_ms = _uint(plan, "validity_ms", U32_MASK, positive=True)
    if validity_window_ms(timestamp_ms, valid_until_ms) != validity_ms:
        raise PairBAdapterError("plan_ttl_mismatch")
    if validity_ms > max_plan_ttl_ms:
        raise PairBAdapterError("plan_ttl_exceeds_pairb_policy")
    requested_validity_ms = _uint(
        plan, "requested_validity_ms", U32_MASK, positive=True
    )
    required_validity_ms = _uint(
        plan, "required_validity_ms", U32_MASK, positive=True
    )
    validity_margin_ms = _integer(plan, "validity_margin_ms")
    if requested_validity_ms != validity_ms:
        raise PairBAdapterError("requested_validity_mismatch")
    if validity_margin_ms != validity_ms - required_validity_ms:
        raise PairBAdapterError("validity_margin_mismatch")

    tangent_point = _vec2(plan, "tangent_point")
    tangent_direction = _vec2(plan, "tangent_direction")
    tangent_phase_rad = _finite(plan, "tangent_phase_rad")
    terminal_length_m = _finite(plan, "terminal_length_m")
    target_front_gap_m = _finite(plan, "target_front_gap_m")
    mini_speed_mps = _finite(plan, "mini_speed_mps")
    carrier_max_speed_mps = _finite(plan, "carrier_max_speed_mps")
    if terminal_length_m <= 0.0:
        raise PairBAdapterError("invalid_terminal_length")
    if target_front_gap_m < 0.0:
        raise PairBAdapterError("invalid_target_front_gap")
    if mini_speed_mps <= 0.0 or carrier_max_speed_mps <= 0.0:
        raise PairBAdapterError("invalid_plan_speed")
    if not 0.999 <= math.hypot(*tangent_direction) <= 1.001:
        raise PairBAdapterError("invalid_tangent_norm")
    if not 0.0 <= tangent_phase_rad < 2.0 * math.pi:
        raise PairBAdapterError("tangent_phase_not_normalized")

    for index, value in enumerate(tangent_point):
        _check_quantized(
            f"tangent_point[{index}]",
            value,
            scale=100.0,
            minimum=-32768,
            maximum=32767,
            tolerance=tolerance.linear_m,
        )
    for index, value in enumerate(tangent_direction):
        _check_quantized(
            f"tangent_direction[{index}]",
            value,
            scale=10_000.0,
            minimum=-32768,
            maximum=32767,
            tolerance=tolerance.direction,
        )
    _check_quantized(
        "tangent_phase_rad",
        tangent_phase_rad,
        scale=18_000.0 / math.pi,
        minimum=0,
        maximum=0xFFFF,
        tolerance=tolerance.angle_rad,
    )
    for name, value in (
        ("terminal_length_m", terminal_length_m),
        ("target_front_gap_m", target_front_gap_m),
    ):
        _check_quantized(
            name,
            value,
            scale=100.0,
            minimum=0,
            maximum=0xFFFF,
            tolerance=tolerance.linear_m,
        )
    for name, value in (
        ("mini_speed_mps", mini_speed_mps),
        ("carrier_max_speed_mps", carrier_max_speed_mps),
    ):
        _check_quantized(
            name,
            value,
            scale=100.0,
            minimum=0,
            maximum=0xFFFF,
            tolerance=tolerance.speed_mps,
        )

    flags = int(PlanFlag.CORRIDOR_VALID)
    if one_orbit_complete:
        flags |= int(PlanFlag.ONE_ORBIT_COMPLETE)
    compact = CorridorPlanCompact(
        plan_schema_version=PLAN_SCHEMA_VERSION,
        plan_id=plan_id,
        seq=sequence,
        timestamp_ms=timestamp_ms,
        valid_until_ms=valid_until_ms,
        rendezvous_x_m=tangent_point[0],
        rendezvous_y_m=tangent_point[1],
        tangent_dir_x=tangent_direction[0],
        tangent_dir_y=tangent_direction[1],
        corridor_length_m=terminal_length_m,
        ahead_distance_m=target_front_gap_m,
        mini_arrival_delay_ms=_uint(
            plan, "mini_arrival_delay_ms", U32_MASK, positive=True
        ),
        trigger_phase_rad=tangent_phase_rad,
        mini_speed_mps=mini_speed_mps,
        carrier_max_speed_mps=carrier_max_speed_mps,
        target_front_gap_m=target_front_gap_m,
        required_validity_ms=required_validity_ms,
        post_tangent_reserve_ms=_uint(
            plan, "post_tangent_reserve_ms", 0xFFFF, positive=True
        ),
        terminal_completion_budget_ms=_uint(
            plan, "terminal_completion_budget_ms", 0xFFFF, positive=True
        ),
        completion_hold_ms=_uint(
            plan, "completion_hold_ms", 0xFFFF, positive=True
        ),
        plan_timing_guard_ms=_uint(plan, "plan_timing_guard_ms", 0xFFFF),
        command_ttl_ms=_uint(plan, "command_ttl_ms", 0xFFFF, positive=True),
        local_command_watchdog_ms=_uint(
            plan, "local_command_watchdog_ms", 0xFFFF, positive=True
        ),
        flags=flags,
        origin_id=origin_id,
    )
    try:
        compact.encode()
    except (OverflowError, ValueError) as exc:
        raise PairBAdapterError(f"invalid_pairb_plan:{exc}") from exc
    return compact


def adapt_ground_plan_command(
    command: object,
    *,
    expected_frame_id: str,
    expected_origin_id: int,
    expected_role: Role | None = None,
    tolerance: QuantizationTolerance = DEFAULT_QUANTIZATION_TOLERANCE,
    max_command_ttl_ms: int = DEFAULT_MAX_COMMAND_TTL_MS,
) -> PlanCommand:
    """Map one GroundPlanCommand without comparing clocks across computers."""

    if _integer(command, "schema_version") != GROUND_COMMAND_SCHEMA_VERSION:
        raise PairBAdapterError("unsupported_command_schema")
    _check_identity(
        command,
        expected_frame_id=expected_frame_id,
        expected_origin_id=expected_origin_id,
    )
    role_text = _enum_text(_field(command, "target_role")).lower()
    if role_text not in ROLE_MAP:
        raise PairBAdapterError("unsupported_target_role")
    role = ROLE_MAP[role_text]
    if expected_role is not None and role != expected_role:
        raise PairBAdapterError("target_role_mismatch")

    phase_text = _enum_text(_field(command, "phase")).upper()
    if phase_text not in PHASE_MAP:
        raise PairBAdapterError("unsupported_command_phase")
    phase = PHASE_MAP[phase_text]
    plan_id = _uint(command, "plan_id", 0xFFFF)
    sequence = _uint(command, "sequence", U32_MASK, positive=True)
    timestamp_ms = _uint(command, "sender_monotonic_ms", U32_MASK)
    valid_until_ms = _uint(
        command, "valid_until_sender_monotonic_ms", U32_MASK
    )
    ttl_ms = _uint(command, "ttl_ms", 0xFFFF, positive=True)
    if ttl_ms > max_command_ttl_ms:
        raise PairBAdapterError("command_ttl_exceeds_pairb_policy")
    if validity_window_ms(timestamp_ms, valid_until_ms) != ttl_ms:
        raise PairBAdapterError("command_ttl_mismatch")

    speed_mps = _finite(command, "body_speed_mps")
    yaw_rate_radps = _finite(command, "yaw_rate_radps")
    max_speed_mps = _finite(command, "max_speed_mps")
    max_accel_mps2 = _finite(command, "max_accel_mps2")
    if speed_mps < 0.0:
        raise PairBAdapterError("reverse_not_supported")
    if max_speed_mps < 0.0 or max_accel_mps2 < 0.0:
        raise PairBAdapterError("invalid_command_limits")
    if speed_mps > max_speed_mps:
        raise PairBAdapterError("command_exceeds_declared_speed")
    if phase in (Phase.HOLD, Phase.STOP, Phase.ABORT) and (
        abs(speed_mps) > 1.0e-12 or abs(yaw_rate_radps) > 1.0e-12
    ):
        raise PairBAdapterError("nonzero_safe_phase")

    _check_quantized(
        "body_speed_mps",
        speed_mps,
        scale=100.0,
        minimum=-32768,
        maximum=32767,
        tolerance=tolerance.speed_mps,
    )
    _check_quantized(
        "yaw_rate_radps",
        yaw_rate_radps,
        scale=18_000.0 / math.pi,
        minimum=-32768,
        maximum=32767,
        tolerance=tolerance.angle_rad,
    )
    _check_quantized(
        "max_speed_mps",
        max_speed_mps,
        scale=100.0,
        minimum=0,
        maximum=0xFFFF,
        tolerance=tolerance.speed_mps,
    )
    _check_quantized(
        "max_accel_mps2",
        max_accel_mps2,
        scale=100.0,
        minimum=0,
        maximum=0xFFFF,
        tolerance=tolerance.accel_mps2,
    )

    compact = PlanCommand(
        plan_id=plan_id,
        role=role,
        phase=phase,
        seq=sequence,
        timestamp_ms=timestamp_ms,
        valid_until_ms=valid_until_ms,
        v_mps=speed_mps,
        omega_radps=yaw_rate_radps,
        duration_ms=ttl_ms,
        distance_m=0.0,
        max_speed_mps=max_speed_mps,
        max_accel_mps2=max_accel_mps2,
        flags=0,
    )
    compact.encode()
    return compact


# Descriptive aliases for callers that prefer transport-oriented names.
corridor_plan_to_pairb = adapt_ground_corridor_plan
plan_command_to_pairb = adapt_ground_plan_command
