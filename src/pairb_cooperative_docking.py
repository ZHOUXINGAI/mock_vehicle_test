#!/usr/bin/env python3

"""Pure planning and temporal coordination for the two-rover docking mock.

This module deliberately owns no ROS node, serial port, MAVLink connection, or
actuator interface.  It produces finite local trajectories and low-rate speed
envelopes that can be transported in Pair B PlanCommand messages.
"""

from __future__ import annotations

import enum
import math
from dataclasses import dataclass
from typing import Iterable, Sequence

try:
    from src.lr24_compact_protocol import (
        CorridorPlanCompact,
        MessageType,
        Phase,
        PlanCommand,
        PlanFlag,
        Role,
        corridor_plan_post_tangent_reserve_ms,
        corridor_plan_required_validity_ms,
    )
    from src.orin2_trajectory_tracker import (
        PolylineTrajectory,
        TrajectoryPoint,
    )
except ModuleNotFoundError:  # Direct execution with src/ on PYTHONPATH.
    from lr24_compact_protocol import (
        CorridorPlanCompact,
        MessageType,
        Phase,
        PlanCommand,
        PlanFlag,
        Role,
        corridor_plan_post_tangent_reserve_ms,
        corridor_plan_required_validity_ms,
    )
    from orin2_trajectory_tracker import PolylineTrajectory, TrajectoryPoint


Vec2 = tuple[float, float]
UINT32_MASK = 0xFFFFFFFF
PAIRB_MISSION_TYPES = frozenset(
    {
        MessageType.MINI_STATE,
        MessageType.CORRIDOR_PLAN,
        MessageType.PLAN_COMMAND,
        MessageType.MISSION_STATUS,
        MessageType.ABORT,
    }
)


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def wrap_pi(angle_rad: float) -> float:
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def norm(vector: Vec2) -> float:
    return math.hypot(vector[0], vector[1])


def distance(a: Vec2, b: Vec2) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def dot(a: Vec2, b: Vec2) -> float:
    return a[0] * b[0] + a[1] * b[1]


def unit(vector: Vec2) -> Vec2:
    length = norm(vector)
    if length <= 1.0e-9:
        raise ValueError("zero-length vector")
    return vector[0] / length, vector[1] / length


def directed_phase_delta(start: float, end: float, turn_direction: str) -> float:
    sign = 1.0 if turn_direction == "ccw" else -1.0
    delta = sign * (end - start)
    while delta < 0.0:
        delta += 2.0 * math.pi
    while delta >= 2.0 * math.pi:
        delta -= 2.0 * math.pi
    return delta


def tangent_direction(phase_rad: float, turn_direction: str) -> Vec2:
    sign = 1.0 if turn_direction == "ccw" else -1.0
    return -sign * math.sin(phase_rad), sign * math.cos(phase_rad)


def pairb_mission_type_allowed(message_type: MessageType) -> bool:
    """Return the strict run-time mission whitelist for Pair B."""

    return message_type in PAIRB_MISSION_TYPES


@dataclass(frozen=True)
class CooperativePlannerConfig:
    orbit_center: Vec2 = (5.0, 3.0)
    orbit_radius_m: float = 2.0
    turn_direction: str = "ccw"
    required_orbit_laps: float = 1.0
    terminal_length_m: float = 6.0
    target_front_gap_m: float = 0.60
    terminal_lead_time_s: float = 2.5
    mini_min_speed_mps: float = 0.12
    mini_speed_mps: float = 0.18
    mini_max_speed_mps: float = 0.20
    mini_terminal_min_speed_mps: float = 0.12
    mini_terminal_max_speed_mps: float = 0.16
    carrier_min_speed_mps: float = 0.0
    carrier_nominal_speed_mps: float = 0.14
    carrier_max_speed_mps: float = 0.16
    carrier_terminal_min_speed_mps: float = 0.12
    carrier_terminal_max_speed_mps: float = 0.16
    carrier_min_tracking_speed_mps: float = 0.045
    carrier_max_accel_mps2: float = 0.12
    carrier_max_decel_mps2: float = 0.12
    mini_max_accel_mps2: float = 0.15
    mini_max_decel_mps2: float = 0.15
    minimum_turn_radius_m: float = 1.20
    orbit_clearance_m: float = 0.20
    path_spacing_m: float = 0.08
    phase_candidates: int = 72
    max_extra_orbits: int = 1
    command_ttl_ms: int = 500
    local_watchdog_ms: int = 750
    plan_validity_ms: int = 90_000

    def validate(self) -> None:
        values = (
            *self.orbit_center,
            self.orbit_radius_m,
            self.required_orbit_laps,
            self.terminal_length_m,
            self.target_front_gap_m,
            self.terminal_lead_time_s,
            self.mini_min_speed_mps,
            self.mini_speed_mps,
            self.mini_max_speed_mps,
            self.mini_terminal_min_speed_mps,
            self.mini_terminal_max_speed_mps,
            self.carrier_min_speed_mps,
            self.carrier_nominal_speed_mps,
            self.carrier_max_speed_mps,
            self.carrier_terminal_min_speed_mps,
            self.carrier_terminal_max_speed_mps,
            self.carrier_min_tracking_speed_mps,
            self.carrier_max_accel_mps2,
            self.carrier_max_decel_mps2,
            self.mini_max_accel_mps2,
            self.mini_max_decel_mps2,
            self.minimum_turn_radius_m,
            self.orbit_clearance_m,
            self.path_spacing_m,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("planner limits must be finite")
        if self.turn_direction not in {"ccw", "cw"}:
            raise ValueError("turn_direction must be ccw or cw")
        if any(
            value <= 0.0
            for value in (
                self.orbit_radius_m,
                self.required_orbit_laps,
                self.terminal_length_m,
                self.mini_min_speed_mps,
                self.mini_speed_mps,
                self.mini_max_speed_mps,
                self.mini_terminal_min_speed_mps,
                self.mini_terminal_max_speed_mps,
                self.carrier_nominal_speed_mps,
                self.carrier_max_speed_mps,
                self.carrier_terminal_min_speed_mps,
                self.carrier_terminal_max_speed_mps,
                self.carrier_min_tracking_speed_mps,
                self.carrier_max_accel_mps2,
                self.carrier_max_decel_mps2,
                self.mini_max_accel_mps2,
                self.mini_max_decel_mps2,
                self.minimum_turn_radius_m,
                self.path_spacing_m,
            )
        ):
            raise ValueError("planner lengths, speeds, acceleration and laps must be positive")
        if self.carrier_min_tracking_speed_mps > self.carrier_max_speed_mps:
            raise ValueError("carrier minimum speed exceeds maximum")
        if not (
            self.mini_min_speed_mps
            <= self.mini_speed_mps
            <= self.mini_max_speed_mps
        ):
            raise ValueError("Mini nominal speed is outside its envelope")
        if not (
            self.carrier_min_speed_mps
            <= self.carrier_nominal_speed_mps
            <= self.carrier_max_speed_mps
        ):
            raise ValueError("Carrier nominal speed is outside its envelope")
        if self.carrier_min_speed_mps < 0.0:
            raise ValueError("Carrier minimum speed cannot be negative")
        if not (
            self.mini_min_speed_mps
            <= self.mini_terminal_min_speed_mps
            <= self.mini_terminal_max_speed_mps
            <= self.mini_max_speed_mps
        ):
            raise ValueError("Mini terminal speed range is outside its envelope")
        if not (
            self.carrier_min_speed_mps
            <= self.carrier_terminal_min_speed_mps
            <= self.carrier_terminal_max_speed_mps
            <= self.carrier_max_speed_mps
        ):
            raise ValueError("Carrier terminal speed range is outside its envelope")
        terminal_speed_overlap(self)
        if self.phase_candidates < 36 or self.max_extra_orbits not in (0, 1):
            raise ValueError("planner search limits are invalid")
        if self.command_ttl_ms <= 0 or self.local_watchdog_ms <= 0:
            raise ValueError("command timing must be positive")


@dataclass(frozen=True)
class TerminalSpeedWindow:
    minimum_mps: float
    maximum_mps: float
    rendezvous_mps: float


def terminal_speed_overlap(config: CooperativePlannerConfig) -> TerminalSpeedWindow:
    """Return the shared terminal speed range or reject an impossible mission."""

    lower = max(
        config.mini_terminal_min_speed_mps,
        config.carrier_terminal_min_speed_mps,
    )
    upper = min(
        config.mini_terminal_max_speed_mps,
        config.carrier_terminal_max_speed_mps,
    )
    if lower > upper + 1.0e-9:
        raise ValueError("terminal_speed_envelopes_do_not_overlap")
    return TerminalSpeedWindow(lower, upper, 0.5 * (lower + upper))


@dataclass(frozen=True)
class CurveMetrics:
    length_m: float
    max_abs_curvature_inv_m: float
    max_curvature_rate_inv_m2: float
    heading_change_rad: float
    start_abs_curvature_inv_m: float
    end_abs_curvature_inv_m: float
    forward_only: bool
    self_intersects: bool
    clears_orbit: bool


@dataclass(frozen=True)
class CooperativeDockingPlan:
    plan_id: int
    origin_id: int
    frame_id: str
    generated_at_ms: int
    carrier_start: Vec2
    carrier_start_yaw_rad: float
    mini_phase_at_plan_rad: float
    orbit_center: Vec2
    orbit_radius_m: float
    turn_direction: str
    tangent_phase_rad: float
    tangent_point: Vec2
    tangent_direction: Vec2
    mini_exit_delta_rad: float
    extra_orbits: int
    mini_exit_delay_s: float
    carrier_start_delay_s: float
    carrier_approach_duration_s: float
    carrier_planned_speed_mps: float
    mini_speed_range_mps: tuple[float, float]
    carrier_speed_range_mps: tuple[float, float]
    terminal_speed_range_mps: tuple[float, float]
    rendezvous_speed_mps: float
    terminal_length_m: float
    target_front_gap_m: float
    carrier_path: PolylineTrajectory
    mini_terminal_path: PolylineTrajectory
    carrier_curve_metrics: CurveMetrics
    candidate_count: int
    score: float

    @property
    def carrier_approach_length_m(self) -> float:
        terminal_start = self.tangent_point
        progress = 0.0
        previous = (self.carrier_path.points[0].x_m, self.carrier_path.points[0].y_m)
        for point in self.carrier_path.points[1:]:
            current = (point.x_m, point.y_m)
            progress += distance(previous, current)
            if distance(current, terminal_start) <= 1.0e-5:
                return progress
            previous = current
        return self.carrier_curve_metrics.length_m


def _minimum_travel_time(distance_m: float, speed_mps: float, accel_mps2: float) -> float:
    accel_distance = speed_mps * speed_mps / accel_mps2
    if distance_m <= accel_distance:
        return 2.0 * math.sqrt(distance_m / accel_mps2)
    return 2.0 * speed_mps / accel_mps2 + (distance_m - accel_distance) / speed_mps


def _bezier_point(control: Sequence[Vec2], u: float) -> Vec2:
    degree = len(control) - 1
    one_minus_u = 1.0 - u
    weights = [
        math.comb(degree, index)
        * one_minus_u ** (degree - index)
        * u**index
        for index in range(degree + 1)
    ]
    return (
        sum(weight * point[0] for weight, point in zip(weights, control)),
        sum(weight * point[1] for weight, point in zip(weights, control)),
    )


def _quintic_derivatives(control: Sequence[Vec2], u: float) -> tuple[Vec2, Vec2]:
    first_control = [
        (5.0 * (end[0] - start[0]), 5.0 * (end[1] - start[1]))
        for start, end in zip(control, control[1:])
    ]
    second_control = [
        (
            4.0 * (end[0] - start[0]),
            4.0 * (end[1] - start[1]),
        )
        for start, end in zip(first_control, first_control[1:])
    ]
    return _bezier_point(first_control, u), _bezier_point(second_control, u)


def _segments_intersect(a: Vec2, b: Vec2, c: Vec2, d: Vec2) -> bool:
    def orientation(p: Vec2, q: Vec2, r: Vec2) -> float:
        return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])

    return orientation(a, b, c) * orientation(a, b, d) < 0.0 and orientation(c, d, a) * orientation(c, d, b) < 0.0


def _self_intersects(points: Sequence[Vec2]) -> bool:
    # Cubic candidates are oversampled for controller quality. Intersection
    # testing at at most 64 evenly spaced points keeps planning bounded without
    # changing the curve topology.
    if len(points) > 24:
        last = len(points) - 1
        points = tuple(points[round(index * last / 23)] for index in range(24))
    for first in range(len(points) - 1):
        for second in range(first + 2, len(points) - 1):
            if second == first + 1:
                continue
            if _segments_intersect(
                points[first], points[first + 1], points[second], points[second + 1]
            ):
                return True
    return False


def sample_heading_constrained_curve(
    start: Vec2,
    start_yaw_rad: float,
    end: Vec2,
    end_yaw_rad: float,
    start_handle_m: float,
    end_handle_m: float,
    spacing_m: float,
) -> tuple[tuple[Vec2, ...], CurveMetrics]:
    """Sample a heading-constrained quintic with zero endpoint curvature."""

    values = (*start, start_yaw_rad, *end, end_yaw_rad, start_handle_m, end_handle_m, spacing_m)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("curve inputs must be finite")
    if start_handle_m <= 0.0 or end_handle_m <= 0.0 or spacing_m <= 0.0:
        raise ValueError("curve handles and spacing must be positive")
    start_dir = (math.cos(start_yaw_rad), math.sin(start_yaw_rad))
    end_dir = (math.cos(end_yaw_rad), math.sin(end_yaw_rad))
    p1 = (
        start[0] + 0.5 * start_handle_m * start_dir[0],
        start[1] + 0.5 * start_handle_m * start_dir[1],
    )
    p2 = (
        start[0] + start_handle_m * start_dir[0],
        start[1] + start_handle_m * start_dir[1],
    )
    p4 = (
        end[0] - 0.5 * end_handle_m * end_dir[0],
        end[1] - 0.5 * end_handle_m * end_dir[1],
    )
    p3 = (
        end[0] - end_handle_m * end_dir[0],
        end[1] - end_handle_m * end_dir[1],
    )
    control = (start, p1, p2, p3, p4, end)
    estimate = sum(distance(a, b) for a, b in zip(control, control[1:]))
    count = max(24, math.ceil(estimate / spacing_m))
    points = tuple(_bezier_point(control, index / count) for index in range(count + 1))
    length_m = sum(distance(a, b) for a, b in zip(points, points[1:]))
    curvatures: list[float] = []
    headings: list[float] = []
    forward_only = True
    for index in range(count + 1):
        u = index / count
        first, second = _quintic_derivatives(control, u)
        speed_squared = dot(first, first)
        if speed_squared <= 1.0e-10:
            forward_only = False
            curvatures.append(math.inf)
            headings.append(headings[-1] if headings else start_yaw_rad)
            continue
        headings.append(math.atan2(first[1], first[0]))
        curvatures.append(
            (first[0] * second[1] - first[1] * second[0])
            / (speed_squared ** 1.5)
        )
    for index, (a, b) in enumerate(zip(points, points[1:])):
        midpoint_u = (index + 0.5) / count
        derivative, _ = _quintic_derivatives(control, midpoint_u)
        if dot((b[0] - a[0], b[1] - a[1]), derivative) <= 0.0:
            forward_only = False
    curvature_rates = []
    for index, (current, following) in enumerate(zip(curvatures, curvatures[1:])):
        ds = distance(points[index], points[index + 1])
        if ds > 1.0e-7:
            curvature_rates.append(abs(following - current) / ds)
    heading_change = sum(abs(wrap_pi(b - a)) for a, b in zip(headings, headings[1:]))
    metrics = CurveMetrics(
        length_m=length_m,
        max_abs_curvature_inv_m=max(abs(value) for value in curvatures),
        max_curvature_rate_inv_m2=max(curvature_rates, default=0.0),
        heading_change_rad=heading_change,
        start_abs_curvature_inv_m=abs(curvatures[0]),
        end_abs_curvature_inv_m=abs(curvatures[-1]),
        forward_only=forward_only,
        self_intersects=_self_intersects(points),
        clears_orbit=True,
    )
    return points, metrics


def _with_orbit_clearance(
    metrics: CurveMetrics,
    points: Sequence[Vec2],
    center: Vec2,
    radius_m: float,
    clearance_m: float,
) -> CurveMetrics:
    # The endpoint is the permitted tangent contact, so a fixed clearance
    # cannot remain nonzero all the way to it. The hard safety condition is no
    # circle penetration; clearance is handled by plan scoring and the shared
    # terminal front-gap contract.
    clears = all(
        distance(point, center) >= radius_m - 1.0e-4
        for point in points[:-1]
    )
    return CurveMetrics(
        length_m=metrics.length_m,
        max_abs_curvature_inv_m=metrics.max_abs_curvature_inv_m,
        max_curvature_rate_inv_m2=metrics.max_curvature_rate_inv_m2,
        heading_change_rad=metrics.heading_change_rad,
        start_abs_curvature_inv_m=metrics.start_abs_curvature_inv_m,
        end_abs_curvature_inv_m=metrics.end_abs_curvature_inv_m,
        forward_only=metrics.forward_only,
        self_intersects=metrics.self_intersects,
        clears_orbit=clears,
    )


def _trajectory_from_sections(
    sections: Iterable[tuple[Sequence[Vec2], float, str]],
) -> PolylineTrajectory:
    points: list[TrajectoryPoint] = []
    for coordinates, speed, phase in sections:
        for coordinate in coordinates:
            point = TrajectoryPoint(coordinate[0], coordinate[1], speed, phase)
            if points and distance(
                (points[-1].x_m, points[-1].y_m), coordinate
            ) <= 1.0e-6:
                points[-1] = point
            else:
                points.append(point)
    return PolylineTrajectory(points)


def _sample_straight(start: Vec2, direction: Vec2, length_m: float, spacing_m: float) -> tuple[Vec2, ...]:
    count = max(1, math.ceil(length_m / spacing_m))
    return tuple(
        (
            start[0] + direction[0] * length_m * index / count,
            start[1] + direction[1] * length_m * index / count,
        )
        for index in range(count + 1)
    )


def build_cooperative_docking_plan(
    *,
    plan_id: int,
    origin_id: int,
    generated_at_ms: int,
    carrier_position: Vec2,
    carrier_yaw_rad: float,
    mini_phase_rad: float,
    qualified_orbit_laps: float,
    config: CooperativePlannerConfig = CooperativePlannerConfig(),
    frame_id: str = "field_enu",
) -> CooperativeDockingPlan:
    """Search a short executable common-tangent plan after one full Mini lap."""

    config.validate()
    values = (*carrier_position, carrier_yaw_rad, mini_phase_rad, qualified_orbit_laps)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("planning state must be finite")
    if plan_id <= 0 or origin_id <= 0 or generated_at_ms < 0 or not frame_id:
        raise ValueError("plan identity and frame are invalid")
    if qualified_orbit_laps + 1.0e-6 < config.required_orbit_laps:
        raise ValueError("mini_full_orbit_not_qualified")
    if distance(carrier_position, config.orbit_center) <= (
        config.orbit_radius_m + config.orbit_clearance_m
    ):
        raise ValueError("carrier_start_inside_protected_orbit")
    terminal_window = terminal_speed_overlap(config)

    candidates: list[tuple[float, dict[str, object]]] = []
    full_orbit_time = 2.0 * math.pi * config.orbit_radius_m / config.mini_speed_mps
    max_curvature = 1.0 / config.minimum_turn_radius_m
    for phase_index in range(config.phase_candidates):
        tangent_phase = 2.0 * math.pi * phase_index / config.phase_candidates
        tangent = (
            config.orbit_center[0] + config.orbit_radius_m * math.cos(tangent_phase),
            config.orbit_center[1] + config.orbit_radius_m * math.sin(tangent_phase),
        )
        tangent_dir = tangent_direction(tangent_phase, config.turn_direction)
        tangent_yaw = math.atan2(tangent_dir[1], tangent_dir[0])
        chord = distance(carrier_position, tangent)
        if chord <= config.path_spacing_m:
            continue
        for start_ratio in (0.25, 0.45, 0.65):
            for end_ratio in (0.25, 0.45, 0.65):
                curve, raw_metrics = sample_heading_constrained_curve(
                    carrier_position,
                    carrier_yaw_rad,
                    tangent,
                    tangent_yaw,
                    max(0.25, chord * start_ratio),
                    max(0.25, chord * end_ratio),
                    config.path_spacing_m,
                )
                metrics = _with_orbit_clearance(
                    raw_metrics,
                    curve,
                    config.orbit_center,
                    config.orbit_radius_m,
                    config.orbit_clearance_m,
                )
                if (
                    not metrics.forward_only
                    or metrics.self_intersects
                    or not metrics.clears_orbit
                    or metrics.max_abs_curvature_inv_m > max_curvature
                ):
                    continue
                minimum_carrier_time = _minimum_travel_time(
                    metrics.length_m,
                    config.carrier_max_speed_mps,
                    config.carrier_max_accel_mps2,
                )
                base_delta = directed_phase_delta(
                    mini_phase_rad,
                    tangent_phase,
                    config.turn_direction,
                )
                base_delay = base_delta * config.orbit_radius_m / config.mini_speed_mps
                extra_orbits = 0
                required_delay = minimum_carrier_time + config.terminal_lead_time_s
                while base_delay + extra_orbits * full_orbit_time < required_delay:
                    extra_orbits += 1
                if extra_orbits > config.max_extra_orbits:
                    continue
                mini_delay = base_delay + extra_orbits * full_orbit_time
                available_duration = mini_delay - config.terminal_lead_time_s
                slow_duration = metrics.length_m / config.carrier_min_tracking_speed_mps
                if available_duration > slow_duration:
                    start_delay = available_duration - slow_duration
                    approach_duration = slow_duration
                else:
                    start_delay = 0.0
                    approach_duration = available_duration
                if approach_duration <= 0.0:
                    continue
                planned_speed = metrics.length_m / approach_duration
                if planned_speed > config.carrier_max_speed_mps + 1.0e-6:
                    continue
                # Extra laps dominate the score. Remaining terms select a
                # short, low-curvature, low-heading-travel candidate.
                score = (
                    1000.0 * extra_orbits
                    + metrics.length_m
                    + 0.15 * base_delta * config.orbit_radius_m
                    + 0.25 * metrics.heading_change_rad
                    + 0.10 * metrics.max_abs_curvature_inv_m
                )
                candidates.append(
                    (
                        score,
                        {
                            "phase": tangent_phase,
                            "tangent": tangent,
                            "direction": tangent_dir,
                            "curve": curve,
                            "metrics": metrics,
                            "base_delta": base_delta,
                            "extra_orbits": extra_orbits,
                            "mini_delay": mini_delay,
                            "start_delay": start_delay,
                            "approach_duration": approach_duration,
                            "planned_speed": planned_speed,
                        },
                    )
                )
    if not candidates:
        raise ValueError("no_forward_executable_tangent_solution")
    score, selected = min(candidates, key=lambda item: item[0])
    tangent = selected["tangent"]
    tangent_dir = selected["direction"]
    curve = selected["curve"]
    planned_speed = float(selected["planned_speed"])
    terminal = _sample_straight(
        tangent,
        tangent_dir,
        config.terminal_length_m,
        config.path_spacing_m,
    )
    carrier_path = _trajectory_from_sections(
        (
            (curve, planned_speed, "CARRIER_APPROACH"),
            (terminal, terminal_window.rendezvous_mps, "SHARED_TERMINAL"),
        )
    )
    mini_terminal_length = max(
        config.path_spacing_m,
        config.terminal_length_m - config.target_front_gap_m,
    )
    mini_terminal = _trajectory_from_sections(
        (
            (
                _sample_straight(
                    tangent,
                    tangent_dir,
                    mini_terminal_length,
                    config.path_spacing_m,
                ),
                terminal_window.rendezvous_mps,
                "MINI_TANGENT_EXIT",
            ),
        )
    )
    return CooperativeDockingPlan(
        plan_id=plan_id,
        origin_id=origin_id,
        frame_id=frame_id,
        generated_at_ms=generated_at_ms,
        carrier_start=carrier_position,
        carrier_start_yaw_rad=carrier_yaw_rad,
        mini_phase_at_plan_rad=mini_phase_rad % (2.0 * math.pi),
        orbit_center=config.orbit_center,
        orbit_radius_m=config.orbit_radius_m,
        turn_direction=config.turn_direction,
        tangent_phase_rad=float(selected["phase"]),
        tangent_point=tangent,
        tangent_direction=tangent_dir,
        mini_exit_delta_rad=float(selected["base_delta"]) + 2.0 * math.pi * int(selected["extra_orbits"]),
        extra_orbits=int(selected["extra_orbits"]),
        mini_exit_delay_s=float(selected["mini_delay"]),
        carrier_start_delay_s=float(selected["start_delay"]),
        carrier_approach_duration_s=float(selected["approach_duration"]),
        carrier_planned_speed_mps=planned_speed,
        mini_speed_range_mps=(config.mini_min_speed_mps, config.mini_max_speed_mps),
        carrier_speed_range_mps=(
            config.carrier_min_speed_mps,
            config.carrier_max_speed_mps,
        ),
        terminal_speed_range_mps=(
            terminal_window.minimum_mps,
            terminal_window.maximum_mps,
        ),
        rendezvous_speed_mps=terminal_window.rendezvous_mps,
        terminal_length_m=config.terminal_length_m,
        target_front_gap_m=config.target_front_gap_m,
        carrier_path=carrier_path,
        mini_terminal_path=mini_terminal,
        carrier_curve_metrics=selected["metrics"],
        candidate_count=len(candidates),
        score=score,
    )


def compact_corridor_plan(
    plan: CooperativeDockingPlan,
    *,
    sequence: int,
    config: CooperativePlannerConfig,
) -> CorridorPlanCompact:
    post_reserve_ms = corridor_plan_post_tangent_reserve_ms(
        3000,
        1000,
        config.command_ttl_ms,
        config.local_watchdog_ms,
        250,
    )
    arrival_ms = int(round(plan.mini_exit_delay_s * 1000.0))
    required_ms = corridor_plan_required_validity_ms(arrival_ms, post_reserve_ms)
    validity_ms = max(config.plan_validity_ms, required_ms)
    return CorridorPlanCompact(
        plan_schema_version=2,
        plan_id=plan.plan_id,
        seq=sequence & UINT32_MASK,
        timestamp_ms=plan.generated_at_ms & UINT32_MASK,
        valid_until_ms=(plan.generated_at_ms + validity_ms) & UINT32_MASK,
        rendezvous_x_m=plan.tangent_point[0],
        rendezvous_y_m=plan.tangent_point[1],
        tangent_dir_x=plan.tangent_direction[0],
        tangent_dir_y=plan.tangent_direction[1],
        corridor_length_m=plan.terminal_length_m,
        ahead_distance_m=plan.target_front_gap_m,
        mini_arrival_delay_ms=arrival_ms,
        trigger_phase_rad=plan.tangent_phase_rad,
        # The compact v1 fields carry terminal speed intent. Full per-vehicle
        # envelopes remain part of the shared mission configuration.
        mini_speed_mps=plan.rendezvous_speed_mps,
        carrier_max_speed_mps=plan.terminal_speed_range_mps[1],
        target_front_gap_m=plan.target_front_gap_m,
        required_validity_ms=required_ms,
        post_tangent_reserve_ms=post_reserve_ms,
        terminal_completion_budget_ms=3000,
        completion_hold_ms=1000,
        plan_timing_guard_ms=250,
        command_ttl_ms=config.command_ttl_ms,
        local_command_watchdog_ms=config.local_watchdog_ms,
        flags=int(PlanFlag.CORRIDOR_VALID | PlanFlag.ONE_ORBIT_COMPLETE),
        origin_id=plan.origin_id,
    )


class CoordinationMode(str, enum.Enum):
    RUN = "RUN"
    SLOW_CARRIER = "SLOW_CARRIER"
    HOLD = "HOLD"
    ABORT = "ABORT"


class CooperationPhase(str, enum.Enum):
    APPROACH = "APPROACH"
    TERMINAL = "TERMINAL"


@dataclass(frozen=True)
class TemporalCoordinatorConfig:
    carrier_min_speed_mps: float = 0.0
    carrier_nominal_speed_mps: float = 0.14
    carrier_max_speed_mps: float = 0.16
    carrier_terminal_min_speed_mps: float = 0.12
    carrier_terminal_max_speed_mps: float = 0.16
    mini_min_speed_mps: float = 0.12
    mini_nominal_speed_mps: float = 0.18
    mini_max_speed_mps: float = 0.20
    mini_terminal_min_speed_mps: float = 0.12
    mini_terminal_max_speed_mps: float = 0.16
    minimum_tracking_speed_mps: float = 0.04
    target_front_gap_m: float = 0.60
    minimum_front_gap_m: float = 0.15
    maximum_front_gap_m: float = 2.5
    terminal_lead_time_s: float = 2.5
    eta_tolerance_s: float = 1.0
    state_stale_s: float = 0.50
    cross_track_hold_m: float = 0.80
    speed_slew_mps2: float = 0.10
    gap_kp_per_s: float = 0.18
    hold_after_s: float = 1.5
    abort_after_s: float = 4.0
    relative_speed_tolerance_mps: float = 0.015
    front_gap_tolerance_m: float = 0.20
    lateral_gap_tolerance_m: float = 0.20
    heading_tolerance_rad: float = 0.20
    yaw_rate_tolerance_radps: float = 0.20
    terminal_capture_hold_s: float = 2.0

    def terminal_speed_window(self) -> TerminalSpeedWindow:
        lower = max(
            self.carrier_terminal_min_speed_mps,
            self.mini_terminal_min_speed_mps,
        )
        upper = min(
            self.carrier_terminal_max_speed_mps,
            self.mini_terminal_max_speed_mps,
        )
        if lower > upper + 1.0e-9:
            raise ValueError("terminal_speed_envelopes_do_not_overlap")
        return TerminalSpeedWindow(lower, upper, 0.5 * (lower + upper))

    def validate(self) -> None:
        values = tuple(
            value
            for value in self.__dict__.values()
            if isinstance(value, (int, float))
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("coordinator limits must be finite")
        if not (
            0.0 <= self.carrier_min_speed_mps
            <= self.carrier_nominal_speed_mps
            <= self.carrier_max_speed_mps
        ):
            raise ValueError("Carrier speed envelope is invalid")
        if not (
            0.0 < self.mini_min_speed_mps
            <= self.mini_nominal_speed_mps
            <= self.mini_max_speed_mps
        ):
            raise ValueError("Mini speed envelope is invalid")
        if not (
            self.carrier_min_speed_mps
            <= self.carrier_terminal_min_speed_mps
            <= self.carrier_terminal_max_speed_mps
            <= self.carrier_max_speed_mps
        ):
            raise ValueError("Carrier terminal speed envelope is invalid")
        if not (
            self.mini_min_speed_mps
            <= self.mini_terminal_min_speed_mps
            <= self.mini_terminal_max_speed_mps
            <= self.mini_max_speed_mps
        ):
            raise ValueError("Mini terminal speed envelope is invalid")
        if any(
            value <= 0.0
            for value in (
                self.minimum_tracking_speed_mps,
                self.target_front_gap_m,
                self.minimum_front_gap_m,
                self.maximum_front_gap_m,
                self.terminal_lead_time_s,
                self.eta_tolerance_s,
                self.state_stale_s,
                self.cross_track_hold_m,
                self.speed_slew_mps2,
                self.gap_kp_per_s,
                self.hold_after_s,
                self.abort_after_s,
                self.relative_speed_tolerance_mps,
                self.front_gap_tolerance_m,
                self.lateral_gap_tolerance_m,
                self.heading_tolerance_rad,
                self.yaw_rate_tolerance_radps,
                self.terminal_capture_hold_s,
            )
        ):
            raise ValueError("coordinator thresholds must be positive")
        if self.hold_after_s >= self.abort_after_s:
            raise ValueError("HOLD must precede ABORT")
        self.terminal_speed_window()


@dataclass(frozen=True)
class CooperationObservation:
    now_s: float
    phase: CooperationPhase
    carrier_remaining_m: float
    mini_remaining_m: float
    carrier_speed_mps: float
    mini_speed_mps: float
    carrier_state_age_s: float
    mini_state_age_s: float
    carrier_cross_track_m: float = 0.0
    mini_cross_track_m: float = 0.0
    front_gap_m: float | None = None
    lateral_gap_m: float = 0.0
    heading_error_rad: float = 0.0
    yaw_rate_error_radps: float = 0.0


@dataclass(frozen=True)
class SpeedEnvelopeDecision:
    mode: CoordinationMode
    carrier_speed_limit_mps: float
    mini_speed_limit_mps: float
    carrier_eta_s: float
    mini_eta_s: float
    eta_error_s: float
    front_gap_m: float | None
    reason: str
    mismatch_duration_s: float
    rendezvous_speed_mps: float
    relative_speed_mps: float
    terminal_capture_duration_s: float
    terminal_capture_qualified: bool


class TemporalCoordinator:
    """Stateful ETA/gap coordinator that never emits wheel-level commands."""

    def __init__(self, config: TemporalCoordinatorConfig = TemporalCoordinatorConfig()) -> None:
        config.validate()
        self.config = config
        self.terminal_speed_window = config.terminal_speed_window()
        self._last_time_s: float | None = None
        self._carrier_limit = 0.0
        self._mini_limit = 0.0
        self._mismatch_started_s: float | None = None
        self._abort_reason: str | None = None
        self._capture_started_s: float | None = None

    def _slew(self, previous: float, target: float, dt_s: float, maximum: float) -> float:
        if dt_s <= 0.0:
            return clamp(target, 0.0, maximum)
        step = self.config.speed_slew_mps2 * max(0.0, min(dt_s, 0.5))
        return clamp(target, previous - step, previous + step)

    def step(self, observation: CooperationObservation) -> SpeedEnvelopeDecision:
        values = (
            observation.now_s,
            observation.carrier_remaining_m,
            observation.mini_remaining_m,
            observation.carrier_speed_mps,
            observation.mini_speed_mps,
            observation.carrier_state_age_s,
            observation.mini_state_age_s,
            observation.carrier_cross_track_m,
            observation.mini_cross_track_m,
            observation.lateral_gap_m,
            observation.heading_error_rad,
            observation.yaw_rate_error_radps,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("coordination observation must be finite")
        if observation.front_gap_m is not None and not math.isfinite(observation.front_gap_m):
            raise ValueError("front gap must be finite")
        if self._last_time_s is not None and observation.now_s < self._last_time_s:
            raise ValueError("coordination time moved backwards")
        dt_s = 0.0 if self._last_time_s is None else observation.now_s - self._last_time_s
        self._last_time_s = observation.now_s

        reason = "synchronized"
        hard_abort: str | None = self._abort_reason
        mismatch = False
        if max(observation.carrier_state_age_s, observation.mini_state_age_s) > self.config.state_stale_s:
            hard_abort = "state_stale"
        elif max(abs(observation.carrier_cross_track_m), abs(observation.mini_cross_track_m)) > self.config.cross_track_hold_m:
            mismatch = True
            reason = "cross_track_outside_envelope"
        elif (
            observation.mini_remaining_m > 0.05
            and observation.mini_speed_mps < self.config.minimum_tracking_speed_mps
        ):
            mismatch = True
            reason = "mini_below_tracking_speed"
        elif (
            observation.mini_remaining_m > 0.05
            and observation.mini_speed_mps
            < self.config.mini_min_speed_mps
            - self.config.relative_speed_tolerance_mps
        ):
            mismatch = True
            reason = "mini_below_speed_envelope"
        elif observation.phase == CooperationPhase.TERMINAL and (
            observation.front_gap_m is None
            or observation.front_gap_m <= self.config.minimum_front_gap_m
        ):
            hard_abort = "carrier_ahead_violation"

        mini_nominal = self.config.mini_nominal_speed_mps
        # MiniState speed is the best short-horizon arrival predictor. The
        # command ceiling remains nominal so Mini can recover, while Carrier
        # is slowed against the measured (not merely requested) Mini speed.
        mini_prediction_speed = clamp(
            observation.mini_speed_mps,
            0.01,
            self.config.mini_max_speed_mps,
        )
        mini_eta = observation.mini_remaining_m / mini_prediction_speed
        desired_carrier_eta = max(0.0, mini_eta - self.config.terminal_lead_time_s)
        carrier_target = self.config.carrier_max_speed_mps
        mode = CoordinationMode.RUN
        if observation.phase == CooperationPhase.APPROACH:
            if observation.carrier_remaining_m > 1.0e-6 and desired_carrier_eta > 1.0e-6:
                carrier_target = min(
                    self.config.carrier_max_speed_mps,
                    observation.carrier_remaining_m / desired_carrier_eta,
                )
            elif observation.carrier_remaining_m <= 1.0e-6:
                carrier_target = self.config.minimum_tracking_speed_mps
            carrier_eta = observation.carrier_remaining_m / max(carrier_target, 1.0e-6)
            eta_error = carrier_eta - desired_carrier_eta
            if carrier_target < self.config.minimum_tracking_speed_mps:
                carrier_target = 0.0
                mode = CoordinationMode.SLOW_CARRIER
                if reason == "synchronized":
                    reason = "mini_late_carrier_wait"
            elif carrier_target < self.config.carrier_max_speed_mps - 0.005:
                mode = CoordinationMode.SLOW_CARRIER
                if reason == "synchronized":
                    reason = "mini_late_carrier_slowdown"
            if eta_error > self.config.eta_tolerance_s:
                mismatch = True
                if reason == "synchronized":
                    reason = "carrier_cannot_arrive_before_mini"
        else:
            gap = observation.front_gap_m
            assert gap is not None or hard_abort is not None
            gap_value = self.config.target_front_gap_m if gap is None else gap
            gap_error = self.config.target_front_gap_m - gap_value
            rendezvous = self.terminal_speed_window.rendezvous_mps
            mini_nominal = clamp(
                rendezvous - self.config.gap_kp_per_s * gap_error,
                self.config.mini_terminal_min_speed_mps,
                self.config.mini_terminal_max_speed_mps,
            )
            carrier_target = clamp(
                rendezvous + self.config.gap_kp_per_s * gap_error,
                self.config.carrier_terminal_min_speed_mps,
                self.config.carrier_terminal_max_speed_mps,
            )
            carrier_eta = observation.carrier_remaining_m / max(carrier_target, 1.0e-6)
            eta_error = carrier_eta - mini_eta
            if gap_value > self.config.target_front_gap_m + 0.10:
                mode = CoordinationMode.SLOW_CARRIER
                reason = "front_gap_large_carrier_slowdown"
            if gap_value > self.config.maximum_front_gap_m:
                mismatch = True
                reason = "front_gap_unsynchronized"

        relative_speed = observation.carrier_speed_mps - observation.mini_speed_mps
        capture_ready = bool(
            observation.phase == CooperationPhase.TERMINAL
            and observation.front_gap_m is not None
            and abs(observation.front_gap_m - self.config.target_front_gap_m)
            <= self.config.front_gap_tolerance_m
            and abs(relative_speed) <= self.config.relative_speed_tolerance_mps
            and abs(observation.lateral_gap_m)
            <= self.config.lateral_gap_tolerance_m
            and abs(observation.heading_error_rad)
            <= self.config.heading_tolerance_rad
            and abs(observation.yaw_rate_error_radps)
            <= self.config.yaw_rate_tolerance_radps
            and hard_abort is None
        )
        if capture_ready:
            if self._capture_started_s is None:
                self._capture_started_s = observation.now_s
            capture_duration = observation.now_s - self._capture_started_s
        else:
            self._capture_started_s = None
            capture_duration = 0.0
        capture_qualified = (
            capture_duration + 1.0e-9 >= self.config.terminal_capture_hold_s
        )
        if capture_qualified and reason == "synchronized":
            reason = "terminal_capture_qualified"

        if hard_abort is not None:
            self._abort_reason = hard_abort
            mode = CoordinationMode.ABORT
            carrier_target = 0.0
            mini_nominal = 0.0
            reason = hard_abort
            mismatch = True
        if mismatch:
            if self._mismatch_started_s is None:
                self._mismatch_started_s = observation.now_s
            mismatch_duration = observation.now_s - self._mismatch_started_s
            if hard_abort is None and mismatch_duration >= self.config.abort_after_s:
                self._abort_reason = "persistent_sync_failure"
                mode = CoordinationMode.ABORT
                carrier_target = 0.0
                mini_nominal = 0.0
                reason = self._abort_reason
            elif hard_abort is None and mismatch_duration >= self.config.hold_after_s:
                mode = CoordinationMode.HOLD
                carrier_target = 0.0
                mini_nominal = 0.0
                reason = "synchronization_hold"
        else:
            self._mismatch_started_s = None
            mismatch_duration = 0.0

        if mode in {CoordinationMode.HOLD, CoordinationMode.ABORT}:
            self._carrier_limit = 0.0
            self._mini_limit = 0.0
            self._capture_started_s = None
            capture_duration = 0.0
            capture_qualified = False
        else:
            self._carrier_limit = self._slew(
                self._carrier_limit,
                carrier_target,
                dt_s,
                self.config.carrier_max_speed_mps,
            )
            self._mini_limit = self._slew(
                self._mini_limit,
                mini_nominal,
                dt_s,
                self.config.mini_max_speed_mps,
            )
        carrier_eta = observation.carrier_remaining_m / max(self._carrier_limit, 1.0e-6)
        mini_eta = observation.mini_remaining_m / max(self._mini_limit, 1.0e-6)
        eta_error = carrier_eta - max(0.0, mini_eta - self.config.terminal_lead_time_s)
        return SpeedEnvelopeDecision(
            mode=mode,
            carrier_speed_limit_mps=self._carrier_limit,
            mini_speed_limit_mps=self._mini_limit,
            carrier_eta_s=carrier_eta,
            mini_eta_s=mini_eta,
            eta_error_s=eta_error,
            front_gap_m=observation.front_gap_m,
            reason=reason,
            mismatch_duration_s=mismatch_duration,
            rendezvous_speed_mps=self.terminal_speed_window.rendezvous_mps,
            relative_speed_mps=relative_speed,
            terminal_capture_duration_s=capture_duration,
            terminal_capture_qualified=capture_qualified,
        )


def build_plan_commands(
    plan: CooperativeDockingPlan,
    decision: SpeedEnvelopeDecision,
    *,
    phase: CooperationPhase,
    sequence: int,
    timestamp_ms: int,
    config: CooperativePlannerConfig,
) -> tuple[PlanCommand, PlanCommand]:
    """Convert one speed-envelope decision into low-rate Pair B commands."""

    if decision.mode == CoordinationMode.ABORT:
        wire_phase = Phase.ABORT
    elif decision.mode == CoordinationMode.HOLD:
        wire_phase = Phase.HOLD
    else:
        wire_phase = Phase.ARC_TO_CORRIDOR if phase == CooperationPhase.APPROACH else Phase.TERMINAL
    valid_until = (timestamp_ms + config.command_ttl_ms) & UINT32_MASK
    common = {
        "plan_id": plan.plan_id,
        "phase": wire_phase,
        "timestamp_ms": timestamp_ms & UINT32_MASK,
        "valid_until_ms": valid_until,
        "omega_radps": 0.0,
        "duration_ms": config.command_ttl_ms,
        "max_accel_mps2": min(config.carrier_max_accel_mps2, config.mini_max_accel_mps2),
        "flags": 0,
    }
    carrier = PlanCommand(
        role=Role.CARRIER,
        seq=sequence & UINT32_MASK,
        v_mps=decision.carrier_speed_limit_mps,
        distance_m=max(0.0, plan.carrier_path.length_m),
        max_speed_mps=decision.carrier_speed_limit_mps,
        **common,
    )
    mini = PlanCommand(
        role=Role.MINI,
        seq=(sequence + 1) & UINT32_MASK,
        v_mps=decision.mini_speed_limit_mps,
        distance_m=plan.terminal_length_m,
        max_speed_mps=decision.mini_speed_limit_mps,
        **common,
    )
    return carrier, mini
