#!/usr/bin/env python3

"""Geometry-based local trajectory tracking for the Orin2 rover.

The tracker consumes an arbitrary finite ENU polyline and produces both the
planner-facing ``v/omega`` primitive and the Mini PX4 BODY_NED ``x/y`` adapter
command.  It deliberately contains no ROS, MAVROS, arming, or mode logic.
"""

from __future__ import annotations

import bisect
import math
from dataclasses import dataclass
from typing import Iterable, Sequence


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def wrap_pi(angle_rad: float) -> float:
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


@dataclass(frozen=True)
class TrajectoryPoint:
    x_m: float
    y_m: float
    requested_speed_mps: float
    phase: str = "TRACK"


@dataclass(frozen=True)
class TrajectorySample:
    x_m: float
    y_m: float
    tangent_yaw_rad: float
    requested_speed_mps: float
    phase: str
    s_m: float
    segment_index: int


@dataclass(frozen=True)
class TrajectoryFeasibilityReport:
    feasible: bool
    max_abs_curvature_inv_m: float
    max_abs_curvature_rate_inv_m2: float
    max_nominal_body_bearing_deg: float
    reasons: tuple[str, ...]


class PolylineTrajectory:
    """Validated finite trajectory with arc-length sampling."""

    def __init__(self, points: Sequence[TrajectoryPoint]) -> None:
        if len(points) < 2:
            raise ValueError("trajectory requires at least two points")
        checked: list[TrajectoryPoint] = []
        cumulative = [0.0]
        segment_lengths: list[float] = []
        for point in points:
            if not all(
                math.isfinite(value)
                for value in (point.x_m, point.y_m, point.requested_speed_mps)
            ):
                raise ValueError("trajectory points must be finite")
            if not 0.0 < point.requested_speed_mps <= 0.25:
                raise ValueError("trajectory requested speed must be within (0, 0.25]")
            checked.append(point)
        for previous, current in zip(checked, checked[1:]):
            length = math.hypot(
                current.x_m - previous.x_m,
                current.y_m - previous.y_m,
            )
            if length <= 1.0e-6:
                raise ValueError("trajectory contains a zero-length segment")
            segment_lengths.append(length)
            cumulative.append(cumulative[-1] + length)
        self.points = tuple(checked)
        self.cumulative_s_m = tuple(cumulative)
        self.segment_lengths_m = tuple(segment_lengths)

    @property
    def length_m(self) -> float:
        return self.cumulative_s_m[-1]

    def sample(self, s_m: float) -> TrajectorySample:
        if not math.isfinite(s_m):
            raise ValueError("trajectory sample distance must be finite")
        s_m = clamp(s_m, 0.0, self.length_m)
        segment = bisect.bisect_right(self.cumulative_s_m, s_m) - 1
        segment = min(max(0, segment), len(self.segment_lengths_m) - 1)
        start = self.points[segment]
        end = self.points[segment + 1]
        length = self.segment_lengths_m[segment]
        ratio = (s_m - self.cumulative_s_m[segment]) / length
        dx = end.x_m - start.x_m
        dy = end.y_m - start.y_m
        return TrajectorySample(
            x_m=start.x_m + ratio * dx,
            y_m=start.y_m + ratio * dy,
            tangent_yaw_rad=math.atan2(dy, dx),
            requested_speed_mps=(
                start.requested_speed_mps
                + ratio * (end.requested_speed_mps - start.requested_speed_mps)
            ),
            phase=start.phase if ratio < 0.5 else end.phase,
            s_m=s_m,
            segment_index=segment,
        )


@dataclass(frozen=True)
class ProjectionState:
    progress_s_m: float = 0.0
    segment_index: int = 0
    initialized: bool = False


@dataclass(frozen=True)
class PathProjection:
    state: ProjectionState
    x_m: float
    y_m: float
    tangent_yaw_rad: float
    cross_track_m: float
    distance_m: float


@dataclass(frozen=True)
class TrajectoryTrackerConfig:
    base_lookahead_m: float = 0.80
    speed_lookahead_gain_sec: float = 0.80
    min_lookahead_m: float = 0.65
    max_lookahead_m: float = 1.60
    projection_backtrack_m: float = 0.20
    projection_ahead_m: float = 4.0
    max_body_bearing_deg: float = 32.0
    # Zero preserves generic stateless behavior. Vehicle adapters can opt into
    # finite steering slew to reject frame-to-frame pose/yaw jumps.
    max_body_bearing_rate_degps: float = 0.0
    # Shared-field ENU uses CCW-positive bearing, while BODY_NED +y points
    # right.  The proven Mini mapping therefore negates a left-bearing demand.
    body_y_for_ccw_sign: float = -1.0
    max_yaw_rate_radps: float = 0.35
    curvature_slowdown_gain: float = 0.70
    reference_curvature_window_m: float = 0.45
    # Bound only the feedback delta around the path's nominal curvature. Zero
    # leaves the generic tracker unbounded; calibrated rover missions opt in.
    max_curvature_correction_inv_m: float = 0.0
    # Convert geometric curvature into the BODY_NED bearing demand used by the
    # PX4 differential-rover adapter. The Orin2 ULog fit supplies this virtual
    # wheelbase; zero preserves the generic uncalibrated tracker.
    curvature_to_body_gain_m: float = 0.0
    # Keep trajectory curvature feed-forward independent from tracking-error
    # feedback. A value of one preserves the original coupled adapter.
    curvature_feedback_gain_ratio: float = 1.0
    # Optional bounded integral removes persistent drivetrain bias. Both
    # values remain zero by default, preserving the stateless generic tracker.
    cross_track_integral_gain_inv_m_per_m_sec: float = 0.0
    cross_track_integral_limit_m_sec: float = 0.0
    minimum_tracking_speed_mps: float = 0.035
    terminal_slowdown_distance_m: float = 1.5
    goal_tolerance_m: float = 0.15
    max_cross_track_m: float = 1.5


@dataclass(frozen=True)
class TrajectoryTrackerState:
    projection: ProjectionState = ProjectionState()
    cross_track_integral_m_sec: float = 0.0
    body_bearing_rad: float = 0.0


def tracker_state_at_route_start() -> TrajectoryTrackerState:
    """Anchor a newly started mission to arc length zero.

    A global nearest-point initialization is useful when attaching to an
    already active route, but it is ambiguous when a closed route's endpoint
    coincides with its start. New missions have an explicit start and must use
    the local projection window from s=0 on their first frame.
    """
    return TrajectoryTrackerState(
        projection=ProjectionState(
            progress_s_m=0.0,
            segment_index=0,
            initialized=True,
        )
    )


@dataclass(frozen=True)
class TrajectoryCommand:
    state: TrajectoryTrackerState
    progress_s_m: float
    remaining_s_m: float
    cross_track_m: float
    lookahead_m: float
    target_x_m: float
    target_y_m: float
    target_bearing_error_rad: float
    curvature_inv_m: float
    raw_curvature_inv_m: float
    reference_curvature_inv_m: float
    feedback_curvature_inv_m: float
    adapter_curvature_inv_m: float
    cross_track_integral_m_sec: float
    v_mps: float
    omega_radps: float
    body_x_mps: float
    body_y_mps: float
    phase: str
    goal_reached: bool
    terminal_missed: bool


def validate_tracker_config(config: TrajectoryTrackerConfig) -> None:
    if not all(math.isfinite(float(value)) for value in vars(config).values()):
        raise ValueError("trajectory tracker configuration must be finite")
    if not 0.2 <= config.min_lookahead_m <= config.base_lookahead_m:
        raise ValueError("minimum lookahead must be positive and no greater than base")
    if not config.base_lookahead_m <= config.max_lookahead_m <= 5.0:
        raise ValueError("maximum lookahead must be at least base and no greater than 5 m")
    if not 0.0 <= config.speed_lookahead_gain_sec <= 5.0:
        raise ValueError("speed lookahead gain must be within [0, 5]")
    if not 0.0 <= config.projection_backtrack_m <= 1.0:
        raise ValueError("projection backtrack must be within [0, 1]")
    if not 0.5 <= config.projection_ahead_m <= 20.0:
        raise ValueError("projection ahead window must be within [0.5, 20]")
    if not 5.0 <= config.max_body_bearing_deg <= 89.0:
        raise ValueError("body bearing limit must be within [5, 89] degrees")
    if config.max_body_bearing_rate_degps != 0.0 and not (
        5.0 <= config.max_body_bearing_rate_degps <= 360.0
    ):
        raise ValueError("body bearing rate must be zero or within [5, 360] deg/s")
    if config.body_y_for_ccw_sign not in {-1.0, 1.0}:
        raise ValueError("BODY y direction sign must be exactly -1 or 1")
    if not 0.05 <= config.max_yaw_rate_radps <= 1.0:
        raise ValueError("yaw-rate limit must be within [0.05, 1.0]")
    if not 0.0 <= config.curvature_slowdown_gain <= 5.0:
        raise ValueError("curvature slowdown gain must be within [0, 5]")
    if not 0.10 <= config.reference_curvature_window_m <= 2.0:
        raise ValueError("reference curvature window must be within [0.10, 2.0]")
    if not 0.0 <= config.max_curvature_correction_inv_m <= 1.0:
        raise ValueError("curvature correction limit must be within [0, 1] 1/m")
    if not 0.0 <= config.curvature_to_body_gain_m <= 8.0:
        raise ValueError("curvature-to-body gain must be within [0, 8] meters")
    if not 0.25 <= config.curvature_feedback_gain_ratio <= 4.0:
        raise ValueError("curvature feedback gain ratio must be within [0.25, 4]")
    if not 0.0 <= config.cross_track_integral_gain_inv_m_per_m_sec <= 0.5:
        raise ValueError("cross-track integral gain must be within [0, 0.5]")
    if not 0.0 <= config.cross_track_integral_limit_m_sec <= 5.0:
        raise ValueError("cross-track integral limit must be within [0, 5]")
    integral_enabled = config.cross_track_integral_gain_inv_m_per_m_sec > 0.0
    if integral_enabled != (config.cross_track_integral_limit_m_sec > 0.0):
        raise ValueError("cross-track integral gain and limit must be enabled together")
    if not 0.01 <= config.minimum_tracking_speed_mps <= 0.10:
        raise ValueError("minimum tracking speed must be within [0.01, 0.10]")
    if not 0.2 <= config.terminal_slowdown_distance_m <= 5.0:
        raise ValueError("terminal slowdown distance must be within [0.2, 5]")
    if not 0.05 <= config.goal_tolerance_m <= 0.5:
        raise ValueError("goal tolerance must be within [0.05, 0.5]")
    if not 0.2 <= config.max_cross_track_m <= 3.0:
        raise ValueError("cross-track limit must be within [0.2, 3]")


def project_onto_trajectory(
    trajectory: PolylineTrajectory,
    state: ProjectionState,
    x_m: float,
    y_m: float,
    config: TrajectoryTrackerConfig,
) -> PathProjection:
    """Project locally without jumping to a distant branch at path crossings."""
    if not all(math.isfinite(value) for value in (x_m, y_m)):
        raise ValueError("projection position must be finite")
    validate_tracker_config(config)
    if state.initialized:
        minimum_s = max(0.0, state.progress_s_m - config.projection_backtrack_m)
        maximum_s = min(
            trajectory.length_m,
            state.progress_s_m + config.projection_ahead_m,
        )
        first_segment = max(
            0, bisect.bisect_right(trajectory.cumulative_s_m, minimum_s) - 2
        )
        last_segment = min(
            len(trajectory.segment_lengths_m) - 1,
            bisect.bisect_right(trajectory.cumulative_s_m, maximum_s),
        )
    else:
        minimum_s = 0.0
        maximum_s = trajectory.length_m
        first_segment = 0
        last_segment = len(trajectory.segment_lengths_m) - 1

    best = None
    for index in range(first_segment, last_segment + 1):
        start = trajectory.points[index]
        end = trajectory.points[index + 1]
        dx = end.x_m - start.x_m
        dy = end.y_m - start.y_m
        length_sq = dx * dx + dy * dy
        ratio = clamp(((x_m - start.x_m) * dx + (y_m - start.y_m) * dy) / length_sq, 0.0, 1.0)
        candidate_s = trajectory.cumulative_s_m[index] + ratio * trajectory.segment_lengths_m[index]
        candidate_s = clamp(candidate_s, minimum_s, maximum_s)
        candidate = trajectory.sample(candidate_s)
        distance_sq = (x_m - candidate.x_m) ** 2 + (y_m - candidate.y_m) ** 2
        if best is None or distance_sq < best[0]:
            best = (distance_sq, candidate)
    if best is None:
        raise RuntimeError("trajectory projection search produced no candidate")

    candidate = best[1]
    progress_s = (
        max(state.progress_s_m, candidate.s_m) if state.initialized else candidate.s_m
    )
    sample = trajectory.sample(progress_s)
    tx = math.cos(sample.tangent_yaw_rad)
    ty = math.sin(sample.tangent_yaw_rad)
    error_x = x_m - sample.x_m
    error_y = y_m - sample.y_m
    cross_track = tx * error_y - ty * error_x
    return PathProjection(
        state=ProjectionState(progress_s, sample.segment_index, True),
        x_m=sample.x_m,
        y_m=sample.y_m,
        tangent_yaw_rad=sample.tangent_yaw_rad,
        cross_track_m=cross_track,
        distance_m=math.hypot(error_x, error_y),
    )


def body_vector_for_bearing(
    requested_speed_mps: float,
    bearing_error_rad: float,
    max_bearing_deg: float,
    body_y_for_ccw_sign: float = -1.0,
) -> tuple[float, float, float]:
    """Preserve command magnitude while converting a local bearing to BODY x/y."""
    if not all(
        math.isfinite(value)
        for value in (
            requested_speed_mps,
            bearing_error_rad,
            max_bearing_deg,
            body_y_for_ccw_sign,
        )
    ):
        raise ValueError("body-vector inputs must be finite")
    if requested_speed_mps < 0.0 or not 0.0 < max_bearing_deg < 90.0:
        raise ValueError("invalid body-vector bounds")
    if body_y_for_ccw_sign not in {-1.0, 1.0}:
        raise ValueError("BODY y direction sign must be exactly -1 or 1")
    limited = clamp(
        wrap_pi(bearing_error_rad),
        -math.radians(max_bearing_deg),
        math.radians(max_bearing_deg),
    )
    return (
        requested_speed_mps * math.cos(limited),
        body_y_for_ccw_sign * requested_speed_mps * math.sin(limited),
        limited,
    )


def slew_limited_body_bearing(
    desired_bearing_rad: float,
    previous_bearing_rad: float,
    max_bearing_deg: float,
    max_rate_degps: float,
    dt_sec: float,
) -> float:
    """Bound the commanded BODY bearing and its frame-to-frame change."""
    if not all(
        math.isfinite(value)
        for value in (
            desired_bearing_rad,
            previous_bearing_rad,
            max_bearing_deg,
            max_rate_degps,
            dt_sec,
        )
    ):
        raise ValueError("body-bearing slew inputs must be finite")
    if not 0.0 < max_bearing_deg < 90.0 or max_rate_degps < 0.0 or dt_sec <= 0.0:
        raise ValueError("invalid body-bearing slew bounds")
    limit_rad = math.radians(max_bearing_deg)
    desired = clamp(wrap_pi(desired_bearing_rad), -limit_rad, limit_rad)
    previous = clamp(wrap_pi(previous_bearing_rad), -limit_rad, limit_rad)
    if max_rate_degps == 0.0:
        return desired
    max_step_rad = math.radians(max_rate_degps) * min(dt_sec, 0.20)
    return clamp(desired, previous - max_step_rad, previous + max_step_rad)


def trajectory_reference_curvature(
    trajectory: PolylineTrajectory,
    s_m: float,
    window_m: float,
) -> float:
    """Estimate signed path curvature from tangent change over arc length."""
    if not math.isfinite(s_m) or not math.isfinite(window_m):
        raise ValueError("reference curvature inputs must be finite")
    if window_m <= 0.0:
        raise ValueError("reference curvature window must be positive")
    half_window = 0.5 * window_m
    lower_s = clamp(s_m - half_window, 0.0, trajectory.length_m)
    upper_s = clamp(s_m + half_window, 0.0, trajectory.length_m)
    span_m = upper_s - lower_s
    if span_m <= 1.0e-6:
        return 0.0
    lower_yaw = trajectory.sample(lower_s).tangent_yaw_rad
    upper_yaw = trajectory.sample(upper_s).tangent_yaw_rad
    return wrap_pi(upper_yaw - lower_yaw) / span_m


def assess_trajectory_feasibility(
    trajectory: PolylineTrajectory,
    *,
    reference_window_m: float,
    curvature_to_body_gain_m: float,
    max_body_bearing_deg: float,
    minimum_bearing_reserve_deg: float,
    max_curvature_rate_inv_m2: float,
    sample_spacing_m: float = 0.05,
) -> TrajectoryFeasibilityReport:
    """Check arbitrary path geometry against one vehicle adapter's limits."""
    values = (
        reference_window_m,
        curvature_to_body_gain_m,
        max_body_bearing_deg,
        minimum_bearing_reserve_deg,
        max_curvature_rate_inv_m2,
        sample_spacing_m,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("trajectory feasibility limits must be finite")
    if reference_window_m <= 0.0 or sample_spacing_m <= 0.0:
        raise ValueError("trajectory feasibility spacing/window must be positive")
    if curvature_to_body_gain_m < 0.0 or max_curvature_rate_inv_m2 <= 0.0:
        raise ValueError("trajectory feasibility curvature limits are invalid")
    if not 0.0 <= minimum_bearing_reserve_deg < max_body_bearing_deg < 90.0:
        raise ValueError("trajectory feasibility bearing limits are invalid")

    count = max(2, math.ceil(trajectory.length_m / sample_spacing_m))
    samples_s = [trajectory.length_m * index / count for index in range(count + 1)]
    curvatures = [
        trajectory_reference_curvature(trajectory, s_m, reference_window_m)
        for s_m in samples_s
    ]
    max_abs_curvature = max(abs(value) for value in curvatures)
    max_abs_rate = max(
        abs(following - current) / (samples_s[index + 1] - samples_s[index])
        for index, (current, following) in enumerate(
            zip(curvatures, curvatures[1:])
        )
    )
    nominal_bearing_deg = math.degrees(
        math.atan(curvature_to_body_gain_m * max_abs_curvature)
    )
    reasons: list[str] = []
    if nominal_bearing_deg > max_body_bearing_deg - minimum_bearing_reserve_deg:
        reasons.append("insufficient_body_bearing_reserve")
    if max_abs_rate > max_curvature_rate_inv_m2:
        reasons.append("curvature_rate_limit")
    return TrajectoryFeasibilityReport(
        feasible=not reasons,
        max_abs_curvature_inv_m=max_abs_curvature,
        max_abs_curvature_rate_inv_m2=max_abs_rate,
        max_nominal_body_bearing_deg=nominal_bearing_deg,
        reasons=tuple(reasons),
    )


def bounded_path_curvature(
    raw_curvature_inv_m: float,
    reference_curvature_inv_m: float,
    max_correction_inv_m: float,
) -> float:
    """Bound tracking feedback around nominal path curvature."""
    if not all(
        math.isfinite(value)
        for value in (
            raw_curvature_inv_m,
            reference_curvature_inv_m,
            max_correction_inv_m,
        )
    ):
        raise ValueError("bounded curvature inputs must be finite")
    if max_correction_inv_m < 0.0:
        raise ValueError("curvature correction limit must be nonnegative")
    if max_correction_inv_m == 0.0:
        return raw_curvature_inv_m
    return reference_curvature_inv_m + clamp(
        raw_curvature_inv_m - reference_curvature_inv_m,
        -max_correction_inv_m,
        max_correction_inv_m,
    )


def curvature_for_body_adapter(
    reference_curvature_inv_m: float,
    combined_curvature_inv_m: float,
    feedback_gain_ratio: float,
) -> float:
    """Apply vehicle calibration separately to path feed-forward and feedback."""
    if not all(
        math.isfinite(value)
        for value in (
            reference_curvature_inv_m,
            combined_curvature_inv_m,
            feedback_gain_ratio,
        )
    ):
        raise ValueError("curvature adapter inputs must be finite")
    if feedback_gain_ratio <= 0.0:
        raise ValueError("curvature feedback gain ratio must be positive")
    return reference_curvature_inv_m + feedback_gain_ratio * (
        combined_curvature_inv_m - reference_curvature_inv_m
    )


def bounded_cross_track_integral_step(
    previous_integral_m_sec: float,
    cross_track_m: float,
    dt_sec: float,
    integral_gain_inv_m_per_m_sec: float,
    integral_limit_m_sec: float,
    raw_feedback_delta_inv_m: float,
    max_correction_inv_m: float,
) -> float:
    """Integrate persistent lateral bias without winding up a saturated turn."""
    values = (
        previous_integral_m_sec,
        cross_track_m,
        dt_sec,
        integral_gain_inv_m_per_m_sec,
        integral_limit_m_sec,
        raw_feedback_delta_inv_m,
        max_correction_inv_m,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("cross-track integral inputs must be finite")
    if dt_sec <= 0.0:
        raise ValueError("cross-track integral dt must be positive")
    if integral_gain_inv_m_per_m_sec < 0.0 or integral_limit_m_sec < 0.0:
        raise ValueError("cross-track integral gain/limit must be nonnegative")
    if max_correction_inv_m < 0.0:
        raise ValueError("curvature correction limit must be nonnegative")
    if integral_gain_inv_m_per_m_sec == 0.0 or integral_limit_m_sec == 0.0:
        return 0.0

    candidate = clamp(
        previous_integral_m_sec + cross_track_m * min(dt_sec, 0.20),
        -integral_limit_m_sec,
        integral_limit_m_sec,
    )
    previous_delta = (
        raw_feedback_delta_inv_m
        - integral_gain_inv_m_per_m_sec * previous_integral_m_sec
    )
    candidate_delta = (
        raw_feedback_delta_inv_m
        - integral_gain_inv_m_per_m_sec * candidate
    )
    if max_correction_inv_m > 0.0:
        candidate_limited = clamp(
            candidate_delta,
            -max_correction_inv_m,
            max_correction_inv_m,
        )
        unwinds_existing_integral = abs(candidate) < abs(previous_integral_m_sec)
        if (
            not unwinds_existing_integral
            and candidate_delta != candidate_limited
            and abs(candidate_delta) > abs(previous_delta)
        ):
            return previous_integral_m_sec
    return candidate


def trajectory_tracking_step(
    trajectory: PolylineTrajectory,
    state: TrajectoryTrackerState,
    x_m: float,
    y_m: float,
    yaw_rad: float,
    measured_speed_mps: float,
    config: TrajectoryTrackerConfig = TrajectoryTrackerConfig(),
    dt_sec: float = 0.05,
    speed_ceiling_mps: float | None = None,
) -> TrajectoryCommand:
    """Compute one fail-bounded geometric tracking command."""
    if not all(
        math.isfinite(value)
        for value in (x_m, y_m, yaw_rad, measured_speed_mps, dt_sec)
    ):
        raise ValueError("trajectory tracking inputs must be finite")
    if measured_speed_mps < 0.0:
        raise ValueError("measured speed must be nonnegative")
    if dt_sec <= 0.0:
        raise ValueError("trajectory tracking dt must be positive")
    if speed_ceiling_mps is not None and (
        not math.isfinite(speed_ceiling_mps)
        or not 0.0 <= speed_ceiling_mps <= 0.25
    ):
        raise ValueError("trajectory speed ceiling must be within [0, 0.25]")
    validate_tracker_config(config)
    projection = project_onto_trajectory(
        trajectory, state.projection, x_m, y_m, config
    )
    remaining = max(0.0, trajectory.length_m - projection.state.progress_s_m)
    goal = trajectory.points[-1]
    # A nonholonomic rover must finish inside a path-aligned terminal gate.
    # Requiring a shrinking Euclidean circle makes the lookahead chase the
    # endpoint sideways and can command a sharp turn immediately before stop.
    terminal_window_reached = remaining <= config.goal_tolerance_m
    goal_reached = bool(
        terminal_window_reached
        and abs(projection.cross_track_m) <= config.goal_tolerance_m
    )
    terminal_missed = terminal_window_reached and not goal_reached
    if goal_reached or terminal_missed:
        return TrajectoryCommand(
            state=TrajectoryTrackerState(
                projection.state,
                state.cross_track_integral_m_sec,
                state.body_bearing_rad,
            ),
            progress_s_m=projection.state.progress_s_m,
            remaining_s_m=remaining,
            cross_track_m=projection.cross_track_m,
            lookahead_m=0.0,
            target_x_m=goal.x_m,
            target_y_m=goal.y_m,
            target_bearing_error_rad=0.0,
            curvature_inv_m=0.0,
            raw_curvature_inv_m=0.0,
            reference_curvature_inv_m=0.0,
            feedback_curvature_inv_m=0.0,
            adapter_curvature_inv_m=0.0,
            cross_track_integral_m_sec=state.cross_track_integral_m_sec,
            v_mps=0.0,
            omega_radps=0.0,
            body_x_mps=0.0,
            body_y_mps=0.0,
            phase="GOAL" if goal_reached else "TERMINAL_MISS",
            goal_reached=goal_reached,
            terminal_missed=terminal_missed,
        )

    lookahead = clamp(
        config.base_lookahead_m
        + config.speed_lookahead_gain_sec * measured_speed_mps,
        config.min_lookahead_m,
        config.max_lookahead_m,
    )
    target_s_m = projection.state.progress_s_m + lookahead
    if target_s_m <= trajectory.length_m:
        target = trajectory.sample(target_s_m)
        target_x_m = target.x_m
        target_y_m = target.y_m
    else:
        # Keep a full geometric lookahead by extending the terminal tangent.
        # This is a steering-only virtual point; progress and completion remain
        # bounded by the finite trajectory itself.
        target = trajectory.sample(trajectory.length_m)
        extension_m = target_s_m - trajectory.length_m
        target_x_m = target.x_m + extension_m * math.cos(target.tangent_yaw_rad)
        target_y_m = target.y_m + extension_m * math.sin(target.tangent_yaw_rad)
    target_distance = math.hypot(target_x_m - x_m, target_y_m - y_m)
    bearing_error = wrap_pi(
        math.atan2(target_y_m - y_m, target_x_m - x_m) - yaw_rad
    )
    effective_lookahead = max(target_distance, config.min_lookahead_m)
    feedback_curvature = 2.0 * math.sin(bearing_error) / effective_lookahead
    reference_curvature = trajectory_reference_curvature(
        trajectory,
        projection.state.progress_s_m,
        config.reference_curvature_window_m,
    )
    raw_feedback_delta = feedback_curvature - reference_curvature
    cross_track_integral = bounded_cross_track_integral_step(
        state.cross_track_integral_m_sec,
        projection.cross_track_m,
        dt_sec,
        config.cross_track_integral_gain_inv_m_per_m_sec,
        config.cross_track_integral_limit_m_sec,
        raw_feedback_delta,
        config.max_curvature_correction_inv_m,
    )
    integral_curvature = (
        config.cross_track_integral_gain_inv_m_per_m_sec
        * cross_track_integral
    )
    # Pure pursuit already contains the nominal path curvature when the rover
    # is exactly on a curve. Bound only its correction around that nominal
    # curvature so accumulated cross-track error cannot trigger an abrupt
    # near-pivot command.
    combined_curvature = bounded_path_curvature(
        feedback_curvature - integral_curvature,
        reference_curvature,
        config.max_curvature_correction_inv_m,
    )
    feedback_curvature_delta = combined_curvature - reference_curvature
    adapter_curvature = curvature_for_body_adapter(
        reference_curvature,
        combined_curvature,
        config.curvature_feedback_gain_ratio,
    )

    requested_speed = min(
        trajectory.sample(projection.state.progress_s_m).requested_speed_mps,
        target.requested_speed_mps,
    )
    if speed_ceiling_mps is not None:
        requested_speed = min(requested_speed, speed_ceiling_mps)
    curvature_factor = 1.0 / (
        1.0 + config.curvature_slowdown_gain * abs(combined_curvature)
    )
    terminal_factor = clamp(
        remaining / config.terminal_slowdown_distance_m,
        0.0,
        1.0,
    )
    speed_floor = min(config.minimum_tracking_speed_mps, requested_speed)
    requested_speed = max(
        speed_floor,
        requested_speed * min(curvature_factor, terminal_factor),
    )
    # atan(K * curvature) is speed-independent and remains smooth below 90
    # degrees. This avoids the measured-speed spikes that previously drove the
    # adapter to an almost-pivoting 89-degree command.
    calibrated_bearing = math.atan(
        config.curvature_to_body_gain_m * adapter_curvature
    )
    adapter_bearing = (
        calibrated_bearing
        if config.curvature_to_body_gain_m > 0.0
        else bearing_error
    )
    adapter_bearing = slew_limited_body_bearing(
        adapter_bearing,
        state.body_bearing_rad,
        config.max_body_bearing_deg,
        config.max_body_bearing_rate_degps,
        dt_sec,
    )
    body_x, body_y, limited_bearing = body_vector_for_bearing(
        requested_speed,
        adapter_bearing,
        config.max_body_bearing_deg,
        config.body_y_for_ccw_sign,
    )
    omega = clamp(
        requested_speed * combined_curvature,
        -config.max_yaw_rate_radps,
        config.max_yaw_rate_radps,
    )
    return TrajectoryCommand(
        state=TrajectoryTrackerState(
            projection.state,
            cross_track_integral,
            limited_bearing,
        ),
        progress_s_m=projection.state.progress_s_m,
        remaining_s_m=remaining,
        cross_track_m=projection.cross_track_m,
        lookahead_m=lookahead,
        target_x_m=target_x_m,
        target_y_m=target_y_m,
        target_bearing_error_rad=limited_bearing,
        curvature_inv_m=combined_curvature,
        raw_curvature_inv_m=feedback_curvature,
        reference_curvature_inv_m=reference_curvature,
        feedback_curvature_inv_m=feedback_curvature_delta,
        adapter_curvature_inv_m=adapter_curvature,
        cross_track_integral_m_sec=cross_track_integral,
        v_mps=requested_speed,
        omega_radps=omega,
        body_x_mps=body_x,
        body_y_mps=body_y,
        phase=target.phase,
        goal_reached=False,
        terminal_missed=False,
    )


def _append_unique(points: list[TrajectoryPoint], point: TrajectoryPoint) -> None:
    if points and math.hypot(point.x_m - points[-1].x_m, point.y_m - points[-1].y_m) <= 1.0e-6:
        points[-1] = point
    else:
        points.append(point)


def sampled_straight(
    start_x_m: float,
    start_y_m: float,
    yaw_rad: float,
    length_m: float,
    speed_mps: float,
    spacing_m: float,
    phase: str,
) -> list[TrajectoryPoint]:
    if not all(
        math.isfinite(value)
        for value in (start_x_m, start_y_m, yaw_rad, length_m, speed_mps, spacing_m)
    ):
        raise ValueError("straight trajectory inputs must be finite")
    if length_m <= 0.0 or spacing_m <= 0.0:
        raise ValueError("straight length and spacing must be positive")
    count = max(1, math.ceil(length_m / spacing_m))
    return [
        TrajectoryPoint(
            start_x_m + math.cos(yaw_rad) * length_m * index / count,
            start_y_m + math.sin(yaw_rad) * length_m * index / count,
            speed_mps,
            phase,
        )
        for index in range(count + 1)
    ]


def sampled_arc(
    start_x_m: float,
    start_y_m: float,
    start_yaw_rad: float,
    radius_m: float,
    sweep_rad: float,
    speed_mps: float,
    spacing_m: float,
    phase: str,
) -> list[TrajectoryPoint]:
    if not all(
        math.isfinite(value)
        for value in (
            start_x_m,
            start_y_m,
            start_yaw_rad,
            radius_m,
            sweep_rad,
            speed_mps,
            spacing_m,
        )
    ):
        raise ValueError("arc trajectory inputs must be finite")
    if radius_m <= 0.0 or spacing_m <= 0.0 or sweep_rad == 0.0:
        raise ValueError("arc radius/spacing must be positive and sweep nonzero")
    side = math.copysign(1.0, sweep_rad)
    center_x = start_x_m - side * radius_m * math.sin(start_yaw_rad)
    center_y = start_y_m + side * radius_m * math.cos(start_yaw_rad)
    radial_start = start_yaw_rad - side * math.pi / 2.0
    count = max(2, math.ceil(abs(radius_m * sweep_rad) / spacing_m))
    points = []
    for index in range(count + 1):
        angle = radial_start + sweep_rad * index / count
        points.append(
            TrajectoryPoint(
                center_x + radius_m * math.cos(angle),
                center_y + radius_m * math.sin(angle),
                speed_mps,
                phase,
            )
        )
    return points


def sampled_quintic_lateral_shift(
    start_x_m: float,
    start_y_m: float,
    start_yaw_rad: float,
    forward_length_m: float,
    lateral_offset_m: float,
    speed_mps: float,
    spacing_m: float,
    phase: str,
) -> list[TrajectoryPoint]:
    """Sample a zero-slope, zero-curvature lateral shift.

    The quintic smoothstep ``10u^3 - 15u^4 + 6u^5`` has zero first and
    second derivatives at both ends.  It therefore joins straight segments
    without the impossible instantaneous curvature reversal of two tangent
    circular arcs.
    """
    if not all(
        math.isfinite(value)
        for value in (
            start_x_m,
            start_y_m,
            start_yaw_rad,
            forward_length_m,
            lateral_offset_m,
            speed_mps,
            spacing_m,
        )
    ):
        raise ValueError("quintic shift inputs must be finite")
    if forward_length_m <= 0.0 or spacing_m <= 0.0 or lateral_offset_m == 0.0:
        raise ValueError("quintic shift length/spacing must be positive and offset nonzero")
    # The lateral polynomial can bend substantially more than its forward
    # extent suggests. Oversample it for stable tangent/curvature estimates.
    count = max(6, math.ceil(3.0 * forward_length_m / spacing_m))
    cos_yaw = math.cos(start_yaw_rad)
    sin_yaw = math.sin(start_yaw_rad)
    points = []
    for index in range(count + 1):
        u = index / count
        blend = 10.0 * u**3 - 15.0 * u**4 + 6.0 * u**5
        local_x = forward_length_m * u
        local_y = lateral_offset_m * blend
        points.append(
            TrajectoryPoint(
                start_x_m + local_x * cos_yaw - local_y * sin_yaw,
                start_y_m + local_x * sin_yaw + local_y * cos_yaw,
                speed_mps,
                phase,
            )
        )
    return points


def sampled_quintic_heading_reversal(
    start_x_m: float,
    start_y_m: float,
    start_yaw_rad: float,
    radius_m: float,
    turn_left: bool,
    speed_mps: float,
    spacing_m: float,
    phase: str,
) -> list[TrajectoryPoint]:
    """Sample a tangent- and curvature-continuous 180-degree reversal.

    The symmetric quintic Bezier has collinear first three and last three
    control points, so curvature is zero where it joins the incoming and
    outgoing straights. ``radius_m`` retains the familiar 2R lane separation;
    the smooth transition uses about 1.25R forward clearance.
    """
    if not all(
        math.isfinite(value)
        for value in (
            start_x_m,
            start_y_m,
            start_yaw_rad,
            radius_m,
            speed_mps,
            spacing_m,
        )
    ):
        raise ValueError("quintic reversal inputs must be finite")
    if radius_m <= 0.0 or speed_mps <= 0.0 or spacing_m <= 0.0:
        raise ValueError("quintic reversal radius/speed/spacing must be positive")

    side = 1.0 if turn_left else -1.0
    entry_control_m = (2.0 / 3.0) * radius_m
    inner_control_m = (5.0 / 3.0) * radius_m
    separation_m = side * 2.0 * radius_m
    controls = (
        (0.0, 0.0),
        (entry_control_m, 0.0),
        (inner_control_m, 0.0),
        (inner_control_m, separation_m),
        (entry_control_m, separation_m),
        (0.0, separation_m),
    )
    # Curvature feasibility is evaluated from polyline tangents. Oversample
    # this high-curvature primitive so tangent quantization cannot create a
    # false curvature-rate spike at control-point joins.
    count = max(24, math.ceil(12.0 * radius_m / spacing_m))
    cos_yaw = math.cos(start_yaw_rad)
    sin_yaw = math.sin(start_yaw_rad)
    points = []
    for index in range(count + 1):
        u = index / count
        one_minus_u = 1.0 - u
        weights = (
            one_minus_u**5,
            5.0 * one_minus_u**4 * u,
            10.0 * one_minus_u**3 * u**2,
            10.0 * one_minus_u**2 * u**3,
            5.0 * one_minus_u * u**4,
            u**5,
        )
        local_x = sum(
            weight * control[0] for weight, control in zip(weights, controls)
        )
        local_y = sum(
            weight * control[1] for weight, control in zip(weights, controls)
        )
        points.append(
            TrajectoryPoint(
                start_x_m + local_x * cos_yaw - local_y * sin_yaw,
                start_y_m + local_x * sin_yaw + local_y * cos_yaw,
                speed_mps,
                phase,
            )
        )
    return points


def build_straight_trajectory(
    start_x_m: float,
    start_y_m: float,
    start_yaw_rad: float,
    distance_m: float,
    speed_mps: float,
    spacing_m: float = 0.15,
) -> PolylineTrajectory:
    return PolylineTrajectory(
        sampled_straight(
            start_x_m,
            start_y_m,
            start_yaw_rad,
            distance_m,
            speed_mps,
            spacing_m,
            "STRAIGHT",
        )
    )


def build_out_and_back_trajectory(
    start_x_m: float,
    start_y_m: float,
    start_yaw_rad: float,
    leg_distance_m: float,
    turn_radius_m: float,
    turn_left: bool,
    straight_speed_mps: float,
    turn_speed_mps: float,
    spacing_m: float = 0.15,
    turn_angle_rad: float = math.pi,
) -> PolylineTrajectory:
    """Build one continuous straight/arc/straight reference path."""
    if not math.isfinite(turn_angle_rad) or not 0.0 < turn_angle_rad <= math.pi:
        raise ValueError("turn angle must be within (0, pi]")
    leg1 = sampled_straight(
        start_x_m,
        start_y_m,
        start_yaw_rad,
        leg_distance_m,
        straight_speed_mps,
        spacing_m,
        "LEG1",
    )
    turn_start = leg1[-1]
    sweep = turn_angle_rad if turn_left else -turn_angle_rad
    if math.isclose(turn_angle_rad, math.pi, abs_tol=1.0e-9):
        turn = sampled_quintic_heading_reversal(
            turn_start.x_m,
            turn_start.y_m,
            start_yaw_rad,
            turn_radius_m,
            turn_left,
            turn_speed_mps,
            spacing_m,
            "UTURN",
        )
    else:
        turn = sampled_arc(
            turn_start.x_m,
            turn_start.y_m,
            start_yaw_rad,
            turn_radius_m,
            sweep,
            turn_speed_mps,
            spacing_m,
            "UTURN",
        )
    turn_end = turn[-1]
    leg2 = sampled_straight(
        turn_end.x_m,
        turn_end.y_m,
        wrap_pi(start_yaw_rad + sweep),
        leg_distance_m,
        straight_speed_mps,
        spacing_m,
        "LEG2",
    )
    points: list[TrajectoryPoint] = []
    for point in (*leg1, *turn, *leg2):
        _append_unique(points, point)
    return PolylineTrajectory(points)


def build_s_bend_return_trajectory(
    start_x_m: float,
    start_y_m: float,
    start_yaw_rad: float,
    initial_straight_m: float,
    turn_radius_m: float,
    straight_speed_mps: float,
    turn_speed_mps: float,
    spacing_m: float = 0.15,
) -> PolylineTrajectory:
    """Build a curvature-continuous S-bend and return-to-start path."""
    if not math.isfinite(initial_straight_m) or initial_straight_m <= 0.0:
        raise ValueError("initial straight distance must be positive")

    leg_out = sampled_straight(
        start_x_m,
        start_y_m,
        start_yaw_rad,
        initial_straight_m,
        straight_speed_mps,
        spacing_m,
        "LEG_OUT",
    )
    s_start = leg_out[-1]
    s_forward_length_m = 4.0 * turn_radius_m
    s_curve = sampled_quintic_lateral_shift(
        s_start.x_m,
        s_start.y_m,
        start_yaw_rad,
        s_forward_length_m,
        -2.0 * turn_radius_m,
        turn_speed_mps,
        spacing_m,
        "S_CURVE",
    )
    uturn_start = s_curve[-1]
    return_turn = sampled_quintic_heading_reversal(
        uturn_start.x_m,
        uturn_start.y_m,
        start_yaw_rad,
        turn_radius_m,
        True,
        turn_speed_mps,
        spacing_m,
        "RETURN_TURN",
    )
    return_start = return_turn[-1]
    return_distance_m = initial_straight_m + s_forward_length_m
    leg_return = sampled_straight(
        return_start.x_m,
        return_start.y_m,
        wrap_pi(start_yaw_rad + math.pi),
        return_distance_m,
        straight_speed_mps,
        spacing_m,
        "LEG_RETURN",
    )

    points: list[TrajectoryPoint] = []
    for point in (
        *leg_out,
        *s_curve,
        *return_turn,
        *leg_return,
    ):
        _append_unique(points, point)
    return PolylineTrajectory(points)


def trajectory_from_xy(
    coordinates: Iterable[tuple[float, float]],
    requested_speed_mps: float,
    phase: str = "TRACK",
) -> PolylineTrajectory:
    """Adapter for future CorridorPlan paths sampled into shared-field ENU."""
    return PolylineTrajectory(
        [
            TrajectoryPoint(x_m, y_m, requested_speed_mps, phase)
            for x_m, y_m in coordinates
        ]
    )
