#!/usr/bin/env python3

"""Guarded C2 outdoor MAVROS Offboard trajectory execution."""

from __future__ import annotations

import argparse
import math
import os
import select
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Sequence

try:
    from src.orin2_trajectory_tracker import (
        PolylineTrajectory,
        TrajectoryTrackerConfig,
        TrajectoryTrackerState,
        TrajectoryFeasibilityReport,
        assess_trajectory_feasibility,
        build_out_and_back_trajectory,
        build_s_bend_return_trajectory,
        build_straight_trajectory,
        tracker_state_at_route_start,
        trajectory_tracking_step,
        validate_tracker_config,
    )
    from src.rover_rviz_trajectory import RvizTrajectoryPublisher
except ModuleNotFoundError:  # Direct execution from the src directory.
    from orin2_trajectory_tracker import (
        PolylineTrajectory,
        TrajectoryTrackerConfig,
        TrajectoryTrackerState,
        TrajectoryFeasibilityReport,
        assess_trajectory_feasibility,
        build_out_and_back_trajectory,
        build_s_bend_return_trajectory,
        build_straight_trajectory,
        tracker_state_at_route_start,
        trajectory_tracking_step,
        validate_tracker_config,
    )
    from rover_rviz_trajectory import RvizTrajectoryPublisher


EXECUTE_PHRASE = "OUTDOOR_FORWARD_5M_AREA_CLEAR_RC_KILL_READY"
ENTRY_HOLD_EXECUTE_PHRASE = "OUTDOOR_ENTRY_HOLD_WHEELS_LIFTED_RC_KILL_READY"
UTURN_EXECUTE_PHRASE = "OUTDOOR_6M_RIGHT_UTURN_6M_AREA_CLEAR_RC_KILL_READY"
LEFT_UTURN_EXECUTE_PHRASE = "OUTDOOR_5M_LEFT_UTURN_5M_AREA_CLEAR_RC_KILL_READY"
S_BEND_RETURN_EXECUTE_PHRASE = "OUTDOOR_S_BEND_RETURN_AREA_CLEAR_RC_KILL_READY"
STATE_MAX_AGE_SEC = 2.0
POSE_MAX_AGE_SEC = 1.0
GPS_MAX_AGE_SEC = 2.0
GPS_RAW_MAX_AGE_SEC = 2.0
MIN_GPS_FIX_TYPE = 3
MIN_SATELLITES_VISIBLE = 6
CONTROL_PERIOD_SEC = 0.05
# PX4 v1.17 differential Offboard computes yaw as atan2(vy, vx). An exact
# zero velocity therefore means global yaw zero, not "keep current heading".
# This tiny BODY_NED +x vector keeps yaw defined while remaining far below one
# PWM count with the measured RO_MAX_THR_SPEED=0.25 m/s configuration.
OFFBOARD_HEADING_HOLD_SPEED_MPS = 1.0e-4
OFFBOARD_PREARM_SETTLE_SEC = 0.30
# Three heading-defined stop frames replace the motion command before Disarm
# without leaving the rover armed in OFFBOARD long enough to hunt at rest.
EXIT_STOP_BURST_SEC = 3 * CONTROL_PERIOD_SEC
ENTRY_PHASE_SPEEDS = {
    "zero_prestream": 0.0,
    "manual_arm_wait": 0.0,
    "request_offboard": OFFBOARD_HEADING_HOLD_SPEED_MPS,
    "verify_offboard": OFFBOARD_HEADING_HOLD_SPEED_MPS,
    "prearm_offboard_settle": OFFBOARD_HEADING_HOLD_SPEED_MPS,
    "request_arm": OFFBOARD_HEADING_HOLD_SPEED_MPS,
    "verify_arm": OFFBOARD_HEADING_HOLD_SPEED_MPS,
    "offboard_stop": OFFBOARD_HEADING_HOLD_SPEED_MPS,
    "motion": None,
}
EXTERNAL_START_GATE_MAX_BUFFER_BYTES = 256
EXTERNAL_RUNTIME_STOP_MAX_BUFFER_BYTES = 256


@dataclass(frozen=True)
class Observation:
    state_present: bool = False
    state_age_sec: float = math.inf
    connected: bool = False
    armed: bool = False
    mode: str = ""
    manual_input: bool = False
    pose_present: bool = False
    pose_age_sec: float = math.inf
    x_m: float = math.nan
    y_m: float = math.nan
    yaw_rad: float = math.nan
    gps_present: bool = False
    gps_age_sec: float = math.inf
    gps_status: int = -1
    latitude_deg: float = math.nan
    longitude_deg: float = math.nan
    gps_raw_present: bool = False
    gps_raw_age_sec: float = math.inf
    gps_fix_type: int = 0
    satellites_visible: int = 0


@dataclass(frozen=True)
class MissionConfig:
    distance_m: float = 5.0
    tolerance_m: float = 0.15
    speed_mps: float = 0.06
    max_speed_mps: float = 0.15
    terminal_speed_mps: float = 0.035
    terminal_slowdown_distance_m: float = 1.5
    max_cross_track_m: float = 1.5
    max_heading_error_deg: float = 35.0
    # Absolute mission ceiling. Individual trajectory runs derive a shorter
    # length-aware deadline while stall and tracking guards remain active.
    max_motion_sec: float = 180.0
    stall_window_sec: float = 8.0
    stall_min_progress_m: float = 0.08
    course_calibration_distance_m: float = 1.0
    course_calibration_speed_mps: float = 0.04
    course_calibration_max_sec: float = 5.0
    course_calibration_max_yaw_change_deg: float = 20.0
    reference_mode: str = "ground_course_rollout"
    steering_trim_mps: float = 0.0
    heading_kp: float = 0.30
    heading_ki: float = 0.02
    heading_kd: float = 0.03
    heading_integral_limit_rad_sec: float = 0.25
    heading_derivative_tau_sec: float = 0.20
    max_steering_mps: float = 0.06
    min_effective_steering_mps: float = 0.03
    heading_deadband_deg: float = 1.0
    cross_track_lookahead_m: float = 0.8
    cross_track_deadband_m: float = 0.15
    cross_track_filter_tau_sec: float = 0.20
    max_path_heading_correction_deg: float = 15.0
    # Vehicle-specific hardware adapter. Geometry and feedback always use
    # positive semantic forward; only the final BODY_NED x publication flips.
    body_forward_sign: float = 1.0
    pose_course_disagreement_enter_deg: float = 8.0
    pose_course_disagreement_exit_deg: float = 4.0
    # 2026-08-06 ground log: negative BODY_NED y increased a physical left
    # deviation. Positive y is therefore the corrective command for positive
    # path-frame cross track on this Orin2 drivetrain.
    steering_direction_sign: float = 1.0
    u_turn: bool = False
    s_bend_return: bool = False
    turn_direction_sign: float = 1.0
    turn_angle_deg: float = 180.0
    turn_tolerance_deg: float = 8.0
    turn_forward_speed_mps: float = 0.05
    turn_lateral_speed_mps: float = 0.04
    turn_max_sec: float = 45.0
    turn_completion_hold_sec: float = 0.30
    turn_stall_window_sec: float = 8.0
    turn_stall_min_progress_deg: float = 5.0
    turn_clearance_radius_m: float = 3.5
    turn_radius_m: float = 3.0
    trajectory_spacing_m: float = 0.15
    tracker_base_lookahead_m: float = 0.80
    tracker_speed_lookahead_gain_sec: float = 0.80
    tracker_min_lookahead_m: float = 0.65
    tracker_max_lookahead_m: float = 1.60
    tracker_projection_backtrack_m: float = 0.20
    tracker_projection_ahead_m: float = 4.0
    tracker_max_body_bearing_deg: float = 32.0
    tracker_max_body_bearing_rate_degps: float = 45.0
    tracker_max_yaw_rate_radps: float = 0.35
    tracker_curvature_slowdown_gain: float = 0.70
    tracker_reference_curvature_window_m: float = 0.45
    tracker_max_curvature_correction_inv_m: float = 0.0
    tracker_curvature_to_body_gain_m: float = 0.0
    tracker_curvature_feedback_gain_ratio: float = 1.0
    tracker_cross_track_integral_gain_inv_m_per_m_sec: float = 0.0
    tracker_cross_track_integral_limit_m_sec: float = 0.0
    tracker_min_nominal_bearing_reserve_deg: float = 3.0
    tracker_max_reference_curvature_rate_inv_m2: float = 1.0


@dataclass(frozen=True)
class HeadingPidState:
    integral_rad_sec: float = 0.0
    previous_error_rad: float | None = None
    derivative_radps: float = 0.0


@dataclass(frozen=True)
class PathFollowerState:
    heading_pid: HeadingPidState = HeadingPidState()
    filtered_cross_track_m: float | None = None


@dataclass(frozen=True)
class PathReference:
    axis_yaw_rad: float
    pose_yaw_zero_rad: float


@dataclass(frozen=True)
class TurnTracker:
    previous_yaw_rad: float
    accumulated_rad: float = 0.0


def validate_config(config: MissionConfig) -> None:
    values = (
        value
        for name, value in vars(config).items()
        if name not in {"u_turn", "s_bend_return", "reference_mode"}
    )
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError("mission configuration must contain only finite values")
    if not 0.5 <= config.distance_m <= 10.0:
        raise ValueError("distance_m must be within [0.5, 10.0]")
    if not 0.0 <= config.tolerance_m < config.distance_m:
        raise ValueError("tolerance_m must be nonnegative and below distance_m")
    if not 0.0 < config.speed_mps <= config.max_speed_mps <= 0.25:
        raise ValueError("speed must be positive and no greater than the 0.25 m/s rover limit")
    if not 0.0 < config.terminal_speed_mps <= config.speed_mps:
        raise ValueError("terminal_speed_mps must be positive and no greater than speed_mps")
    if not 0.5 <= config.terminal_slowdown_distance_m <= 3.0:
        raise ValueError("terminal_slowdown_distance_m must be within [0.5, 3.0]")
    if not 0.1 <= config.max_cross_track_m <= 2.0:
        raise ValueError("max_cross_track_m must be within [0.1, 2.0]")
    if not 5.0 <= config.max_heading_error_deg <= 60.0:
        raise ValueError("max_heading_error_deg must be within [5, 60]")
    if not 10.0 <= config.max_motion_sec <= 180.0:
        raise ValueError("max_motion_sec must be within [10, 180]")
    if not 2.0 <= config.stall_window_sec <= 30.0:
        raise ValueError("stall_window_sec must be within [2, 30]")
    if not 0.01 <= config.stall_min_progress_m <= 1.0:
        raise ValueError("stall_min_progress_m must be within [0.01, 1.0]")
    if not -0.04 <= config.steering_trim_mps <= 0.04:
        raise ValueError("steering_trim_mps must be within [-0.04, 0.04]")
    if not 0.0 <= config.heading_kp <= 1.0:
        raise ValueError("heading_kp must be within [0, 1.0]")
    if not 0.0 <= config.heading_ki <= 0.2:
        raise ValueError("heading_ki must be within [0, 0.2]")
    if not 0.0 <= config.heading_kd <= 0.2:
        raise ValueError("heading_kd must be within [0, 0.2]")
    if not 0.0 < config.heading_integral_limit_rad_sec <= 1.0:
        raise ValueError("heading_integral_limit_rad_sec must be within (0, 1.0]")
    if not 0.01 <= config.heading_derivative_tau_sec <= 1.0:
        raise ValueError("heading_derivative_tau_sec must be within [0.01, 1.0]")
    if not 0.01 <= config.max_steering_mps <= 0.08:
        raise ValueError("max_steering_mps must be within [0.01, 0.08]")
    if abs(config.steering_trim_mps) > config.max_steering_mps:
        raise ValueError("steering_trim_mps must not exceed max_steering_mps")
    if not 0.0 <= config.min_effective_steering_mps <= config.max_steering_mps:
        raise ValueError("min_effective_steering_mps must be within steering limit")
    if not 0.1 <= config.heading_deadband_deg <= 3.0:
        raise ValueError("heading_deadband_deg must be within [0.1, 3.0]")
    if not 0.5 <= config.cross_track_lookahead_m <= 10.0:
        raise ValueError("cross_track_lookahead_m must be within [0.5, 10.0]")
    if not 0.0 <= config.cross_track_deadband_m <= 0.25:
        raise ValueError("cross_track_deadband_m must be within [0, 0.25]")
    if not 0.05 <= config.cross_track_filter_tau_sec <= 2.0:
        raise ValueError("cross_track_filter_tau_sec must be within [0.05, 2.0]")
    if not 2.0 <= config.max_path_heading_correction_deg <= 25.0:
        raise ValueError("max_path_heading_correction_deg must be within [2, 25]")
    if config.body_forward_sign not in {-1.0, 1.0}:
        raise ValueError("body_forward_sign must be exactly -1 or 1")
    if not 3.0 <= config.pose_course_disagreement_enter_deg <= 20.0:
        raise ValueError("pose/course disagreement entry must be within [3, 20]")
    if not 1.0 <= config.pose_course_disagreement_exit_deg < config.pose_course_disagreement_enter_deg:
        raise ValueError("pose/course disagreement exit must be below entry threshold")
    if config.steering_direction_sign not in {-1.0, 1.0}:
        raise ValueError("steering_direction_sign must be exactly -1 or 1")
    if config.u_turn and config.s_bend_return:
        raise ValueError("u_turn and s_bend_return are mutually exclusive")
    if not 0.75 <= config.course_calibration_distance_m <= 1.5:
        raise ValueError("course calibration distance must be within [0.75, 1.5]")
    if config.distance_m <= config.course_calibration_distance_m + config.tolerance_m:
        raise ValueError("mission distance must extend beyond course calibration")
    if not 0.02 <= config.course_calibration_speed_mps <= config.speed_mps:
        raise ValueError("course calibration speed must be within [0.02, speed_mps]")
    if not 3.0 <= config.course_calibration_max_sec <= 8.0:
        raise ValueError("course calibration timeout must be within [3, 8]")
    if not 10.0 <= config.course_calibration_max_yaw_change_deg <= 30.0:
        raise ValueError("course calibration yaw limit must be within [10, 30]")
    if config.reference_mode not in {"ground_course_rollout", "initial_yaw"}:
        raise ValueError("reference_mode must be ground_course_rollout or initial_yaw")
    if not 90.0 <= config.turn_angle_deg <= 180.0:
        raise ValueError("turn_angle_deg must be within [90, 180]")
    if config.turn_direction_sign not in {-1.0, 1.0}:
        raise ValueError("turn_direction_sign must be exactly -1 or 1")
    if not 1.0 <= config.turn_tolerance_deg <= 15.0:
        raise ValueError("turn_tolerance_deg must be within [1, 15]")
    if not 0.02 <= config.turn_forward_speed_mps <= config.max_speed_mps:
        raise ValueError("turn_forward_speed_mps must be within [0.02, max_speed_mps]")
    if not 0.02 <= config.turn_lateral_speed_mps <= 0.08:
        raise ValueError("turn_lateral_speed_mps must be within [0.02, 0.08]")
    if math.hypot(config.turn_forward_speed_mps, config.turn_lateral_speed_mps) > config.max_speed_mps:
        raise ValueError("combined turn command must not exceed max_speed_mps")
    if not 10.0 <= config.turn_max_sec <= 60.0:
        raise ValueError("turn_max_sec must be within [10, 60]")
    if not 0.2 <= config.turn_completion_hold_sec <= 1.0:
        raise ValueError("turn_completion_hold_sec must be within [0.2, 1.0]")
    if not 3.0 <= config.turn_stall_window_sec <= 15.0:
        raise ValueError("turn_stall_window_sec must be within [3, 15]")
    if not 1.0 <= config.turn_stall_min_progress_deg <= 30.0:
        raise ValueError("turn_stall_min_progress_deg must be within [1, 30]")
    if not 0.5 <= config.turn_clearance_radius_m <= 5.0:
        raise ValueError("turn_clearance_radius_m must be within [0.5, 5.0]")
    if not 1.0 <= config.turn_radius_m <= config.turn_clearance_radius_m:
        raise ValueError("turn_radius_m must be within [1.0, turn clearance]")
    if not 0.05 <= config.trajectory_spacing_m <= 0.5:
        raise ValueError("trajectory_spacing_m must be within [0.05, 0.5]")
    if not 0.0 <= config.tracker_min_nominal_bearing_reserve_deg <= 10.0:
        raise ValueError("nominal path bearing reserve must be within [0, 10]")
    if not 0.1 <= config.tracker_max_reference_curvature_rate_inv_m2 <= 10.0:
        raise ValueError("reference curvature rate limit must be within [0.1, 10]")
    validate_tracker_config(tracker_config_from_mission(config))
    report = mission_trajectory_feasibility_report(config)
    if not report.feasible:
        raise ValueError(
            "trajectory geometry infeasible:"
            + ",".join(report.reasons)
            + f" curvature={report.max_abs_curvature_inv_m:.3f}"
            + f" curvature_rate={report.max_abs_curvature_rate_inv_m2:.3f}"
            + f" nominal_bearing={report.max_nominal_body_bearing_deg:.2f}deg"
        )


def tracker_config_from_mission(config: MissionConfig) -> TrajectoryTrackerConfig:
    return TrajectoryTrackerConfig(
        base_lookahead_m=config.tracker_base_lookahead_m,
        speed_lookahead_gain_sec=config.tracker_speed_lookahead_gain_sec,
        min_lookahead_m=config.tracker_min_lookahead_m,
        max_lookahead_m=config.tracker_max_lookahead_m,
        projection_backtrack_m=config.tracker_projection_backtrack_m,
        projection_ahead_m=config.tracker_projection_ahead_m,
        max_body_bearing_deg=config.tracker_max_body_bearing_deg,
        max_body_bearing_rate_degps=config.tracker_max_body_bearing_rate_degps,
        max_yaw_rate_radps=config.tracker_max_yaw_rate_radps,
        curvature_slowdown_gain=config.tracker_curvature_slowdown_gain,
        reference_curvature_window_m=config.tracker_reference_curvature_window_m,
        max_curvature_correction_inv_m=(
            config.tracker_max_curvature_correction_inv_m
        ),
        curvature_to_body_gain_m=(
            config.tracker_curvature_to_body_gain_m
        ),
        curvature_feedback_gain_ratio=(
            config.tracker_curvature_feedback_gain_ratio
        ),
        cross_track_integral_gain_inv_m_per_m_sec=(
            config.tracker_cross_track_integral_gain_inv_m_per_m_sec
        ),
        cross_track_integral_limit_m_sec=(
            config.tracker_cross_track_integral_limit_m_sec
        ),
        minimum_tracking_speed_mps=config.terminal_speed_mps,
        terminal_slowdown_distance_m=config.terminal_slowdown_distance_m,
        goal_tolerance_m=config.tolerance_m,
        max_cross_track_m=config.max_cross_track_m,
    )


def build_relative_mission_trajectory(
    config: MissionConfig,
    start_x_m: float,
    start_y_m: float,
    trajectory_yaw_rad: float,
) -> PolylineTrajectory:
    if not all(math.isfinite(value) for value in (start_x_m, start_y_m, trajectory_yaw_rad)):
        raise ValueError("relative mission trajectory origin must be finite")
    if config.s_bend_return:
        return build_s_bend_return_trajectory(
            start_x_m,
            start_y_m,
            trajectory_yaw_rad,
            config.distance_m,
            config.turn_radius_m,
            straight_speed_mps=config.speed_mps,
            turn_speed_mps=config.turn_forward_speed_mps,
            spacing_m=config.trajectory_spacing_m,
        )
    if config.u_turn:
        return build_out_and_back_trajectory(
            start_x_m,
            start_y_m,
            trajectory_yaw_rad,
            config.distance_m,
            config.turn_radius_m,
            turn_left=config.turn_direction_sign < 0.0,
            straight_speed_mps=config.speed_mps,
            turn_speed_mps=config.turn_forward_speed_mps,
            spacing_m=config.trajectory_spacing_m,
            turn_angle_rad=math.radians(config.turn_angle_deg),
        )
    return build_straight_trajectory(
        start_x_m,
        start_y_m,
        trajectory_yaw_rad,
        config.distance_m,
        config.speed_mps,
        spacing_m=config.trajectory_spacing_m,
    )


def mission_trajectory_feasibility_report(
    config: MissionConfig,
) -> TrajectoryFeasibilityReport:
    trajectory = build_relative_mission_trajectory(config, 0.0, 0.0, 0.0)
    return assess_trajectory_feasibility(
        trajectory,
        reference_window_m=config.tracker_reference_curvature_window_m,
        curvature_to_body_gain_m=config.tracker_curvature_to_body_gain_m,
        max_body_bearing_deg=config.tracker_max_body_bearing_deg,
        minimum_bearing_reserve_deg=config.tracker_min_nominal_bearing_reserve_deg,
        max_curvature_rate_inv_m2=config.tracker_max_reference_curvature_rate_inv_m2,
    )


def build_mission_trajectory(
    config: MissionConfig,
    start: Observation,
    initial_yaw_rad: float | None = None,
) -> PolylineTrajectory:
    if not navigation_ready(start):
        raise ValueError("mission trajectory requires fresh navigation")
    trajectory_yaw_rad = start.yaw_rad if initial_yaw_rad is None else initial_yaw_rad
    if not math.isfinite(trajectory_yaw_rad):
        raise ValueError("mission trajectory yaw must be finite")
    return build_relative_mission_trajectory(
        config,
        start.x_m,
        start.y_m,
        trajectory_yaw_rad,
    )


def trajectory_motion_timeout_sec(
    trajectory_length_m: float,
    max_motion_sec: float,
) -> float:
    """Return a finite route-scaled timeout below the configured hard ceiling."""
    if not math.isfinite(trajectory_length_m) or trajectory_length_m <= 0.0:
        raise ValueError("trajectory length must be finite and positive")
    if not math.isfinite(max_motion_sec) or max_motion_sec <= 0.0:
        raise ValueError("maximum motion time must be finite and positive")
    return min(max_motion_sec, max(30.0, 2.5 * trajectory_length_m))


def fitted_forward_course_yaw(
    points: Sequence[tuple[float, float]],
    minimum_displacement_m: float,
) -> float:
    """Fit the directed ground-course axis of a bounded straight rollout."""
    if len(points) < 3:
        raise ValueError("course fit requires at least three points")
    if not 0.5 <= minimum_displacement_m <= 2.0:
        raise ValueError("minimum displacement must be within [0.5, 2.0]")
    if not all(math.isfinite(value) for point in points for value in point):
        raise ValueError("course fit points must be finite")
    start_x, start_y = points[0]
    end_x, end_y = points[-1]
    if math.hypot(end_x - start_x, end_y - start_y) < minimum_displacement_m:
        raise ValueError("course fit displacement is too short")
    mean_x = sum(point[0] for point in points) / len(points)
    mean_y = sum(point[1] for point in points) / len(points)
    covariance_xx = sum((point[0] - mean_x) ** 2 for point in points)
    covariance_xy = sum(
        (point[0] - mean_x) * (point[1] - mean_y) for point in points
    )
    covariance_yy = sum((point[1] - mean_y) ** 2 for point in points)
    yaw_rad = 0.5 * math.atan2(
        2.0 * covariance_xy,
        covariance_xx - covariance_yy,
    )
    if math.cos(yaw_rad) * (end_x - start_x) + math.sin(yaw_rad) * (
        end_y - start_y
    ) < 0.0:
        yaw_rad = wrap_pi(yaw_rad + math.pi)
    return yaw_rad


def tracking_yaw_with_course_offset(
    pose_yaw_rad: float,
    body_course_offset_rad: float,
) -> float:
    """Map pose yaw to the calibrated physical ground-course heading."""
    if not all(math.isfinite(value) for value in (pose_yaw_rad, body_course_offset_rad)):
        raise ValueError("tracking yaw inputs must be finite")
    return wrap_pi(pose_yaw_rad + body_course_offset_rad)


def _finite_pose(observation: Observation) -> bool:
    return all(
        math.isfinite(value)
        for value in (observation.x_m, observation.y_m, observation.yaw_rad)
    )


def _finite_gps(observation: Observation) -> bool:
    return all(
        math.isfinite(value)
        for value in (observation.latitude_deg, observation.longitude_deg)
    )


def navigation_ready(observation: Observation) -> bool:
    return bool(
        observation.state_present
        and observation.state_age_sec <= STATE_MAX_AGE_SEC
        and observation.connected
        and observation.pose_present
        and observation.pose_age_sec <= POSE_MAX_AGE_SEC
        and _finite_pose(observation)
        and observation.gps_present
        and observation.gps_age_sec <= GPS_MAX_AGE_SEC
        and observation.gps_status >= 0
        and _finite_gps(observation)
        and observation.gps_raw_present
        and observation.gps_raw_age_sec <= GPS_RAW_MAX_AGE_SEC
        and observation.gps_fix_type >= MIN_GPS_FIX_TYPE
        and observation.satellites_visible >= MIN_SATELLITES_VISIBLE
    )


def safe_manual_prestate(observation: Observation) -> bool:
    return bool(
        navigation_ready(observation)
        and not observation.armed
        and observation.mode.upper() == "MANUAL"
        and observation.manual_input
    )


def external_start_gate_token(plan_id: int) -> bytes:
    if not 0 < plan_id <= 0xFFFF:
        raise ValueError("external start plan_id must be within uint16 and nonzero")
    return f"PAIRB_START plan_id={plan_id}\n".encode("ascii")


def validate_external_start_gate_args(
    fd: int | None,
    plan_id: int | None,
    timeout_sec: float,
) -> None:
    if (fd is None) != (plan_id is None):
        raise ValueError(
            "external start gate requires both --external-start-gate-fd and "
            "--external-start-plan-id"
        )
    if fd is None:
        return
    if fd < 3:
        raise ValueError("external start gate fd must be at least 3")
    external_start_gate_token(plan_id)
    if not math.isfinite(timeout_sec) or not 1.0 <= timeout_sec <= 600.0:
        raise ValueError("external start gate timeout must be within [1, 600] seconds")


def consume_external_start_gate_bytes(
    buffered: bytes,
    incoming: bytes,
    plan_id: int,
) -> tuple[bytes, bool, str | None]:
    expected = external_start_gate_token(plan_id)
    combined = buffered + incoming
    if len(combined) > EXTERNAL_START_GATE_MAX_BUFFER_BYTES:
        return b"", False, "external_start_gate_buffer_overflow"
    if b"\n" not in combined:
        return combined, False, None
    line, trailing = combined.split(b"\n", 1)
    candidate = line + b"\n"
    if trailing:
        return b"", False, "external_start_gate_trailing_data"
    if candidate != expected:
        return b"", False, "external_start_gate_token_mismatch"
    return b"", True, None


def external_runtime_stop_token(plan_id: int) -> bytes:
    external_start_gate_token(plan_id)
    return f"PAIRB_RUNTIME_COMPLETE plan_id={plan_id}\n".encode("ascii")


def validate_external_runtime_stop_args(
    runtime_stop_fd: int | None,
    plan_id: int | None,
) -> None:
    if runtime_stop_fd is None:
        return
    if runtime_stop_fd < 3 or plan_id is None:
        raise ValueError("runtime stop FD requires a valid external start plan ID")
    external_runtime_stop_token(plan_id)


def consume_external_runtime_stop_bytes(
    buffered: bytes,
    incoming: bytes,
    plan_id: int,
) -> tuple[bytes, bool, str | None]:
    expected = external_runtime_stop_token(plan_id)
    combined = buffered + incoming
    if len(combined) > EXTERNAL_RUNTIME_STOP_MAX_BUFFER_BYTES:
        return b"", False, "external_runtime_stop_buffer_overflow"
    if b"\n" not in combined:
        return combined, False, None
    line, trailing = combined.split(b"\n", 1)
    if trailing:
        return b"", False, "external_runtime_stop_trailing_data"
    if line + b"\n" != expected:
        return b"", False, "external_runtime_stop_token_mismatch"
    return b"", True, None


def manual_arm_ready(observation: Observation) -> bool:
    return bool(
        navigation_ready(observation)
        and observation.armed
        and observation.mode.upper() == "MANUAL"
    )


def motion_fault(observation: Observation) -> str | None:
    if not observation.state_present:
        return "state_missing"
    if observation.state_age_sec > STATE_MAX_AGE_SEC:
        return "state_stale"
    if not observation.connected:
        return "mavros_disconnected"
    if not observation.pose_present:
        return "local_pose_missing"
    if observation.pose_age_sec > POSE_MAX_AGE_SEC:
        return "local_pose_stale"
    if not _finite_pose(observation):
        return "local_pose_nonfinite"
    if not observation.gps_present:
        return "gps_missing"
    if observation.gps_age_sec > GPS_MAX_AGE_SEC:
        return "gps_stale"
    if observation.gps_status < 0 or not _finite_gps(observation):
        return "gps_no_fix"
    if not observation.gps_raw_present:
        return "gps_raw_missing"
    if observation.gps_raw_age_sec > GPS_RAW_MAX_AGE_SEC:
        return "gps_raw_stale"
    if observation.gps_fix_type < MIN_GPS_FIX_TYPE:
        return "gps_not_3d"
    if observation.satellites_visible < MIN_SATELLITES_VISIBLE:
        return "gps_satellites_low"
    if not observation.armed:
        return "unexpected_disarm"
    if observation.mode.upper() != "OFFBOARD":
        return "offboard_exit"
    return None


def wrap_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def turn_progress_step(
    tracker: TurnTracker,
    current_yaw_rad: float,
    direction_sign: float = 1.0,
) -> TurnTracker:
    """Accumulate directed yaw across +/-pi; +1 is the proven physical right turn."""
    if direction_sign not in {-1.0, 1.0}:
        raise ValueError("direction_sign must be exactly -1 or 1")
    if not all(
        math.isfinite(value)
        for value in (
            tracker.previous_yaw_rad,
            tracker.accumulated_rad,
            current_yaw_rad,
        )
    ):
        raise ValueError("turn tracker values must be finite")
    yaw_step = wrap_pi(current_yaw_rad - tracker.previous_yaw_rad)
    accumulated = max(0.0, tracker.accumulated_rad - direction_sign * yaw_step)
    return TurnTracker(current_yaw_rad, accumulated)


def effective_semicircle_radius_m(
    start_x_m: float,
    start_y_m: float,
    end_x_m: float,
    end_y_m: float,
) -> float:
    """Estimate an achieved 180-degree turn radius from its endpoint chord."""
    values = (start_x_m, start_y_m, end_x_m, end_y_m)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("turn endpoint coordinates must be finite")
    return 0.5 * math.hypot(end_x_m - start_x_m, end_y_m - start_y_m)


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def heading_pid_step(
    config: MissionConfig,
    state: HeadingPidState,
    heading_error_rad: float,
    dt_sec: float,
) -> tuple[float, HeadingPidState]:
    """Return bounded BODY_NED lateral steering for the captured start yaw."""
    if not math.isfinite(heading_error_rad) or not math.isfinite(dt_sec) or dt_sec <= 0.0:
        raise ValueError("heading PID inputs must be finite and dt_sec must be positive")
    dt_sec = min(dt_sec, 0.20)
    error = wrap_pi(heading_error_rad)
    raw_derivative = 0.0
    if state.previous_error_rad is not None:
        raw_derivative = wrap_pi(error - state.previous_error_rad) / dt_sec
    alpha = dt_sec / (config.heading_derivative_tau_sec + dt_sec)
    derivative = state.derivative_radps + alpha * (
        raw_derivative - state.derivative_radps
    )
    candidate_integral = clamp(
        state.integral_rad_sec + error * dt_sec,
        -config.heading_integral_limit_rad_sec,
        config.heading_integral_limit_rad_sec,
    )

    def output_for(integral: float) -> float:
        return (
            config.steering_trim_mps
            + config.heading_kp * error
            + config.heading_ki * integral
            + config.heading_kd * derivative
        )

    raw_output = output_for(candidate_integral)
    output = clamp(raw_output, -config.max_steering_mps, config.max_steering_mps)
    if output != raw_output and error * raw_output > 0.0:
        candidate_integral = state.integral_rad_sec
        output = clamp(
            output_for(candidate_integral),
            -config.max_steering_mps,
            config.max_steering_mps,
        )
    if abs(math.degrees(error)) < config.heading_deadband_deg:
        output = 0.0
        candidate_integral *= max(0.0, 1.0 - 2.0 * dt_sec)
    elif 0.0 < abs(output) < config.min_effective_steering_mps:
        output = math.copysign(config.min_effective_steering_mps, output)
    return output, HeadingPidState(candidate_integral, error, derivative)


def path_following_step(
    config: MissionConfig,
    state: PathFollowerState,
    cross_track_m: float,
    heading_error_rad: float,
    dt_sec: float,
) -> tuple[float, float, float, PathFollowerState]:
    """Track the start-frame x axis with an outer cross-track loop and yaw PID."""
    if not all(math.isfinite(item) for item in (cross_track_m, heading_error_rad, dt_sec)):
        raise ValueError("path follower inputs must be finite")
    if dt_sec <= 0.0:
        raise ValueError("path follower dt_sec must be positive")
    dt_sec = min(dt_sec, 0.20)
    if state.filtered_cross_track_m is None:
        filtered_cross = cross_track_m
    else:
        alpha = dt_sec / (config.cross_track_filter_tau_sec + dt_sec)
        filtered_cross = state.filtered_cross_track_m + alpha * (
            cross_track_m - state.filtered_cross_track_m
        )
    effective_cross = math.copysign(
        max(0.0, abs(filtered_cross) - config.cross_track_deadband_m),
        filtered_cross,
    )
    desired_heading_offset = -math.atan2(
        effective_cross, config.cross_track_lookahead_m
    )
    max_offset = math.radians(config.max_path_heading_correction_deg)
    desired_heading_offset = clamp(desired_heading_offset, -max_offset, max_offset)
    heading_control_error = wrap_pi(heading_error_rad - desired_heading_offset)
    lateral_command, heading_pid = heading_pid_step(
        config, state.heading_pid, heading_control_error, dt_sec
    )
    lateral_command *= config.steering_direction_sign
    return (
        lateral_command,
        desired_heading_offset,
        heading_control_error,
        PathFollowerState(heading_pid, filtered_cross),
    )


def build_current_yaw_reference(observation: Observation) -> PathReference:
    """Capture the current body heading without commanding a yaw alignment."""
    if not navigation_ready(observation):
        raise ValueError("current-yaw reference requires fresh navigation")
    return PathReference(
        axis_yaw_rad=observation.yaw_rad,
        pose_yaw_zero_rad=observation.yaw_rad,
    )


def track_metrics(
    start_x_m: float,
    start_y_m: float,
    start_yaw_rad: float,
    current_x_m: float,
    current_y_m: float,
    current_yaw_rad: float,
) -> tuple[float, float, float, float]:
    dx = current_x_m - start_x_m
    dy = current_y_m - start_y_m
    heading_x = math.cos(start_yaw_rad)
    heading_y = math.sin(start_yaw_rad)
    along_track_m = dx * heading_x + dy * heading_y
    cross_track_m = -dx * heading_y + dy * heading_x
    displacement_m = math.hypot(dx, dy)
    heading_error_rad = wrap_pi(current_yaw_rad - start_yaw_rad)
    return along_track_m, cross_track_m, displacement_m, heading_error_rad


def position_course_error(along_track_m: float, cross_track_m: float) -> float:
    """Return path-relative course from displacement, independent of compass yaw."""
    if not math.isfinite(along_track_m) or not math.isfinite(cross_track_m):
        raise ValueError("position course inputs must be finite")
    if along_track_m == 0.0 and cross_track_m == 0.0:
        return 0.0
    return math.atan2(cross_track_m, along_track_m)


def position_course_limit_exceeded(
    course_error_rad: float,
    displacement_m: float,
    max_error_deg: float,
    minimum_baseline_m: float = 0.75,
) -> bool:
    """Ignore undefined short-baseline GNSS course, then enforce the normal limit."""
    if not all(
        math.isfinite(value)
        for value in (course_error_rad, displacement_m, max_error_deg, minimum_baseline_m)
    ):
        raise ValueError("position course gate inputs must be finite")
    if displacement_m < 0.0 or minimum_baseline_m <= 0.0 or max_error_deg <= 0.0:
        raise ValueError("position course gate bounds must be positive")
    return bool(
        displacement_m >= minimum_baseline_m
        and abs(math.degrees(course_error_rad)) > max_error_deg
    )


def feedback_directions_consistent(
    cross_track_m: float,
    course_error_rad: float,
    pose_heading_delta_rad: float,
    config: MissionConfig,
) -> bool:
    """Reject steering when meaningful position and attitude trends conflict."""
    if not all(
        math.isfinite(item)
        for item in (cross_track_m, course_error_rad, pose_heading_delta_rad)
    ):
        return False
    if abs(cross_track_m) <= config.cross_track_deadband_m:
        return True
    heading_threshold = math.radians(config.heading_deadband_deg)
    if abs(course_error_rad) <= heading_threshold:
        return True
    if abs(pose_heading_delta_rad) <= heading_threshold:
        return True
    return course_error_rad * pose_heading_delta_rad >= 0.0


def select_straight_heading_feedback(
    cross_track_m: float,
    course_error_rad: float,
    pose_heading_delta_rad: float,
    displacement_m: float,
    config: MissionConfig,
    minimum_course_baseline_m: float = 0.75,
    previous_source: str = "",
) -> tuple[float, str]:
    """Prefer pose yaw, but use established position course when they conflict."""
    values = (
        cross_track_m,
        course_error_rad,
        pose_heading_delta_rad,
        displacement_m,
        minimum_course_baseline_m,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("straight heading feedback inputs must be finite")
    if displacement_m < 0.0 or minimum_course_baseline_m <= 0.0:
        raise ValueError("straight heading feedback baseline must be positive")
    if displacement_m < minimum_course_baseline_m:
        return pose_heading_delta_rad, "pose_yaw_short_baseline"
    disagreement_deg = abs(
        math.degrees(wrap_pi(pose_heading_delta_rad - course_error_rad))
    )
    disagreement_limit_deg = (
        config.pose_course_disagreement_exit_deg
        if previous_source == "position_course_fallback"
        else config.pose_course_disagreement_enter_deg
    )
    if disagreement_deg > disagreement_limit_deg:
        return course_error_rad, "position_course_fallback"
    if feedback_directions_consistent(
        cross_track_m,
        course_error_rad,
        pose_heading_delta_rad,
        config,
    ):
        return pose_heading_delta_rad, "pose_yaw"
    return course_error_rad, "position_course_fallback"


def commanded_forward_speed(config: MissionConfig, along_track_m: float) -> float:
    """Use a bounded terminal speed to reduce overshoot before zero/disarm."""
    if not math.isfinite(along_track_m):
        raise ValueError("along_track_m must be finite")
    remaining_m = config.distance_m - along_track_m
    if remaining_m <= config.terminal_slowdown_distance_m:
        return config.terminal_speed_mps
    return config.speed_mps


def reset_path_follower_for_conflict(cross_track_m: float) -> PathFollowerState:
    """Clear PID memory while retaining a finite filter seed for diagnostics."""
    if not math.isfinite(cross_track_m):
        raise ValueError("cross_track_m must be finite")
    return PathFollowerState(
        heading_pid=HeadingPidState(), filtered_cross_track_m=cross_track_m
    )


def entry_phase_speed_mps(phase: str, config: MissionConfig) -> float:
    """Keep the OFFBOARD handoff heading-defined; move after mode proof."""
    try:
        speed_mps = ENTRY_PHASE_SPEEDS[phase]
    except KeyError as exc:
        raise ValueError(f"unknown entry phase: {phase}") from exc
    return config.speed_mps if speed_mps is None else speed_mps


def recovery_hold_speed_mps(observation: Observation) -> float:
    """Avoid PX4's zero-vector yaw ambiguity until Disarm is freshly proven."""
    if (
        observation.state_present
        and observation.state_age_sec <= STATE_MAX_AGE_SEC
        and observation.connected
        and not observation.armed
    ):
        return 0.0
    return OFFBOARD_HEADING_HOLD_SPEED_MPS


def validate_entry_hold_sec(value: float | None) -> None:
    if value is None:
        return
    if not math.isfinite(value) or not 1.0 <= value <= 5.0:
        raise ValueError("entry-only hold must be finite and within [1, 5] seconds")


def entry_phase_lateral_mps(phase: str, config: MissionConfig) -> float:
    """Apply the measured left trim only after OFFBOARD motion is observed."""
    if phase not in ENTRY_PHASE_SPEEDS:
        raise ValueError(f"unknown entry phase: {phase}")
    return config.steering_trim_mps if phase == "motion" else 0.0


def adapt_body_forward_mps(speed_mps: float, config: MissionConfig) -> float:
    """Map semantic forward speed to this rover's physical BODY_NED x sign."""
    return config.body_forward_sign * speed_mps


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--confirm", default="")
    parser.add_argument("--namespace", default="/mavros")
    parser.add_argument("--rviz-topic-prefix", default="/orin2/offboard")
    parser.add_argument("--disable-rviz-trajectory", action="store_true")
    parser.add_argument("--distance-m", type=float, default=5.0)
    parser.add_argument("--tolerance-m", type=float, default=0.15)
    parser.add_argument("--speed-mps", type=float, default=0.06)
    parser.add_argument("--max-speed-mps", type=float, default=0.15)
    parser.add_argument("--terminal-speed-mps", type=float, default=0.035)
    parser.add_argument("--terminal-slowdown-distance-m", type=float, default=1.5)
    parser.add_argument("--max-cross-track-m", type=float, default=1.5)
    parser.add_argument("--max-heading-error-deg", type=float, default=35.0)
    parser.add_argument("--max-motion-sec", type=float, default=180.0)
    parser.add_argument("--stall-window-sec", type=float, default=8.0)
    parser.add_argument("--stall-min-progress-m", type=float, default=0.08)
    parser.add_argument("--course-calibration-distance-m", type=float, default=1.0)
    parser.add_argument("--course-calibration-speed-mps", type=float, default=0.04)
    parser.add_argument("--course-calibration-max-sec", type=float, default=5.0)
    parser.add_argument("--course-calibration-max-yaw-change-deg", type=float, default=20.0)
    parser.add_argument(
        "--reference-mode",
        choices=("ground_course_rollout", "initial_yaw"),
        default="ground_course_rollout",
    )
    parser.add_argument("--trajectory-artifact-dir")
    parser.add_argument("--steering-trim-mps", type=float, default=0.0)
    parser.add_argument("--heading-kp", type=float, default=0.30)
    parser.add_argument("--heading-ki", type=float, default=0.02)
    parser.add_argument("--heading-kd", type=float, default=0.03)
    parser.add_argument("--heading-integral-limit-rad-sec", type=float, default=0.25)
    parser.add_argument("--heading-derivative-tau-sec", type=float, default=0.20)
    parser.add_argument("--max-steering-mps", type=float, default=0.06)
    parser.add_argument("--min-effective-steering-mps", type=float, default=0.03)
    parser.add_argument("--heading-deadband-deg", type=float, default=1.0)
    parser.add_argument("--cross-track-lookahead-m", type=float, default=0.8)
    parser.add_argument("--cross-track-deadband-m", type=float, default=0.15)
    parser.add_argument("--cross-track-filter-tau-sec", type=float, default=0.20)
    parser.add_argument("--max-path-heading-correction-deg", type=float, default=15.0)
    parser.add_argument("--body-forward-sign", type=float, default=1.0)
    parser.add_argument("--steering-direction-sign", type=float, default=1.0)
    parser.add_argument("--u-turn", action="store_true")
    parser.add_argument("--s-bend-return", action="store_true")
    parser.add_argument("--turn-direction-sign", type=float, default=1.0)
    parser.add_argument("--turn-angle-deg", type=float, default=180.0)
    parser.add_argument("--turn-tolerance-deg", type=float, default=8.0)
    parser.add_argument("--turn-forward-speed-mps", type=float, default=0.05)
    parser.add_argument("--turn-lateral-speed-mps", type=float, default=0.04)
    parser.add_argument("--turn-max-sec", type=float, default=45.0)
    parser.add_argument("--turn-completion-hold-sec", type=float, default=0.30)
    parser.add_argument("--turn-stall-window-sec", type=float, default=8.0)
    parser.add_argument("--turn-stall-min-progress-deg", type=float, default=5.0)
    parser.add_argument("--turn-clearance-radius-m", type=float, default=3.5)
    parser.add_argument("--turn-radius-m", type=float, default=3.0)
    parser.add_argument("--trajectory-spacing-m", type=float, default=0.15)
    parser.add_argument("--tracker-base-lookahead-m", type=float, default=0.80)
    parser.add_argument("--tracker-speed-lookahead-gain-sec", type=float, default=0.80)
    parser.add_argument("--tracker-min-lookahead-m", type=float, default=0.65)
    parser.add_argument("--tracker-max-lookahead-m", type=float, default=1.60)
    parser.add_argument("--tracker-projection-backtrack-m", type=float, default=0.20)
    parser.add_argument("--tracker-projection-ahead-m", type=float, default=4.0)
    parser.add_argument("--tracker-max-body-bearing-deg", type=float)
    parser.add_argument(
        "--tracker-max-body-bearing-rate-degps", type=float, default=45.0
    )
    parser.add_argument("--tracker-max-yaw-rate-radps", type=float, default=0.35)
    parser.add_argument("--tracker-curvature-slowdown-gain", type=float, default=0.70)
    parser.add_argument("--tracker-reference-curvature-window-m", type=float, default=0.45)
    parser.add_argument("--tracker-max-curvature-correction-inv-m", type=float)
    parser.add_argument("--tracker-curvature-to-body-gain-m", type=float)
    parser.add_argument("--tracker-curvature-feedback-gain-ratio", type=float, default=1.0)
    parser.add_argument(
        "--tracker-cross-track-integral-gain-inv-m-per-m-sec",
        type=float,
        default=0.0,
    )
    parser.add_argument(
        "--tracker-cross-track-integral-limit-m-sec",
        type=float,
        default=0.0,
    )
    parser.add_argument("--tracker-min-nominal-bearing-reserve-deg", type=float, default=3.0)
    parser.add_argument("--tracker-max-reference-curvature-rate-inv-m2", type=float, default=1.0)
    parser.add_argument("--entry-only-hold-sec", type=float)
    parser.add_argument("--external-start-gate-fd", type=int)
    parser.add_argument("--external-start-plan-id", type=int)
    parser.add_argument("--external-start-gate-timeout-sec", type=float, default=300.0)
    parser.add_argument("--external-runtime-stop-fd", type=int)
    parser.add_argument("--recover-only", action="store_true")
    parser.add_argument("--verify-only", action="store_true")
    return parser


def config_from_args(args: argparse.Namespace) -> MissionConfig:
    config = MissionConfig(
        distance_m=args.distance_m,
        tolerance_m=args.tolerance_m,
        speed_mps=args.speed_mps,
        max_speed_mps=args.max_speed_mps,
        terminal_speed_mps=args.terminal_speed_mps,
        terminal_slowdown_distance_m=args.terminal_slowdown_distance_m,
        max_cross_track_m=args.max_cross_track_m,
        max_heading_error_deg=args.max_heading_error_deg,
        max_motion_sec=args.max_motion_sec,
        stall_window_sec=args.stall_window_sec,
        stall_min_progress_m=args.stall_min_progress_m,
        course_calibration_distance_m=args.course_calibration_distance_m,
        course_calibration_speed_mps=args.course_calibration_speed_mps,
        course_calibration_max_sec=args.course_calibration_max_sec,
        course_calibration_max_yaw_change_deg=args.course_calibration_max_yaw_change_deg,
        reference_mode=args.reference_mode,
        steering_trim_mps=args.steering_trim_mps,
        heading_kp=args.heading_kp,
        heading_ki=args.heading_ki,
        heading_kd=args.heading_kd,
        heading_integral_limit_rad_sec=args.heading_integral_limit_rad_sec,
        heading_derivative_tau_sec=args.heading_derivative_tau_sec,
        max_steering_mps=args.max_steering_mps,
        min_effective_steering_mps=args.min_effective_steering_mps,
        heading_deadband_deg=args.heading_deadband_deg,
        cross_track_lookahead_m=args.cross_track_lookahead_m,
        cross_track_deadband_m=args.cross_track_deadband_m,
        cross_track_filter_tau_sec=args.cross_track_filter_tau_sec,
        max_path_heading_correction_deg=args.max_path_heading_correction_deg,
        body_forward_sign=args.body_forward_sign,
        steering_direction_sign=args.steering_direction_sign,
        u_turn=args.u_turn,
        s_bend_return=args.s_bend_return,
        turn_direction_sign=args.turn_direction_sign,
        turn_angle_deg=args.turn_angle_deg,
        turn_tolerance_deg=args.turn_tolerance_deg,
        turn_forward_speed_mps=args.turn_forward_speed_mps,
        turn_lateral_speed_mps=args.turn_lateral_speed_mps,
        turn_max_sec=args.turn_max_sec,
        turn_completion_hold_sec=args.turn_completion_hold_sec,
        turn_stall_window_sec=args.turn_stall_window_sec,
        turn_stall_min_progress_deg=args.turn_stall_min_progress_deg,
        turn_clearance_radius_m=args.turn_clearance_radius_m,
        turn_radius_m=args.turn_radius_m,
        trajectory_spacing_m=args.trajectory_spacing_m,
        tracker_base_lookahead_m=args.tracker_base_lookahead_m,
        tracker_speed_lookahead_gain_sec=args.tracker_speed_lookahead_gain_sec,
        tracker_min_lookahead_m=args.tracker_min_lookahead_m,
        tracker_max_lookahead_m=args.tracker_max_lookahead_m,
        tracker_projection_backtrack_m=args.tracker_projection_backtrack_m,
        tracker_projection_ahead_m=args.tracker_projection_ahead_m,
        tracker_max_body_bearing_deg=(
            args.tracker_max_body_bearing_deg
            if args.tracker_max_body_bearing_deg is not None
            else (89.0 if args.u_turn or args.s_bend_return else 32.0)
        ),
        tracker_max_body_bearing_rate_degps=(
            args.tracker_max_body_bearing_rate_degps
        ),
        tracker_max_yaw_rate_radps=args.tracker_max_yaw_rate_radps,
        tracker_curvature_slowdown_gain=args.tracker_curvature_slowdown_gain,
        tracker_reference_curvature_window_m=(
            args.tracker_reference_curvature_window_m
        ),
        tracker_max_curvature_correction_inv_m=(
            args.tracker_max_curvature_correction_inv_m
            if args.tracker_max_curvature_correction_inv_m is not None
            else (0.12 if args.u_turn or args.s_bend_return else 0.0)
        ),
        tracker_curvature_to_body_gain_m=(
            args.tracker_curvature_to_body_gain_m
            if args.tracker_curvature_to_body_gain_m is not None
            else (3.5 if args.u_turn or args.s_bend_return else 0.0)
        ),
        tracker_curvature_feedback_gain_ratio=(
            args.tracker_curvature_feedback_gain_ratio
        ),
        tracker_cross_track_integral_gain_inv_m_per_m_sec=(
            args.tracker_cross_track_integral_gain_inv_m_per_m_sec
        ),
        tracker_cross_track_integral_limit_m_sec=(
            args.tracker_cross_track_integral_limit_m_sec
        ),
        tracker_min_nominal_bearing_reserve_deg=(
            args.tracker_min_nominal_bearing_reserve_deg
        ),
        tracker_max_reference_curvature_rate_inv_m2=(
            args.tracker_max_reference_curvature_rate_inv_m2
        ),
    )
    validate_config(config)
    return config


def print_plan(config: MissionConfig, entry_only_hold_sec: float | None = None) -> None:
    turn_name = "RIGHT" if config.turn_direction_sign > 0.0 else "LEFT"
    if config.s_bend_return:
        title = f"{config.distance_m:g}M S-BEND RETURN-TO-START"
    elif config.u_turn:
        title = f"{config.distance_m:g}M {turn_name} U-TURN {config.distance_m:g}M"
    else:
        title = "FORWARD-ONLY"
    if entry_only_hold_sec is not None:
        title = f"WHEELS-LIFTED ENTRY HOLD {entry_only_hold_sec:.1f}S"
    print(f"C2 OUTDOOR {title} OFFBOARD PLAN")
    leg_text = " initial straight" if config.s_bend_return else (" per leg" if config.u_turn else "")
    print(
        f"distance={config.distance_m:.2f}m{leg_text} "
        f"PX4_velocity_request={config.speed_mps:.3f} (not calibrated ground m/s)"
    )
    print(
        f"terminal_speed={config.terminal_speed_mps:.3f}m/s over final "
        f"{config.terminal_slowdown_distance_m:.2f}m"
    )
    print("navigation=real GNSS + real MAVROS local position; fake EV/GPS forbidden")
    print("entry=program requests OFFBOARD while disarmed, settles, then Arms once")
    print(
        f"handoff=heading-defined {OFFBOARD_HEADING_HOLD_SPEED_MPS:.4f}m/s "
        "BODY +x stop vector through mode transition; motion starts immediately "
        "after OFFBOARD proof"
    )
    print(
        "motion=one continuous arc-length ENU trajectory -> geometric v/omega "
        "-> bounded BODY_NED x/y; no yaw setpoint"
    )
    preview = build_relative_mission_trajectory(config, 0.0, 0.0, 0.0)
    print(
        f"motion_timeout=route-scaled {trajectory_motion_timeout_sec(preview.length_m, config.max_motion_sec):.1f}s "
        f"for {preview.length_m:.3f}m; hard ceiling={config.max_motion_sec:.1f}s; "
        f"stall watchdog={config.stall_window_sec:.1f}s"
    )
    if config.u_turn:
        print(
            f"turn={turn_name.lower()} sampled semicircle radius="
            f"{config.turn_radius_m:.2f}m request={config.turn_forward_speed_mps:.3f}; "
            "the same tracker remains active through both straights and the arc"
        )
        print(
            f"turn_clearance=reserve at least {config.turn_clearance_radius_m:.1f}m "
            "radius plus rover margin; actual radius is measured from the run"
        )
    elif config.s_bend_return:
        print(
            f"route=straight {config.distance_m:.1f}m -> quintic S shift "
            f"{4.0 * config.turn_radius_m:.1f}m forward/"
            f"{2.0 * config.turn_radius_m:.1f}m right -> smooth left 180deg -> return; "
            f"turn_radius={config.turn_radius_m:.2f}m length={preview.length_m:.3f}m"
        )
        print(
            f"route_clearance=approximately {config.distance_m + 5.25 * config.turn_radius_m:.1f}m "
            f"along-track by {2.0 * config.turn_radius_m:.1f}m lateral plus rover margin"
        )
    print(
        "tracker=monotonic local projection + adaptive lookahead; no fixed cross-track "
        f"deadband; lookahead={config.tracker_min_lookahead_m:.2f}.."
        f"{config.tracker_max_lookahead_m:.2f}m"
    )
    print(
        f"adapter=constant BODY command magnitude, bearing limit="
        f"+/-{config.tracker_max_body_bearing_deg:.1f}deg, bearing slew="
        f"{config.tracker_max_body_bearing_rate_degps:.1f}deg/s, "
        "ENU CCW -> BODY_NED y sign=-1"
    )
    print(
        f"vehicle_forward_adapter=semantic +forward -> BODY_NED x sign "
        f"{config.body_forward_sign:+.0f}"
    )
    print(
        "curvature_adapter="
        f"atan({config.tracker_curvature_to_body_gain_m:.2f}m * "
        f"(reference + {config.tracker_curvature_feedback_gain_ratio:.2f} * feedback)), "
        f"reference window={config.tracker_reference_curvature_window_m:.2f}m, "
        f"feedback delta=+/-{config.tracker_max_curvature_correction_inv_m:.2f} 1/m"
    )
    print(
        "cross_track_integral="
        f"gain={config.tracker_cross_track_integral_gain_inv_m_per_m_sec:.3f} "
        f"limit=+/-{config.tracker_cross_track_integral_limit_m_sec:.2f}m*s "
        "with saturation anti-windup"
    )
    feasibility = mission_trajectory_feasibility_report(config)
    print(
        "trajectory_feasibility="
        f"curvature_max={feasibility.max_abs_curvature_inv_m:.3f} 1/m, "
        f"curvature_rate_max={feasibility.max_abs_curvature_rate_inv_m2:.3f} 1/m^2, "
        f"nominal_body_bearing_max={feasibility.max_nominal_body_bearing_deg:.2f}deg, "
        f"reserve={config.tracker_min_nominal_bearing_reserve_deg:.1f}deg, "
        "status=PASS"
    )
    print(
        f"slowdown=curvature gain {config.tracker_curvature_slowdown_gain:.2f}, "
        f"terminal {config.terminal_slowdown_distance_m:.2f}m to request floor "
        f"{config.terminal_speed_mps:.3f}"
    )
    if entry_only_hold_sec is None and config.reference_mode == "ground_course_rollout":
        print(
            f"relative_frame={config.course_calibration_distance_m:.2f}m straight "
            f"rollout at {config.course_calibration_speed_mps:.3f} request, then "
            "ground-course fit; no magnetometer-only path initialization"
        )
    elif entry_only_hold_sec is None:
        print(
            "relative_frame=initial fresh pose position+yaw; no rollout, "
            "no pre-yaw alignment, tracking starts on the first motion frame"
        )
    print("exit=heading-defined stop burst -> Disarm -> MANUAL -> verified safe state")
    print(
        "rviz=planned path, actual path, vehicle pose and lookahead target are "
        "published during trajectory execution"
    )
    if entry_only_hold_sec is not None:
        print("entry-only=no trajectory execution; heading-defined stop vector only")


def run_live(args: argparse.Namespace, config: MissionConfig) -> int:
    try:
        import rclpy
        from geometry_msgs.msg import PoseStamped, TwistStamped
        from mavros_msgs.msg import GPSRAW, State, StatusText
        from mavros_msgs.srv import CommandBool, SetMode
        from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
        from rcl_interfaces.srv import GetParameters, SetParameters
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from rclpy.signals import SignalHandlerOptions
        from sensor_msgs.msg import NavSatFix
    except ImportError as exc:
        print(f"REFUSED: ROS/MAVROS dependency unavailable: {exc}")
        return 2

    namespace = args.namespace.rstrip("/")

    class ForwardNode(Node):
        def __init__(self) -> None:
            super().__init__("orin2_outdoor_forward_5m")
            self.state = None
            self.state_rx = 0.0
            self.pose = None
            self.pose_rx = 0.0
            self.gps = None
            self.gps_rx = 0.0
            self.gps_raw = None
            self.gps_raw_rx = 0.0
            self.last_status = None
            self.trajectory_visualizer = None
            self.trajectory_visualizer_error = None
            self.publisher = self.create_publisher(
                TwistStamped, f"{namespace}/setpoint_velocity/cmd_vel", 10
            )
            self.create_subscription(State, f"{namespace}/state", self._state_cb, 10)
            self.create_subscription(
                PoseStamped,
                f"{namespace}/local_position/pose",
                self._pose_cb,
                qos_profile_sensor_data,
            )
            self.create_subscription(
                NavSatFix,
                f"{namespace}/global_position/raw/fix",
                self._gps_cb,
                qos_profile_sensor_data,
            )
            self.create_subscription(
                GPSRAW,
                f"{namespace}/gpsstatus/gps1/raw",
                self._gps_raw_cb,
                qos_profile_sensor_data,
            )
            self.create_subscription(
                StatusText,
                f"{namespace}/statustext/recv",
                self._status_cb,
                qos_profile_sensor_data,
            )
            self.get_frame = self.create_client(
                GetParameters, f"{namespace}/setpoint_velocity/get_parameters"
            )
            self.set_frame = self.create_client(
                SetParameters, f"{namespace}/setpoint_velocity/set_parameters"
            )
            self.arming = self.create_client(CommandBool, f"{namespace}/cmd/arming")
            self.set_mode = self.create_client(SetMode, f"{namespace}/set_mode")
            if not args.disable_rviz_trajectory:
                self.trajectory_visualizer = RvizTrajectoryPublisher(
                    self, args.rviz_topic_prefix
                )
                print(
                    f"RVIZ_TRAJECTORY_READY prefix={args.rviz_topic_prefix}",
                    flush=True,
                )

        def _state_cb(self, message) -> None:
            self.state = message
            self.state_rx = time.monotonic()

        def _pose_cb(self, message) -> None:
            self.pose = message
            self.pose_rx = time.monotonic()

        def _gps_cb(self, message) -> None:
            self.gps = message
            self.gps_rx = time.monotonic()

        def _gps_raw_cb(self, message) -> None:
            self.gps_raw = message
            self.gps_raw_rx = time.monotonic()

        def _status_cb(self, message) -> None:
            item = (int(message.severity), str(message.text))
            if item != self.last_status:
                self.last_status = item
                print(f"PX4_STATUS severity={item[0]} text={item[1]!r}", flush=True)

        def local_frame_id(self) -> str:
            if self.pose is None:
                return "map"
            return str(self.pose.header.frame_id) or "map"

        def set_rviz_plan(self, points) -> None:
            if self.trajectory_visualizer is None:
                return
            try:
                self.trajectory_visualizer.set_plan(points, self.local_frame_id())
            except Exception as exc:
                self.trajectory_visualizer_error = str(exc)
                self.trajectory_visualizer = None
                print(f"RVIZ_TRAJECTORY_DISABLED error={exc!r}", flush=True)

        def start_rviz_actual(self) -> None:
            if self.trajectory_visualizer is None:
                return
            try:
                self.trajectory_visualizer.start_actual(self.local_frame_id())
            except Exception as exc:
                self.trajectory_visualizer_error = str(exc)
                self.trajectory_visualizer = None
                print(f"RVIZ_TRAJECTORY_DISABLED error={exc!r}", flush=True)

        def seed_rviz_actual(self, points) -> None:
            if self.trajectory_visualizer is None:
                return
            try:
                self.trajectory_visualizer.seed_actual(points)
            except Exception as exc:
                self.trajectory_visualizer_error = str(exc)
                self.trajectory_visualizer = None
                print(f"RVIZ_TRAJECTORY_DISABLED error={exc!r}", flush=True)

        def update_rviz_actual(
            self, observation: Observation, tracking_yaw_rad: float
        ) -> None:
            if self.trajectory_visualizer is None:
                return
            try:
                self.trajectory_visualizer.update_actual(
                    observation.x_m,
                    observation.y_m,
                    tracking_yaw_rad,
                )
            except Exception as exc:
                self.trajectory_visualizer_error = str(exc)
                self.trajectory_visualizer = None
                print(f"RVIZ_TRAJECTORY_DISABLED error={exc!r}", flush=True)

        def update_rviz_trajectory(
            self,
            observation: Observation,
            tracking_yaw_rad: float,
            target_x_m: float,
            target_y_m: float,
        ) -> None:
            if self.trajectory_visualizer is None:
                return
            try:
                self.trajectory_visualizer.update(
                    observation.x_m,
                    observation.y_m,
                    tracking_yaw_rad,
                    target_x_m,
                    target_y_m,
                )
            except Exception as exc:
                self.trajectory_visualizer_error = str(exc)
                self.trajectory_visualizer = None
                print(f"RVIZ_TRAJECTORY_DISABLED error={exc!r}", flush=True)

        def write_rviz_artifacts(self, directory: str | None) -> None:
            if directory is None or self.trajectory_visualizer is None:
                return
            try:
                planned, actual = self.trajectory_visualizer.write_artifacts(
                    Path(directory)
                )
                print(
                    f"TRAJECTORY_ARTIFACTS planned={planned} actual={actual}",
                    flush=True,
                )
            except Exception as exc:
                print(f"TRAJECTORY_ARTIFACT_ERROR error={exc!r}", flush=True)

        def observation(self) -> Observation:
            now = time.monotonic()
            pose = self.pose.pose if self.pose is not None else None
            yaw = math.nan
            if pose is not None:
                q = pose.orientation
                yaw = math.atan2(
                    2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z),
                )
            return Observation(
                state_present=self.state is not None,
                state_age_sec=now - self.state_rx if self.state is not None else math.inf,
                connected=bool(self.state and self.state.connected),
                armed=bool(self.state and self.state.armed),
                mode=str(self.state.mode) if self.state is not None else "",
                manual_input=bool(self.state and self.state.manual_input),
                pose_present=pose is not None,
                pose_age_sec=now - self.pose_rx if pose is not None else math.inf,
                x_m=float(pose.position.x) if pose is not None else math.nan,
                y_m=float(pose.position.y) if pose is not None else math.nan,
                yaw_rad=yaw,
                gps_present=self.gps is not None,
                gps_age_sec=now - self.gps_rx if self.gps is not None else math.inf,
                gps_status=int(self.gps.status.status) if self.gps is not None else -1,
                latitude_deg=float(self.gps.latitude) if self.gps is not None else math.nan,
                longitude_deg=float(self.gps.longitude) if self.gps is not None else math.nan,
                gps_raw_present=self.gps_raw is not None,
                gps_raw_age_sec=(
                    now - self.gps_raw_rx if self.gps_raw is not None else math.inf
                ),
                gps_fix_type=(
                    int(self.gps_raw.fix_type) if self.gps_raw is not None else 0
                ),
                satellites_visible=(
                    int(self.gps_raw.satellites_visible)
                    if self.gps_raw is not None
                    else 0
                ),
            )

        def publish(self, speed_mps: float, lateral_mps: float = 0.0) -> None:
            message = TwistStamped()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = "base_link"
            message.twist.linear.x = float(adapt_body_forward_mps(speed_mps, config))
            message.twist.linear.y = float(lateral_mps)
            message.twist.linear.z = 0.0
            message.twist.angular.x = 0.0
            message.twist.angular.y = 0.0
            message.twist.angular.z = 0.0
            self.publisher.publish(message)

        def spin_publish(self, duration_sec: float, speed_mps: float = 0.0) -> None:
            deadline = time.monotonic() + max(0.0, duration_sec)
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.0)
                self.publish(speed_mps)
                time.sleep(CONTROL_PERIOD_SEC)

        def wait_for(
            self,
            predicate: Callable[[Observation], bool],
            timeout_sec: float,
            label: str,
            *,
            publish_speed_mps: float | None = 0.0,
        ) -> Observation | None:
            deadline = time.monotonic() + timeout_sec
            last_log = 0.0
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.0)
                observation = self.observation()
                if publish_speed_mps is not None:
                    self.publish(publish_speed_mps)
                if predicate(observation):
                    print(f"GATE_PASS {label} {observation}", flush=True)
                    return observation
                if time.monotonic() - last_log >= 2.0:
                    last_log = time.monotonic()
                    print(f"GATE_WAIT {label} {observation}", flush=True)
                time.sleep(CONTROL_PERIOD_SEC)
            print(f"GATE_FAIL {label} {self.observation()}", flush=True)
            return None

        def _call(
            self,
            client,
            request,
            label: str,
            timeout_sec: float = 5.0,
            *,
            publish_speed_mps: float = 0.0,
        ):
            if not client.wait_for_service(timeout_sec=timeout_sec):
                raise RuntimeError(f"service unavailable: {label}")
            future = client.call_async(request)
            deadline = time.monotonic() + timeout_sec
            while rclpy.ok() and time.monotonic() < deadline and not future.done():
                rclpy.spin_once(self, timeout_sec=0.0)
                self.publish(publish_speed_mps)
                time.sleep(CONTROL_PERIOD_SEC)
            if not future.done():
                future.cancel()
                raise RuntimeError(f"service timeout: {label}")
            if future.exception() is not None:
                raise RuntimeError(f"service error {label}: {future.exception()}")
            return future.result()

        def configure_body_ned(self) -> None:
            request = SetParameters.Request()
            request.parameters = [
                Parameter(
                    name="mav_frame",
                    value=ParameterValue(
                        type=ParameterType.PARAMETER_STRING,
                        string_value="BODY_NED",
                    ),
                )
            ]
            result = self._call(self.set_frame, request, "set BODY_NED")
            if not result.results or not result.results[0].successful:
                raise RuntimeError("MAVROS rejected BODY_NED")
            verify = GetParameters.Request()
            verify.names = ["mav_frame"]
            values = self._call(self.get_frame, verify, "verify BODY_NED").values
            value = values[0].string_value if values else ""
            if value.upper() not in {"BODY_NED", "8"}:
                raise RuntimeError(f"BODY_NED verification failed: {value!r}")
            print(f"BODY_NED_VERIFIED value={value!r}", flush=True)

        def request_offboard(self, entry_speed_mps: float) -> None:
            request = SetMode.Request()
            request.custom_mode = "OFFBOARD"
            response = self._call(
                self.set_mode,
                request,
                "request OFFBOARD",
                publish_speed_mps=entry_speed_mps,
            )
            if not response.mode_sent:
                raise RuntimeError("OFFBOARD request rejected")
            print("OFFBOARD_REQUEST_ACCEPTED", flush=True)

        def request_arm_once(self, entry_speed_mps: float) -> None:
            request = CommandBool.Request()
            request.value = True
            response = self._call(
                self.arming,
                request,
                "request Arm once",
                publish_speed_mps=entry_speed_mps,
            )
            print(
                f"ARM_ONCE_RESPONSE success={response.success} result={response.result}",
                flush=True,
            )
            if not response.success:
                raise RuntimeError(f"Arm once rejected result={response.result}")

        def recover(self) -> bool:
            stop_speed_mps = recovery_hold_speed_mps(self.observation())
            print(
                f"EXIT_STOP_BURST {EXIT_STOP_BURST_SEC:.2f}s "
                f"body_x={stop_speed_mps:.4f} then immediate Disarm",
                flush=True,
            )
            self.spin_publish(EXIT_STOP_BURST_SEC, stop_speed_mps)
            observation = self.observation()
            if observation.state_present and observation.connected and observation.armed:
                request = CommandBool.Request()
                request.value = False
                response = self._call(
                    self.arming,
                    request,
                    "request Disarm",
                    publish_speed_mps=stop_speed_mps,
                )
                print(
                    f"DISARM_RESPONSE success={response.success} result={response.result}",
                    flush=True,
                )
            if self.wait_for(
                lambda item: item.state_present and item.connected and not item.armed,
                8.0,
                "disarmed",
                publish_speed_mps=stop_speed_mps,
            ) is None:
                return False
            request = SetMode.Request()
            request.custom_mode = "MANUAL"
            response = self._call(self.set_mode, request, "request MANUAL")
            print(f"MANUAL_RESPONSE mode_sent={response.mode_sent}", flush=True)
            return self.wait_for(
                lambda item: bool(
                    item.state_present
                    and item.state_age_sec <= STATE_MAX_AGE_SEC
                    and item.connected
                    and not item.armed
                    and item.mode.upper() == "MANUAL"
                    and item.manual_input
                ),
                10.0,
                "final MANUAL/disarmed",
            ) is not None

    rclpy.init(args=None, signal_handler_options=SignalHandlerOptions.NO)
    node = ForwardNode()
    primary_error: str | None = None
    result_code = 7
    recovery_required = not args.verify_only and not args.recover_only
    runtime_stop_buffer = b""
    if args.external_runtime_stop_fd is not None:
        os.set_blocking(args.external_runtime_stop_fd, False)

    def poll_external_runtime_stop() -> tuple[bool, str | None]:
        nonlocal runtime_stop_buffer
        if args.external_runtime_stop_fd is None:
            return False, None
        assert args.external_start_plan_id is not None
        readable, _, _ = select.select([args.external_runtime_stop_fd], [], [], 0.0)
        if not readable:
            return False, None
        try:
            incoming = os.read(
                args.external_runtime_stop_fd,
                EXTERNAL_RUNTIME_STOP_MAX_BUFFER_BYTES,
            )
        except BlockingIOError:
            return False, None
        if not incoming:
            return False, "external_runtime_stop_closed"
        runtime_stop_buffer, accepted, error = consume_external_runtime_stop_bytes(
            runtime_stop_buffer,
            incoming,
            args.external_start_plan_id,
        )
        return accepted, error

    def run_trajectory(
        trajectory: PolylineTrajectory,
        start: Observation,
        body_course_offset_rad: float,
        initial_actual_points: Sequence[tuple[float, float]] = (),
    ) -> tuple[Observation | None, str | None]:
        """Track any finite ENU trajectory with one continuous controller state."""
        node.set_rviz_plan(trajectory.points)
        if len(initial_actual_points) >= 2:
            node.seed_rviz_actual(initial_actual_points)
        tracker_config = tracker_config_from_mission(config)
        tracker_state = tracker_state_at_route_start()
        motion_start = time.monotonic()
        motion_timeout_sec = trajectory_motion_timeout_sec(
            trajectory.length_m,
            config.max_motion_sec,
        )
        print(
            f"TRAJECTORY_TIMEOUT length={trajectory.length_m:.3f}m "
            f"effective={motion_timeout_sec:.1f}s "
            f"hard_ceiling={config.max_motion_sec:.1f}s",
            flush=True,
        )
        previous_time = motion_start
        previous_x = start.x_m
        previous_y = start.y_m
        filtered_speed_mps = 0.0
        speed_filter_tau_sec = 0.40
        stall_start = motion_start
        stall_progress_m = 0.0
        last_log = 0.0
        phase = ""
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            observation = node.observation()
            fault = motion_fault(observation)
            if fault is not None:
                return None, f"trajectory:{fault}"
            external_complete, external_error = poll_external_runtime_stop()
            if external_error is not None:
                return None, f"trajectory:{external_error}"
            if external_complete:
                node.publish(entry_phase_speed_mps("offboard_stop", config), 0.0)
                print(
                    "PAIRB_RUNTIME_COMPLETE_ACCEPTED; starting safe recovery",
                    flush=True,
                )
                return observation, None
            now = time.monotonic()
            elapsed = now - motion_start
            if elapsed > motion_timeout_sec:
                return None, "trajectory:motion_timeout"
            dt_sec = max(now - previous_time, 1.0e-3)
            measured_speed_mps = math.hypot(
                observation.x_m - previous_x,
                observation.y_m - previous_y,
            ) / dt_sec
            speed_alpha = dt_sec / (speed_filter_tau_sec + dt_sec)
            filtered_speed_mps += speed_alpha * (
                measured_speed_mps - filtered_speed_mps
            )
            previous_time = now
            previous_x = observation.x_m
            previous_y = observation.y_m

            tracking_yaw_rad = tracking_yaw_with_course_offset(
                observation.yaw_rad,
                body_course_offset_rad,
            )
            command = trajectory_tracking_step(
                trajectory,
                tracker_state,
                observation.x_m,
                observation.y_m,
                tracking_yaw_rad,
                filtered_speed_mps,
                tracker_config,
                dt_sec,
            )
            tracker_state = command.state
            node.update_rviz_trajectory(
                observation,
                tracking_yaw_rad,
                command.target_x_m,
                command.target_y_m,
            )
            if command.terminal_missed:
                node.publish(entry_phase_speed_mps("offboard_stop", config), 0.0)
                print(
                    "TRAJECTORY_TERMINAL_MISS "
                    f"remaining={command.remaining_s_m:.3f} "
                    f"cross={command.cross_track_m:+.3f}; heading-defined stop",
                    flush=True,
                )
                return None, (
                    "trajectory:terminal_cross_track_miss:"
                    f"{command.cross_track_m:.3f}"
                )
            if command.goal_reached:
                node.publish(entry_phase_speed_mps("offboard_stop", config), 0.0)
                print(
                    f"TRAJECTORY_TARGET_REACHED progress={command.progress_s_m:.3f} "
                    f"length={trajectory.length_m:.3f} "
                    f"cross={command.cross_track_m:+.3f}",
                    flush=True,
                )
                return observation, None
            if abs(command.cross_track_m) > tracker_config.max_cross_track_m:
                return None, (
                    "trajectory:cross_track_limit:"
                    f"{command.cross_track_m:.3f}"
                )
            path_heading = trajectory.sample(command.progress_s_m).tangent_yaw_rad
            path_heading_error = wrap_pi(tracking_yaw_rad - path_heading)
            if abs(math.degrees(path_heading_error)) > config.max_heading_error_deg:
                return None, (
                    "trajectory:path_heading_error_limit:"
                    f"{math.degrees(path_heading_error):.2f}"
                )
            if now - stall_start >= config.stall_window_sec:
                gained_m = command.progress_s_m - stall_progress_m
                if gained_m < config.stall_min_progress_m:
                    return None, (
                        f"trajectory:progress_stall:{gained_m:.3f}m/"
                        f"{config.stall_window_sec:.1f}s"
                    )
                stall_start = now
                stall_progress_m = command.progress_s_m
            node.publish(command.body_x_mps, command.body_y_mps)
            if command.phase != phase:
                print(
                    f"TRAJECTORY_PHASE {phase or 'ENTRY'}->{command.phase} "
                    f"progress={command.progress_s_m:.3f}",
                    flush=True,
                )
                phase = command.phase
            if now - last_log >= 0.5:
                last_log = now
                print(
                    f"TRAJECTORY_PROGRESS elapsed={elapsed:.1f}s phase={command.phase} "
                    f"s={command.progress_s_m:.3f}/{trajectory.length_m:.3f} "
                    f"remaining={command.remaining_s_m:.3f} "
                    f"cross={command.cross_track_m:+.3f} "
                    f"path_heading_error_deg={math.degrees(path_heading_error):+.2f} "
                    f"target_bearing_deg={math.degrees(command.target_bearing_error_rad):+.2f} "
                    f"lookahead={command.lookahead_m:.3f} "
                    f"curvature={command.curvature_inv_m:+.3f} "
                    f"raw_curvature={command.raw_curvature_inv_m:+.3f} "
                    f"reference_curvature={command.reference_curvature_inv_m:+.3f} "
                    f"feedback_curvature={command.feedback_curvature_inv_m:+.3f} "
                    f"adapter_curvature={command.adapter_curvature_inv_m:+.3f} "
                    f"cross_integral={command.cross_track_integral_m_sec:+.3f} "
                    f"measured_speed={filtered_speed_mps:.3f} "
                    f"primitive=({command.v_mps:.3f},{command.omega_radps:+.3f}) "
                    f"body=({command.body_x_mps:.3f},{command.body_y_mps:+.3f}) "
                    f"body_norm={math.hypot(command.body_x_mps, command.body_y_mps):.3f}",
                    flush=True,
                )
            time.sleep(CONTROL_PERIOD_SEC)
        return None, "trajectory:ros_shutdown"

    def run_straight_leg(
        label: str,
        start: Observation,
        reference: PathReference,
    ) -> tuple[Observation | None, str | None]:
        motion_start = time.monotonic()
        control_time = motion_start
        follower = PathFollowerState()
        feedback_source = ""
        stall_start = motion_start
        stall_along = 0.0
        last_log = 0.0
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            observation = node.observation()
            fault = motion_fault(observation)
            if fault is not None:
                return None, f"{label}:{fault}"
            elapsed = time.monotonic() - motion_start
            if elapsed > config.max_motion_sec:
                return None, f"{label}:motion_timeout"
            along, cross, displacement, _ = track_metrics(
                start.x_m,
                start.y_m,
                reference.axis_yaw_rad,
                observation.x_m,
                observation.y_m,
                reference.axis_yaw_rad,
            )
            pose_heading_delta = wrap_pi(
                observation.yaw_rad - reference.pose_yaw_zero_rad
            )
            course_error = position_course_error(along, cross)
            if abs(cross) > config.max_cross_track_m:
                return None, f"{label}:cross_track_limit:{cross:.3f}"
            if position_course_limit_exceeded(
                course_error, displacement, config.max_heading_error_deg
            ):
                return None, (
                    f"{label}:position_course_error_limit:"
                    f"{math.degrees(course_error):.2f}"
                )
            if abs(math.degrees(pose_heading_delta)) > config.max_heading_error_deg:
                return None, (
                    f"{label}:pose_heading_delta_limit:"
                    f"{math.degrees(pose_heading_delta):.2f}"
                )
            now = time.monotonic()
            if now - stall_start >= config.stall_window_sec:
                if along - stall_along < config.stall_min_progress_m:
                    return None, (
                        f"{label}:progress_stall:{along - stall_along:.3f}m/"
                        f"{config.stall_window_sec:.1f}s"
                    )
                stall_start = now
                stall_along = along
            if along >= config.distance_m - config.tolerance_m:
                print(
                    f"{label}_TARGET_REACHED along={along:.3f} cross={cross:.3f} "
                    f"displacement={displacement:.3f}",
                    flush=True,
                )
                return observation, None
            selected_heading_error, selected_feedback_source = (
                select_straight_heading_feedback(
                    cross,
                    course_error,
                    pose_heading_delta,
                    displacement,
                    config,
                    previous_source=feedback_source,
                )
            )
            if selected_feedback_source != feedback_source:
                follower = reset_path_follower_for_conflict(cross)
                print(
                    f"{label}_FEEDBACK_SOURCE {feedback_source or 'initial'}"
                    f"->{selected_feedback_source}",
                    flush=True,
                )
                feedback_source = selected_feedback_source
            (
                lateral_command,
                desired_heading_offset,
                heading_control_error,
                follower,
            ) = path_following_step(
                config,
                follower,
                cross,
                selected_heading_error,
                max(now - control_time, 1e-3),
            )
            control_time = now
            forward_speed = commanded_forward_speed(config, along)
            node.publish(forward_speed, lateral_command)
            if now - last_log >= 1.0:
                last_log = now
                filtered_cross = follower.filtered_cross_track_m
                filtered_cross_text = (
                    "n/a" if filtered_cross is None else f"{filtered_cross:+.3f}"
                )
                print(
                    f"{label}_PROGRESS elapsed={elapsed:.1f}s along={along:.3f} "
                    f"cross={cross:.3f} displacement={displacement:.3f} "
                    f"course_error_deg={math.degrees(course_error):.2f} "
                    f"pose_heading_delta_deg={math.degrees(pose_heading_delta):.2f} "
                    f"feedback_source={feedback_source} "
                    f"filtered_cross={filtered_cross_text} "
                    f"desired_heading_offset_deg={math.degrees(desired_heading_offset):+.2f} "
                    f"heading_control_error_deg={math.degrees(heading_control_error):+.2f} "
                    f"lateral_cmd={lateral_command:+.3f} "
                    f"forward_cmd={forward_speed:.3f} "
                    f"pid_i={follower.heading_pid.integral_rad_sec:+.3f}",
                    flush=True,
                )
            time.sleep(CONTROL_PERIOD_SEC)
        return None, f"{label}:ros_shutdown"

    def run_u_turn(start: Observation) -> tuple[Observation | None, str | None]:
        turn_label = "RIGHT_UTURN" if config.turn_direction_sign > 0.0 else "LEFT_UTURN"
        error_label = "right_u_turn" if config.turn_direction_sign > 0.0 else "left_u_turn"
        tracker = TurnTracker(start.yaw_rad)
        threshold_rad = math.radians(
            config.turn_angle_deg - config.turn_tolerance_deg
        )
        motion_start = time.monotonic()
        stall_start = motion_start
        stall_progress_rad = 0.0
        threshold_since: float | None = None
        last_log = 0.0
        previous_x = start.x_m
        previous_y = start.y_m
        path_length_m = 0.0
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            observation = node.observation()
            fault = motion_fault(observation)
            if fault is not None:
                return None, f"{error_label}:{fault}"
            now = time.monotonic()
            elapsed = now - motion_start
            if elapsed > config.turn_max_sec:
                return None, (
                    f"{error_label}:timeout:"
                    f"{math.degrees(tracker.accumulated_rad):.1f}deg"
                )
            path_length_m += math.hypot(
                observation.x_m - previous_x,
                observation.y_m - previous_y,
            )
            previous_x = observation.x_m
            previous_y = observation.y_m
            tracker = turn_progress_step(
                tracker, observation.yaw_rad, config.turn_direction_sign
            )
            if now - stall_start >= config.turn_stall_window_sec:
                gained_deg = math.degrees(
                    tracker.accumulated_rad - stall_progress_rad
                )
                if gained_deg < config.turn_stall_min_progress_deg:
                    return None, f"{error_label}:yaw_stall:{gained_deg:.1f}deg"
                stall_start = now
                stall_progress_rad = tracker.accumulated_rad
            threshold_met = tracker.accumulated_rad >= threshold_rad
            if threshold_met:
                if threshold_since is None:
                    threshold_since = now
                node.publish(entry_phase_speed_mps("offboard_stop", config), 0.0)
            else:
                threshold_since = None
                node.publish(
                    config.turn_forward_speed_mps,
                    config.turn_direction_sign * config.turn_lateral_speed_mps,
                )
            hold_sec = 0.0 if threshold_since is None else now - threshold_since
            if now - last_log >= 0.5:
                last_log = now
                chord_m = math.hypot(
                    observation.x_m - start.x_m,
                    observation.y_m - start.y_m,
                )
                print(
                    f"{turn_label}_PROGRESS elapsed={elapsed:.1f}s "
                    f"yaw_progress_deg={math.degrees(tracker.accumulated_rad):.1f} "
                    f"hold={hold_sec:.2f}s chord={chord_m:.3f}m "
                    f"path_length={path_length_m:.3f}m "
                    f"cmd=({0.0 if threshold_met else config.turn_forward_speed_mps:.3f},"
                    f"{0.0 if threshold_met else config.turn_direction_sign * config.turn_lateral_speed_mps:+.3f})",
                    flush=True,
                )
            if threshold_met and hold_sec >= config.turn_completion_hold_sec:
                chord_radius = effective_semicircle_radius_m(
                    start.x_m,
                    start.y_m,
                    observation.x_m,
                    observation.y_m,
                )
                arc_radius = path_length_m / math.pi
                print(
                    f"{turn_label}_TARGET_REACHED "
                    f"yaw_progress_deg={math.degrees(tracker.accumulated_rad):.1f} "
                    f"chord_radius_estimate={chord_radius:.3f}m "
                    f"arc_radius_estimate={arc_radius:.3f}m",
                    flush=True,
                )
                return observation, None
            time.sleep(CONTROL_PERIOD_SEC)
        return None, f"{error_label}:ros_shutdown"

    def wait_for_external_start_gate() -> tuple[bool, str | None]:
        if args.external_start_gate_fd is None:
            return True, None
        assert args.external_start_plan_id is not None
        gate_fd = args.external_start_gate_fd
        expected = external_start_gate_token(args.external_start_plan_id).decode().strip()
        deadline = time.monotonic() + args.external_start_gate_timeout_sec
        buffered = b""
        os.set_blocking(gate_fd, False)
        print(
            "EXTERNAL_START_GATE_READY "
            f"source=PairB plan_id={args.external_start_plan_id} "
            f"timeout_sec={args.external_start_gate_timeout_sec:.1f} "
            "state=MANUAL/disarmed no_setpoint_publication=True "
            f"expected_token={expected}",
            flush=True,
        )
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.0)
            observation = node.observation()
            if not safe_manual_prestate(observation):
                return False, "external_start_gate_safe_prestate_lost"
            readable, _, _ = select.select([gate_fd], [], [], 0.0)
            if readable:
                try:
                    incoming = os.read(
                        gate_fd,
                        EXTERNAL_START_GATE_MAX_BUFFER_BYTES,
                    )
                except BlockingIOError:
                    incoming = b""
                if not incoming:
                    return False, "external_start_gate_closed"
                buffered, accepted, error = consume_external_start_gate_bytes(
                    buffered,
                    incoming,
                    args.external_start_plan_id,
                )
                if error is not None:
                    return False, error
                if accepted:
                    print(
                        "EXTERNAL_START_GATE_ACCEPTED "
                        f"source=PairB plan_id={args.external_start_plan_id}; "
                        "beginning existing zero prestream",
                        flush=True,
                    )
                    return True, None
            time.sleep(CONTROL_PERIOD_SEC)
        return False, "external_start_gate_timeout"

    try:
        if args.recover_only:
            result_code = 0 if node.recover() else 8
            return result_code
        if args.verify_only:
            result = node.wait_for(
                safe_manual_prestate,
                15.0,
                "verify safe final state",
                publish_speed_mps=None,
            )
            result_code = 0 if result is not None else 8
            return result_code

        if node.wait_for(safe_manual_prestate, 30.0, "real-GPS MANUAL/disarmed preflight") is None:
            result_code = 3
            return result_code
        node.configure_body_ned()
        gate_accepted, gate_error = wait_for_external_start_gate()
        if not gate_accepted:
            primary_error = gate_error
            print(f"MISSION_ABORT reason={primary_error}", flush=True)
            result_code = 9
            return result_code
        print("ZERO_PRESTREAM 2.0s", flush=True)
        node.spin_publish(2.0, entry_phase_speed_mps("zero_prestream", config))
        request_speed = entry_phase_speed_mps("request_offboard", config)
        verify_speed = entry_phase_speed_mps("verify_offboard", config)
        for attempt in range(1, 4):
            node.request_offboard(request_speed)
            entered = node.wait_for(
                lambda item: bool(
                    navigation_ready(item)
                    and not item.armed
                    and item.mode.upper() == "OFFBOARD"
                ),
                3.0,
                f"disarmed OFFBOARD observed attempt={attempt}",
                publish_speed_mps=verify_speed,
            )
            if entered is not None:
                break
        else:
            primary_error = "offboard_not_observed"
            result_code = 5
            return result_code

        settle_speed = entry_phase_speed_mps("prearm_offboard_settle", config)
        print(
            f"OFFBOARD_PREARM_SETTLE {OFFBOARD_PREARM_SETTLE_SEC:.2f}s "
            f"body_x={settle_speed:.4f}",
            flush=True,
        )
        node.spin_publish(OFFBOARD_PREARM_SETTLE_SEC, settle_speed)
        settled = node.observation()
        if not (
            navigation_ready(settled)
            and not settled.armed
            and settled.mode.upper() == "OFFBOARD"
        ):
            primary_error = "offboard_prearm_settle_state_invalid"
            result_code = 5
            return result_code

        arm_speed = entry_phase_speed_mps("request_arm", config)
        node.request_arm_once(arm_speed)
        armed_observation = node.wait_for(
            lambda item: bool(
                navigation_ready(item)
                and item.armed
                and item.mode.upper() == "OFFBOARD"
            ),
            2.0,
            "armed OFFBOARD observed after single Arm request",
            publish_speed_mps=entry_phase_speed_mps("verify_arm", config),
        )
        if armed_observation is None:
            primary_error = "single_arm_not_observed"
            result_code = 4
            return result_code

        print(
            f"OFFBOARD_ENTRY_OBSERVED position_delta_m="
            f"{math.hypot(armed_observation.x_m - entered.x_m, armed_observation.y_m - entered.y_m):.3f} "
            f"yaw_delta_deg={math.degrees(wrap_pi(armed_observation.yaw_rad - entered.yaw_rad)):+.2f}; "
            f"relative_reference_pending_course_fit={args.entry_only_hold_sec is None}",
            flush=True,
        )
        if args.entry_only_hold_sec is not None:
            hold_start = time.monotonic()
            hold_speed = entry_phase_speed_mps("offboard_stop", config)
            print(
                f"ENTRY_ONLY_HOLD_START duration={args.entry_only_hold_sec:.1f}s "
                f"body=({hold_speed:.4f},+0.0000) no_trajectory=True",
                flush=True,
            )
            while rclpy.ok() and time.monotonic() - hold_start < args.entry_only_hold_sec:
                rclpy.spin_once(node, timeout_sec=0.0)
                observation = node.observation()
                fault = motion_fault(observation)
                if fault is not None:
                    primary_error = f"entry_only:{fault}"
                    print(f"ENTRY_ONLY_ABORT reason={primary_error}", flush=True)
                    result_code = 6
                    return result_code
                node.publish(hold_speed, 0.0)
                time.sleep(CONTROL_PERIOD_SEC)
            print("ENTRY_ONLY_HOLD_COMPLETE; starting safe recovery", flush=True)
            result_code = 0
            return result_code

        if config.reference_mode == "initial_yaw":
            trajectory = build_mission_trajectory(
                config,
                armed_observation,
                initial_yaw_rad=armed_observation.yaw_rad,
            )
            print(
                f"TRAJECTORY_REFERENCE_LOCKED start=({armed_observation.x_m:.3f},"
                f"{armed_observation.y_m:.3f}) "
                f"pose_yaw_deg={math.degrees(armed_observation.yaw_rad):.2f} "
                f"course_yaw_deg={math.degrees(armed_observation.yaw_rad):.2f} "
                "body_course_offset_deg=+0.00 "
                f"length={trajectory.length_m:.3f}m points={len(trajectory.points)} "
                "source=initial_pose_yaw no_rollout=True no_pre_yaw_alignment=True",
                flush=True,
            )
            mission_end, primary_error = run_trajectory(
                trajectory,
                armed_observation,
                0.0,
            )
            if mission_end is None:
                print(f"MISSION_ABORT reason={primary_error}", flush=True)
                result_code = 6
                return result_code
            print("TRAJECTORY_COMPLETE; starting safe recovery", flush=True)
            result_code = 0
            return result_code

        calibration_start = time.monotonic()
        node.start_rviz_actual()
        calibration_points = [(armed_observation.x_m, armed_observation.y_m)]
        calibration_end = armed_observation
        last_log = 0.0
        print(
            "COURSE_CALIBRATION_START "
            f"distance={config.course_calibration_distance_m:.2f}m "
            f"body=({config.course_calibration_speed_mps:.3f},+0.000) "
            "steering_disabled=True",
            flush=True,
        )
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            observation = node.observation()
            fault = motion_fault(observation)
            if fault is not None:
                primary_error = f"course_calibration:{fault}"
                result_code = 6
                return result_code
            elapsed = time.monotonic() - calibration_start
            displacement_m = math.hypot(
                observation.x_m - armed_observation.x_m,
                observation.y_m - armed_observation.y_m,
            )
            yaw_change_deg = math.degrees(
                wrap_pi(observation.yaw_rad - armed_observation.yaw_rad)
            )
            if abs(yaw_change_deg) > config.course_calibration_max_yaw_change_deg:
                primary_error = f"course_calibration:yaw_change:{yaw_change_deg:.2f}"
                result_code = 6
                return result_code
            if elapsed > config.course_calibration_max_sec:
                primary_error = f"course_calibration:timeout:{displacement_m:.3f}m"
                result_code = 6
                return result_code
            if math.hypot(
                observation.x_m - calibration_points[-1][0],
                observation.y_m - calibration_points[-1][1],
            ) >= 0.01:
                calibration_points.append((observation.x_m, observation.y_m))
            node.update_rviz_actual(observation, observation.yaw_rad)
            if (
                displacement_m >= config.course_calibration_distance_m
                and len(calibration_points) >= 3
            ):
                endpoint = (observation.x_m, observation.y_m)
                if calibration_points[-1] != endpoint:
                    calibration_points.append(endpoint)
                calibration_end = observation
                break
            node.publish(config.course_calibration_speed_mps, 0.0)
            if time.monotonic() - last_log >= 0.5:
                last_log = time.monotonic()
                print(
                    f"COURSE_CALIBRATION_PROGRESS elapsed={elapsed:.1f}s "
                    f"distance={displacement_m:.3f}m "
                    f"yaw_change_deg={yaw_change_deg:+.2f}",
                    flush=True,
                )
            time.sleep(CONTROL_PERIOD_SEC)

        fitted_yaw_rad = fitted_forward_course_yaw(
            calibration_points,
            config.course_calibration_distance_m,
        )
        yaw_disagreement_deg = math.degrees(
            wrap_pi(fitted_yaw_rad - calibration_end.yaw_rad)
        )
        if abs(yaw_disagreement_deg) > config.course_calibration_max_yaw_change_deg:
            primary_error = (
                "course_calibration:pose_course_disagreement:"
                f"{yaw_disagreement_deg:.2f}deg"
            )
            result_code = 6
            return result_code
        body_course_offset_rad = math.radians(yaw_disagreement_deg)
        trajectory = build_mission_trajectory(
            config,
            armed_observation,
            initial_yaw_rad=fitted_yaw_rad,
        )
        print(
            f"TRAJECTORY_REFERENCE_LOCKED start=({armed_observation.x_m:.3f},"
            f"{armed_observation.y_m:.3f}) "
            f"pose_yaw_deg={math.degrees(armed_observation.yaw_rad):.2f} "
            f"course_yaw_deg={math.degrees(fitted_yaw_rad):.2f} "
            f"body_course_offset_deg={yaw_disagreement_deg:+.2f} "
            f"length={trajectory.length_m:.3f}m points={len(trajectory.points)} "
            "source=measured_ground_course no_pre_yaw_alignment=True",
            flush=True,
        )
        mission_end, primary_error = run_trajectory(
            trajectory,
            calibration_end,
            body_course_offset_rad,
            calibration_points,
        )
        if mission_end is None:
            print(f"MISSION_ABORT reason={primary_error}", flush=True)
            result_code = 6
            return result_code
        print("TRAJECTORY_COMPLETE; starting safe recovery", flush=True)
        result_code = 0
        return result_code
    except KeyboardInterrupt:
        primary_error = "keyboard_interrupt"
        print("FORWARD_ABORT reason=keyboard_interrupt", flush=True)
        result_code = 130
        return result_code
    except Exception as exc:
        primary_error = f"exception:{exc}"
        print(f"FORWARD_ABORT reason={primary_error}", flush=True)
        result_code = 7
        return result_code
    finally:
        recovery_override: int | None = None
        if recovery_required:
            try:
                safe = node.recover()
                print(f"FINAL_RECOVERY_SAFE={safe} primary_error={primary_error!r}", flush=True)
                if not safe and result_code == 0:
                    recovery_override = 8
            except Exception as exc:
                print(f"FINAL_RECOVERY_SAFE=False recovery_exception={exc!r}", flush=True)
                if result_code == 0:
                    recovery_override = 8
        node.write_rviz_artifacts(args.trajectory_artifact_dir)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if recovery_override is not None:
            return recovery_override


def execute_phrase_for(
    config: MissionConfig,
    entry_only_hold_sec: float | None = None,
) -> str:
    if entry_only_hold_sec is not None:
        return ENTRY_HOLD_EXECUTE_PHRASE
    if config.s_bend_return:
        return S_BEND_RETURN_EXECUTE_PHRASE
    if config.u_turn and config.turn_direction_sign < 0.0:
        return LEFT_UTURN_EXECUTE_PHRASE
    if config.u_turn:
        return UTURN_EXECUTE_PHRASE
    return EXECUTE_PHRASE


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        config = config_from_args(args)
        validate_entry_hold_sec(args.entry_only_hold_sec)
        validate_external_start_gate_args(
            args.external_start_gate_fd,
            args.external_start_plan_id,
            args.external_start_gate_timeout_sec,
        )
        validate_external_runtime_stop_args(
            args.external_runtime_stop_fd,
            args.external_start_plan_id,
        )
    except ValueError as exc:
        print(f"REFUSED: {exc}")
        return 2
    print_plan(config, args.entry_only_hold_sec)
    if not args.execute and not args.recover_only and not args.verify_only:
        print("DRY RUN ONLY: no ROS import, no process start, no setpoint publication.")
        return 0
    execute_phrase = execute_phrase_for(config, args.entry_only_hold_sec)
    if args.execute and args.confirm != execute_phrase:
        print(f"REFUSED: required --confirm {execute_phrase}")
        return 2
    return run_live(args, config)


if __name__ == "__main__":
    raise SystemExit(main())
