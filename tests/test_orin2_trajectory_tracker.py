import math
import unittest

from src.orin2_trajectory_tracker import (
    PolylineTrajectory,
    ProjectionState,
    TrajectoryPoint,
    TrajectoryTrackerConfig,
    TrajectoryTrackerState,
    assess_trajectory_feasibility,
    bounded_cross_track_integral_step,
    bounded_path_curvature,
    body_vector_for_bearing,
    build_out_and_back_trajectory,
    build_s_bend_return_trajectory,
    build_straight_trajectory,
    project_onto_trajectory,
    curvature_for_body_adapter,
    sampled_arc,
    sampled_quintic_heading_reversal,
    sampled_quintic_lateral_shift,
    slew_limited_body_bearing,
    tracker_state_at_route_start,
    trajectory_from_xy,
    trajectory_reference_curvature,
    trajectory_tracking_step,
    validate_tracker_config,
    wrap_pi,
)


class TrajectoryGeometryTests(unittest.TestCase):
    def test_polyline_samples_by_arc_length(self):
        trajectory = trajectory_from_xy([(0.0, 0.0), (2.0, 0.0), (2.0, 2.0)], 0.06)
        self.assertAlmostEqual(trajectory.length_m, 4.0)
        sample = trajectory.sample(3.0)
        self.assertAlmostEqual(sample.x_m, 2.0)
        self.assertAlmostEqual(sample.y_m, 1.0)
        self.assertAlmostEqual(sample.tangent_yaw_rad, math.pi / 2.0)

    def test_projection_progress_is_monotonic(self):
        trajectory = trajectory_from_xy([(0.0, 0.0), (5.0, 0.0)], 0.06)
        config = TrajectoryTrackerConfig()
        first = project_onto_trajectory(
            trajectory, ProjectionState(), 2.0, 0.2, config
        )
        second = project_onto_trajectory(
            trajectory, first.state, 1.7, 0.1, config
        )
        self.assertAlmostEqual(first.state.progress_s_m, 2.0)
        self.assertEqual(second.state.progress_s_m, first.state.progress_s_m)

    def test_projection_window_does_not_jump_across_a_path_crossing(self):
        trajectory = trajectory_from_xy(
            [(-2.0, 0.0), (2.0, 0.0), (0.0, 2.0), (0.0, -2.0)], 0.06
        )
        config = TrajectoryTrackerConfig(projection_ahead_m=1.0)
        state = ProjectionState(progress_s_m=1.8, segment_index=0, initialized=True)
        projected = project_onto_trajectory(trajectory, state, 0.0, 0.0, config)
        self.assertLess(projected.state.progress_s_m, 3.0)

    def test_new_mission_does_not_project_closed_route_start_to_endpoint(self):
        trajectory = build_s_bend_return_trajectory(
            0.0,
            0.0,
            0.0,
            initial_straight_m=6.0,
            turn_radius_m=3.0,
            straight_speed_mps=0.10,
            turn_speed_mps=0.10,
        )
        config = TrajectoryTrackerConfig(
            projection_ahead_m=4.0,
            goal_tolerance_m=0.15,
        )

        command = trajectory_tracking_step(
            trajectory,
            tracker_state_at_route_start(),
            x_m=0.003,
            y_m=-0.004,
            yaw_rad=0.0,
            measured_speed_mps=0.0,
            config=config,
            dt_sec=0.05,
        )

        self.assertFalse(command.goal_reached)
        self.assertLess(command.progress_s_m, 0.1)
        self.assertGreater(command.remaining_s_m, 47.0)

    def test_validated_external_speed_ceiling_only_reduces_command_magnitude(self):
        trajectory = build_straight_trajectory(0.0, 0.0, 0.0, 5.0, 0.10)
        command = trajectory_tracking_step(
            trajectory,
            tracker_state_at_route_start(),
            x_m=0.0,
            y_m=0.0,
            yaw_rad=0.0,
            measured_speed_mps=0.10,
            speed_ceiling_mps=0.05,
        )
        self.assertAlmostEqual(command.v_mps, 0.05)
        self.assertAlmostEqual(math.hypot(command.body_x_mps, command.body_y_mps), 0.05)

    def test_zero_speed_ceiling_holds_without_changing_tracking_phase(self):
        trajectory = build_straight_trajectory(0.0, 0.0, 0.0, 5.0, 0.10)
        command = trajectory_tracking_step(
            trajectory,
            tracker_state_at_route_start(),
            x_m=0.0,
            y_m=0.0,
            yaw_rad=0.0,
            measured_speed_mps=0.0,
            speed_ceiling_mps=0.0,
        )
        self.assertEqual(command.v_mps, 0.0)
        self.assertEqual(command.omega_radps, 0.0)
        self.assertEqual(command.body_x_mps, 0.0)
        self.assertEqual(command.body_y_mps, 0.0)
        self.assertFalse(command.goal_reached)

    def test_invalid_speed_ceiling_fails_closed(self):
        trajectory = build_straight_trajectory(0.0, 0.0, 0.0, 5.0, 0.10)
        for invalid in (-0.01, 0.251, math.nan):
            with self.assertRaisesRegex(ValueError, "speed ceiling"):
                trajectory_tracking_step(
                    trajectory,
                    tracker_state_at_route_start(),
                    x_m=0.0,
                    y_m=0.0,
                    yaw_rad=0.0,
                    measured_speed_mps=0.0,
                    speed_ceiling_mps=invalid,
                )

    def test_body_ned_adapter_preserves_magnitude_and_inverts_ccw_y(self):
        x_mps, y_mps, limited = body_vector_for_bearing(
            0.06, math.radians(20.0), 32.0
        )
        self.assertAlmostEqual(math.hypot(x_mps, y_mps), 0.06)
        self.assertLess(y_mps, 0.0)
        self.assertAlmostEqual(math.degrees(limited), 20.0)

    def test_body_ned_adapter_bounds_target_bearing(self):
        x_mps, y_mps, limited = body_vector_for_bearing(
            0.06, math.radians(-80.0), 30.0
        )
        self.assertAlmostEqual(math.hypot(x_mps, y_mps), 0.06)
        self.assertGreater(y_mps, 0.0)
        self.assertAlmostEqual(math.degrees(limited), -30.0)

    def test_body_bearing_slew_rejects_abrupt_direction_flip(self):
        previous = math.radians(16.0)
        limited = slew_limited_body_bearing(
            math.radians(-25.0), previous, 25.0, 45.0, 0.20
        )
        self.assertAlmostEqual(math.degrees(limited), 7.0)

    def test_tracker_body_bearing_slew_is_continuous_across_frames(self):
        trajectory = build_straight_trajectory(0.0, 0.0, 0.0, 5.0, 0.10)
        config = TrajectoryTrackerConfig(
            max_body_bearing_deg=25.0,
            max_body_bearing_rate_degps=45.0,
            curvature_to_body_gain_m=0.94,
        )
        state = TrajectoryTrackerState(body_bearing_rad=math.radians(16.0))
        command = trajectory_tracking_step(
            trajectory,
            state,
            1.0,
            0.5,
            math.radians(-30.0),
            0.7,
            config,
            0.20,
        )
        self.assertLessEqual(
            abs(command.target_bearing_error_rad - state.body_bearing_rad),
            math.radians(9.0) + 1.0e-9,
        )
        self.assertEqual(
            command.state.body_bearing_rad,
            command.target_bearing_error_rad,
        )

    def test_left_and_right_arcs_have_expected_geometry(self):
        left = sampled_arc(0.0, 0.0, 0.0, 3.0, math.pi, 0.05, 0.2, "ARC")
        right = sampled_arc(0.0, 0.0, 0.0, 3.0, -math.pi, 0.05, 0.2, "ARC")
        self.assertAlmostEqual(left[-1].x_m, 0.0, places=6)
        self.assertAlmostEqual(left[-1].y_m, 6.0, places=6)
        self.assertAlmostEqual(right[-1].x_m, 0.0, places=6)
        self.assertAlmostEqual(right[-1].y_m, -6.0, places=6)

    def test_reference_curvature_matches_two_meter_arc_and_straight(self):
        straight = build_straight_trajectory(0.0, 0.0, 0.0, 5.0, 0.06)
        left = PolylineTrajectory(
            sampled_arc(0.0, 0.0, 0.0, 2.0, math.pi, 0.05, 0.10, "ARC")
        )
        right = PolylineTrajectory(
            sampled_arc(0.0, 0.0, 0.0, 2.0, -math.pi, 0.05, 0.10, "ARC")
        )
        self.assertAlmostEqual(
            trajectory_reference_curvature(straight, 2.5, 0.45), 0.0
        )
        self.assertAlmostEqual(
            trajectory_reference_curvature(left, left.length_m / 2.0, 0.45),
            0.5,
            delta=0.08,
        )
        self.assertAlmostEqual(
            trajectory_reference_curvature(right, right.length_m / 2.0, 0.45),
            -0.5,
            delta=0.08,
        )

    def test_out_and_back_is_one_continuous_generic_trajectory(self):
        trajectory = build_out_and_back_trajectory(
            0.0, 0.0, 0.0, 5.0, 3.2, True, 0.06, 0.05
        )
        self.assertAlmostEqual(trajectory.points[-1].x_m, 0.0, places=5)
        self.assertAlmostEqual(trajectory.points[-1].y_m, 6.4, places=5)
        phases = {point.phase for point in trajectory.points}
        self.assertEqual(phases, {"LEG1", "UTURN", "LEG2"})
        self.assertGreater(trajectory.length_m, 20.0)

    def test_non_semicircle_arc_uses_requested_angle(self):
        trajectory = build_out_and_back_trajectory(
            0.0,
            0.0,
            0.0,
            2.0,
            2.0,
            True,
            0.06,
            0.05,
            turn_angle_rad=math.pi / 2.0,
        )
        arc_end = next(
            point
            for point, following in zip(trajectory.points, trajectory.points[1:])
            if point.phase == "UTURN" and following.phase == "LEG2"
        )
        self.assertAlmostEqual(arc_end.x_m, 4.0, delta=0.2)
        self.assertAlmostEqual(arc_end.y_m, 2.0, delta=0.2)

    def test_s_bend_return_is_continuous_and_returns_to_start(self):
        trajectory = build_s_bend_return_trajectory(
            0.0, 0.0, 0.0, 6.0, 3.0, 0.10, 0.10
        )
        self.assertGreater(trajectory.length_m, 46.0)
        self.assertLess(trajectory.length_m, 49.0)
        self.assertAlmostEqual(trajectory.points[-1].x_m, 0.0, places=6)
        self.assertAlmostEqual(trajectory.points[-1].y_m, 0.0, places=6)
        phases = {point.phase for point in trajectory.points}
        self.assertEqual(
            phases,
            {"LEG_OUT", "S_CURVE", "RETURN_TURN", "LEG_RETURN"},
        )
        self.assertGreaterEqual(max(point.x_m for point in trajectory.points), 21.6)
        self.assertAlmostEqual(min(point.y_m for point in trajectory.points), -6.0)

    def test_quintic_reversal_has_zero_curvature_joins(self):
        points = sampled_quintic_heading_reversal(
            0.0, 0.0, 0.0, 3.0, True, 0.10, 0.05, "TURN"
        )
        trajectory = PolylineTrajectory(points)
        self.assertAlmostEqual(points[-1].x_m, 0.0, places=6)
        self.assertAlmostEqual(points[-1].y_m, 6.0, places=6)
        self.assertAlmostEqual(
            trajectory.sample(0.02).tangent_yaw_rad,
            0.0,
            delta=0.02,
        )
        self.assertAlmostEqual(
            abs(trajectory.sample(trajectory.length_m - 0.02).tangent_yaw_rad),
            math.pi,
            delta=0.02,
        )
        self.assertLess(
            abs(trajectory_reference_curvature(trajectory, 0.0, 0.45)),
            0.10,
        )
        self.assertLess(
            abs(
                trajectory_reference_curvature(
                    trajectory, trajectory.length_m, 0.45
                )
            ),
            0.10,
        )

    def test_quintic_shift_has_straight_tangents_and_s_curvature(self):
        points = sampled_quintic_lateral_shift(
            0.0, 0.0, 0.0, 12.0, -6.0, 0.10, 0.05, "S"
        )
        start_yaw = math.atan2(
            points[1].y_m - points[0].y_m,
            points[1].x_m - points[0].x_m,
        )
        end_yaw = math.atan2(
            points[-1].y_m - points[-2].y_m,
            points[-1].x_m - points[-2].x_m,
        )
        self.assertAlmostEqual(start_yaw, 0.0, delta=0.001)
        self.assertAlmostEqual(end_yaw, 0.0, delta=0.001)
        slopes = [
            (following.y_m - point.y_m) / (following.x_m - point.x_m)
            for point, following in zip(points, points[1:])
        ]
        slope_changes = [b - a for a, b in zip(slopes, slopes[1:])]
        self.assertLess(min(slope_changes), 0.0)
        self.assertGreater(max(slope_changes), 0.0)

    def test_feasibility_gate_accepts_smooth_route_and_rejects_tight_reversal(self):
        smooth = build_s_bend_return_trajectory(
            0.0, 0.0, 0.0, 6.0, 3.0, 0.10, 0.10
        )
        accepted = assess_trajectory_feasibility(
            smooth,
            reference_window_m=0.45,
            curvature_to_body_gain_m=0.94,
            max_body_bearing_deg=25.0,
            minimum_bearing_reserve_deg=3.0,
            max_curvature_rate_inv_m2=1.0,
        )
        self.assertTrue(accepted.feasible, accepted)
        self.assertLess(accepted.max_nominal_body_bearing_deg, 22.0)
        self.assertLess(accepted.max_abs_curvature_rate_inv_m2, 1.0)

        right = sampled_arc(0.0, 0.0, 0.0, 2.0, -math.pi / 2, 0.10, 0.05, "R")
        left = sampled_arc(
            right[-1].x_m,
            right[-1].y_m,
            -math.pi / 2,
            2.0,
            math.pi / 2,
            0.10,
            0.05,
            "L",
        )
        points = []
        for point in (*right, *left):
            if not points or math.hypot(point.x_m - points[-1].x_m, point.y_m - points[-1].y_m) > 1e-6:
                points.append(point)
        rejected = assess_trajectory_feasibility(
            PolylineTrajectory(points),
            reference_window_m=0.45,
            curvature_to_body_gain_m=0.94,
            max_body_bearing_deg=25.0,
            minimum_bearing_reserve_deg=3.0,
            max_curvature_rate_inv_m2=2.3,
        )
        self.assertFalse(rejected.feasible)
        self.assertTrue(rejected.reasons)


class TrajectoryControllerTests(unittest.TestCase):
    def test_curvature_feedback_is_bounded_around_reference_not_zero(self):
        self.assertAlmostEqual(bounded_path_curvature(0.80, 0.50, 0.12), 0.62)
        self.assertAlmostEqual(bounded_path_curvature(0.20, 0.50, 0.12), 0.38)
        self.assertAlmostEqual(bounded_path_curvature(-0.20, 0.0, 0.12), -0.12)
        self.assertAlmostEqual(bounded_path_curvature(0.80, 0.50, 0.0), 0.80)

    def test_body_adapter_separates_reference_and_feedback_gain(self):
        self.assertAlmostEqual(curvature_for_body_adapter(0.4, 0.3, 1.0), 0.3)
        self.assertAlmostEqual(curvature_for_body_adapter(0.4, 0.3, 1.5), 0.25)
        self.assertAlmostEqual(curvature_for_body_adapter(0.4, 0.4, 2.0), 0.4)

    def test_bounded_cross_track_integral_accumulates_and_anti_windups(self):
        accumulated = bounded_cross_track_integral_step(
            0.0, 0.2, 0.1, 0.1, 1.0, -0.02, 0.12
        )
        self.assertAlmostEqual(accumulated, 0.02)
        saturated = bounded_cross_track_integral_step(
            accumulated, 0.5, 0.1, 0.1, 1.0, -0.20, 0.12
        )
        self.assertEqual(saturated, accumulated)
        unwound = bounded_cross_track_integral_step(
            accumulated, -0.1, 0.1, 0.1, 1.0, 0.20, 0.12
        )
        self.assertLess(unwound, accumulated)

    def test_feedback_ratio_strengthens_straight_correction_without_changing_speed(self):
        trajectory = build_straight_trajectory(0.0, 0.0, 0.0, 5.0, 0.12)
        base = trajectory_tracking_step(
            trajectory,
            TrajectoryTrackerState(),
            0.0,
            0.4,
            0.0,
            0.7,
            TrajectoryTrackerConfig(
                max_curvature_correction_inv_m=0.12,
                curvature_to_body_gain_m=0.94,
            ),
        )
        stronger = trajectory_tracking_step(
            trajectory,
            TrajectoryTrackerState(),
            0.0,
            0.4,
            0.0,
            0.7,
            TrajectoryTrackerConfig(
                max_curvature_correction_inv_m=0.12,
                curvature_to_body_gain_m=0.94,
                curvature_feedback_gain_ratio=1.28,
            ),
        )
        self.assertGreater(
            abs(stronger.target_bearing_error_rad),
            abs(base.target_bearing_error_rad),
        )
        self.assertAlmostEqual(stronger.v_mps, base.v_mps)

    def test_cross_track_integral_is_bounded_in_tracker_state(self):
        trajectory = build_straight_trajectory(0.0, 0.0, 0.0, 5.0, 0.12)
        config = TrajectoryTrackerConfig(
            max_curvature_correction_inv_m=0.5,
            curvature_to_body_gain_m=0.94,
            curvature_feedback_gain_ratio=1.28,
            cross_track_integral_gain_inv_m_per_m_sec=0.04,
            cross_track_integral_limit_m_sec=0.10,
        )
        state = TrajectoryTrackerState()
        for _ in range(20):
            command = trajectory_tracking_step(
                trajectory, state, 0.5, 0.2, 0.0, 0.5, config, 0.1
            )
            state = command.state
        self.assertGreater(state.cross_track_integral_m_sec, 0.0)
        self.assertLessEqual(state.cross_track_integral_m_sec, 0.10)
        self.assertLess(command.feedback_curvature_inv_m, 0.0)

    def test_small_cross_track_error_is_corrected_without_fixed_deadband(self):
        trajectory = build_straight_trajectory(0.0, 0.0, 0.0, 5.0, 0.06)
        command = trajectory_tracking_step(
            trajectory,
            TrajectoryTrackerState(),
            0.0,
            0.08,
            0.0,
            0.5,
        )
        self.assertGreater(command.body_y_mps, 0.0)
        self.assertLess(command.omega_radps, 0.0)

    def test_command_vector_does_not_accelerate_when_turning(self):
        trajectory = build_straight_trajectory(0.0, 0.0, 0.0, 5.0, 0.06)
        command = trajectory_tracking_step(
            trajectory,
            TrajectoryTrackerState(),
            0.0,
            0.8,
            0.0,
            0.5,
        )
        self.assertLessEqual(math.hypot(command.body_x_mps, command.body_y_mps), 0.06)
        self.assertLess(command.body_x_mps, command.v_mps)

    def test_curvature_and_terminal_distance_reduce_speed(self):
        straight = build_straight_trajectory(0.0, 0.0, 0.0, 5.0, 0.06)
        cruise = trajectory_tracking_step(
            straight, TrajectoryTrackerState(), 1.0, 0.0, 0.0, 0.5
        )
        near_goal_state = TrajectoryTrackerState(
            ProjectionState(progress_s_m=4.6, segment_index=30, initialized=True)
        )
        terminal = trajectory_tracking_step(
            straight, near_goal_state, 4.6, 0.0, 0.0, 0.5
        )
        self.assertGreater(cruise.v_mps, terminal.v_mps)
        self.assertGreaterEqual(terminal.v_mps, TrajectoryTrackerConfig().minimum_tracking_speed_mps)

    def test_arbitrary_s_curve_uses_same_tracker(self):
        coordinates = [
            (index * 0.25, 0.7 * math.sin(index * 0.25 * math.pi / 4.0))
            for index in range(49)
        ]
        trajectory = trajectory_from_xy(coordinates, 0.06, "CORRIDOR")
        command = trajectory_tracking_step(
            trajectory,
            TrajectoryTrackerState(),
            0.0,
            -0.25,
            0.0,
            0.4,
        )
        self.assertEqual(command.phase, "CORRIDOR")
        self.assertGreater(command.omega_radps, 0.0)
        self.assertLess(command.body_y_mps, 0.0)

    def test_calibrated_curvature_adapter_tracks_two_meter_arc_without_pivot(self):
        trajectory = PolylineTrajectory(
            sampled_arc(0.0, 0.0, 0.0, 2.0, math.pi, 0.05, 0.10, "ARC")
        )
        sample = trajectory.sample(trajectory.length_m / 2.0)
        command = trajectory_tracking_step(
            trajectory,
            TrajectoryTrackerState(
                ProjectionState(
                    progress_s_m=sample.s_m,
                    segment_index=sample.segment_index,
                    initialized=True,
                )
            ),
            sample.x_m,
            sample.y_m,
            sample.tangent_yaw_rad,
            0.40,
            TrajectoryTrackerConfig(
                max_body_bearing_deg=89.0,
                curvature_to_body_gain_m=3.5,
            ),
        )
        self.assertAlmostEqual(command.reference_curvature_inv_m, 0.5, delta=0.08)
        self.assertGreater(math.degrees(command.target_bearing_error_rad), 40.0)
        self.assertLess(math.degrees(command.target_bearing_error_rad), 65.0)
        self.assertGreater(command.body_x_mps, 0.0)
        self.assertLess(command.body_y_mps, 0.0)

    def test_goal_is_zero_output(self):
        trajectory = build_straight_trajectory(0.0, 0.0, 0.0, 5.0, 0.06)
        state = TrajectoryTrackerState(
            ProjectionState(progress_s_m=trajectory.length_m, segment_index=32, initialized=True)
        )
        command = trajectory_tracking_step(
            trajectory, state, 5.0, 0.0, 0.0, 0.0
        )
        self.assertTrue(command.goal_reached)
        self.assertFalse(command.terminal_missed)
        self.assertEqual(
            (command.v_mps, command.omega_radps, command.body_x_mps, command.body_y_mps),
            (0.0, 0.0, 0.0, 0.0),
        )

    def test_terminal_gate_checks_along_and_cross_separately(self):
        trajectory = build_straight_trajectory(0.0, 0.0, 0.0, 5.0, 0.06)
        state = TrajectoryTrackerState(
            ProjectionState(progress_s_m=4.8, segment_index=31, initialized=True)
        )
        command = trajectory_tracking_step(
            trajectory, state, 4.89, 0.11, 0.0, 0.4
        )
        self.assertTrue(command.goal_reached)
        self.assertFalse(command.terminal_missed)
        self.assertEqual((command.body_x_mps, command.body_y_mps), (0.0, 0.0))

    def test_terminal_cross_track_miss_is_zero_and_fail_closed(self):
        trajectory = build_straight_trajectory(0.0, 0.0, 0.0, 5.0, 0.06)
        state = TrajectoryTrackerState(
            ProjectionState(progress_s_m=4.8, segment_index=31, initialized=True)
        )
        command = trajectory_tracking_step(
            trajectory, state, 4.89, -0.16, 0.0, 0.4
        )
        self.assertFalse(command.goal_reached)
        self.assertTrue(command.terminal_missed)
        self.assertEqual((command.body_x_mps, command.body_y_mps), (0.0, 0.0))
        self.assertEqual(command.target_bearing_error_rad, 0.0)

    def test_tracker_config_rejects_unsafe_bounds(self):
        validate_tracker_config(TrajectoryTrackerConfig())
        with self.assertRaises(ValueError):
            validate_tracker_config(TrajectoryTrackerConfig(max_body_bearing_deg=90.0))
        with self.assertRaises(ValueError):
            validate_tracker_config(TrajectoryTrackerConfig(body_y_for_ccw_sign=0.0))
        with self.assertRaises(ValueError):
            validate_tracker_config(
                TrajectoryTrackerConfig(max_body_bearing_rate_degps=4.0)
            )
        with self.assertRaises(ValueError):
            validate_tracker_config(
                TrajectoryTrackerConfig(curvature_to_body_gain_m=9.0)
            )
        with self.assertRaises(ValueError):
            validate_tracker_config(
                TrajectoryTrackerConfig(max_curvature_correction_inv_m=1.1)
            )
        with self.assertRaises(ValueError):
            validate_tracker_config(
                TrajectoryTrackerConfig(curvature_feedback_gain_ratio=0.0)
            )
        with self.assertRaises(ValueError):
            validate_tracker_config(
                TrajectoryTrackerConfig(
                    cross_track_integral_gain_inv_m_per_m_sec=0.04,
                    cross_track_integral_limit_m_sec=0.0,
                )
            )

    def test_kinematic_replay_tracks_general_s_curve(self):
        coordinates = [
            (index * 0.20, 0.65 * math.sin(index * 0.20 * math.pi / 4.0))
            for index in range(61)
        ]
        trajectory = trajectory_from_xy(coordinates, 0.055, "CORRIDOR")
        state = TrajectoryTrackerState()
        x_m, y_m, yaw_rad = 0.0, -0.30, 0.0
        dt_sec = 0.05
        for _ in range(1400):
            command = trajectory_tracking_step(
                trajectory, state, x_m, y_m, yaw_rad, 0.45
            )
            state = command.state
            if command.goal_reached:
                break
            actual_speed = 8.0 * command.v_mps
            yaw_rate = -10.0 * command.body_y_mps
            yaw_rad = wrap_pi(yaw_rad + yaw_rate * dt_sec)
            x_m += actual_speed * math.cos(yaw_rad) * dt_sec
            y_m += actual_speed * math.sin(yaw_rad) * dt_sec
        self.assertTrue(command.goal_reached)
        self.assertLess(math.hypot(x_m - trajectory.points[-1].x_m, y_m - trajectory.points[-1].y_m), 0.20)

    def test_kinematic_replay_tracks_continuous_u_path_without_phase_reset(self):
        trajectory = build_out_and_back_trajectory(
            0.0, 0.0, 0.0, 5.0, 3.2, True, 0.055, 0.045
        )
        state = TrajectoryTrackerState()
        x_m, y_m, yaw_rad = 0.0, -0.15, math.radians(3.0)
        dt_sec = 0.05
        phases = []
        maximum_cross_m = 0.0
        for _ in range(2600):
            command = trajectory_tracking_step(
                trajectory, state, x_m, y_m, yaw_rad, 0.42
            )
            state = command.state
            maximum_cross_m = max(maximum_cross_m, abs(command.cross_track_m))
            if not phases or phases[-1] != command.phase:
                phases.append(command.phase)
            if command.goal_reached:
                break
            actual_speed = 8.0 * command.v_mps
            yaw_rate = -10.0 * command.body_y_mps
            yaw_rad = wrap_pi(yaw_rad + yaw_rate * dt_sec)
            x_m += actual_speed * math.cos(yaw_rad) * dt_sec
            y_m += actual_speed * math.sin(yaw_rad) * dt_sec
        self.assertTrue(command.goal_reached)
        self.assertEqual(phases[:3], ["LEG1", "UTURN", "LEG2"])
        self.assertLess(maximum_cross_m, 0.40)
        self.assertLess(
            math.hypot(
                x_m - trajectory.points[-1].x_m,
                y_m - trajectory.points[-1].y_m,
            ),
            0.20,
        )

    def test_orin1_adapter_replay_tracks_smooth_closed_complex_path(self):
        trajectory = build_s_bend_return_trajectory(
            0.0, 0.0, 0.0, 6.0, 3.0, 0.10, 0.10
        )
        config = TrajectoryTrackerConfig(
            base_lookahead_m=1.20,
            min_lookahead_m=1.00,
            max_lookahead_m=2.00,
            max_body_bearing_deg=25.0,
            max_body_bearing_rate_degps=45.0,
            curvature_to_body_gain_m=0.94,
            curvature_feedback_gain_ratio=1.28,
            max_curvature_correction_inv_m=0.24,
            cross_track_integral_gain_inv_m_per_m_sec=0.04,
            cross_track_integral_limit_m_sec=1.0,
            curvature_slowdown_gain=0.0,
            minimum_tracking_speed_mps=0.10,
            terminal_slowdown_distance_m=1.5,
            goal_tolerance_m=0.15,
            max_cross_track_m=1.5,
        )
        state = TrajectoryTrackerState()
        x_m, y_m, yaw_rad = 0.0, -0.10, math.radians(2.0)
        dt_sec = 0.05
        maximum_cross_m = 0.0
        for _ in range(2400):
            command = trajectory_tracking_step(
                trajectory, state, x_m, y_m, yaw_rad, 0.70, config, dt_sec
            )
            state = command.state
            maximum_cross_m = max(maximum_cross_m, abs(command.cross_track_m))
            if command.goal_reached:
                break
            actual_speed_mps = 7.0 * command.v_mps
            yaw_rate_radps = -13.0 * command.body_y_mps
            yaw_rad = wrap_pi(yaw_rad + yaw_rate_radps * dt_sec)
            x_m += actual_speed_mps * math.cos(yaw_rad) * dt_sec
            y_m += actual_speed_mps * math.sin(yaw_rad) * dt_sec
        self.assertTrue(command.goal_reached)
        self.assertLess(maximum_cross_m, 0.20)
        self.assertLess(math.hypot(x_m, y_m), 0.15)

    def test_split_feedback_and_integral_reject_persistent_rover_bias(self):
        trajectory = build_out_and_back_trajectory(
            0.0, 0.0, 0.0, 5.0, 3.0, False, 0.12, 0.12
        )
        common = dict(
            base_lookahead_m=1.20,
            min_lookahead_m=1.00,
            max_lookahead_m=2.00,
            max_body_bearing_deg=25.0,
            max_curvature_correction_inv_m=0.12,
            curvature_slowdown_gain=0.0,
            minimum_tracking_speed_mps=0.10,
            goal_tolerance_m=0.25,
        )
        baseline = TrajectoryTrackerConfig(
            **common,
            curvature_to_body_gain_m=0.80,
        )
        improved = TrajectoryTrackerConfig(
            **common,
            curvature_to_body_gain_m=0.94,
            curvature_feedback_gain_ratio=1.28,
            cross_track_integral_gain_inv_m_per_m_sec=0.04,
            cross_track_integral_limit_m_sec=1.0,
        )

        def replay(config):
            state = TrajectoryTrackerState()
            x_m = y_m = yaw_rad = 0.0
            maximum_cross_m = 0.0
            for _ in range(2000):
                command = trajectory_tracking_step(
                    trajectory,
                    state,
                    x_m,
                    y_m,
                    yaw_rad,
                    0.70,
                    config,
                    0.05,
                )
                state = command.state
                maximum_cross_m = max(
                    maximum_cross_m, abs(command.cross_track_m)
                )
                if command.goal_reached or command.terminal_missed:
                    return command, maximum_cross_m
                actual_speed_mps = 6.0 * command.v_mps
                yaw_rate_radps = -8.0 * command.body_y_mps + 0.08
                yaw_rad = wrap_pi(yaw_rad + yaw_rate_radps * 0.05)
                x_m += actual_speed_mps * math.cos(yaw_rad) * 0.05
                y_m += actual_speed_mps * math.sin(yaw_rad) * 0.05
            self.fail("biased trajectory replay did not terminate")

        baseline_command, baseline_cross = replay(baseline)
        improved_command, improved_cross = replay(improved)
        self.assertTrue(baseline_command.goal_reached)
        self.assertTrue(improved_command.goal_reached)
        self.assertLess(improved_cross, baseline_cross)

    def test_kinematic_replay_tracks_one_orbit_then_tangent_exit(self):
        radius_m = 4.5
        points = []
        samples = 144
        for index in range(samples + 1):
            angle = 2.0 * math.pi * index / samples
            points.append(
                TrajectoryPoint(
                    radius_m * math.cos(angle),
                    radius_m * math.sin(angle),
                    0.05,
                    "ORBIT",
                )
            )
        for index in range(1, 41):
            points.append(
                TrajectoryPoint(radius_m, index * 0.15, 0.055, "TERMINAL")
            )
        trajectory = PolylineTrajectory(points)
        state = TrajectoryTrackerState()
        x_m, y_m, yaw_rad = radius_m, 0.0, math.pi / 2.0
        dt_sec = 0.05
        phases = set()
        for _ in range(3600):
            command = trajectory_tracking_step(
                trajectory, state, x_m, y_m, yaw_rad, 0.42
            )
            state = command.state
            phases.add(command.phase)
            if command.goal_reached:
                break
            actual_speed = 8.0 * command.v_mps
            yaw_rate = -10.0 * command.body_y_mps
            yaw_rad = wrap_pi(yaw_rad + yaw_rate * dt_sec)
            x_m += actual_speed * math.cos(yaw_rad) * dt_sec
            y_m += actual_speed * math.sin(yaw_rad) * dt_sec
        self.assertTrue(command.goal_reached)
        self.assertIn("ORBIT", phases)
        self.assertIn("TERMINAL", phases)
        self.assertGreater(command.progress_s_m, 2.0 * math.pi * radius_m)
        self.assertLess(
            math.hypot(
                x_m - trajectory.points[-1].x_m,
                y_m - trajectory.points[-1].y_m,
            ),
            0.20,
        )


if __name__ == "__main__":
    unittest.main()
