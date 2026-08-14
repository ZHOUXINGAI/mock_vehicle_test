import math
import unittest

from src.orin2_outdoor_forward_5m import (
    CONTROL_PERIOD_SEC,
    EXIT_STOP_BURST_SEC,
    OFFBOARD_HEADING_HOLD_SPEED_MPS,
    OFFBOARD_PREARM_SETTLE_SEC,
    STATE_MAX_AGE_SEC,
    HeadingPidState,
    PathFollowerState,
    TurnTracker,
    adapt_body_forward_mps,
    build_parser,
    build_mission_trajectory,
    build_current_yaw_reference,
    commanded_forward_speed,
    config_from_args,
    MissionConfig,
    Observation,
    entry_phase_speed_mps,
    entry_phase_lateral_mps,
    execute_phrase_for,
    external_runtime_stop_token,
    external_start_gate_token,
    consume_external_runtime_stop_bytes,
    consume_external_start_gate_bytes,
    feedback_directions_consistent,
    fitted_forward_course_yaw,
    effective_semicircle_radius_m,
    heading_pid_step,
    path_following_step,
    position_course_error,
    position_course_limit_exceeded,
    reset_path_follower_for_conflict,
    recovery_hold_speed_mps,
    manual_arm_ready,
    mission_trajectory_feasibility_report,
    motion_fault,
    navigation_ready,
    safe_manual_prestate,
    select_straight_heading_feedback,
    track_metrics,
    trajectory_motion_timeout_sec,
    tracker_config_from_mission,
    tracking_yaw_with_course_offset,
    turn_progress_step,
    validate_config,
    validate_entry_hold_sec,
    validate_external_runtime_stop_args,
    validate_external_start_gate_args,
)


def ready_observation(**changes):
    values = dict(
        state_present=True,
        state_age_sec=0.1,
        connected=True,
        armed=False,
        mode="MANUAL",
        manual_input=True,
        pose_present=True,
        pose_age_sec=0.1,
        x_m=0.0,
        y_m=0.0,
        yaw_rad=0.0,
        gps_present=True,
        gps_age_sec=0.1,
        gps_status=0,
        latitude_deg=22.0,
        longitude_deg=114.0,
        gps_raw_present=True,
        gps_raw_age_sec=0.1,
        gps_fix_type=3,
        satellites_visible=10,
    )
    values.update(changes)
    return Observation(**values)


class OutdoorForwardGeometryTests(unittest.TestCase):
    def test_trajectory_timeout_scales_with_length_below_hard_ceiling(self):
        self.assertEqual(trajectory_motion_timeout_sec(5.0, 180.0), 30.0)
        self.assertAlmostEqual(
            trajectory_motion_timeout_sec(46.791, 180.0),
            116.9775,
        )
        self.assertEqual(trajectory_motion_timeout_sec(100.0, 180.0), 180.0)
        with self.assertRaises(ValueError):
            trajectory_motion_timeout_sec(math.nan, 180.0)

    def test_vehicle_forward_adapter_is_explicit_and_zero_preserving(self):
        self.assertEqual(adapt_body_forward_mps(0.06, MissionConfig()), 0.06)
        reversed_config = MissionConfig(body_forward_sign=-1.0)
        self.assertEqual(adapt_body_forward_mps(0.06, reversed_config), -0.06)
        self.assertEqual(adapt_body_forward_mps(0.0, reversed_config), 0.0)

    def test_exit_stop_burst_is_three_frames_without_long_offboard_hold(self):
        self.assertAlmostEqual(EXIT_STOP_BURST_SEC, 3 * CONTROL_PERIOD_SEC)
        self.assertLessEqual(EXIT_STOP_BURST_SEC, 0.2)

    def test_track_metrics_at_zero_heading(self):
        along, cross, displacement, heading_error = track_metrics(
            1.0, 2.0, 0.0, 5.0, 2.5, math.radians(10.0)
        )
        self.assertAlmostEqual(along, 4.0)
        self.assertAlmostEqual(cross, 0.5)
        self.assertAlmostEqual(displacement, math.hypot(4.0, 0.5))
        self.assertAlmostEqual(math.degrees(heading_error), 10.0)

    def test_track_metrics_follow_rotated_heading(self):
        along, cross, _, _ = track_metrics(
            0.0, 0.0, math.pi / 2.0, 0.2, 3.0, math.pi / 2.0
        )
        self.assertAlmostEqual(along, 3.0)
        self.assertAlmostEqual(cross, -0.2)

    def test_ground_course_fit_rejects_magnetometer_bias(self):
        points = [
            (0.00, 0.00),
            (-0.20, -0.01),
            (-0.40, 0.01),
            (-0.60, -0.01),
            (-0.80, 0.01),
            (-1.01, 0.00),
        ]
        course = fitted_forward_course_yaw(points, 1.0)
        self.assertLess(abs(math.degrees(abs(course) - math.pi)), 1.0)

    def test_ground_course_fit_is_directed_and_bounded(self):
        course = fitted_forward_course_yaw(
            [(0.0, 0.0), (0.4, 0.4), (0.8, 0.8)],
            1.0,
        )
        self.assertAlmostEqual(math.degrees(course), 45.0)
        with self.assertRaises(ValueError):
            fitted_forward_course_yaw([(0.0, 0.0), (1.0, 0.0)], 1.0)
        with self.assertRaises(ValueError):
            fitted_forward_course_yaw(
                [(0.0, 0.0), (0.2, 0.0), (0.4, 0.0)],
                1.0,
            )

    def test_relative_trajectory_accepts_measured_course_override(self):
        start = ready_observation(x_m=2.0, y_m=3.0, yaw_rad=math.radians(30.0))
        trajectory = build_mission_trajectory(
            MissionConfig(distance_m=6.0),
            start,
            initial_yaw_rad=math.radians(-20.0),
        )
        self.assertAlmostEqual(
            math.degrees(trajectory.sample(0.0).tangent_yaw_rad),
            -20.0,
        )

    def test_initial_yaw_reference_starts_at_pose_without_rollout(self):
        start = ready_observation(x_m=2.0, y_m=3.0, yaw_rad=math.radians(17.0))
        config = MissionConfig(distance_m=10.0, reference_mode="initial_yaw")
        trajectory = build_mission_trajectory(
            config,
            start,
            initial_yaw_rad=start.yaw_rad,
        )
        first = trajectory.sample(0.0)
        self.assertAlmostEqual(first.x_m, start.x_m)
        self.assertAlmostEqual(first.y_m, start.y_m)
        self.assertAlmostEqual(first.tangent_yaw_rad, start.yaw_rad)
        self.assertAlmostEqual(trajectory.length_m, 10.0)

    def test_initial_yaw_reference_cli_is_explicit(self):
        parser = build_parser()
        config = config_from_args(
            parser.parse_args(["--reference-mode", "initial_yaw"])
        )
        self.assertEqual(config.reference_mode, "initial_yaw")

    def test_s_bend_return_cli_builds_closed_route(self):
        parser = build_parser()
        config = config_from_args(
            parser.parse_args(
                [
                    "--s-bend-return",
                    "--distance-m",
                    "6.0",
                    "--turn-radius-m",
                    "3.0",
                    "--speed-mps",
                    "0.10",
                    "--turn-forward-speed-mps",
                    "0.10",
                    "--terminal-speed-mps",
                    "0.10",
                    "--reference-mode",
                    "initial_yaw",
                ]
            )
        )
        trajectory = build_mission_trajectory(config, ready_observation())
        self.assertTrue(config.s_bend_return)
        self.assertAlmostEqual(trajectory.points[-1].x_m, 0.0, places=6)
        self.assertAlmostEqual(trajectory.points[-1].y_m, 0.0, places=6)

    def test_s_bend_return_and_u_turn_are_mutually_exclusive(self):
        parser = build_parser()
        with self.assertRaises(ValueError):
            config_from_args(parser.parse_args(["--s-bend-return", "--u-turn"]))

    def test_s_bend_return_has_distinct_exact_confirmation_phrase(self):
        config = MissionConfig(s_bend_return=True)
        self.assertEqual(
            execute_phrase_for(config),
            "OUTDOOR_S_BEND_RETURN_AREA_CLEAR_RC_KILL_READY",
        )
        self.assertNotEqual(execute_phrase_for(config), execute_phrase_for(MissionConfig()))

    def test_smooth_s_bend_passes_generic_vehicle_feasibility_gate(self):
        config = MissionConfig(
            distance_m=6.0,
            speed_mps=0.10,
            terminal_speed_mps=0.10,
            max_speed_mps=0.22,
            s_bend_return=True,
            turn_radius_m=3.0,
            turn_forward_speed_mps=0.10,
            tracker_max_body_bearing_deg=25.0,
            tracker_curvature_to_body_gain_m=0.94,
            tracker_max_curvature_correction_inv_m=0.24,
        )
        validate_config(config)
        report = mission_trajectory_feasibility_report(config)
        self.assertTrue(report.feasible)
        self.assertLess(report.max_nominal_body_bearing_deg, 22.0)

    def test_tight_r2_route_is_rejected_before_live_execution(self):
        config = MissionConfig(
            distance_m=6.0,
            speed_mps=0.10,
            terminal_speed_mps=0.10,
            max_speed_mps=0.22,
            s_bend_return=True,
            turn_radius_m=2.0,
            turn_forward_speed_mps=0.10,
            tracker_max_body_bearing_deg=25.0,
            tracker_curvature_to_body_gain_m=0.94,
            tracker_max_curvature_correction_inv_m=0.24,
        )
        with self.assertRaisesRegex(ValueError, "trajectory geometry infeasible"):
            validate_config(config)

    def test_course_offset_prevents_false_heading_correction_at_handoff(self):
        pose_yaw = math.radians(-173.73)
        measured_course = math.radians(-179.04)
        offset = math.atan2(
            math.sin(measured_course - pose_yaw),
            math.cos(measured_course - pose_yaw),
        )
        effective_yaw = tracking_yaw_with_course_offset(pose_yaw, offset)
        self.assertAlmostEqual(math.degrees(effective_yaw), -179.04, places=2)
        with self.assertRaises(ValueError):
            tracking_yaw_with_course_offset(math.nan, offset)

    def test_run_252_replay_course_fit_reduces_reference_cross_track(self):
        magnetometer_yaw = math.radians(-173.73)
        measured_along_cross = [
            (0.00, 0.000),
            (0.28, -0.013),
            (0.51, -0.026),
            (1.02, -0.095),
            (2.02, -0.200),
            (3.04, -0.328),
            (6.05, -0.407),
        ]

        def to_enu(along, cross):
            return (
                along * math.cos(magnetometer_yaw)
                - cross * math.sin(magnetometer_yaw),
                along * math.sin(magnetometer_yaw)
                + cross * math.cos(magnetometer_yaw),
            )

        points = [to_enu(along, cross) for along, cross in measured_along_cross]
        fitted_yaw = fitted_forward_course_yaw(points[:4], 1.0)
        end_x, end_y = points[-1]
        _, fitted_cross, _, _ = track_metrics(
            0.0, 0.0, fitted_yaw, end_x, end_y, fitted_yaw
        )
        self.assertLess(abs(fitted_cross), abs(measured_along_cross[-1][1]) * 0.50)
        self.assertAlmostEqual(math.degrees(fitted_yaw), -179.04, delta=1.0)

    def test_offboard_handoff_keeps_heading_defined_without_post_mode_dwell(self):
        config = MissionConfig(speed_mps=0.12)
        self.assertEqual(entry_phase_speed_mps("zero_prestream", config), 0.0)
        self.assertEqual(entry_phase_speed_mps("manual_arm_wait", config), 0.0)
        self.assertEqual(
            entry_phase_speed_mps("request_offboard", config),
            OFFBOARD_HEADING_HOLD_SPEED_MPS,
        )
        self.assertEqual(
            entry_phase_speed_mps("verify_offboard", config),
            OFFBOARD_HEADING_HOLD_SPEED_MPS,
        )
        self.assertEqual(
            entry_phase_speed_mps("offboard_stop", config),
            OFFBOARD_HEADING_HOLD_SPEED_MPS,
        )
        for phase in ("prearm_offboard_settle", "request_arm", "verify_arm"):
            self.assertEqual(
                entry_phase_speed_mps(phase, config),
                OFFBOARD_HEADING_HOLD_SPEED_MPS,
            )
        self.assertEqual(entry_phase_speed_mps("motion", config), 0.12)
        self.assertTrue(math.isfinite(OFFBOARD_HEADING_HOLD_SPEED_MPS))
        self.assertGreater(OFFBOARD_HEADING_HOLD_SPEED_MPS, 0.0)
        self.assertLessEqual(OFFBOARD_HEADING_HOLD_SPEED_MPS, 1.0e-4)
        self.assertGreaterEqual(OFFBOARD_PREARM_SETTLE_SEC, 0.2)
        self.assertLessEqual(OFFBOARD_PREARM_SETTLE_SEC, 0.5)

    def test_recovery_uses_heading_defined_stop_until_fresh_disarm(self):
        self.assertEqual(
            recovery_hold_speed_mps(ready_observation(armed=True, mode="OFFBOARD")),
            OFFBOARD_HEADING_HOLD_SPEED_MPS,
        )
        self.assertEqual(
            recovery_hold_speed_mps(
                ready_observation(
                    armed=False,
                    mode="MANUAL",
                    state_age_sec=STATE_MAX_AGE_SEC + 0.01,
                )
            ),
            OFFBOARD_HEADING_HOLD_SPEED_MPS,
        )
        self.assertEqual(
            recovery_hold_speed_mps(ready_observation(armed=False, mode="MANUAL")),
            0.0,
        )

    def test_entry_only_hold_is_short_and_bounded(self):
        validate_entry_hold_sec(None)
        validate_entry_hold_sec(1.0)
        validate_entry_hold_sec(2.0)
        validate_entry_hold_sec(5.0)
        for invalid in (0.0, 5.01, math.nan, math.inf):
            with self.assertRaises(ValueError):
                validate_entry_hold_sec(invalid)

    def test_left_trim_is_motion_only(self):
        config = MissionConfig(steering_trim_mps=-0.012)
        self.assertEqual(entry_phase_lateral_mps("zero_prestream", config), 0.0)
        self.assertEqual(entry_phase_lateral_mps("manual_arm_wait", config), 0.0)
        self.assertEqual(entry_phase_lateral_mps("request_offboard", config), 0.0)
        self.assertEqual(entry_phase_lateral_mps("verify_offboard", config), 0.0)
        self.assertEqual(entry_phase_lateral_mps("offboard_stop", config), 0.0)
        self.assertEqual(entry_phase_lateral_mps("request_arm", config), 0.0)
        self.assertEqual(entry_phase_lateral_mps("motion", config), -0.012)

    def test_heading_pid_uses_measured_error_with_proven_sign(self):
        config = MissionConfig()
        straight, state = heading_pid_step(config, HeadingPidState(), 0.0, 0.05)
        right_drift, _ = heading_pid_step(
            config, state, math.radians(-4.0), 0.05
        )
        left_drift, _ = heading_pid_step(
            config, HeadingPidState(), math.radians(4.0), 0.05
        )
        self.assertEqual(straight, 0.0)
        self.assertLessEqual(right_drift, -config.min_effective_steering_mps)
        self.assertGreaterEqual(left_drift, config.min_effective_steering_mps)

    def test_heading_pid_deadband_suppresses_ineffective_chatter(self):
        config = MissionConfig()
        for error_deg in (-0.99, 0.0, 0.99):
            output, _ = heading_pid_step(
                config, HeadingPidState(), math.radians(error_deg), 0.05
            )
            self.assertEqual(output, 0.0)

    def test_heading_pid_is_bounded_and_anti_windup(self):
        config = MissionConfig()
        state = HeadingPidState()
        outputs = []
        for _ in range(200):
            output, state = heading_pid_step(config, state, math.radians(-30.0), 0.05)
            outputs.append(output)
        self.assertTrue(all(abs(item) <= config.max_steering_mps for item in outputs))
        self.assertLessEqual(
            abs(state.integral_rad_sec), config.heading_integral_limit_rad_sec
        )

    def test_heading_pid_rejects_invalid_time_step(self):
        with self.assertRaises(ValueError):
            heading_pid_step(MissionConfig(), HeadingPidState(), 0.0, 0.0)

    def test_heading_pid_replay_matches_latest_six_meter_log(self):
        config = MissionConfig()
        state = HeadingPidState()
        commands = []
        for error_deg, dt_sec in (
            (2.36, 0.05),
            (2.12, 1.0),
            (-0.80, 1.1),
            (0.79, 1.0),
            (2.05, 1.0),
            (3.48, 1.0),
            (5.14, 1.1),
        ):
            command, state = heading_pid_step(
                config, state, math.radians(error_deg), dt_sec
            )
            commands.append(command)
        self.assertGreaterEqual(commands[0], config.min_effective_steering_mps)
        self.assertEqual(commands[2], 0.0)
        self.assertEqual(commands[3], 0.0)
        self.assertGreaterEqual(commands[-1], config.min_effective_steering_mps)
        self.assertTrue(all(abs(item) <= config.max_steering_mps for item in commands))

    def test_path_follower_corrects_cross_track_in_measured_physical_direction(self):
        config = MissionConfig()
        right_command, right_target, _, _ = path_following_step(
            config, PathFollowerState(), 0.40, 0.0, 0.05
        )
        left_command, left_target, _, _ = path_following_step(
            config, PathFollowerState(), -0.40, 0.0, 0.05
        )
        self.assertGreaterEqual(right_command, config.min_effective_steering_mps)
        self.assertLess(right_target, 0.0)
        self.assertLessEqual(left_command, -config.min_effective_steering_mps)
        self.assertGreater(left_target, 0.0)

    def test_path_follower_ignores_small_cross_track_noise(self):
        command, target, control_error, _ = path_following_step(
            MissionConfig(), PathFollowerState(), 0.14, 0.0, 0.05
        )
        self.assertEqual(command, 0.0)
        self.assertEqual(target, 0.0)
        self.assertEqual(control_error, 0.0)

    def test_path_follower_bounds_return_heading(self):
        config = MissionConfig()
        command, target, _, _ = path_following_step(
            config, PathFollowerState(), 5.0, 0.0, 0.05
        )
        self.assertGreater(command, 0.0)
        self.assertAlmostEqual(
            math.degrees(target), -config.max_path_heading_correction_deg
        )

    def test_path_axis_captures_current_yaw_without_alignment(self):
        observation = ready_observation(
            armed=True, mode="OFFBOARD", manual_input=False, yaw_rad=0.37
        )
        reference = build_current_yaw_reference(observation)
        self.assertEqual(reference.axis_yaw_rad, 0.37)
        self.assertEqual(reference.pose_yaw_zero_rad, 0.37)

    def test_position_course_error_uses_track_displacement_not_pose_yaw(self):
        self.assertAlmostEqual(
            position_course_error(2.0, 0.4), math.atan2(0.4, 2.0)
        )
        self.assertAlmostEqual(
            position_course_error(2.0, -0.4), math.atan2(-0.4, 2.0)
        )
        self.assertEqual(position_course_error(0.0, 0.0), 0.0)

    def test_position_course_limit_waits_for_stable_baseline(self):
        large_error = math.radians(121.7)
        self.assertFalse(position_course_limit_exceeded(large_error, 0.15, 35.0))
        self.assertFalse(position_course_limit_exceeded(math.radians(34.9), 0.75, 35.0))
        self.assertTrue(position_course_limit_exceeded(large_error, 0.75, 35.0))

    def test_feedback_gate_rejects_conflicting_position_and_pose_trends(self):
        config = MissionConfig()
        self.assertFalse(
            feedback_directions_consistent(
                -0.30, math.radians(-4.0), math.radians(6.0), config
            )
        )
        self.assertTrue(
            feedback_directions_consistent(
                0.30, math.radians(4.0), math.radians(6.0), config
            )
        )
        self.assertTrue(
            feedback_directions_consistent(
                -0.13, math.radians(-2.0), math.radians(6.0), config
            )
        )

    def test_straight_feedback_falls_back_to_established_position_course(self):
        config = MissionConfig()
        heading, source = select_straight_heading_feedback(
            -0.32,
            math.radians(-9.2),
            math.radians(3.7),
            2.0,
            config,
        )
        self.assertEqual(source, "position_course_fallback")
        self.assertAlmostEqual(heading, math.radians(-9.2))
        command, _, _, _ = path_following_step(
            config, PathFollowerState(), -0.32, heading, 0.05
        )
        self.assertLessEqual(command, -config.min_effective_steering_mps)

    def test_straight_feedback_uses_pose_before_course_baseline(self):
        pose_heading = math.radians(4.0)
        heading, source = select_straight_heading_feedback(
            -0.20,
            math.radians(-120.0),
            pose_heading,
            0.20,
            MissionConfig(),
        )
        self.assertEqual(source, "pose_yaw_short_baseline")
        self.assertEqual(heading, pose_heading)

    def test_position_course_fallback_does_not_exit_on_large_yaw_disagreement(self):
        course = math.radians(0.44)
        heading, source = select_straight_heading_feedback(
            0.036,
            course,
            math.radians(25.54),
            4.743,
            MissionConfig(),
            previous_source="position_course_fallback",
        )
        self.assertEqual(source, "position_course_fallback")
        self.assertEqual(heading, course)

    def test_position_course_fallback_has_exit_hysteresis(self):
        config = MissionConfig()
        course = math.radians(1.0)
        _, retained = select_straight_heading_feedback(
            0.05,
            course,
            math.radians(6.0),
            2.0,
            config,
            previous_source="position_course_fallback",
        )
        heading, exited = select_straight_heading_feedback(
            0.05,
            course,
            math.radians(3.0),
            2.0,
            config,
            previous_source="position_course_fallback",
        )
        self.assertEqual(retained, "position_course_fallback")
        self.assertEqual(exited, "pose_yaw")
        self.assertAlmostEqual(heading, math.radians(3.0))

    def test_terminal_speed_reduces_command_before_target(self):
        config = MissionConfig(distance_m=6.0)
        self.assertEqual(commanded_forward_speed(config, 4.0), config.speed_mps)
        self.assertEqual(
            commanded_forward_speed(config, 4.6), config.terminal_speed_mps
        )

    def test_feedback_conflict_reset_clears_pid_and_keeps_loggable_cross(self):
        state = reset_path_follower_for_conflict(-0.23)
        self.assertEqual(state.heading_pid, HeadingPidState())
        self.assertEqual(state.filtered_cross_track_m, -0.23)

    def test_unknown_entry_phase_fails_closed(self):
        with self.assertRaises(ValueError):
            entry_phase_speed_mps("yaw_alignment", MissionConfig())

    def test_right_turn_progress_accumulates_across_yaw_wrap(self):
        tracker = TurnTracker(math.radians(-170.0))
        for yaw_deg in (170.0, 120.0, 60.0, 10.0):
            tracker = turn_progress_step(tracker, math.radians(yaw_deg), 1.0)
        self.assertAlmostEqual(math.degrees(tracker.accumulated_rad), 180.0)

    def test_wrong_way_yaw_does_not_create_right_turn_progress(self):
        tracker = turn_progress_step(TurnTracker(0.0), math.radians(15.0), 1.0)
        self.assertEqual(tracker.accumulated_rad, 0.0)

    def test_left_turn_progress_accumulates_across_yaw_wrap(self):
        tracker = TurnTracker(math.radians(170.0))
        for yaw_deg in (-170.0, -120.0, -60.0, -10.0):
            tracker = turn_progress_step(tracker, math.radians(yaw_deg), -1.0)
        self.assertAlmostEqual(math.degrees(tracker.accumulated_rad), 180.0)

    def test_semicircle_radius_uses_half_endpoint_chord(self):
        self.assertAlmostEqual(effective_semicircle_radius_m(1.0, 2.0, 1.0, 5.0), 1.5)

    def test_live_mission_builds_one_continuous_generic_u_trajectory(self):
        config = MissionConfig(
            distance_m=5.0,
            u_turn=True,
            turn_direction_sign=-1.0,
        )
        self.assertEqual(config.turn_radius_m, 3.0)
        trajectory = build_mission_trajectory(config, ready_observation())
        phases = {point.phase for point in trajectory.points}
        self.assertEqual(phases, {"LEG1", "UTURN", "LEG2"})
        self.assertAlmostEqual(trajectory.points[-1].x_m, 0.0, places=5)
        self.assertAlmostEqual(
            trajectory.points[-1].y_m, 2.0 * config.turn_radius_m, places=5
        )

    def test_live_tracker_has_no_cross_track_deadband_and_keeps_body_ned_sign(self):
        tracker = tracker_config_from_mission(MissionConfig())
        self.assertEqual(tracker.body_y_for_ccw_sign, -1.0)
        self.assertEqual(tracker.minimum_tracking_speed_mps, 0.035)
        self.assertEqual(tracker.max_body_bearing_rate_degps, 45.0)

    def test_u_turn_gets_more_authority_without_relaxing_straight_default(self):
        parser = build_parser()
        straight = config_from_args(parser.parse_args([]))
        u_turn = config_from_args(parser.parse_args(["--u-turn"]))
        explicit = config_from_args(
            parser.parse_args(["--u-turn", "--tracker-max-body-bearing-deg", "60"])
        )
        self.assertEqual(straight.tracker_max_body_bearing_deg, 32.0)
        self.assertEqual(u_turn.tracker_max_body_bearing_deg, 89.0)
        self.assertEqual(explicit.tracker_max_body_bearing_deg, 60.0)
        self.assertEqual(straight.tracker_curvature_to_body_gain_m, 0.0)
        self.assertEqual(u_turn.tracker_curvature_to_body_gain_m, 3.5)
        self.assertEqual(straight.tracker_max_curvature_correction_inv_m, 0.0)
        self.assertEqual(u_turn.tracker_max_curvature_correction_inv_m, 0.12)

    def test_generic_tracker_feedback_and_integral_cli_are_explicit(self):
        parser = build_parser()
        config = config_from_args(
            parser.parse_args(
                [
                    "--u-turn",
                    "--tracker-curvature-to-body-gain-m",
                    "0.94",
                    "--tracker-curvature-feedback-gain-ratio",
                    "1.28",
                    "--tracker-cross-track-integral-gain-inv-m-per-m-sec",
                    "0.04",
                    "--tracker-cross-track-integral-limit-m-sec",
                    "1.0",
                ]
            )
        )
        tracker = tracker_config_from_mission(config)
        self.assertAlmostEqual(tracker.curvature_to_body_gain_m, 0.94)
        self.assertAlmostEqual(tracker.curvature_feedback_gain_ratio, 1.28)
        self.assertAlmostEqual(
            tracker.cross_track_integral_gain_inv_m_per_m_sec, 0.04
        )
        self.assertAlmostEqual(tracker.cross_track_integral_limit_m_sec, 1.0)

    def test_body_forward_sign_cli_defaults_positive_and_accepts_orin1_adapter(self):
        parser = build_parser()
        default_config = config_from_args(parser.parse_args([]))
        orin1_config = config_from_args(
            parser.parse_args(["--body-forward-sign", "-1"])
        )
        self.assertEqual(default_config.body_forward_sign, 1.0)
        self.assertEqual(orin1_config.body_forward_sign, -1.0)


class OutdoorForwardGateTests(unittest.TestCase):
    def test_external_start_gate_is_disabled_by_default(self):
        args = build_parser().parse_args([])
        self.assertIsNone(args.external_start_gate_fd)
        self.assertIsNone(args.external_start_plan_id)
        validate_external_start_gate_args(
            args.external_start_gate_fd,
            args.external_start_plan_id,
            args.external_start_gate_timeout_sec,
        )

    def test_external_start_gate_requires_paired_bounded_arguments(self):
        validate_external_start_gate_args(7, 42, 300.0)
        for values in ((7, None, 300.0), (None, 42, 300.0)):
            with self.assertRaises(ValueError):
                validate_external_start_gate_args(*values)
        for timeout in (0.9, 600.1, math.nan):
            with self.assertRaises(ValueError):
                validate_external_start_gate_args(7, 42, timeout)

    def test_external_start_gate_accepts_only_exact_pairb_plan_token(self):
        token = external_start_gate_token(42)
        buffered, accepted, error = consume_external_start_gate_bytes(
            b"", token[:8], 42
        )
        self.assertFalse(accepted)
        self.assertIsNone(error)
        buffered, accepted, error = consume_external_start_gate_bytes(
            buffered, token[8:], 42
        )
        self.assertEqual(buffered, b"")
        self.assertTrue(accepted)
        self.assertIsNone(error)

        _, accepted, error = consume_external_start_gate_bytes(
            b"", b"PAIRB_START plan_id=41\n", 42
        )
        self.assertFalse(accepted)
        self.assertEqual(error, "external_start_gate_token_mismatch")

    def test_runtime_stop_requires_plan_and_exact_completion_token(self):
        validate_external_runtime_stop_args(9, 42)
        for values in ((9, None), (2, 42)):
            with self.assertRaises(ValueError):
                validate_external_runtime_stop_args(*values)

        token = external_runtime_stop_token(42)
        buffered, accepted, error = consume_external_runtime_stop_bytes(
            b"", token[:10], 42
        )
        self.assertFalse(accepted)
        self.assertIsNone(error)
        buffered, accepted, error = consume_external_runtime_stop_bytes(
            buffered, token[10:], 42
        )
        self.assertEqual(buffered, b"")
        self.assertTrue(accepted)
        self.assertIsNone(error)

        _, accepted, error = consume_external_runtime_stop_bytes(
            b"", b"PAIRB_RUNTIME_COMPLETE plan_id=41\n", 42
        )
        self.assertFalse(accepted)
        self.assertEqual(error, "external_runtime_stop_token_mismatch")

    def test_safe_manual_prestate_requires_real_navigation(self):
        self.assertTrue(navigation_ready(ready_observation()))
        self.assertTrue(safe_manual_prestate(ready_observation()))
        self.assertFalse(safe_manual_prestate(ready_observation(gps_status=-1)))
        self.assertFalse(safe_manual_prestate(ready_observation(gps_fix_type=2)))
        self.assertFalse(safe_manual_prestate(ready_observation(satellites_visible=5)))
        self.assertFalse(safe_manual_prestate(ready_observation(pose_age_sec=2.0)))
        self.assertFalse(safe_manual_prestate(ready_observation(mode="AUTO.LOITER")))

    def test_manual_arm_gate_is_manual_and_armed(self):
        self.assertTrue(manual_arm_ready(ready_observation(armed=True)))
        self.assertFalse(manual_arm_ready(ready_observation(armed=False)))
        self.assertFalse(manual_arm_ready(ready_observation(armed=True, mode="OFFBOARD")))

    def test_motion_faults_fail_closed(self):
        active = ready_observation(armed=True, mode="OFFBOARD", manual_input=False)
        self.assertIsNone(motion_fault(active))
        self.assertEqual(motion_fault(ready_observation(armed=False, mode="OFFBOARD")), "unexpected_disarm")
        self.assertEqual(motion_fault(ready_observation(armed=True, mode="MANUAL")), "offboard_exit")
        self.assertEqual(motion_fault(ready_observation(armed=True, mode="OFFBOARD", gps_status=-1)), "gps_no_fix")
        self.assertEqual(motion_fault(ready_observation(armed=True, mode="OFFBOARD", gps_fix_type=2)), "gps_not_3d")
        self.assertEqual(motion_fault(ready_observation(armed=True, mode="OFFBOARD", satellites_visible=5)), "gps_satellites_low")
        self.assertEqual(motion_fault(ready_observation(armed=True, mode="OFFBOARD", pose_age_sec=2.0)), "local_pose_stale")

    def test_configuration_is_bounded(self):
        validate_config(MissionConfig())
        with self.assertRaises(ValueError):
            validate_config(MissionConfig(distance_m=50.0))
        with self.assertRaises(ValueError):
            validate_config(MissionConfig(speed_mps=0.3, max_speed_mps=0.3))
        with self.assertRaises(ValueError):
            validate_config(MissionConfig(stall_window_sec=1.0))
        with self.assertRaises(ValueError):
            validate_config(MissionConfig(course_calibration_distance_m=0.5))
        with self.assertRaises(ValueError):
            validate_config(MissionConfig(course_calibration_speed_mps=0.1))
        with self.assertRaises(ValueError):
            validate_config(MissionConfig(steering_trim_mps=-0.05))
        with self.assertRaises(ValueError):
            validate_config(MissionConfig(max_steering_mps=0.005))
        with self.assertRaises(ValueError):
            validate_config(
                MissionConfig(steering_trim_mps=-0.02, max_steering_mps=0.01)
            )
        with self.assertRaises(ValueError):
            validate_config(
                MissionConfig(min_effective_steering_mps=0.07, max_steering_mps=0.06)
            )
        with self.assertRaises(ValueError):
            validate_config(MissionConfig(cross_track_lookahead_m=0.1))
        with self.assertRaises(ValueError):
            validate_config(MissionConfig(steering_direction_sign=0.0))
        with self.assertRaises(ValueError):
            validate_config(MissionConfig(body_forward_sign=0.0))
        validate_config(MissionConfig(u_turn=True))
        with self.assertRaises(ValueError):
            validate_config(MissionConfig(u_turn=True, turn_angle_deg=181.0))
        with self.assertRaises(ValueError):
            validate_config(MissionConfig(u_turn=True, turn_direction_sign=0.0))
        with self.assertRaises(ValueError):
            validate_config(
                MissionConfig(
                    u_turn=True,
                    turn_forward_speed_mps=0.15,
                    turn_lateral_speed_mps=0.08,
                )
            )
        with self.assertRaises(ValueError):
            validate_config(MissionConfig(u_turn=True, turn_max_sec=61.0))


if __name__ == "__main__":
    unittest.main()
