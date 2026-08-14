from __future__ import annotations

import math
import json
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from src.lr24_compact_protocol import (
    CorridorPlanCompact,
    HealthFlag,
    MessageType,
    Phase,
    PlanCommand,
    PlanFlag,
    MiniState,
)
from src.pairb_cooperative_docking import (
    CooperationObservation,
    CooperationPhase,
    CooperativePlannerConfig,
    CoordinationMode,
    TemporalCoordinator,
    TemporalCoordinatorConfig,
    build_cooperative_docking_plan,
    build_plan_commands,
    compact_corridor_plan,
    pairb_mission_type_allowed,
    sample_heading_constrained_curve,
    terminal_speed_overlap,
    wrap_pi,
)
from scripts.run_pairb_virtual_mini_hil_rviz import (
    align_relative_trace,
    load_replay_bundle,
    mavros_topic,
    pairb_mini_state_safe,
    transform_point,
)
from scripts.run_pairb_hitl_state_relay import (
    HitlStatePacket,
    decode_packet,
    encode_packet,
)
from scripts.render_pairb_cooperative_xy_gif import _sample_indices
from src.rover_rviz_trajectory import TracePoint


def build_plan(**overrides):
    values = {
        "plan_id": 17,
        "origin_id": 9,
        "generated_at_ms": 1000,
        "carrier_position": (0.0, 0.0),
        "carrier_yaw_rad": 0.0,
        "mini_phase_rad": 0.0,
        "qualified_orbit_laps": 1.0,
        "config": CooperativePlannerConfig(),
    }
    values.update(overrides)
    return build_cooperative_docking_plan(**values)


class CooperativePlannerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.config = CooperativePlannerConfig(phase_candidates=36)
        cls.plan = build_plan(config=cls.config)

    def test_rejects_plan_before_complete_orbit(self) -> None:
        with self.assertRaisesRegex(ValueError, "full_orbit"):
            build_plan(qualified_orbit_laps=0.999, config=self.config)

    def test_plan_is_forward_short_and_executable(self) -> None:
        metrics = self.plan.carrier_curve_metrics
        self.assertTrue(metrics.forward_only)
        self.assertFalse(metrics.self_intersects)
        self.assertTrue(metrics.clears_orbit)
        self.assertLess(metrics.start_abs_curvature_inv_m, 1.0e-8)
        self.assertLess(metrics.end_abs_curvature_inv_m, 1.0e-8)
        self.assertLessEqual(
            metrics.max_abs_curvature_inv_m,
            1.0 / self.config.minimum_turn_radius_m + 1.0e-6,
        )
        self.assertLessEqual(self.plan.extra_orbits, 1)
        self.assertGreater(self.plan.candidate_count, 0)

    def test_carrier_curve_honors_start_and_terminal_headings(self) -> None:
        start = self.plan.carrier_path.sample(0.001)
        tangent = self.plan.carrier_path.sample(self.plan.carrier_approach_length_m - 0.001)
        expected_terminal = math.atan2(
            self.plan.tangent_direction[1], self.plan.tangent_direction[0]
        )
        self.assertLess(abs(wrap_pi(start.tangent_yaw_rad)), 0.04)
        self.assertLess(abs(wrap_pi(tangent.tangent_yaw_rad - expected_terminal)), 0.04)

    def test_terminal_paths_share_line_and_finish_carrier_ahead(self) -> None:
        carrier_terminal = self.plan.carrier_path.sample(
            self.plan.carrier_path.length_m
        )
        mini_terminal = self.plan.mini_terminal_path.sample(
            self.plan.mini_terminal_path.length_m
        )
        self.assertAlmostEqual(
            wrap_pi(carrier_terminal.tangent_yaw_rad - mini_terminal.tangent_yaw_rad),
            0.0,
            places=6,
        )
        terminal_delta = (
            carrier_terminal.x_m - mini_terminal.x_m,
            carrier_terminal.y_m - mini_terminal.y_m,
        )
        along_gap = (
            terminal_delta[0] * self.plan.tangent_direction[0]
            + terminal_delta[1] * self.plan.tangent_direction[1]
        )
        lateral_gap = (
            -terminal_delta[0] * self.plan.tangent_direction[1]
            + terminal_delta[1] * self.plan.tangent_direction[0]
        )
        self.assertAlmostEqual(along_gap, self.plan.target_front_gap_m, places=6)
        self.assertAlmostEqual(lateral_gap, 0.0, places=6)

    def test_no_unnecessary_extra_orbit_for_default_geometry(self) -> None:
        self.assertEqual(self.plan.extra_orbits, 0)

    def test_terminal_speed_overlap_selects_common_rover_speed(self) -> None:
        window = terminal_speed_overlap(self.config)
        self.assertAlmostEqual(window.minimum_mps, 0.12)
        self.assertAlmostEqual(window.maximum_mps, 0.16)
        self.assertAlmostEqual(window.rendezvous_mps, 0.14)
        self.assertAlmostEqual(self.plan.rendezvous_speed_mps, 0.14)
        carrier_terminal = next(
            point
            for point in self.plan.carrier_path.points
            if point.phase == "SHARED_TERMINAL"
        )
        self.assertAlmostEqual(carrier_terminal.requested_speed_mps, 0.14)
        self.assertTrue(
            all(
                abs(point.requested_speed_mps - 0.14) < 1.0e-9
                for point in self.plan.mini_terminal_path.points
            )
        )

    def test_rejects_nonoverlapping_terminal_speed_envelopes(self) -> None:
        config = CooperativePlannerConfig(
            mini_terminal_min_speed_mps=0.17,
            mini_terminal_max_speed_mps=0.20,
            carrier_terminal_min_speed_mps=0.12,
            carrier_terminal_max_speed_mps=0.16,
        )
        with self.assertRaisesRegex(ValueError, "do_not_overlap"):
            build_plan(config=config)

    def test_compact_corridor_round_trip_contains_contract(self) -> None:
        compact = compact_corridor_plan(self.plan, sequence=8, config=self.config)
        decoded = CorridorPlanCompact.decode(compact.encode())
        self.assertEqual(decoded.plan_id, self.plan.plan_id)
        self.assertEqual(decoded.origin_id, self.plan.origin_id)
        self.assertTrue(decoded.flags & int(PlanFlag.ONE_ORBIT_COMPLETE))
        self.assertTrue(decoded.flags & int(PlanFlag.CORRIDOR_VALID))
        self.assertAlmostEqual(decoded.trigger_phase_rad, self.plan.tangent_phase_rad, places=3)

    def test_curve_primitive_rejects_nonfinite_input(self) -> None:
        with self.assertRaises(ValueError):
            sample_heading_constrained_curve(
                (0.0, 0.0), math.nan, (1.0, 1.0), 0.0, 1.0, 1.0, 0.1
            )


class PairBContractTests(unittest.TestCase):
    def test_runtime_whitelist_contains_only_high_level_mission_messages(self) -> None:
        expected = {
            MessageType.MINI_STATE,
            MessageType.CORRIDOR_PLAN,
            MessageType.PLAN_COMMAND,
            MessageType.MISSION_STATUS,
            MessageType.ABORT,
        }
        actual = {message for message in MessageType if pairb_mission_type_allowed(message)}
        self.assertEqual(actual, expected)
        self.assertFalse(pairb_mission_type_allowed(MessageType.STAGED_MISSION_PLAN))
        self.assertFalse(pairb_mission_type_allowed(MessageType.FIELD_ORIGIN))


class ReadOnlyHilReplayTests(unittest.TestCase):
    def test_replay_plan_contains_continuous_orbit_exit_and_tangent(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "timeline.csv").write_text(
                "time_s,mission_phase,coordination_mode,carrier_x_m,carrier_y_m,mini_x_m,mini_y_m,carrier_speed_mps,mini_speed_mps,carrier_speed_limit_mps,front_gap_m,reason\n"
                "0.0,ORBIT_QUALIFICATION,HOLD,0,0,7,3,0,0.18,0,nan,pending\n"
                "0.1,ORBIT_QUALIFICATION,HOLD,0,0,6.99,3.1,0,0.18,0,nan,pending\n"
                "70.0,APPROACH,RUN,0,0,7,3,0.1,0.18,0.1,nan,ready\n",
                encoding="ascii",
            )
            terminal_points = [
                [5.0 - 0.1 * index, 5.0, 0.1, "T"] for index in range(21)
            ]
            plan_payload = {
                "carrier_path": [[0, 0, 0.1, "A"], [5, 5, 0.1, "T"]],
                "mini_terminal_path": terminal_points,
                "orbit_center": [5, 3],
                "orbit_radius_m": 2,
                "turn_direction": "ccw",
                "mini_phase_at_plan_rad": 0,
                "mini_exit_delta_rad": math.pi / 2.0,
                "carrier_speed_range_mps": [0.0, 0.16],
                "mini_speed_range_mps": [0.12, 0.20],
                "terminal_speed_range_mps": [0.12, 0.16],
                "rendezvous_speed_mps": 0.14,
                "terminal_capture_required_s": 2.0,
            }
            (root / "plan.json").write_text(
                json.dumps(plan_payload) + "\n",
                encoding="ascii",
            )
            bundle = load_replay_bundle(root)
        jumps = [
            math.hypot(b.x_m - a.x_m, b.y_m - a.y_m)
            for a, b in zip(bundle.mini_plan, bundle.mini_plan[1:])
        ]
        self.assertLess(max(jumps), 0.20)
        self.assertAlmostEqual(bundle.mini_plan[-1].x_m, 3.0)
        self.assertAlmostEqual(bundle.mini_plan[-1].y_m, 5.0)
        self.assertEqual(bundle.carrier_speed_range_mps, (0.0, 0.16))
        self.assertEqual(bundle.mini_speed_range_mps, (0.12, 0.20))
        self.assertAlmostEqual(bundle.rendezvous_speed_mps, 0.14)

    def test_hil_transform_anchors_shadow_to_real_carrier_pose(self) -> None:
        transformed = transform_point(
            TracePoint(2.0, 0.0, 0.0),
            TracePoint(10.0, 20.0, math.pi / 2.0),
        )
        self.assertAlmostEqual(transformed.x_m, 10.0)
        self.assertAlmostEqual(transformed.y_m, 22.0)
        self.assertAlmostEqual(transformed.yaw_rad, math.pi / 2.0)

    def test_dual_hil_aligns_independent_mini_local_frame(self) -> None:
        aligned = align_relative_trace(
            TracePoint(12.0, 21.0, math.pi / 2.0),
            TracePoint(10.0, 20.0, 0.0),
            TracePoint(5.0, 8.0, math.pi / 2.0),
        )
        self.assertAlmostEqual(aligned.x_m, 4.0)
        self.assertAlmostEqual(aligned.y_m, 10.0)
        self.assertAlmostEqual(abs(aligned.yaw_rad), math.pi)

    def test_dual_hil_uses_explicit_namespaced_mavros_topics(self) -> None:
        self.assertEqual(mavros_topic("/carrier/mavros", "/state"), "/carrier/mavros/state")
        self.assertEqual(
            mavros_topic("mini/mavros", "/local_position/pose"),
            "/mini/mavros/local_position/pose",
        )
        with self.assertRaises(ValueError):
            mavros_topic("/", "/state")

    def test_hil_source_has_no_vehicle_command_interface(self) -> None:
        source = (
            REPO_ROOT / "scripts/run_pairb_virtual_mini_hil_rviz.py"
        ).read_text(encoding="ascii")
        self.assertNotIn("create_client(", source)
        self.assertNotIn("/mavros/setpoint", source)
        self.assertNotIn("/mavros/cmd/arming", source)
        self.assertNotIn("/mavros/set_mode", source)
        self.assertIn("--require-real-mini", source)
        self.assertIn('f"{role}_vehicle_armed"', source)

    def test_pairb_hil_requires_connected_disarmed_pose(self) -> None:
        health = int(
            HealthFlag.POSITION_VALID
            | HealthFlag.YAW_VALID
            | HealthFlag.PX4_CONNECTED
            | HealthFlag.DISARMED
        )
        state = MiniState(1, 7, 10, 1.0, 2.0, 0.0, 0.0, 0.2, 0.0, health, 0)
        self.assertEqual(pairb_mini_state_safe(state), (True, "ready"))
        unsafe = MiniState(1, 8, 20, 1.0, 2.0, 0.0, 0.0, 0.2, 0.0, health & ~int(HealthFlag.DISARMED), 0)
        self.assertFalse(pairb_mini_state_safe(unsafe)[0])

    def test_hitl_relay_packet_is_strict_and_round_trips(self) -> None:
        packet = HitlStatePacket(
            schema=1,
            role="mini",
            seq=7,
            connected=True,
            armed=False,
            mode="MANUAL",
            manual_input=True,
            x_m=1.0,
            y_m=2.0,
            z_m=0.0,
            qx=0.0,
            qy=0.0,
            qz=0.0,
            qw=1.0,
        )
        self.assertEqual(decode_packet(encode_packet(packet)), packet)
        self.assertAlmostEqual(packet.yaw_rad, 0.0)
        with self.assertRaises(ValueError):
            decode_packet(b'{"schema":1}')

    def test_hitl_relay_is_send_only_and_has_no_vehicle_command_interface(self) -> None:
        source = (
            REPO_ROOT / "scripts/run_pairb_hitl_state_relay.py"
        ).read_text(encoding="ascii")
        self.assertIn("sendto(", source)
        self.assertNotIn("recvfrom(", source)
        self.assertNotIn("create_client(", source)
        self.assertNotIn("create_publisher(", source)
        self.assertNotIn("/setpoint", source)
        self.assertNotIn("/cmd/arming", source)

    def test_gif_sampling_preserves_four_times_log_speed(self) -> None:
        rows = [SimpleNamespace(time_s=index * 0.05) for index in range(3179)]
        indices, duration_ms = _sample_indices(rows, 4.0, 12.0, 600)
        playback_s = (len(indices) - 1) * duration_ms / 1000.0
        self.assertEqual(indices[0], 0)
        self.assertEqual(indices[-1], len(rows) - 1)
        self.assertAlmostEqual(rows[-1].time_s / playback_s, 4.0, delta=0.01)


class TemporalCoordinatorTests(unittest.TestCase):
    def setUp(self) -> None:
        self.config = TemporalCoordinatorConfig(speed_slew_mps2=1.0)

    def observation(self, **overrides) -> CooperationObservation:
        values = {
            "now_s": 0.0,
            "phase": CooperationPhase.APPROACH,
            "carrier_remaining_m": 2.0,
            "mini_remaining_m": 3.0,
            "carrier_speed_mps": 0.12,
            "mini_speed_mps": 0.18,
            "carrier_state_age_s": 0.05,
            "mini_state_age_s": 0.05,
        }
        values.update(overrides)
        return CooperationObservation(**values)

    def test_mini_lag_reduces_carrier_speed_envelope(self) -> None:
        nominal = TemporalCoordinator(self.config).step(self.observation())
        lagged = TemporalCoordinator(self.config).step(
            self.observation(mini_speed_mps=0.13)
        )
        self.assertLess(lagged.carrier_speed_limit_mps, nominal.carrier_speed_limit_mps)
        self.assertEqual(lagged.mode, CoordinationMode.SLOW_CARRIER)
        self.assertIn("mini_late", lagged.reason)

    def test_terminal_large_gap_slows_carrier(self) -> None:
        decision = TemporalCoordinator(self.config).step(
            self.observation(
                phase=CooperationPhase.TERMINAL,
                carrier_remaining_m=4.0,
                mini_remaining_m=4.7,
                front_gap_m=1.2,
            )
        )
        self.assertEqual(decision.mode, CoordinationMode.SLOW_CARRIER)
        self.assertLess(decision.carrier_speed_limit_mps, self.config.carrier_max_speed_mps)

    def test_terminal_capture_requires_sustained_relative_state_match(self) -> None:
        coordinator = TemporalCoordinator(self.config)
        values = {
            "phase": CooperationPhase.TERMINAL,
            "carrier_remaining_m": 1.0,
            "mini_remaining_m": 1.0,
            "carrier_speed_mps": 0.14,
            "mini_speed_mps": 0.14,
            "front_gap_m": 0.60,
            "lateral_gap_m": 0.05,
            "heading_error_rad": 0.03,
            "yaw_rate_error_radps": 0.02,
        }
        first = coordinator.step(self.observation(now_s=0.0, **values))
        middle = coordinator.step(self.observation(now_s=1.0, **values))
        qualified = coordinator.step(self.observation(now_s=2.0, **values))
        self.assertFalse(first.terminal_capture_qualified)
        self.assertFalse(middle.terminal_capture_qualified)
        self.assertTrue(qualified.terminal_capture_qualified)
        self.assertAlmostEqual(qualified.relative_speed_mps, 0.0)
        self.assertEqual(qualified.reason, "terminal_capture_qualified")

    def test_terminal_capture_resets_on_relative_speed_error(self) -> None:
        coordinator = TemporalCoordinator(self.config)
        common = {
            "phase": CooperationPhase.TERMINAL,
            "carrier_remaining_m": 1.0,
            "mini_remaining_m": 1.0,
            "front_gap_m": 0.60,
        }
        coordinator.step(
            self.observation(
                now_s=0.0,
                carrier_speed_mps=0.14,
                mini_speed_mps=0.14,
                **common,
            )
        )
        reset = coordinator.step(
            self.observation(
                now_s=1.0,
                carrier_speed_mps=0.16,
                mini_speed_mps=0.12,
                **common,
            )
        )
        self.assertFalse(reset.terminal_capture_qualified)
        self.assertEqual(reset.terminal_capture_duration_s, 0.0)

    def test_coordinator_rejects_no_terminal_speed_overlap(self) -> None:
        with self.assertRaisesRegex(ValueError, "do_not_overlap"):
            TemporalCoordinator(
                TemporalCoordinatorConfig(
                    carrier_terminal_min_speed_mps=0.10,
                    carrier_terminal_max_speed_mps=0.11,
                    mini_terminal_min_speed_mps=0.12,
                )
            )

    def test_carrier_behind_aborts_immediately(self) -> None:
        decision = TemporalCoordinator(self.config).step(
            self.observation(
                phase=CooperationPhase.TERMINAL,
                front_gap_m=0.0,
            )
        )
        self.assertEqual(decision.mode, CoordinationMode.ABORT)
        self.assertEqual(decision.reason, "carrier_ahead_violation")
        self.assertEqual(decision.carrier_speed_limit_mps, 0.0)
        self.assertEqual(decision.mini_speed_limit_mps, 0.0)

    def test_stale_state_aborts_immediately(self) -> None:
        decision = TemporalCoordinator(self.config).step(
            self.observation(mini_state_age_s=0.51)
        )
        self.assertEqual(decision.mode, CoordinationMode.ABORT)
        self.assertEqual(decision.reason, "state_stale")

    def test_persistent_unrecoverable_error_holds_then_aborts(self) -> None:
        coordinator = TemporalCoordinator(self.config)
        kwargs = {
            "carrier_remaining_m": 8.0,
            "mini_remaining_m": 0.5,
            "mini_speed_mps": 0.18,
        }
        first = coordinator.step(self.observation(now_s=0.0, **kwargs))
        held = coordinator.step(self.observation(now_s=2.0, **kwargs))
        aborted = coordinator.step(self.observation(now_s=5.0, **kwargs))
        self.assertNotEqual(first.mode, CoordinationMode.ABORT)
        self.assertEqual(held.mode, CoordinationMode.HOLD)
        self.assertEqual(aborted.mode, CoordinationMode.ABORT)
        self.assertEqual(aborted.reason, "persistent_sync_failure")

    def test_persistent_mini_stall_holds_then_aborts(self) -> None:
        coordinator = TemporalCoordinator(self.config)
        kwargs = {"mini_speed_mps": 0.01}
        first = coordinator.step(self.observation(now_s=0.0, **kwargs))
        held = coordinator.step(self.observation(now_s=2.0, **kwargs))
        aborted = coordinator.step(self.observation(now_s=5.0, **kwargs))
        self.assertEqual(first.reason, "mini_below_tracking_speed")
        self.assertEqual(held.mode, CoordinationMode.HOLD)
        self.assertEqual(aborted.mode, CoordinationMode.ABORT)

    def test_commands_are_low_rate_envelopes_and_round_trip(self) -> None:
        planner_config = CooperativePlannerConfig(phase_candidates=36)
        plan = build_plan(config=planner_config)
        decision = TemporalCoordinator(self.config).step(self.observation())
        carrier, mini = build_plan_commands(
            plan,
            decision,
            phase=CooperationPhase.APPROACH,
            sequence=12,
            timestamp_ms=5000,
            config=planner_config,
        )
        carrier = PlanCommand.decode(carrier.encode())
        mini = PlanCommand.decode(mini.encode())
        self.assertEqual(carrier.phase, Phase.ARC_TO_CORRIDOR)
        self.assertEqual(mini.phase, Phase.ARC_TO_CORRIDOR)
        self.assertAlmostEqual(carrier.max_speed_mps, decision.carrier_speed_limit_mps, places=2)
        self.assertAlmostEqual(mini.max_speed_mps, decision.mini_speed_limit_mps, places=2)
        self.assertEqual(carrier.omega_radps, 0.0)
        self.assertEqual(mini.omega_radps, 0.0)


if __name__ == "__main__":
    unittest.main()
