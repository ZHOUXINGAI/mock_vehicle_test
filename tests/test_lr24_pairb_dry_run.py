#!/usr/bin/env python3

from __future__ import annotations

import contextlib
import io
import sys
import unittest
from pathlib import Path
from unittest.mock import patch


REPO_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_DIR / "src"))
sys.path.insert(0, str(REPO_DIR / "scripts"))

from lr24_compact_protocol import (
    FieldOrigin,
    MessageType,
    Phase,
    PlanCommand,
    Role,
    StagedMissionFlag,
    StagedMissionPlan,
    encode_frame,
)
from lr24_live_follower import CarrierLocalFollower, ExecutorCounters
from lr24_pairb_dry_run import (
    build_parser,
    dry_run_exit_code,
    format_executor_summary,
    make_carrier_local_hold,
    mini_role,
)


class FakeTransport:
    description = "in-memory-no-motion"

    def __init__(self, incoming: bytes) -> None:
        self.incoming = incoming
        self.sent: list[bytes] = []
        self.closed = False

    def receive(self, timeout_sec: float) -> list[bytes]:
        del timeout_sec
        if not self.incoming:
            return []
        incoming, self.incoming = self.incoming, b""
        return [incoming]

    def send(self, frame: bytes) -> None:
        self.sent.append(frame)

    def close(self) -> None:
        self.closed = True


class DryRunNoMotionBoundaryTest(unittest.TestCase):
    def test_shared_ros_environment_is_host_local_by_default(self) -> None:
        env_script = (REPO_DIR / "scripts" / "env.sh").read_text(encoding="utf-8")
        self.assertIn(
            'export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"',
            env_script,
        )

    def test_mavros_state_source_is_explicit_and_cannot_simulate_orbit(self) -> None:
        args = build_parser().parse_args(
            [
                "mini",
                "--confirm-no-motion",
                "--state-source",
                "mavros-local",
                "--simulate-orbit",
            ]
        )
        with self.assertRaisesRegex(SystemExit, "cannot be used"):
            mini_role(args)

    def test_carrier_local_hold_exercises_zero_executor(self) -> None:
        args = build_parser().parse_args(["carrier"])
        command = make_carrier_local_hold(args)
        self.assertEqual(command.role, Role.CARRIER)
        self.assertEqual(command.phase, Phase.HOLD)
        self.assertEqual((command.v_mps, command.omega_radps), (0.0, 0.0))

        follower = CarrierLocalFollower()
        outcome = follower.apply_command(command, 10000)
        self.assertEqual(
            (outcome.executor_output.v_mps, outcome.executor_output.omega_radps),
            (0.0, 0.0),
        )
        summary = format_executor_summary(
            "carrier_local", follower.executor.counters
        )
        self.assertIn("executor_mode=no_motion", summary)
        self.assertIn("zero_output_count=1", summary)
        self.assertIn("nonzero_output_count=0", summary)

    def test_nonzero_output_counter_controls_process_failure(self) -> None:
        clean = ExecutorCounters(executor_decisions=2, zero_output_count=2)
        violated = ExecutorCounters(
            executor_decisions=1,
            nonzero_output_count=1,
        )
        self.assertEqual(dry_run_exit_code(clean), 0)
        self.assertEqual(dry_run_exit_code(clean, violated), 1)

    def test_mini_role_records_follower_summary_without_hardware(self) -> None:
        origin = FieldOrigin(1, 1, 900, 31.0, 121.0, 5.0)
        hold = PlanCommand(
            plan_id=1,
            role=Role.MINI,
            phase=Phase.HOLD,
            seq=1,
            timestamp_ms=1000,
            valid_until_ms=1500,
            v_mps=0.0,
            omega_radps=0.0,
            duration_ms=500,
            distance_m=0.0,
            max_speed_mps=0.0,
            max_accel_mps2=0.0,
        )
        incoming = (
            encode_frame(MessageType.FIELD_ORIGIN, origin.encode())
            + encode_frame(MessageType.PLAN_COMMAND, hold.encode())
        )
        transport = FakeTransport(incoming)
        args = build_parser().parse_args(
            ["mini", "--confirm-no-motion", "--duration-sec", "0.2"]
        )
        ticks = iter(index * 0.01 for index in range(1000))
        output = io.StringIO()
        with patch(
            "lr24_pairb_dry_run.open_transport", return_value=transport
        ), patch(
            "lr24_pairb_dry_run.time.monotonic", side_effect=lambda: next(ticks)
        ), contextlib.redirect_stdout(output):
            result = mini_role(args)

        self.assertEqual(result, 0)
        self.assertTrue(transport.closed)
        self.assertTrue(transport.sent)
        summary = output.getvalue()
        self.assertIn("executor_mode=no_motion", summary)
        self.assertIn("nonzero_output_count=0", summary)
        self.assertIn("executor=zero_hold", summary)

    def test_staged_trajectory_is_received_but_motion_is_blocked(self) -> None:
        plan = StagedMissionPlan(
            schema_version=1,
            plan_id=7,
            seq=1,
            timestamp_ms=1000,
            valid_until_ms=121000,
            lead_delay_ms=5000,
            lead_distance_m=2.0,
            lateral_offset_m=6.0,
            straight_distance_m=5.0,
            turn_radius_m=3.0,
            mini_speed_mps=0.12,
            carrier_speed_mps=0.06,
            flags=int(StagedMissionFlag.S_BEND_RETURN),
        )
        command = PlanCommand(
            plan_id=7,
            role=Role.MINI,
            phase=Phase.TRAJECTORY,
            seq=1,
            timestamp_ms=1100,
            valid_until_ms=1600,
            v_mps=0.12,
            omega_radps=0.0,
            duration_ms=500,
            distance_m=5.0,
            max_speed_mps=0.12,
            max_accel_mps2=0.3,
        )
        incoming = (
            encode_frame(MessageType.STAGED_MISSION_PLAN, plan.encode())
            + encode_frame(MessageType.PLAN_COMMAND, command.encode())
        )
        transport = FakeTransport(incoming)
        args = build_parser().parse_args(
            ["mini", "--confirm-no-motion", "--duration-sec", "0.2"]
        )
        ticks = iter(index * 0.01 for index in range(1000))
        output = io.StringIO()
        with patch(
            "lr24_pairb_dry_run.open_transport", return_value=transport
        ), patch(
            "lr24_pairb_dry_run.time.monotonic", side_effect=lambda: next(ticks)
        ), contextlib.redirect_stdout(output):
            result = mini_role(args)

        self.assertEqual(result, 0)
        summary = output.getvalue()
        self.assertIn("staged_plans_rx=1", summary)
        self.assertIn("staged_plan_seq_gaps=0", summary)
        blocked_field = next(
            field
            for field in summary.split()
            if field.startswith("blocked_motion_count=")
        )
        self.assertGreaterEqual(int(blocked_field.split("=", 1)[1]), 1)
        self.assertIn("nonzero_output_count=0", summary)

    def test_180_second_live_plan_is_accepted_only_with_matching_local_bound(self) -> None:
        plan = StagedMissionPlan(
            schema_version=1,
            plan_id=8,
            seq=1,
            timestamp_ms=1000,
            valid_until_ms=181000,
            lead_delay_ms=5000,
            lead_distance_m=2.0,
            lateral_offset_m=6.0,
            straight_distance_m=5.0,
            turn_radius_m=3.0,
            mini_speed_mps=0.12,
            carrier_speed_mps=0.06,
            flags=int(StagedMissionFlag.S_BEND_RETURN),
        )
        transport = FakeTransport(
            encode_frame(MessageType.STAGED_MISSION_PLAN, plan.encode())
        )
        args = build_parser().parse_args(
            [
                "mini",
                "--confirm-no-motion",
                "--duration-sec",
                "0.2",
                "--local-max-plan-ttl-ms",
                "180000",
            ]
        )
        ticks = iter(index * 0.01 for index in range(1000))
        output = io.StringIO()
        with patch(
            "lr24_pairb_dry_run.open_transport", return_value=transport
        ), patch(
            "lr24_pairb_dry_run.time.monotonic", side_effect=lambda: next(ticks)
        ), contextlib.redirect_stdout(output):
            result = mini_role(args)

        self.assertEqual(result, 0)
        self.assertIn("staged_plans_rx=1", output.getvalue())
        self.assertIn("rejected=0", output.getvalue())

        args.local_max_plan_ttl_ms = 180001
        with self.assertRaisesRegex(SystemExit, "within"):
            mini_role(args)

    def test_staged_sender_cli_is_explicit_and_bounded(self) -> None:
        args = build_parser().parse_args(
            [
                "carrier",
                "--phase",
                "trajectory",
                "--send-staged-mission-plan",
                "--v-mps",
                "0.12",
                "--max-speed-mps",
                "0.12",
                "--allow-nonhold-command",
            ]
        )
        self.assertTrue(args.send_staged_mission_plan)
        self.assertEqual(args.staged_lead_delay_ms, 5000)
        self.assertEqual(args.staged_lead_distance_m, 2.0)
        self.assertEqual(args.staged_mini_speed_mps, 0.12)


if __name__ == "__main__":
    unittest.main()
