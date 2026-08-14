from __future__ import annotations

import dataclasses
import unittest
import sys
from pathlib import Path

REPO_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_DIR / "src"))

from lr24_command_guard import CommandGuardPolicy, Decision, MiniCommandGate
from lr24_compact_protocol import (
    AbortReason,
    Frame,
    HealthFlag,
    MessageType,
    MiniState,
    Phase,
    Role,
)
from pairb_staged_chase import (
    ChasePhase,
    MINI_TERMINAL_STATUS_GRACE_MS,
    REQUIRED_MINI_HEALTH,
    StagedChaseConfig,
    StagedChaseCoordinator,
    build_staged_mission_plan,
    terminal_gap_is_reached,
    terminal_gap_is_unsafe,
    terminal_gap_metrics,
    validate_staged_chase_config,
)


def state(seq: int, x_m: float, health: int = REQUIRED_MINI_HEALTH) -> MiniState:
    return MiniState(1, seq, 1000 + seq * 100, x_m, 0.0, 0.5, 0.0, 0.0, 0.0, health, 1)


class StagedChaseTests(unittest.TestCase):
    def setUp(self) -> None:
        self.config = StagedChaseConfig(lead_delay_ms=2000, lead_distance_m=1.0)
        self.coordinator = StagedChaseCoordinator(self.config)
        self.coordinator.set_carrier_ready(True)
        self.coordinator.accept_mini_state(state(0, 0.0), 1000)

    def test_no_authorization_remains_zero_hold(self) -> None:
        decision = self.coordinator.step(1000)
        self.assertEqual(decision.phase, ChasePhase.HOLD)
        self.assertEqual(decision.remote_command.phase, Phase.HOLD)
        self.assertEqual(decision.remote_command.v_mps, 0.0)
        self.assertFalse(decision.start_local_carrier)

    def test_mini_starts_first_and_carrier_requires_time_and_distance(self) -> None:
        self.coordinator.authorize_start()
        first = self.coordinator.step(1000)
        self.assertEqual(first.phase, ChasePhase.MINI_ACTIVE)
        self.assertEqual(first.remote_command.phase, Phase.TRAJECTORY)
        self.assertFalse(first.start_local_carrier)

        self.coordinator.accept_mini_state(state(1, 1.2), 2000)
        early = self.coordinator.step(2000)
        self.assertEqual(early.phase, ChasePhase.MINI_ACTIVE)
        self.assertFalse(early.start_local_carrier)

        self.coordinator.accept_mini_state(state(2, 1.3), 3000)
        start = self.coordinator.step(3000)
        self.assertEqual(start.phase, ChasePhase.BOTH_ACTIVE)
        self.assertTrue(start.start_local_carrier)
        self.coordinator.accept_mini_state(state(3, 1.4), 3100)
        repeated = self.coordinator.step(3100)
        self.assertFalse(repeated.start_local_carrier)

    def test_stale_pairb_state_aborts_both(self) -> None:
        self.coordinator.authorize_start()
        self.coordinator.step(1000)
        failed = self.coordinator.step(1800)
        self.assertEqual(failed.phase, ChasePhase.ABORTED)
        self.assertEqual(failed.abort.reason, AbortReason.LINK_STALE)
        self.assertTrue(failed.stop_local_carrier)
        self.assertEqual(failed.remote_command.phase, Phase.STOP)

    def test_active_carrier_readiness_loss_aborts_both(self) -> None:
        self.coordinator.authorize_start()
        self.coordinator.step(1000)
        self.coordinator.set_carrier_ready(False)
        self.coordinator.accept_mini_state(state(1, 0.1), 1100)
        failed = self.coordinator.step(1100)
        self.assertEqual(failed.phase, ChasePhase.ABORTED)
        self.assertEqual(failed.abort.reason, AbortReason.LOCAL_SAFETY)
        self.assertEqual(failed.reason, "carrier_local_not_ready")

    def test_fresh_invalid_mini_health_is_state_invalid(self) -> None:
        self.coordinator.authorize_start()
        self.coordinator.step(1000)
        invalid = REQUIRED_MINI_HEALTH & ~int(HealthFlag.RC_STOP_READY)
        self.coordinator.accept_mini_state(state(1, 0.1, invalid), 1100)
        failed = self.coordinator.step(1100)
        self.assertEqual(failed.abort.reason, AbortReason.STATE_INVALID)
        self.assertEqual(failed.reason, "mini_health_invalid")

    def test_missing_executor_health_never_starts(self) -> None:
        missing = REQUIRED_MINI_HEALTH & ~int(HealthFlag.EXECUTOR_READY)
        coordinator = StagedChaseCoordinator(self.config)
        coordinator.set_carrier_ready(True)
        coordinator.authorize_start()
        coordinator.accept_mini_state(state(0, 0.0, missing), 1000)
        decision = coordinator.step(1000)
        self.assertEqual(decision.phase, ChasePhase.HOLD)
        self.assertEqual(decision.remote_command.phase, Phase.HOLD)

    def test_mismatched_shared_origin_never_starts(self) -> None:
        coordinator = StagedChaseCoordinator(self.config)
        coordinator.set_carrier_ready(True)
        coordinator.authorize_start()
        wrong_origin = state(0, 0.0).__class__(
            **{**state(0, 0.0).__dict__, "origin_id": 99}
        )
        coordinator.accept_mini_state(wrong_origin, 1000)

        decision = coordinator.step(1000)

        self.assertEqual(decision.phase, ChasePhase.HOLD)
        self.assertEqual(decision.remote_command.phase, Phase.HOLD)

    def test_plan_then_trajectory_passes_gate_and_wrong_id_fails(self) -> None:
        gate = MiniCommandGate(CommandGuardPolicy(target_role=Role.MINI))
        plan = build_staged_mission_plan(self.config, seq=1, sender_monotonic_ms=1000)
        accepted_plan = gate.ingest(
            Frame(MessageType.STAGED_MISSION_PLAN, plan.encode()), 5000
        )
        self.assertEqual(accepted_plan.decision, Decision.ACCEPT)

        self.coordinator.authorize_start()
        command = self.coordinator.step(1000).remote_command
        accepted = gate.ingest(Frame(MessageType.PLAN_COMMAND, command.encode()), 5100)
        self.assertEqual(accepted.decision, Decision.ACCEPT)

        wrong = command.__class__(**{**command.__dict__, "plan_id": 99, "seq": command.seq + 1})
        rejected = gate.ingest(Frame(MessageType.PLAN_COMMAND, wrong.encode()), 5200)
        self.assertEqual(rejected.reason, "staged_plan_id_mismatch")

    def test_config_rejects_carrier_faster_than_mini(self) -> None:
        with self.assertRaises(ValueError):
            validate_staged_chase_config(
                StagedChaseConfig(mini_speed_mps=0.05, carrier_speed_mps=0.06)
            )

    def test_terminal_gap_uses_shared_mini_heading_axis(self) -> None:
        carrier = MiniState(
            2, 1, 1000, 8.0, 3.0, 0.0, 0.0, 0.0, 0.0,
            REQUIRED_MINI_HEALTH, 1,
        )
        mini = MiniState(
            1, 2, 1000, 10.0, 3.0, 0.0, 0.0, 0.0, 0.0,
            REQUIRED_MINI_HEALTH, 1,
        )

        metrics = terminal_gap_metrics(carrier, mini, expected_origin_id=1)

        self.assertAlmostEqual(metrics.distance_m, 2.0)
        self.assertAlmostEqual(metrics.longitudinal_gap_m, 2.0)
        self.assertAlmostEqual(metrics.lateral_offset_m, 0.0)
        self.assertTrue(terminal_gap_is_reached(metrics, self.config))
        self.assertFalse(terminal_gap_is_unsafe(metrics, self.config))

    def test_terminal_gap_rejects_lateral_offset_and_origin_mismatch(self) -> None:
        carrier = MiniState(
            2, 1, 1000, 8.0, 1.5, 0.0, 0.0, 0.0, 0.0,
            REQUIRED_MINI_HEALTH, 1,
        )
        mini = MiniState(
            1, 2, 1000, 10.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            REQUIRED_MINI_HEALTH, 1,
        )
        metrics = terminal_gap_metrics(carrier, mini, expected_origin_id=1)
        self.assertFalse(terminal_gap_is_reached(metrics, self.config))
        with self.assertRaisesRegex(ValueError, "origin"):
            terminal_gap_metrics(
                dataclasses.replace(carrier, origin_id=2),
                mini,
                expected_origin_id=1,
            )

    def test_terminal_collision_guard_is_independent_of_target_window(self) -> None:
        carrier = MiniState(
            2, 1, 1000, 9.6, 0.0, 0.0, 0.0, 0.0, 0.0,
            REQUIRED_MINI_HEALTH, 1,
        )
        mini = MiniState(
            1, 2, 1000, 10.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            REQUIRED_MINI_HEALTH, 1,
        )
        metrics = terminal_gap_metrics(carrier, mini, expected_origin_id=1)
        self.assertTrue(terminal_gap_is_unsafe(metrics, self.config))
        self.assertFalse(terminal_gap_is_reached(metrics, self.config))

    def test_completed_mini_is_held_stopped_while_carrier_finishes(self) -> None:
        self.coordinator.authorize_start()
        self.coordinator.step(1000)
        self.coordinator.accept_mini_state(state(1, 1.2), 3000)
        started = self.coordinator.step(3000)
        self.assertEqual(started.phase, ChasePhase.BOTH_ACTIVE)
        self.coordinator.mark_mini_complete()
        finishing = self.coordinator.step(4000)
        self.assertEqual(finishing.remote_command.phase, Phase.STOP)
        self.assertFalse(finishing.stop_local_carrier)
        self.coordinator.mark_carrier_complete()
        completed = self.coordinator.step(4100)
        self.assertEqual(completed.phase, ChasePhase.COMPLETE)
        self.assertTrue(completed.stop_local_carrier)

    def test_post_run_health_frame_waits_for_complete_status(self) -> None:
        self.coordinator.authorize_start()
        self.coordinator.step(1000)
        self.coordinator.accept_mini_state(state(1, 1.2), 3000)
        self.assertEqual(self.coordinator.step(3000).phase, ChasePhase.BOTH_ACTIVE)
        post_run_health = REQUIRED_MINI_HEALTH & ~int(HealthFlag.EXECUTOR_READY)
        self.coordinator.accept_mini_state(state(2, 1.3, post_run_health), 3100)

        grace = self.coordinator.step(3100)
        self.assertEqual(grace.phase, ChasePhase.BOTH_ACTIVE)
        self.assertEqual(grace.reason, "awaiting_mini_terminal_status")
        self.assertEqual(grace.remote_command.phase, Phase.STOP)
        self.assertFalse(grace.stop_local_carrier)

        self.coordinator.mark_mini_complete()
        finishing = self.coordinator.step(3200)
        self.assertEqual(finishing.reason, "mini_complete_carrier_finishing")
        self.assertFalse(finishing.stop_local_carrier)

    def test_post_run_health_without_complete_aborts_after_grace(self) -> None:
        self.coordinator.authorize_start()
        self.coordinator.step(1000)
        self.coordinator.accept_mini_state(state(1, 1.2), 3000)
        self.coordinator.step(3000)
        post_run_health = REQUIRED_MINI_HEALTH & ~int(HealthFlag.EXECUTOR_READY)
        self.coordinator.accept_mini_state(state(2, 1.3, post_run_health), 3100)
        self.assertEqual(
            self.coordinator.step(3100).reason,
            "awaiting_mini_terminal_status",
        )

        failed = self.coordinator.step(
            3100 + MINI_TERMINAL_STATUS_GRACE_MS + 1
        )
        self.assertEqual(failed.phase, ChasePhase.ABORTED)
        self.assertEqual(failed.reason, "mini_health_invalid")
        self.assertEqual(failed.abort.reason, AbortReason.STATE_INVALID)

    def test_mini_completion_before_carrier_start_aborts(self) -> None:
        self.coordinator.authorize_start()
        self.coordinator.step(1000)
        self.coordinator.mark_mini_complete()
        failed = self.coordinator.step(1100)
        self.assertEqual(failed.phase, ChasePhase.ABORTED)
        self.assertEqual(failed.abort.reason, AbortReason.LOCAL_SAFETY)


if __name__ == "__main__":
    unittest.main()
