#!/usr/bin/env python3

from __future__ import annotations

import sys
import unittest
from pathlib import Path


REPO_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_DIR / "src"))

from lr24_command_guard import CommandGuardPolicy, Decision, GateResult
from lr24_compact_protocol import (
    Abort,
    AbortReason,
    CorridorPlanCompact,
    FieldOrigin,
    Frame,
    MessageType,
    Phase,
    PlanCommand,
    PlanFlag,
    Role,
)
from lr24_live_follower import (
    CarrierLocalFollower,
    ExecutorDecision,
    MiniLiveFollower,
    NoMotionExecutor,
)


def frame(msg_type: MessageType, payload: bytes) -> Frame:
    return Frame(msg_type, payload)


def origin() -> FieldOrigin:
    return FieldOrigin(3, 1, 900, 31.0, 121.0, 5.0)


def plan() -> CorridorPlanCompact:
    return CorridorPlanCompact(
        plan_schema_version=2,
        plan_id=7,
        seq=1,
        timestamp_ms=1000,
        valid_until_ms=33000,
        rendezvous_x_m=0.89,
        rendezvous_y_m=-4.41,
        tangent_dir_x=0.9803,
        tangent_dir_y=0.1974,
        corridor_length_m=8.0,
        ahead_distance_m=0.35,
        mini_arrival_delay_ms=25000,
        trigger_phase_rad=4.911,
        mini_speed_mps=0.9,
        carrier_max_speed_mps=0.7,
        target_front_gap_m=0.35,
        required_validity_ms=28400,
        post_tangent_reserve_ms=3350,
        terminal_completion_budget_ms=2000,
        completion_hold_ms=500,
        plan_timing_guard_ms=100,
        command_ttl_ms=500,
        local_command_watchdog_ms=750,
        flags=int(PlanFlag.CORRIDOR_VALID | PlanFlag.ONE_ORBIT_COMPLETE),
        origin_id=3,
    )


def command(
    phase: Phase,
    *,
    role: Role = Role.MINI,
    seq: int = 1,
    ttl_ms: int = 500,
    v_mps: float = 0.0,
    omega_radps: float = 0.0,
) -> PlanCommand:
    return PlanCommand(
        plan_id=7,
        role=role,
        phase=phase,
        seq=seq,
        timestamp_ms=1000,
        valid_until_ms=1000 + ttl_ms,
        v_mps=v_mps,
        omega_radps=omega_radps,
        duration_ms=ttl_ms,
        distance_m=0.0,
        max_speed_mps=0.9,
        max_accel_mps2=0.5,
        flags=0,
    )


class NoMotionExecutorTest(unittest.TestCase):
    def test_hold_stop_abort_and_fallback_are_exact_zero(self) -> None:
        executor = NoMotionExecutor()
        cases = (
            GateResult(Decision.HOLD, "hold"),
            GateResult(Decision.STOP, "watchdog"),
            GateResult(Decision.ABORT, "abort"),
            GateResult(Decision.ACCEPT, "corridor_plan"),
        )
        for result in cases:
            output = executor.apply(result)
            self.assertEqual((output.v_mps, output.omega_radps), (0.0, 0.0))
        self.assertEqual(executor.counters.zero_output_count, 4)
        self.assertEqual(executor.counters.nonzero_output_count, 0)

    def test_motion_phase_and_nonzero_safe_phase_are_reported_blocked(self) -> None:
        executor = NoMotionExecutor()
        requests = (
            command(Phase.ORBIT),
            command(Phase.HOLD, seq=2, v_mps=0.1),
            command(Phase.ABORT, seq=3, omega_radps=0.1),
        )
        for request in requests:
            output = executor.apply(
                GateResult(Decision.REJECT, "gate_reject"),
                request,
            )
            self.assertEqual(output.decision, ExecutorDecision.BLOCKED_MOTION)
            self.assertEqual((output.v_mps, output.omega_radps), (0.0, 0.0))
        self.assertEqual(executor.counters.blocked_motion_count, 3)
        self.assertEqual(executor.counters.nonzero_output_count, 0)


class MiniLiveFollowerTest(unittest.TestCase):
    def setUp(self) -> None:
        self.follower = MiniLiveFollower(
            CommandGuardPolicy(command_watchdog_ms=750)
        )

    def prime_plan(self) -> None:
        self.follower.ingest(frame(MessageType.FIELD_ORIGIN, origin().encode()), 9000)
        self.follower.ingest(frame(MessageType.CORRIDOR_PLAN, plan().encode()), 9100)

    def test_hold_ttl_and_watchdog_fallback_stay_zero(self) -> None:
        self.prime_plan()
        hold = command(Phase.HOLD, ttl_ms=500)
        outcome = self.follower.ingest(
            frame(MessageType.PLAN_COMMAND, hold.encode()),
            10000,
        )
        self.assertEqual(outcome.gate_result.decision, Decision.HOLD)
        self.assertEqual(outcome.executor_output.decision, ExecutorDecision.ZERO_HOLD)
        expired = self.follower.poll(10500)
        self.assertEqual(expired.gate_result.reason, "command_expired")
        self.assertEqual(expired.executor_output.decision, ExecutorDecision.ZERO_STOP)

        orbit = command(Phase.ORBIT, seq=2, ttl_ms=1000, v_mps=0.6, omega_radps=0.2)
        blocked = self.follower.ingest(
            frame(MessageType.PLAN_COMMAND, orbit.encode()),
            11000,
        )
        self.assertEqual(blocked.gate_result.decision, Decision.ACCEPT)
        self.assertEqual(blocked.executor_output.decision, ExecutorDecision.BLOCKED_MOTION)
        watchdog = self.follower.poll(11750)
        self.assertEqual(watchdog.gate_result.reason, "command_watchdog")
        self.assertEqual(watchdog.executor_output.decision, ExecutorDecision.ZERO_STOP)
        self.assertEqual(self.follower.executor.counters.nonzero_output_count, 0)

    def test_abort_latches_until_explicit_local_clear(self) -> None:
        abort = Abort(Role.CARRIER, AbortReason.OPERATOR, 7, 1, 1200)
        outcome = self.follower.ingest(
            frame(MessageType.ABORT, abort.encode()),
            10000,
        )
        self.assertEqual(outcome.gate_result.decision, Decision.ABORT)
        self.assertEqual(outcome.executor_output.decision, ExecutorDecision.ZERO_ABORT)
        self.assertEqual(self.follower.poll(20000).gate_result.decision, Decision.ABORT)

        rejected = self.follower.ingest(
            frame(MessageType.PLAN_COMMAND, command(Phase.HOLD).encode()),
            20001,
        )
        self.assertEqual(rejected.gate_result.reason, "abort_latched")
        self.follower.clear_abort_locally()
        self.assertEqual(self.follower.poll(20002).gate_result.reason, "no_command")

    def test_malformed_command_fails_closed_with_zero_output(self) -> None:
        outcome = self.follower.ingest(
            frame(MessageType.PLAN_COMMAND, bytes(30)),
            10000,
        )
        self.assertEqual(outcome.gate_result.decision, Decision.REJECT)
        self.assertTrue(outcome.gate_result.reason.startswith("malformed_frame:"))
        self.assertEqual(outcome.executor_output.decision, ExecutorDecision.BLOCKED_GATE)
        self.assertEqual(
            (outcome.executor_output.v_mps, outcome.executor_output.omega_radps),
            (0.0, 0.0),
        )


class CarrierLocalFollowerTest(unittest.TestCase):
    def test_local_hold_is_zero_and_motion_is_blocked(self) -> None:
        follower = CarrierLocalFollower()
        hold = follower.apply_command(
            command(Phase.HOLD, role=Role.CARRIER),
            10000,
        )
        self.assertEqual(hold.gate_result.decision, Decision.HOLD)
        self.assertEqual(hold.executor_output.decision, ExecutorDecision.ZERO_HOLD)

        motion = follower.apply_command(
            command(Phase.ARC_TO_CORRIDOR, role=Role.CARRIER, seq=2, v_mps=0.4),
            10100,
        )
        self.assertEqual(motion.executor_output.decision, ExecutorDecision.BLOCKED_MOTION)
        self.assertEqual(follower.executor.counters.nonzero_output_count, 0)


if __name__ == "__main__":
    unittest.main()
