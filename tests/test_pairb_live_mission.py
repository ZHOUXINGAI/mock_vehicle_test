from __future__ import annotations

import sys
import unittest
from pathlib import Path

REPO_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_DIR / "src"))

from lr24_command_guard import CommandGuardPolicy
from lr24_compact_protocol import (
    Abort,
    AbortReason,
    Frame,
    MessageType,
    Phase,
    PlanCommand,
    Role,
)
from pairb_live_mission import MiniMissionEndpointCore, WorkerPhase
from pairb_staged_chase import StagedChaseConfig, build_staged_mission_plan


def frame(msg_type: MessageType, payload: bytes) -> Frame:
    return Frame(msg_type, payload)


def trajectory(seq: int, *, plan_id: int = 1) -> PlanCommand:
    return PlanCommand(
        plan_id=plan_id,
        role=Role.MINI,
        phase=Phase.TRAJECTORY,
        seq=seq,
        timestamp_ms=1000 + seq * 100,
        valid_until_ms=1500 + seq * 100,
        v_mps=0.12,
        omega_radps=0.0,
        duration_ms=500,
        distance_m=5.0,
        max_speed_mps=0.12,
        max_accel_mps2=0.30,
    )


class PairBLiveMissionCoreTest(unittest.TestCase):
    def setUp(self) -> None:
        self.core = MiniMissionEndpointCore(
            CommandGuardPolicy(target_role=Role.MINI)
        )
        plan = build_staged_mission_plan(
            StagedChaseConfig(), seq=1, sender_monotonic_ms=1000
        )
        self.core.ingest(
            frame(MessageType.STAGED_MISSION_PLAN, plan.encode()), 5000
        )

    def test_never_releases_before_local_safe_prestate(self) -> None:
        decision = self.core.ingest(
            frame(MessageType.PLAN_COMMAND, trajectory(1).encode()), 5100
        )
        self.assertFalse(decision.release_start_gate)
        self.assertFalse(decision.stop_worker)
        self.assertEqual(decision.reason, "local_prestate_not_ready")
        self.assertEqual(self.core.worker_phase, WorkerPhase.WAITING)

    def test_first_valid_trajectory_releases_once(self) -> None:
        self.core.set_local_prestate_ready(True)
        first = self.core.ingest(
            frame(MessageType.PLAN_COMMAND, trajectory(1).encode()), 5100
        )
        second = self.core.ingest(
            frame(MessageType.PLAN_COMMAND, trajectory(2).encode()), 5200
        )
        self.assertTrue(first.release_start_gate)
        self.assertFalse(second.release_start_gate)
        self.assertEqual(second.reason, "trajectory_watchdog_refresh")
        self.assertEqual(self.core.worker_phase, WorkerPhase.RUNNING)

    def test_watchdog_stops_and_never_restarts(self) -> None:
        self.core.set_local_prestate_ready(True)
        self.core.ingest(frame(MessageType.PLAN_COMMAND, trajectory(1).encode()), 5100)
        stopped = self.core.poll(5600)
        self.assertTrue(stopped.stop_worker)
        self.assertEqual(self.core.worker_phase, WorkerPhase.STOPPED)
        retry = self.core.ingest(
            frame(MessageType.PLAN_COMMAND, trajectory(2).encode()), 5700
        )
        self.assertFalse(retry.release_start_gate)
        self.assertTrue(retry.stop_worker)
        self.assertEqual(retry.reason, "restart_refused")

    def test_completed_worker_ignores_inflight_trajectory_refresh(self) -> None:
        self.core.set_local_prestate_ready(True)
        self.core.ingest(frame(MessageType.PLAN_COMMAND, trajectory(1).encode()), 5100)
        self.core.set_worker_result(0)

        refresh = self.core.ingest(
            frame(MessageType.PLAN_COMMAND, trajectory(2).encode()), 5200
        )

        self.assertFalse(refresh.release_start_gate)
        self.assertFalse(refresh.stop_worker)
        self.assertEqual(refresh.reason, "worker_complete_hold")
        self.assertEqual(self.core.worker_phase, WorkerPhase.COMPLETE)

    def test_abort_stops_waiting_worker(self) -> None:
        abort = Abort(Role.CARRIER, AbortReason.OPERATOR, 1, 1, 2000)
        decision = self.core.ingest(
            frame(MessageType.ABORT, abort.encode()), 5100
        )
        self.assertTrue(decision.stop_worker)
        self.assertEqual(self.core.worker_phase, WorkerPhase.STOPPED)

    def test_wrong_plan_never_releases(self) -> None:
        self.core.set_local_prestate_ready(True)
        decision = self.core.ingest(
            frame(
                MessageType.PLAN_COMMAND,
                trajectory(1, plan_id=99).encode(),
            ),
            5100,
        )
        self.assertFalse(decision.release_start_gate)
        self.assertEqual(decision.reason, "staged_plan_id_mismatch")


if __name__ == "__main__":
    unittest.main()
