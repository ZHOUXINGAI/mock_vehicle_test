#!/usr/bin/env python3

"""Pure safety core between Pair B commands and a gated local mission worker."""

from __future__ import annotations

import enum
from dataclasses import dataclass

try:
    from lr24_command_guard import CommandGuardPolicy, Decision, GateResult, MiniCommandGate
    from lr24_compact_protocol import Frame, MessageType, Phase, Role
except ImportError:
    from src.lr24_command_guard import CommandGuardPolicy, Decision, GateResult, MiniCommandGate
    from src.lr24_compact_protocol import Frame, MessageType, Phase, Role


class WorkerPhase(str, enum.Enum):
    WAITING = "waiting"
    RUNNING = "running"
    COMPLETE = "complete"
    FAILED = "failed"
    STOPPED = "stopped"


@dataclass(frozen=True)
class WorkerDecision:
    release_start_gate: bool
    stop_worker: bool
    reason: str
    gate_result: GateResult


class MiniMissionEndpointCore:
    """Latch one Pair B start and fail closed on command loss or Abort."""

    def __init__(self, policy: CommandGuardPolicy | None = None) -> None:
        self.gate = MiniCommandGate(
            policy or CommandGuardPolicy(target_role=Role.MINI)
        )
        self.worker_phase = WorkerPhase.WAITING
        self.local_prestate_ready = False
        self.start_released = False

    def set_local_prestate_ready(self, ready: bool) -> None:
        self.local_prestate_ready = bool(ready)

    def set_worker_result(self, return_code: int) -> None:
        if self.worker_phase == WorkerPhase.STOPPED:
            return
        self.worker_phase = (
            WorkerPhase.COMPLETE if return_code == 0 else WorkerPhase.FAILED
        )

    def mark_worker_stopped(self) -> None:
        self.worker_phase = WorkerPhase.STOPPED

    def ingest(self, frame: Frame, received_monotonic_ms: int) -> WorkerDecision:
        result = self.gate.ingest(frame, received_monotonic_ms)
        if result.decision in {Decision.ABORT, Decision.STOP}:
            return self._stop(result, result.reason)
        if result.decision == Decision.REJECT:
            return WorkerDecision(False, False, result.reason, result)
        command = result.command
        if command is None or command.phase != Phase.TRAJECTORY:
            return WorkerDecision(False, False, result.reason, result)
        if self.start_released:
            # Carrier may have one TRAJECTORY refresh in flight when Mini first
            # reports COMPLETE.  Completion is terminal success, not a restart.
            if self.worker_phase == WorkerPhase.COMPLETE:
                return WorkerDecision(False, False, "worker_complete_hold", result)
            if self.worker_phase != WorkerPhase.RUNNING:
                return WorkerDecision(False, True, "restart_refused", result)
            return WorkerDecision(False, False, "trajectory_watchdog_refresh", result)
        if self.worker_phase != WorkerPhase.WAITING:
            return WorkerDecision(False, True, "worker_not_waiting", result)
        if not self.local_prestate_ready:
            return WorkerDecision(False, False, "local_prestate_not_ready", result)
        self.start_released = True
        self.worker_phase = WorkerPhase.RUNNING
        return WorkerDecision(True, False, "pairb_trajectory_start", result)

    def poll(self, now_monotonic_ms: int) -> WorkerDecision:
        result = self.gate.poll(now_monotonic_ms)
        if result.decision in {Decision.ABORT, Decision.STOP}:
            return self._stop(result, result.reason)
        return WorkerDecision(False, False, result.reason, result)

    def _stop(self, result: GateResult, reason: str) -> WorkerDecision:
        should_stop = self.worker_phase in {WorkerPhase.WAITING, WorkerPhase.RUNNING}
        if should_stop:
            self.worker_phase = WorkerPhase.STOPPED
        return WorkerDecision(False, should_stop, reason, result)
