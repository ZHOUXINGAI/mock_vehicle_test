#!/usr/bin/env python3

"""Fail-closed Pair B follower boundaries with a zero-motion executor only."""

from __future__ import annotations

import struct
from dataclasses import dataclass
from enum import Enum

from lr24_command_guard import (
    CommandGuardPolicy,
    Decision,
    GateResult,
    MiniCommandGate,
)
from lr24_compact_protocol import Frame, MessageType, Phase, PlanCommand, Role


class ExecutorDecision(str, Enum):
    ZERO_HOLD = "zero_hold"
    ZERO_STOP = "zero_stop"
    ZERO_ABORT = "zero_abort"
    ZERO_FALLBACK = "zero_fallback"
    BLOCKED_MOTION = "blocked_motion"
    BLOCKED_GATE = "blocked_gate"


@dataclass(frozen=True)
class ExecutorOutput:
    decision: ExecutorDecision
    reason: str
    v_mps: float = 0.0
    omega_radps: float = 0.0

    @property
    def blocked(self) -> bool:
        return self.decision in {
            ExecutorDecision.BLOCKED_MOTION,
            ExecutorDecision.BLOCKED_GATE,
        }


@dataclass
class ExecutorCounters:
    executor_decisions: int = 0
    zero_output_count: int = 0
    blocked_motion_count: int = 0
    nonzero_output_count: int = 0


@dataclass(frozen=True)
class FollowerOutcome:
    gate_result: GateResult
    executor_output: ExecutorOutput


class NoMotionExecutor:
    """The only backend in this slice; it has no hardware-facing methods."""

    mode = "no_motion"

    def __init__(self) -> None:
        self.counters = ExecutorCounters()

    def apply(
        self,
        gate_result: GateResult,
        requested_command: PlanCommand | None = None,
    ) -> ExecutorOutput:
        command = requested_command or gate_result.command
        motion_requested = command is not None and (
            command.phase not in (Phase.HOLD, Phase.STOP, Phase.ABORT)
            or abs(command.v_mps) > 1.0e-12
            or abs(command.omega_radps) > 1.0e-12
        )

        if motion_requested:
            output = ExecutorOutput(
                ExecutorDecision.BLOCKED_MOTION,
                f"no_motion_blocks:{command.phase.name}",
            )
        elif gate_result.decision == Decision.REJECT:
            output = ExecutorOutput(
                ExecutorDecision.BLOCKED_GATE,
                f"gate_reject:{gate_result.reason}",
            )
        elif gate_result.decision == Decision.ABORT:
            output = ExecutorOutput(ExecutorDecision.ZERO_ABORT, gate_result.reason)
        elif gate_result.decision == Decision.STOP:
            output = ExecutorOutput(ExecutorDecision.ZERO_STOP, gate_result.reason)
        elif gate_result.decision == Decision.HOLD:
            output = ExecutorOutput(ExecutorDecision.ZERO_HOLD, gate_result.reason)
        else:
            output = ExecutorOutput(
                ExecutorDecision.ZERO_FALLBACK,
                gate_result.reason,
            )

        self.counters.executor_decisions += 1
        if output.v_mps == 0.0 and output.omega_radps == 0.0:
            self.counters.zero_output_count += 1
        else:
            self.counters.nonzero_output_count += 1
        if output.decision == ExecutorDecision.BLOCKED_MOTION:
            self.counters.blocked_motion_count += 1
        return output


class MiniLiveFollower:
    """Own the Mini Pair B gate and feed every result to no_motion output."""

    def __init__(
        self,
        policy: CommandGuardPolicy | None = None,
        executor: NoMotionExecutor | None = None,
    ) -> None:
        self.gate = MiniCommandGate(policy)
        self.executor = executor or NoMotionExecutor()

    def ingest(self, frame: Frame, received_monotonic_ms: int) -> FollowerOutcome:
        requested_command: PlanCommand | None = None
        if frame.msg_type == MessageType.PLAN_COMMAND:
            try:
                requested_command = PlanCommand.decode(frame.payload)
            except (ValueError, struct.error):
                requested_command = None
        try:
            gate_result = self.gate.ingest(frame, received_monotonic_ms)
        except (ValueError, struct.error) as exc:
            gate_result = GateResult(
                Decision.REJECT,
                f"malformed_frame:{type(exc).__name__}",
            )
        output = self.executor.apply(gate_result, requested_command)
        return FollowerOutcome(gate_result, output)

    def poll(self, now_monotonic_ms: int) -> FollowerOutcome:
        gate_result = self.gate.poll(now_monotonic_ms)
        output = self.executor.apply(gate_result)
        return FollowerOutcome(gate_result, output)

    def clear_abort_locally(self) -> None:
        self.gate.clear_abort_locally()


class CarrierLocalFollower:
    """Validate a Carrier-local command before the same no_motion boundary."""

    def __init__(
        self,
        policy: CommandGuardPolicy | None = None,
        executor: NoMotionExecutor | None = None,
    ) -> None:
        if policy is None:
            policy = CommandGuardPolicy(target_role=Role.CARRIER)
        elif policy.target_role != Role.CARRIER:
            raise ValueError("CarrierLocalFollower requires target_role=Role.CARRIER")
        self.gate = MiniCommandGate(policy)
        self.executor = executor or NoMotionExecutor()

    def apply_command(
        self,
        command: PlanCommand,
        received_monotonic_ms: int,
    ) -> FollowerOutcome:
        try:
            frame = Frame(MessageType.PLAN_COMMAND, command.encode())
            gate_result = self.gate.ingest(frame, received_monotonic_ms)
        except (OverflowError, ValueError, struct.error) as exc:
            gate_result = GateResult(
                Decision.REJECT,
                f"invalid_local_command:{type(exc).__name__}",
            )
        output = self.executor.apply(gate_result, command)
        return FollowerOutcome(gate_result, output)

    def poll(self, now_monotonic_ms: int) -> FollowerOutcome:
        gate_result = self.gate.poll(now_monotonic_ms)
        output = self.executor.apply(gate_result)
        return FollowerOutcome(gate_result, output)

    def clear_abort_locally(self) -> None:
        self.gate.clear_abort_locally()
