#!/usr/bin/env python3

"""Safety gate for Pair B plans and body-frame rover commands.

This module has no ROS, MAVROS, PX4, or serial dependencies. It validates
decoded LR24 frames and returns decisions for a separate local executor. It
never arms a vehicle and never publishes a motor command.
"""

from __future__ import annotations

import enum
import math
from dataclasses import dataclass

from lr24_compact_protocol import (
    Abort,
    CorridorPlanCompact,
    FieldOrigin,
    Frame,
    MessageType,
    Phase,
    PlanCommand,
    PlanFlag,
    Role,
    sequence_is_newer,
)


class Decision(enum.Enum):
    ACCEPT = "accept"
    HOLD = "hold"
    STOP = "stop"
    ABORT = "abort"
    REJECT = "reject"


@dataclass(frozen=True)
class GateResult:
    decision: Decision
    reason: str
    command: PlanCommand | None = None
    corridor_plan: CorridorPlanCompact | None = None
    abort: Abort | None = None


@dataclass(frozen=True)
class CommandGuardPolicy:
    target_role: Role = Role.MINI
    max_linear_speed_mps: float = 1.0
    max_yaw_rate_radps: float = 0.6
    max_accel_mps2: float = 0.5
    max_command_duration_ms: int = 5000
    max_command_ttl_ms: int = 2000
    max_plan_ttl_ms: int = 120000
    command_watchdog_ms: int = 750


class MiniCommandGate:
    """Validate Pair B downlink before it can reach a Mini executor.

    Sender timestamps use that sender's CLOCK_BOOTTIME milliseconds. The
    receiver never compares the sender timestamp with its own clock. Instead,
    it derives a TTL from valid_until_ms - timestamp_ms and starts that TTL at
    local receipt. Sequence checks and the local watchdog reject delayed or
    stopped streams.
    """

    def __init__(self, policy: CommandGuardPolicy | None = None) -> None:
        self.policy = policy or CommandGuardPolicy()
        self.active_plan: CorridorPlanCompact | None = None
        self.active_origin: FieldOrigin | None = None
        self.active_command: PlanCommand | None = None
        self.abort_latched: Abort | None = None
        self._last_plan_seq: int | None = None
        self._last_origin_seq: int | None = None
        self._last_command_seq: int | None = None
        self._command_received_ms: int | None = None
        self._command_expiry_ms: int | None = None

    def ingest(self, frame: Frame, received_monotonic_ms: int) -> GateResult:
        if frame.msg_type == MessageType.ABORT:
            abort = Abort.decode(frame.payload)
            self.abort_latched = abort
            self.active_command = None
            self._command_expiry_ms = None
            return GateResult(Decision.ABORT, f"abort:{abort.reason.name}", abort=abort)

        if self.abort_latched is not None:
            return GateResult(Decision.REJECT, "abort_latched")

        if frame.msg_type == MessageType.FIELD_ORIGIN:
            origin = FieldOrigin.decode(frame.payload)
            reason = self._validate_origin(origin)
            if reason:
                return GateResult(Decision.REJECT, reason)
            if self.active_origin is not None and origin.origin_id != self.active_origin.origin_id:
                self.active_plan = None
                self.active_command = None
            self.active_origin = origin
            self._last_origin_seq = origin.seq
            return GateResult(Decision.ACCEPT, "field_origin")

        if frame.msg_type == MessageType.CORRIDOR_PLAN:
            plan = CorridorPlanCompact.decode(frame.payload)
            reason = self._validate_plan(plan)
            if reason:
                return GateResult(Decision.REJECT, reason)
            self.active_plan = plan
            self._last_plan_seq = plan.seq
            return GateResult(Decision.ACCEPT, "corridor_plan", corridor_plan=plan)

        if frame.msg_type == MessageType.PLAN_COMMAND:
            command = PlanCommand.decode(frame.payload)
            reason = self._validate_command(command)
            if reason:
                return GateResult(Decision.REJECT, reason)
            if command.phase == Phase.ABORT:
                self.active_command = None
                return GateResult(Decision.ABORT, "abort_phase")
            self.active_command = command
            self._last_command_seq = command.seq
            self._command_received_ms = int(received_monotonic_ms)
            self._command_expiry_ms = int(received_monotonic_ms) + command.validity_ms
            if command.phase == Phase.HOLD:
                return GateResult(Decision.HOLD, "hold_command", command=command)
            if command.phase == Phase.STOP:
                return GateResult(Decision.STOP, "stop_command", command=command)
            return GateResult(Decision.ACCEPT, "motion_command", command=command)

        return GateResult(Decision.REJECT, f"unsupported_type:{frame.msg_type.name}")

    def poll(self, now_monotonic_ms: int) -> GateResult:
        if self.abort_latched is not None:
            return GateResult(
                Decision.ABORT,
                f"abort_latched:{self.abort_latched.reason.name}",
                abort=self.abort_latched,
            )
        if self.active_command is None:
            return GateResult(Decision.HOLD, "no_command")
        assert self._command_received_ms is not None
        assert self._command_expiry_ms is not None
        if int(now_monotonic_ms) >= self._command_expiry_ms:
            self.active_command = None
            return GateResult(Decision.STOP, "command_expired")
        if (
            int(now_monotonic_ms) - self._command_received_ms
            >= self.policy.command_watchdog_ms
        ):
            self.active_command = None
            return GateResult(Decision.STOP, "command_watchdog")
        if self.active_command.phase == Phase.HOLD:
            return GateResult(Decision.HOLD, "hold_active", command=self.active_command)
        if self.active_command.phase == Phase.STOP:
            return GateResult(Decision.STOP, "stop_active", command=self.active_command)
        return GateResult(Decision.ACCEPT, "command_active", command=self.active_command)

    def clear_abort_locally(self) -> None:
        """Clear the latch only after a local operator completes safety checks."""

        self.abort_latched = None
        self.active_command = None
        self._command_received_ms = None
        self._command_expiry_ms = None

    def _validate_plan(self, plan: CorridorPlanCompact) -> str | None:
        if self._last_plan_seq is not None and not sequence_is_newer(
            plan.seq, self._last_plan_seq
        ):
            return "duplicate_or_old_plan_seq"
        if not (plan.flags & int(PlanFlag.CORRIDOR_VALID)):
            return "corridor_not_valid"
        if self.active_origin is None:
            return "plan_without_field_origin"
        if plan.origin_id != self.active_origin.origin_id:
            return "origin_id_mismatch"
        if not 0 < plan.validity_ms <= self.policy.max_plan_ttl_ms:
            return "invalid_plan_ttl"
        tangent_norm = math.hypot(plan.tangent_dir_x, plan.tangent_dir_y)
        if not 0.98 <= tangent_norm <= 1.02:
            return "invalid_tangent_norm"
        if plan.corridor_length_m <= 0.0:
            return "invalid_corridor_length"
        if plan.mini_speed_mps <= 0.0:
            return "invalid_mini_speed"
        if plan.carrier_max_speed_mps <= 0.0:
            return "invalid_carrier_speed"
        return None

    def _validate_origin(self, origin: FieldOrigin) -> str | None:
        if self._last_origin_seq is not None and not sequence_is_newer(
            origin.seq, self._last_origin_seq
        ):
            return "duplicate_or_old_origin_seq"
        if origin.origin_id == 0:
            return "invalid_origin_id"
        if not -90.0 <= origin.latitude_deg <= 90.0:
            return "invalid_origin_latitude"
        if not -180.0 <= origin.longitude_deg <= 180.0:
            return "invalid_origin_longitude"
        return None

    def _validate_command(self, command: PlanCommand) -> str | None:
        if command.role != self.policy.target_role:
            return "wrong_target_role"
        if self._last_command_seq is not None and not sequence_is_newer(
            command.seq, self._last_command_seq
        ):
            return "duplicate_or_old_command_seq"
        if not 0 < command.validity_ms <= self.policy.max_command_ttl_ms:
            return "invalid_command_ttl"
        if not 0 < command.duration_ms <= self.policy.max_command_duration_ms:
            return "invalid_command_duration"
        if command.v_mps < 0.0:
            return "reverse_not_allowed"
        if abs(command.v_mps) > self.policy.max_linear_speed_mps:
            return "local_linear_limit"
        if abs(command.omega_radps) > self.policy.max_yaw_rate_radps:
            return "local_yaw_rate_limit"
        if command.max_speed_mps > self.policy.max_linear_speed_mps:
            return "declared_linear_limit"
        if command.max_accel_mps2 > self.policy.max_accel_mps2:
            return "declared_accel_limit"
        if command.max_speed_mps > 0.0 and command.v_mps > command.max_speed_mps:
            return "command_exceeds_declared_speed"
        if command.phase in (Phase.HOLD, Phase.STOP) and (
            abs(command.v_mps) > 1.0e-6 or abs(command.omega_radps) > 1.0e-6
        ):
            return "nonzero_hold_or_stop"
        if command.phase in (Phase.ARC_TO_CORRIDOR, Phase.TERMINAL):
            if self.active_plan is None:
                return "motion_without_corridor_plan"
            if command.plan_id != self.active_plan.plan_id:
                return "plan_id_mismatch"
        return None
