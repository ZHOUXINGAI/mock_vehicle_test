#!/usr/bin/env python3

"""Pure Pair B coordinator for a Mini-first, Carrier-delayed chase test."""

from __future__ import annotations

import enum
import math
from dataclasses import dataclass

try:
    from lr24_compact_protocol import (
        Abort,
        AbortReason,
        HealthFlag,
        MiniState,
        Phase,
        PlanCommand,
        Role,
        StagedMissionFlag,
        StagedMissionPlan,
        U32_MASK,
        sequence_is_newer,
    )
except ImportError:
    from src.lr24_compact_protocol import (
        Abort,
        AbortReason,
        HealthFlag,
        MiniState,
        Phase,
        PlanCommand,
        Role,
        StagedMissionFlag,
        StagedMissionPlan,
        U32_MASK,
        sequence_is_newer,
    )


REQUIRED_MINI_HEALTH = int(
    HealthFlag.POSITION_VALID
    | HealthFlag.VELOCITY_VALID
    | HealthFlag.YAW_VALID
    | HealthFlag.ORIGIN_VALID
    | HealthFlag.PX4_CONNECTED
    | HealthFlag.RC_STOP_READY
    | HealthFlag.EXECUTOR_READY
)
TERMINAL_MINI_HEALTH = int(
    HealthFlag.POSITION_VALID
    | HealthFlag.YAW_VALID
    | HealthFlag.ORIGIN_VALID
    | HealthFlag.PX4_CONNECTED
)
MINI_TERMINAL_STATUS_GRACE_MS = 500


class ChasePhase(str, enum.Enum):
    HOLD = "hold"
    MINI_ACTIVE = "mini_active"
    BOTH_ACTIVE = "both_active"
    COMPLETE = "complete"
    ABORTED = "aborted"


@dataclass(frozen=True)
class StagedChaseConfig:
    plan_id: int = 1
    straight_distance_m: float = 5.0
    turn_radius_m: float = 3.0
    lateral_offset_m: float = 6.0
    mini_speed_mps: float = 0.12
    carrier_speed_mps: float = 0.06
    lead_delay_ms: int = 5000
    lead_distance_m: float = 2.0
    terminal_gap_m: float = 2.0
    terminal_gap_tolerance_m: float = 0.75
    terminal_lateral_tolerance_m: float = 1.0
    terminal_heading_tolerance_deg: float = 35.0
    terminal_confirm_samples: int = 3
    terminal_collision_guard_m: float = 0.75
    command_ttl_ms: int = 500
    command_duration_ms: int = 500
    state_timeout_ms: int = 750
    mission_timeout_ms: int = 120000
    plan_validity_ms: int = 120000


@dataclass(frozen=True)
class ChaseDecision:
    phase: ChasePhase
    remote_command: PlanCommand | None
    start_local_carrier: bool
    stop_local_carrier: bool
    abort: Abort | None
    lead_distance_m: float
    reason: str


@dataclass(frozen=True)
class TerminalGapMetrics:
    distance_m: float
    longitudinal_gap_m: float
    lateral_offset_m: float
    heading_error_rad: float


def terminal_gap_metrics(
    carrier: MiniState,
    mini: MiniState,
    *,
    expected_origin_id: int,
) -> TerminalGapMetrics:
    """Measure Carrier relative to the stopped Mini in shared field ENU."""

    required = int(HealthFlag.POSITION_VALID | HealthFlag.YAW_VALID | HealthFlag.ORIGIN_VALID)
    if expected_origin_id == 0:
        raise ValueError("terminal gap requires a nonzero shared origin")
    for label, state in (("Carrier", carrier), ("Mini", mini)):
        if state.origin_id != expected_origin_id:
            raise ValueError(f"{label} origin does not match the shared field")
        if state.health & required != required:
            raise ValueError(f"{label} pose is not valid in the shared field")
        if not all(math.isfinite(value) for value in (state.x_m, state.y_m, state.yaw_rad)):
            raise ValueError(f"{label} pose is not finite")

    dx = mini.x_m - carrier.x_m
    dy = mini.y_m - carrier.y_m
    forward_x = math.cos(mini.yaw_rad)
    forward_y = math.sin(mini.yaw_rad)
    return TerminalGapMetrics(
        distance_m=math.hypot(dx, dy),
        longitudinal_gap_m=forward_x * dx + forward_y * dy,
        lateral_offset_m=-forward_y * dx + forward_x * dy,
        heading_error_rad=math.atan2(
            math.sin(carrier.yaw_rad - mini.yaw_rad),
            math.cos(carrier.yaw_rad - mini.yaw_rad),
        ),
    )


def terminal_gap_is_reached(
    metrics: TerminalGapMetrics,
    config: StagedChaseConfig,
) -> bool:
    return bool(
        metrics.longitudinal_gap_m > 0.0
        and abs(metrics.longitudinal_gap_m - config.terminal_gap_m)
        <= config.terminal_gap_tolerance_m
        and abs(metrics.lateral_offset_m) <= config.terminal_lateral_tolerance_m
        and abs(math.degrees(metrics.heading_error_rad))
        <= config.terminal_heading_tolerance_deg
    )


def terminal_gap_is_unsafe(
    metrics: TerminalGapMetrics,
    config: StagedChaseConfig,
) -> bool:
    return metrics.distance_m < config.terminal_collision_guard_m


def validate_staged_chase_config(config: StagedChaseConfig) -> None:
    numeric = (
        config.straight_distance_m,
        config.turn_radius_m,
        config.lateral_offset_m,
        config.mini_speed_mps,
        config.carrier_speed_mps,
        config.lead_distance_m,
        config.terminal_gap_m,
        config.terminal_gap_tolerance_m,
        config.terminal_lateral_tolerance_m,
        config.terminal_heading_tolerance_deg,
        config.terminal_collision_guard_m,
    )
    if not all(math.isfinite(value) for value in numeric):
        raise ValueError("staged chase configuration must be finite")
    if not 0 < config.plan_id <= 0xFFFF:
        raise ValueError("plan_id must be within uint16 and nonzero")
    if not 1.0 <= config.straight_distance_m <= 10.0:
        raise ValueError("straight distance must be within [1, 10] m")
    if not 1.0 <= config.turn_radius_m <= 5.0:
        raise ValueError("turn radius must be within [1, 5] m")
    if abs(config.lateral_offset_m) > 2.0 * config.turn_radius_m + 1.0e-6:
        raise ValueError("lateral offset exceeds the S-bend geometry")
    if not 0.01 <= config.carrier_speed_mps <= config.mini_speed_mps <= 0.25:
        raise ValueError("Carrier must not be faster than Mini in stage one")
    if not 1000 <= config.lead_delay_ms <= 30000:
        raise ValueError("lead delay must be within [1, 30] seconds")
    if not 0.5 <= config.lead_distance_m <= 10.0:
        raise ValueError("lead distance must be within [0.5, 10] m")
    if not 1.0 <= config.terminal_gap_m <= 5.0:
        raise ValueError("terminal gap must be within [1, 5] m")
    if not 0.25 <= config.terminal_gap_tolerance_m <= 1.5:
        raise ValueError("terminal gap tolerance must be within [0.25, 1.5] m")
    if not 0.25 <= config.terminal_lateral_tolerance_m <= 2.0:
        raise ValueError("terminal lateral tolerance must be within [0.25, 2] m")
    if not 5.0 <= config.terminal_heading_tolerance_deg <= 60.0:
        raise ValueError("terminal heading tolerance must be within [5, 60] degrees")
    if not 2 <= config.terminal_confirm_samples <= 10:
        raise ValueError("terminal confirmation samples must be within [2, 10]")
    if not 0.25 <= config.terminal_collision_guard_m < config.terminal_gap_m:
        raise ValueError("terminal collision guard must be below the target gap")
    if not 200 <= config.command_ttl_ms <= 2000:
        raise ValueError("command TTL must be within [200, 2000] ms")
    if not 200 <= config.command_duration_ms <= 5000:
        raise ValueError("command duration must be within [200, 5000] ms")
    if not 250 <= config.state_timeout_ms <= 2000:
        raise ValueError("state timeout must be within [250, 2000] ms")
    if not 10000 <= config.mission_timeout_ms <= 180000:
        raise ValueError("mission timeout must be within [10, 180] seconds")
    if not config.mission_timeout_ms <= config.plan_validity_ms <= 180000:
        raise ValueError("plan validity must cover the bounded mission")


def build_staged_mission_plan(
    config: StagedChaseConfig,
    *,
    seq: int,
    sender_monotonic_ms: int,
) -> StagedMissionPlan:
    validate_staged_chase_config(config)
    return StagedMissionPlan(
        schema_version=1,
        plan_id=config.plan_id,
        seq=seq & U32_MASK,
        timestamp_ms=sender_monotonic_ms & U32_MASK,
        valid_until_ms=(sender_monotonic_ms + config.plan_validity_ms) & U32_MASK,
        lead_delay_ms=config.lead_delay_ms,
        lead_distance_m=config.lead_distance_m,
        lateral_offset_m=config.lateral_offset_m,
        straight_distance_m=config.straight_distance_m,
        turn_radius_m=config.turn_radius_m,
        mini_speed_mps=config.mini_speed_mps,
        carrier_speed_mps=config.carrier_speed_mps,
        flags=int(StagedMissionFlag.S_BEND_RETURN),
    )


class StagedChaseCoordinator:
    """Latch starts once and fail closed on stale Pair B MiniState."""

    def __init__(self, config: StagedChaseConfig = StagedChaseConfig()) -> None:
        validate_staged_chase_config(config)
        self.config = config
        self.phase = ChasePhase.HOLD
        self.authorized = False
        self.carrier_ready = False
        self.mini_state: MiniState | None = None
        self.mini_state_rx_ms: int | None = None
        self._last_mini_seq: int | None = None
        self._mini_start_xy: tuple[float, float] | None = None
        self._mini_start_ms: int | None = None
        self._mission_start_ms: int | None = None
        self._command_seq = 0
        self._abort_seq = 0
        self._carrier_start_emitted = False
        self._carrier_complete = False
        self._mini_complete = False
        self._mini_terminal_status_grace_start_ms: int | None = None
        self._abort_reason: AbortReason | None = None

    def accept_mini_state(self, state: MiniState, received_local_ms: int) -> bool:
        if state.vehicle_id != 1:
            return False
        if self._last_mini_seq is not None and not sequence_is_newer(
            state.seq, self._last_mini_seq
        ):
            return False
        self._last_mini_seq = state.seq
        self.mini_state = state
        self.mini_state_rx_ms = int(received_local_ms)
        return True

    def set_carrier_ready(self, ready: bool) -> None:
        self.carrier_ready = bool(ready)

    def authorize_start(self) -> None:
        self.authorized = True

    def mark_carrier_complete(self) -> None:
        self._carrier_complete = True

    def mark_mini_complete(self) -> None:
        self._mini_complete = True

    @property
    def mini_complete(self) -> bool:
        return self._mini_complete

    @property
    def carrier_complete(self) -> bool:
        return self._carrier_complete

    def request_abort(self, reason: AbortReason = AbortReason.OPERATOR) -> None:
        self._abort_reason = reason
        self.phase = ChasePhase.ABORTED

    def _mini_ready(self, now_ms: int) -> bool:
        return bool(
            self.mini_state is not None
            and self.mini_state_rx_ms is not None
            and 0 <= now_ms - self.mini_state_rx_ms <= self.config.state_timeout_ms
            and self.mini_state.origin_id == self.config.plan_id
            and (self.mini_state.health & REQUIRED_MINI_HEALTH)
            == REQUIRED_MINI_HEALTH
        )

    def _mini_terminal_status_candidate(self, now_ms: int) -> bool:
        """Accept only fresh shared pose while COMPLETE overtakes MiniState."""
        return bool(
            self.mini_state is not None
            and self.mini_state_rx_ms is not None
            and 0 <= now_ms - self.mini_state_rx_ms <= self.config.state_timeout_ms
            and self.mini_state.origin_id == self.config.plan_id
            and (self.mini_state.health & TERMINAL_MINI_HEALTH)
            == TERMINAL_MINI_HEALTH
            and all(
                math.isfinite(value)
                for value in (
                    self.mini_state.x_m,
                    self.mini_state.y_m,
                    self.mini_state.yaw_rad,
                )
            )
        )

    def _lead_distance(self) -> float:
        if self.mini_state is None or self._mini_start_xy is None:
            return 0.0
        return math.hypot(
            self.mini_state.x_m - self._mini_start_xy[0],
            self.mini_state.y_m - self._mini_start_xy[1],
        )

    def _remote_command(self, now_ms: int, phase: Phase) -> PlanCommand:
        speed = self.config.mini_speed_mps if phase == Phase.TRAJECTORY else 0.0
        command = PlanCommand(
            plan_id=self.config.plan_id,
            role=Role.MINI,
            phase=phase,
            seq=self._command_seq,
            timestamp_ms=now_ms & U32_MASK,
            valid_until_ms=(now_ms + self.config.command_ttl_ms) & U32_MASK,
            v_mps=speed,
            omega_radps=0.0,
            duration_ms=self.config.command_duration_ms,
            distance_m=self.config.straight_distance_m,
            max_speed_mps=self.config.mini_speed_mps,
            max_accel_mps2=0.30,
            flags=0,
        )
        self._command_seq = (self._command_seq + 1) & U32_MASK
        return command

    def _abort(self, now_ms: int, reason: AbortReason, text: str) -> ChaseDecision:
        self._abort_reason = reason
        self.phase = ChasePhase.ABORTED
        abort = Abort(
            source_role=Role.CARRIER,
            reason=reason,
            plan_id=self.config.plan_id,
            seq=self._abort_seq,
            timestamp_ms=now_ms & U32_MASK,
        )
        self._abort_seq = (self._abort_seq + 1) & U32_MASK
        return ChaseDecision(
            self.phase,
            self._remote_command(now_ms, Phase.STOP),
            False,
            True,
            abort,
            self._lead_distance(),
            text,
        )

    def step(self, now_ms: int) -> ChaseDecision:
        now_ms = int(now_ms)
        if self.phase == ChasePhase.ABORTED:
            return self._abort(
                now_ms,
                self._abort_reason or AbortReason.UNSPECIFIED,
                "abort_latched",
            )
        if self.phase == ChasePhase.COMPLETE:
            return ChaseDecision(
                self.phase,
                self._remote_command(now_ms, Phase.STOP),
                False,
                True,
                None,
                self._lead_distance(),
                "complete_hold",
            )
        if self.phase == ChasePhase.MINI_ACTIVE and self._mini_complete:
            return self._abort(
                now_ms,
                AbortReason.LOCAL_SAFETY,
                "mini_completed_before_carrier_start",
            )
        if self.phase == ChasePhase.BOTH_ACTIVE and self._mini_complete:
            if self._carrier_complete:
                self.phase = ChasePhase.COMPLETE
                return ChaseDecision(
                    self.phase,
                    self._remote_command(now_ms, Phase.STOP),
                    False,
                    True,
                    None,
                    self._lead_distance(),
                    "both_complete",
                )
            return ChaseDecision(
                self.phase,
                self._remote_command(now_ms, Phase.STOP),
                False,
                False,
                None,
                self._lead_distance(),
                "mini_complete_carrier_finishing",
            )
        mini_ready = self._mini_ready(now_ms)
        if mini_ready:
            self._mini_terminal_status_grace_start_ms = None
        elif (
            self.phase == ChasePhase.BOTH_ACTIVE
            and self._mini_terminal_status_candidate(now_ms)
        ):
            if self._mini_terminal_status_grace_start_ms is None:
                self._mini_terminal_status_grace_start_ms = now_ms
            grace_age_ms = now_ms - self._mini_terminal_status_grace_start_ms
            if grace_age_ms <= MINI_TERMINAL_STATUS_GRACE_MS:
                return ChaseDecision(
                    self.phase,
                    self._remote_command(now_ms, Phase.STOP),
                    False,
                    False,
                    None,
                    self._lead_distance(),
                    "awaiting_mini_terminal_status",
                )
        if not self.authorized or not self.carrier_ready or not mini_ready:
            if self.phase != ChasePhase.HOLD and not self.carrier_ready:
                return self._abort(
                    now_ms,
                    AbortReason.LOCAL_SAFETY,
                    "carrier_local_not_ready",
                )
            if self.phase != ChasePhase.HOLD and not mini_ready:
                state_stale = bool(
                    self.mini_state_rx_ms is None
                    or now_ms - self.mini_state_rx_ms > self.config.state_timeout_ms
                )
                return self._abort(
                    now_ms,
                    AbortReason.LINK_STALE if state_stale else AbortReason.STATE_INVALID,
                    "mini_state_stale" if state_stale else "mini_health_invalid",
                )
            return ChaseDecision(
                ChasePhase.HOLD,
                self._remote_command(now_ms, Phase.HOLD),
                False,
                False,
                None,
                self._lead_distance(),
                "waiting_for_authorized_ready_state",
            )

        if self.phase == ChasePhase.HOLD:
            assert self.mini_state is not None
            self.phase = ChasePhase.MINI_ACTIVE
            self._mini_start_xy = (self.mini_state.x_m, self.mini_state.y_m)
            self._mini_start_ms = now_ms
            self._mission_start_ms = now_ms

        assert self._mission_start_ms is not None
        if now_ms - self._mission_start_ms >= self.config.mission_timeout_ms:
            return self._abort(now_ms, AbortReason.LOCAL_SAFETY, "mission_timeout")

        start_local = False
        if self.phase == ChasePhase.MINI_ACTIVE:
            assert self._mini_start_ms is not None
            delay_ready = now_ms - self._mini_start_ms >= self.config.lead_delay_ms
            distance_ready = self._lead_distance() >= self.config.lead_distance_m
            if delay_ready and distance_ready:
                self.phase = ChasePhase.BOTH_ACTIVE
                if not self._carrier_start_emitted:
                    start_local = True
                    self._carrier_start_emitted = True

        return ChaseDecision(
            self.phase,
            self._remote_command(now_ms, Phase.TRAJECTORY),
            start_local,
            False,
            None,
            self._lead_distance(),
            "mini_leading" if self.phase == ChasePhase.MINI_ACTIVE else "both_tracking",
        )
