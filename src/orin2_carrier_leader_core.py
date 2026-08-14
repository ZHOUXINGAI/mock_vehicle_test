#!/usr/bin/env python3

"""Pure coordination core for Orin2-as-Carrier and Orin1-as-Mini."""

from __future__ import annotations

import importlib
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

try:
    from src.easydocking_pairb_adapter import (
        adapt_ground_corridor_plan,
        adapt_ground_plan_command,
    )
    from src.lr24_compact_protocol import HealthFlag, MiniState, PlanFlag, Role
except ModuleNotFoundError:  # Direct execution with src/ on PYTHONPATH.
    from easydocking_pairb_adapter import (
        adapt_ground_corridor_plan,
        adapt_ground_plan_command,
    )
    from lr24_compact_protocol import HealthFlag, MiniState, PlanFlag, Role


DEFAULT_EASYDOCKING_SRC = Path("/home/seeed/easydocking/src")
ORIN2_CARRIER_SYSTEM_ID = 2
ORIN1_MINI_SYSTEM_ID = 1
FULL_EXECUTOR_HEALTH = int(
    HealthFlag.POSITION_VALID
    | HealthFlag.VELOCITY_VALID
    | HealthFlag.YAW_VALID
    | HealthFlag.PX4_CONNECTED
    | HealthFlag.RC_STOP_READY
    | HealthFlag.EXECUTOR_READY
    | HealthFlag.ORIGIN_VALID
)
NO_MOTION_HEALTH = int(
    HealthFlag.POSITION_VALID
    | HealthFlag.VELOCITY_VALID
    | HealthFlag.YAW_VALID
    | HealthFlag.ORIGIN_VALID
)


@dataclass(frozen=True)
class SharedVehicleState:
    vehicle_id: int
    sequence: int
    sender_monotonic_ms: int
    received_local_ms: int
    x_m: float
    y_m: float
    vx_mps: float
    vy_mps: float
    yaw_rad: float
    frame_id: str = "field_enu"
    origin_id: int = 1
    health_ok: bool = True


@dataclass(frozen=True)
class PairBLeaderOutput:
    phase: str
    local_carrier_command: Any
    local_carrier_compact: Any
    remote_mini_command: Any
    corridor_plan: Any | None
    stable_orbit_laps: float
    abort_reason: str | None


@dataclass(frozen=True)
class PlannedPoint:
    x_m: float
    y_m: float


@dataclass(frozen=True)
class ParallelStraightPlan:
    """First two-rover test: preserve a Carrier-ahead gap on one line."""

    plan_id: int
    frame_id: str
    origin_id: int
    speed_mps: float
    distance_m: float
    initial_front_gap_m: float
    carrier_path: tuple[PlannedPoint, ...]
    mini_path: tuple[PlannedPoint, ...]


def _finite_pair(name: str, value: tuple[float, float]) -> None:
    if len(value) != 2 or not all(math.isfinite(component) for component in value):
        raise ValueError(f"{name} must contain two finite values")


def build_parallel_straight_plan(
    *,
    plan_id: int = 1,
    frame_id: str = "field_enu",
    origin_id: int = 1,
    carrier_start: tuple[float, float] = (1.5, 0.0),
    mini_start: tuple[float, float] = (0.0, 0.0),
    heading_rad: float = 0.0,
    distance_m: float = 3.0,
    speed_mps: float = 0.05,
    spacing_m: float = 0.10,
    maximum_initial_lateral_error_m: float = 0.20,
) -> ParallelStraightPlan:
    """Build the safest useful first coordinated-motion geometry.

    Both rovers receive the same finite displacement and speed.  Carrier starts
    ahead on the same axis, so the plan tests coordination without gap closing.
    """

    _finite_pair("carrier_start", carrier_start)
    _finite_pair("mini_start", mini_start)
    scalars = (heading_rad, distance_m, speed_mps, spacing_m)
    if not all(math.isfinite(value) for value in scalars):
        raise ValueError("parallel plan values must be finite")
    if plan_id <= 0 or origin_id <= 0 or not frame_id:
        raise ValueError("positive plan/origin IDs and frame_id are required")
    if distance_m <= 0.0 or speed_mps <= 0.0 or spacing_m <= 0.0:
        raise ValueError("distance, speed and spacing must be positive")
    direction = (math.cos(heading_rad), math.sin(heading_rad))
    lateral = (-direction[1], direction[0])
    relative = (
        carrier_start[0] - mini_start[0],
        carrier_start[1] - mini_start[1],
    )
    front_gap = relative[0] * direction[0] + relative[1] * direction[1]
    lateral_error = relative[0] * lateral[0] + relative[1] * lateral[1]
    if front_gap <= 0.0:
        raise ValueError("Carrier must start ahead of Mini")
    if abs(lateral_error) > maximum_initial_lateral_error_m:
        raise ValueError("initial lateral separation exceeds shared-corridor limit")

    steps = max(1, int(math.ceil(distance_m / spacing_m)))

    def sample(start: tuple[float, float]) -> tuple[PlannedPoint, ...]:
        points = []
        for index in range(steps + 1):
            along = distance_m * index / steps
            points.append(
                PlannedPoint(
                    start[0] + along * direction[0],
                    start[1] + along * direction[1],
                )
            )
        return tuple(points)

    return ParallelStraightPlan(
        plan_id=plan_id,
        frame_id=frame_id,
        origin_id=origin_id,
        speed_mps=speed_mps,
        distance_m=distance_m,
        initial_front_gap_m=front_gap,
        carrier_path=sample(carrier_start),
        mini_path=sample(mini_start),
    )


def _load_easydocking(source_dir: Path) -> tuple[Any, Any]:
    source_dir = source_dir.expanduser().resolve()
    if not (source_dir / "ground_docking_leader.py").is_file():
        raise FileNotFoundError(f"EasyDocking source unavailable: {source_dir}")
    source_text = str(source_dir)
    if source_text not in sys.path:
        sys.path.insert(0, source_text)
    geometry = importlib.import_module("ground_corridor_geometry")
    leader = importlib.import_module("ground_docking_leader")
    return geometry, leader


class Orin2CarrierLeaderCore:
    """Transport-free EasyDocking leader with explicit physical role IDs."""

    def __init__(
        self,
        *,
        easydocking_src: Path = DEFAULT_EASYDOCKING_SRC,
        production_ready: bool = False,
        config_overrides: dict[str, Any] | None = None,
    ) -> None:
        _geometry, leader_module = _load_easydocking(easydocking_src)
        config_values: dict[str, Any] = {
            "carrier_vehicle_id": ORIN2_CARRIER_SYSTEM_ID,
            "mini_vehicle_id": ORIN1_MINI_SYSTEM_ID,
        }
        config_values.update(config_overrides or {})
        self.config = leader_module.LeaderConfig(**config_values)
        if (
            self.config.carrier_vehicle_id != ORIN2_CARRIER_SYSTEM_ID
            or self.config.mini_vehicle_id != ORIN1_MINI_SYSTEM_ID
        ):
            raise ValueError("Orin2 Carrier role map must remain system 2 -> system 1")
        self._vehicle_state_type = leader_module.VehicleState
        self.leader = leader_module.GroundDockingLeader(self.config)
        self.required_mini_health = (
            FULL_EXECUTOR_HEALTH if production_ready else NO_MOTION_HEALTH
        )

    def start_attempt(self, attempt_id: int, now_local_ms: int) -> None:
        self.leader.start_attempt(attempt_id=attempt_id, now_local_ms=now_local_ms)

    def _easy_state(self, state: SharedVehicleState):
        return self._vehicle_state_type(
            vehicle_id=state.vehicle_id,
            sequence=state.sequence,
            sender_monotonic_ms=state.sender_monotonic_ms,
            received_local_ms=state.received_local_ms,
            frame_id=state.frame_id,
            origin_id=state.origin_id,
            position=(state.x_m, state.y_m),
            velocity=(state.vx_mps, state.vy_mps),
            yaw_rad=state.yaw_rad,
            health_ok=state.health_ok,
        )

    def accept_local_carrier_state(self, state: SharedVehicleState) -> None:
        if state.vehicle_id != ORIN2_CARRIER_SYSTEM_ID:
            raise ValueError("local Carrier state must come from MAV_SYS_ID 2")
        self.leader.accept_carrier_state(self._easy_state(state))

    def accept_pairb_mini_state(
        self,
        state: MiniState,
        *,
        received_local_ms: int,
    ) -> bool:
        if state.vehicle_id != ORIN1_MINI_SYSTEM_ID:
            raise ValueError("Pair B MiniState must come from MAV_SYS_ID 1")
        health_ok = (
            state.health & self.required_mini_health
        ) == self.required_mini_health
        shared = SharedVehicleState(
            vehicle_id=state.vehicle_id,
            sequence=state.seq,
            sender_monotonic_ms=state.timestamp_ms,
            received_local_ms=received_local_ms,
            x_m=state.x_m,
            y_m=state.y_m,
            vx_mps=state.vx_mps,
            vy_mps=state.vy_mps,
            yaw_rad=state.yaw_rad,
            frame_id=self.config.frame_id,
            origin_id=state.origin_id,
            health_ok=health_ok,
        )
        return bool(self.leader.accept_mini_state(self._easy_state(shared)))

    def step(
        self,
        *,
        now_local_ms: int,
        sender_monotonic_ms: int,
    ) -> PairBLeaderOutput:
        snapshot = self.leader.step(
            now_local_ms=now_local_ms,
            sender_monotonic_ms=sender_monotonic_ms,
        )
        local_compact = adapt_ground_plan_command(
            snapshot.carrier_command,
            expected_frame_id=self.config.frame_id,
            expected_origin_id=self.config.origin_id,
            expected_role=Role.CARRIER,
        )
        remote_compact = adapt_ground_plan_command(
            snapshot.mini_command,
            expected_frame_id=self.config.frame_id,
            expected_origin_id=self.config.origin_id,
            expected_role=Role.MINI,
        )
        corridor = None
        if snapshot.plan is not None:
            corridor = adapt_ground_corridor_plan(
                snapshot.plan,
                expected_frame_id=self.config.frame_id,
                expected_origin_id=self.config.origin_id,
                one_orbit_complete=(
                    snapshot.stable_orbit_laps
                    >= self.config.required_stable_orbit_laps
                ),
            )
            if not corridor.flags & int(PlanFlag.ONE_ORBIT_COMPLETE):
                raise RuntimeError("qualified plan omitted one-orbit proof")
        return PairBLeaderOutput(
            phase=snapshot.phase.value,
            local_carrier_command=snapshot.carrier_command,
            local_carrier_compact=local_compact,
            remote_mini_command=remote_compact,
            corridor_plan=corridor,
            stable_orbit_laps=snapshot.stable_orbit_laps,
            abort_reason=snapshot.abort_reason,
        )
