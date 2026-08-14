#!/usr/bin/env python3

"""Dual-rover Pair B trajectory visualization and artifact recording."""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, replace
from pathlib import Path

try:
    from lr24_compact_protocol import HealthFlag, MiniState
    from orin2_trajectory_tracker import build_s_bend_return_trajectory
    from rover_rviz_trajectory import RvizTrajectoryPublisher, TracePoint
except ImportError:
    from src.lr24_compact_protocol import HealthFlag, MiniState
    from src.orin2_trajectory_tracker import build_s_bend_return_trajectory
    from src.rover_rviz_trajectory import RvizTrajectoryPublisher, TracePoint


POSE_HEALTH = HealthFlag.POSITION_VALID | HealthFlag.YAW_VALID


def wrap_pi(angle_rad: float) -> float:
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def state_has_pose(state: MiniState) -> bool:
    return bool(
        HealthFlag(state.health) & POSE_HEALTH == POSE_HEALTH
        and all(math.isfinite(value) for value in (state.x_m, state.y_m, state.yaw_rad))
    )


def states_share_expected_origin(
    carrier_state: MiniState,
    mini_state: MiniState,
    expected_origin_id: int,
) -> bool:
    return bool(
        expected_origin_id > 0
        and carrier_state.origin_id == expected_origin_id
        and mini_state.origin_id == expected_origin_id
    )


@dataclass(frozen=True)
class StartFrameAlignment:
    """Map Mini's local start frame onto Carrier's local start frame."""

    carrier_start: TracePoint
    mini_start: TracePoint

    @staticmethod
    def from_states(carrier: MiniState, mini: MiniState) -> "StartFrameAlignment":
        if not state_has_pose(carrier) or not state_has_pose(mini):
            raise ValueError("both vehicles need valid position and yaw")
        return StartFrameAlignment(
            carrier_start=TracePoint(carrier.x_m, carrier.y_m, carrier.yaw_rad),
            mini_start=TracePoint(mini.x_m, mini.y_m, mini.yaw_rad),
        )

    def mini_to_display(self, state: MiniState) -> TracePoint:
        if not state_has_pose(state):
            raise ValueError("Mini state needs valid position and yaw")
        dx = state.x_m - self.mini_start.x_m
        dy = state.y_m - self.mini_start.y_m
        mini_cos = math.cos(self.mini_start.yaw_rad)
        mini_sin = math.sin(self.mini_start.yaw_rad)
        forward_m = mini_cos * dx + mini_sin * dy
        left_m = -mini_sin * dx + mini_cos * dy
        carrier_cos = math.cos(self.carrier_start.yaw_rad)
        carrier_sin = math.sin(self.carrier_start.yaw_rad)
        return TracePoint(
            self.carrier_start.x_m + carrier_cos * forward_m - carrier_sin * left_m,
            self.carrier_start.y_m + carrier_sin * forward_m + carrier_cos * left_m,
            wrap_pi(
                self.carrier_start.yaw_rad
                + state.yaw_rad
                - self.mini_start.yaw_rad
            ),
        )


@dataclass(frozen=True)
class FormationRegistration:
    """Register Mini GPS bias and heading bias in the shared Carrier ENU."""

    carrier_start: TracePoint
    mini_raw_start: TracePoint
    mini_registered_start: TracePoint

    @staticmethod
    def from_states(
        carrier: MiniState,
        mini: MiniState,
        *,
        mini_ahead_m: float,
        mini_left_m: float = 0.0,
    ) -> "FormationRegistration":
        if not state_has_pose(carrier) or not state_has_pose(mini):
            raise ValueError("formation registration needs valid position and yaw")
        if not all(math.isfinite(value) for value in (mini_ahead_m, mini_left_m)):
            raise ValueError("formation offset must be finite")
        carrier_start = TracePoint(carrier.x_m, carrier.y_m, carrier.yaw_rad)
        cosine = math.cos(carrier.yaw_rad)
        sine = math.sin(carrier.yaw_rad)
        registered_start = TracePoint(
            carrier.x_m + cosine * mini_ahead_m - sine * mini_left_m,
            carrier.y_m + sine * mini_ahead_m + cosine * mini_left_m,
            carrier.yaw_rad,
        )
        return FormationRegistration(
            carrier_start=carrier_start,
            mini_raw_start=TracePoint(mini.x_m, mini.y_m, mini.yaw_rad),
            mini_registered_start=registered_start,
        )

    def transform_state(self, state: MiniState) -> MiniState:
        if not state_has_pose(state):
            raise ValueError("Mini state needs valid position and yaw")
        # Both GPS positions already use the same FieldOrigin ENU axes. Correct
        # only the independent GNSS offset; rotating ENU displacement by the
        # magnetometer yaw delta would create a growing cross-track error.
        return replace(
            state,
            x_m=self.mini_registered_start.x_m + state.x_m - self.mini_raw_start.x_m,
            y_m=self.mini_registered_start.y_m + state.y_m - self.mini_raw_start.y_m,
            yaw_rad=wrap_pi(
                self.carrier_start.yaw_rad
                + state.yaw_rad
                - self.mini_raw_start.yaw_rad
            ),
        )

def build_display_plan(config: object, start: TracePoint, speed_mps: float):
    return build_s_bend_return_trajectory(
        start.x_m,
        start.y_m,
        start.yaw_rad,
        float(config.straight_distance_m),
        float(config.turn_radius_m),
        speed_mps,
        speed_mps,
        0.15,
    )


class PairBDualTrajectoryRecorder:
    """Publish and persist two normalized planned/actual trajectory pairs."""

    def __init__(
        self,
        node: object,
        config: object,
        artifact_dir: Path,
        *,
        require_shared_origin: bool = False,
    ) -> None:
        self.config = config
        self.artifact_dir = artifact_dir
        self.require_shared_origin = require_shared_origin
        self.carrier = RvizTrajectoryPublisher(node, "/pairb/carrier")
        self.mini = RvizTrajectoryPublisher(node, "/pairb/mini")
        # Clear volatile actual paths immediately so RViz cannot retain the
        # previous plan's last trace while this run waits for both vehicles.
        self.carrier.start_actual("map")
        self.mini.start_actual("map")
        self.alignment: StartFrameAlignment | None = None
        self.shared_origin_id = 0

    @property
    def initialized(self) -> bool:
        return self.alignment is not None

    def observe(self, carrier_state: MiniState, mini_state: MiniState) -> bool:
        if not state_has_pose(carrier_state) or not state_has_pose(mini_state):
            return False
        if self.alignment is None:
            shared_origin = states_share_expected_origin(
                carrier_state,
                mini_state,
                int(self.config.plan_id),
            )
            if self.require_shared_origin and not shared_origin:
                return False
            self.alignment = StartFrameAlignment.from_states(carrier_state, mini_state)
            if shared_origin:
                self.shared_origin_id = carrier_state.origin_id
            carrier_plan = build_display_plan(
                self.config,
                self.alignment.carrier_start,
                float(self.config.carrier_speed_mps),
            )
            mini_plan = build_display_plan(
                self.config,
                (
                    self.alignment.mini_start
                    if self.shared_origin_id
                    else self.alignment.carrier_start
                ),
                float(self.config.mini_speed_mps),
            )
            self.carrier.set_plan(carrier_plan.points, "map")
            self.mini.set_plan(mini_plan.points, "map")

        self.carrier.update_actual(
            carrier_state.x_m,
            carrier_state.y_m,
            carrier_state.yaw_rad,
        )
        if self.shared_origin_id:
            self.mini.update_actual(
                mini_state.x_m,
                mini_state.y_m,
                mini_state.yaw_rad,
            )
        else:
            self.mini.update_actual(*self._mini_display_tuple(mini_state))
        return True

    def _mini_display_tuple(self, state: MiniState) -> tuple[float, float, float]:
        assert self.alignment is not None
        point = self.alignment.mini_to_display(state)
        return point.x_m, point.y_m, point.yaw_rad

    def write_artifacts(self) -> Path | None:
        if self.alignment is None:
            return None
        carrier_paths = self.carrier.write_artifacts(self.artifact_dir / "carrier")
        mini_paths = self.mini.write_artifacts(self.artifact_dir / "mini")
        metadata_path = self.artifact_dir / "alignment.json"
        metadata_path.parent.mkdir(parents=True, exist_ok=True)
        metadata_path.write_text(
            json.dumps(
                {
                    "frame": (
                        "shared_field_enu" if self.shared_origin_id else "carrier_local_map"
                    ),
                    "origin_id": self.shared_origin_id,
                    "alignment": (
                        "shared Carrier-GPS FieldOrigin"
                        if self.shared_origin_id
                        else "independent starts translated and yaw-aligned"
                    ),
                    "physical_relative_position_valid": bool(self.shared_origin_id),
                    "carrier_start": self.alignment.carrier_start.__dict__,
                    "mini_local_start": self.alignment.mini_start.__dict__,
                    "carrier_csv": [str(path) for path in carrier_paths],
                    "mini_csv": [str(path) for path in mini_paths],
                },
                indent=2,
                sort_keys=True,
            )
            + "\n",
            encoding="ascii",
        )
        return self.artifact_dir
