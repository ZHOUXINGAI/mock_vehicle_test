from __future__ import annotations

import math
import sys
import unittest
from dataclasses import replace
from pathlib import Path

REPO_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_DIR))

from src.lr24_compact_protocol import HealthFlag, MiniState
from src.pairb_dual_trajectory import (
    FormationRegistration,
    StartFrameAlignment,
    build_display_plan,
    state_has_pose,
    states_share_expected_origin,
)
from src.pairb_staged_chase import StagedChaseConfig
from src.rover_rviz_trajectory import TracePoint


def state(
    x_m: float,
    y_m: float,
    yaw_rad: float,
    health: int | None = None,
    origin_id: int = 0,
) -> MiniState:
    return MiniState(
        vehicle_id=1,
        seq=0,
        timestamp_ms=0,
        x_m=x_m,
        y_m=y_m,
        vx_mps=0.0,
        vy_mps=0.0,
        yaw_rad=yaw_rad,
        omega_radps=0.0,
        health=(
            int(HealthFlag.POSITION_VALID | HealthFlag.YAW_VALID)
            if health is None
            else health
        ),
        origin_id=origin_id,
    )


class PairBDualTrajectoryTest(unittest.TestCase):
    def test_known_half_meter_lead_registers_position_and_yaw_bias(self) -> None:
        carrier = state(10.0, 20.0, math.pi / 2.0, origin_id=9)
        mini_raw = state(13.0, 17.0, 0.0, origin_id=9)
        registration = FormationRegistration.from_states(
            carrier,
            mini_raw,
            mini_ahead_m=0.5,
        )
        self.assertAlmostEqual(registration.mini_registered_start.x_m, 10.0)
        self.assertAlmostEqual(registration.mini_registered_start.y_m, 20.5)
        moved = replace(
            mini_raw,
            x_m=15.0,
            y_m=18.0,
            vx_mps=2.0,
            vy_mps=-0.5,
            yaw_rad=0.2,
        )
        transformed = registration.transform_state(moved)
        # Shared FieldOrigin positions and velocities stay on the common ENU
        # axes; only the fixed initial GNSS offset is removed.
        self.assertAlmostEqual(transformed.x_m, 12.0, places=6)
        self.assertAlmostEqual(transformed.y_m, 21.5, places=6)
        self.assertAlmostEqual(transformed.vx_mps, 2.0, places=6)
        self.assertAlmostEqual(transformed.vy_mps, -0.5, places=6)
        self.assertAlmostEqual(transformed.yaw_rad, math.pi / 2.0 + 0.2)

    def test_shared_live_display_waits_for_exact_plan_origin(self) -> None:
        carrier = state(0.0, 0.0, 0.0, origin_id=8111)
        mini = state(1.0, 0.0, 0.0, origin_id=8111)
        self.assertTrue(states_share_expected_origin(carrier, mini, 8111))
        self.assertFalse(states_share_expected_origin(carrier, mini, 8110))
        self.assertFalse(
            states_share_expected_origin(
                carrier,
                state(1.0, 0.0, 0.0, origin_id=0),
                8111,
            )
        )

    def test_requires_position_and_yaw_health(self) -> None:
        self.assertTrue(state_has_pose(state(0.0, 0.0, 0.0)))
        self.assertFalse(
            state_has_pose(state(0.0, 0.0, 0.0, int(HealthFlag.POSITION_VALID)))
        )

    def test_mini_start_maps_exactly_to_carrier_start(self) -> None:
        alignment = StartFrameAlignment.from_states(
            state(10.0, -3.0, math.pi / 2.0),
            state(-7.0, 4.0, -math.pi / 4.0),
        )
        displayed = alignment.mini_to_display(state(-7.0, 4.0, -math.pi / 4.0))
        self.assertAlmostEqual(displayed.x_m, 10.0)
        self.assertAlmostEqual(displayed.y_m, -3.0)
        self.assertAlmostEqual(displayed.yaw_rad, math.pi / 2.0)

    def test_mini_motion_is_rotated_into_carrier_start_heading(self) -> None:
        alignment = StartFrameAlignment.from_states(
            state(2.0, 3.0, math.pi / 2.0),
            state(10.0, 20.0, 0.0),
        )
        displayed = alignment.mini_to_display(state(12.0, 21.0, 0.2))
        self.assertAlmostEqual(displayed.x_m, 1.0, places=6)
        self.assertAlmostEqual(displayed.y_m, 5.0, places=6)
        self.assertAlmostEqual(displayed.yaw_rad, math.pi / 2.0 + 0.2, places=6)

    def test_both_display_plans_share_geometry(self) -> None:
        config = StagedChaseConfig()
        start = TracePoint(2.0, 3.0, 0.4)
        carrier = build_display_plan(config, start, config.carrier_speed_mps)
        mini = build_display_plan(config, start, config.mini_speed_mps)
        self.assertEqual(len(carrier.points), len(mini.points))
        self.assertAlmostEqual(carrier.length_m, mini.length_m)
        self.assertEqual(
            [(point.x_m, point.y_m) for point in carrier.points],
            [(point.x_m, point.y_m) for point in mini.points],
        )


if __name__ == "__main__":
    unittest.main()
