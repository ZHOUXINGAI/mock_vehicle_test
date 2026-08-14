from __future__ import annotations

import math
import unittest
from pathlib import Path

from src.lr24_compact_protocol import (
    CorridorPlanCompact,
    HealthFlag,
    MiniState,
    Phase,
    PlanCommand,
    Role,
)
from src.orin2_carrier_leader_core import (
    FULL_EXECUTOR_HEALTH,
    NO_MOTION_HEALTH,
    ORIN1_MINI_SYSTEM_ID,
    ORIN2_CARRIER_SYSTEM_ID,
    Orin2CarrierLeaderCore,
    SharedVehicleState,
    build_parallel_straight_plan,
)


class ParallelStraightPlanTests(unittest.TestCase):
    def test_first_stage_preserves_carrier_ahead_gap(self) -> None:
        plan = build_parallel_straight_plan()
        self.assertAlmostEqual(plan.initial_front_gap_m, 1.5)
        self.assertAlmostEqual(plan.carrier_path[-1].x_m, 4.5)
        self.assertAlmostEqual(plan.mini_path[-1].x_m, 3.0)
        for carrier, mini in zip(plan.carrier_path, plan.mini_path):
            self.assertAlmostEqual(carrier.x_m - mini.x_m, 1.5)
            self.assertAlmostEqual(carrier.y_m, mini.y_m)

    def test_first_stage_rejects_carrier_behind_or_off_corridor(self) -> None:
        with self.assertRaisesRegex(ValueError, "ahead"):
            build_parallel_straight_plan(carrier_start=(-0.1, 0.0))
        with self.assertRaisesRegex(ValueError, "lateral"):
            build_parallel_straight_plan(carrier_start=(1.5, 0.3))


@unittest.skipUnless(
    Path("/home/seeed/easydocking/src/ground_docking_leader.py").is_file(),
    "local EasyDocking source checkout is unavailable",
)
class ReversedCarrierLeaderTests(unittest.TestCase):
    def setUp(self) -> None:
        self.core = Orin2CarrierLeaderCore(
            config_overrides={"plan_validity_ms": 60_000}
        )
        self.core.start_attempt(41, 0)
        self.now_ms = 0
        self.sender_ms = 100_000
        self.sequence = 0

    def feed(self, phase_rad: float):
        speed = self.core.config.mini_speed_mps
        radius = self.core.config.orbit_radius_m
        mini = MiniState(
            vehicle_id=ORIN1_MINI_SYSTEM_ID,
            seq=self.sequence,
            timestamp_ms=self.sender_ms,
            x_m=radius * math.cos(phase_rad),
            y_m=radius * math.sin(phase_rad),
            vx_mps=-speed * math.sin(phase_rad),
            vy_mps=speed * math.cos(phase_rad),
            yaw_rad=phase_rad + math.pi / 2.0,
            omega_radps=speed / radius,
            health=FULL_EXECUTOR_HEALTH,
            origin_id=self.core.config.origin_id,
        )
        carrier = SharedVehicleState(
            vehicle_id=ORIN2_CARRIER_SYSTEM_ID,
            sequence=self.sequence,
            sender_monotonic_ms=self.sender_ms,
            received_local_ms=self.now_ms,
            x_m=-7.0,
            y_m=-6.0,
            vx_mps=0.0,
            vy_mps=0.0,
            yaw_rad=0.0,
            origin_id=self.core.config.origin_id,
        )
        self.core.accept_local_carrier_state(carrier)
        self.core.accept_pairb_mini_state(mini, received_local_ms=self.now_ms)
        output = self.core.step(
            now_local_ms=self.now_ms,
            sender_monotonic_ms=self.sender_ms,
        )
        self.sequence += 1
        self.now_ms += 100
        self.sender_ms += 100
        return output

    def test_role_map_and_pairb_plan_are_end_to_end(self) -> None:
        self.assertEqual(self.core.config.carrier_vehicle_id, 2)
        self.assertEqual(self.core.config.mini_vehicle_id, 1)
        phase = 0.0
        angular_step = (
            self.core.config.mini_speed_mps
            / self.core.config.orbit_radius_m
            * 0.1
        )
        output = self.feed(phase)
        self.assertEqual(output.local_carrier_compact.role, Role.CARRIER)
        self.assertEqual(output.local_carrier_compact.phase, Phase.HOLD)
        self.assertEqual(output.remote_mini_command.role, Role.MINI)
        self.assertEqual(output.remote_mini_command.phase, Phase.ORBIT)
        self.assertIsNone(output.corridor_plan)

        for _ in range(400):
            phase += angular_step
            output = self.feed(phase)
            if output.corridor_plan is not None:
                break
        self.assertIsNotNone(output.corridor_plan)
        self.assertGreaterEqual(output.stable_orbit_laps, 1.0)
        command_roundtrip = PlanCommand.decode(output.remote_mini_command.encode())
        plan_roundtrip = CorridorPlanCompact.decode(output.corridor_plan.encode())
        self.assertEqual(command_roundtrip.role, Role.MINI)
        self.assertEqual(plan_roundtrip.origin_id, self.core.config.origin_id)

    def test_wrong_physical_ids_and_missing_production_health_fail_closed(self) -> None:
        wrong = MiniState(
            vehicle_id=2,
            seq=1,
            timestamp_ms=1,
            x_m=4.5,
            y_m=0.0,
            vx_mps=0.0,
            vy_mps=0.9,
            yaw_rad=math.pi / 2.0,
            omega_radps=0.2,
            health=NO_MOTION_HEALTH,
            origin_id=1,
        )
        with self.assertRaisesRegex(ValueError, "MAV_SYS_ID 1"):
            self.core.accept_pairb_mini_state(wrong, received_local_ms=0)

        production = Orin2CarrierLeaderCore(production_ready=True)
        production.start_attempt(42, 0)
        missing_executor = MiniState(
            vehicle_id=1,
            seq=1,
            timestamp_ms=1,
            x_m=4.5,
            y_m=0.0,
            vx_mps=0.0,
            vy_mps=0.9,
            yaw_rad=math.pi / 2.0,
            omega_radps=0.2,
            health=int(
                HealthFlag.POSITION_VALID
                | HealthFlag.VELOCITY_VALID
                | HealthFlag.YAW_VALID
                | HealthFlag.ORIGIN_VALID
            ),
            origin_id=1,
        )
        self.assertTrue(
            production.required_mini_health & int(HealthFlag.EXECUTOR_READY)
        )
        self.assertFalse(
            production.accept_pairb_mini_state(
                missing_executor, received_local_ms=0
            )
        )


if __name__ == "__main__":
    unittest.main()
