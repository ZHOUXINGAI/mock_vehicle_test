#!/usr/bin/env python3

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path


REPO_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_DIR / "src"))

from lr24_compact_protocol import FieldOrigin, HealthFlag
from mavros_mini_state_source import (
    AngularRateSample,
    GlobalPositionSample,
    MiniStateAccumulator,
    PoseSample,
    StateSample,
    VelocitySample,
    quaternion_yaw,
)


class MiniStateAccumulatorTest(unittest.TestCase):
    def test_fresh_live_samples_set_telemetry_bits_but_not_execution_bits(self) -> None:
        source = MiniStateAccumulator(sample_timeout_sec=2.0)
        source.state = StateSample(True, False, "MANUAL", True, 10.0)
        source.pose = PoseSample(1.2, -0.4, 0.0, 0.0, math.sin(0.25), math.cos(0.25), 10.1)
        source.velocity = VelocitySample(0.3, -0.1, 10.1)
        source.angular_rate = AngularRateSample(0.2, 10.1)

        state = source.build(vehicle_id=1, seq=7, timestamp_ms=1234, now_mono=10.2)

        expected = (
            HealthFlag.POSITION_VALID
            | HealthFlag.VELOCITY_VALID
            | HealthFlag.YAW_VALID
            | HealthFlag.PX4_CONNECTED
            | HealthFlag.DISARMED
            | HealthFlag.MANUAL_INPUT
        )
        self.assertEqual(state.health, int(expected))
        self.assertAlmostEqual(state.yaw_rad, 0.5)
        self.assertEqual(state.origin_id, 0)
        self.assertFalse(state.health & int(HealthFlag.ORIGIN_VALID))
        self.assertFalse(state.health & int(HealthFlag.RC_STOP_READY))
        self.assertFalse(state.health & int(HealthFlag.EXECUTOR_READY))

    def test_arm_state_is_visible_in_compact_health_without_granting_execution(self) -> None:
        source = MiniStateAccumulator(sample_timeout_sec=2.0)
        source.state = StateSample(True, True, "OFFBOARD", False, 10.0)

        state = source.build(vehicle_id=1, seq=1, timestamp_ms=100, now_mono=10.1)

        self.assertTrue(state.health & int(HealthFlag.PX4_CONNECTED))
        self.assertFalse(state.health & int(HealthFlag.DISARMED))
        self.assertFalse(state.health & int(HealthFlag.MANUAL_INPUT))
        self.assertFalse(state.health & int(HealthFlag.EXECUTOR_READY))

    def test_stale_and_nonfinite_samples_clear_bits_and_values(self) -> None:
        source = MiniStateAccumulator(sample_timeout_sec=2.0)
        source.state = StateSample(True, False, "MANUAL", True, 1.0)
        source.pose = PoseSample(math.nan, 2.0, 0.0, 0.0, 0.0, 1.0, 4.5)
        source.velocity = VelocitySample(1.0, 2.0, 1.0)
        source.angular_rate = AngularRateSample(math.inf, 4.5)

        state = source.build(vehicle_id=1, seq=0, timestamp_ms=0, now_mono=5.0)

        self.assertEqual(state.health, int(HealthFlag.YAW_VALID))
        self.assertEqual((state.x_m, state.y_m), (0.0, 0.0))
        self.assertEqual((state.vx_mps, state.vy_mps), (0.0, 0.0))
        self.assertEqual(state.omega_radps, 0.0)

    def test_carrier_field_origin_converts_fresh_gps_to_shared_enu(self) -> None:
        source = MiniStateAccumulator(sample_timeout_sec=2.0)
        origin = FieldOrigin(42, 0, 1000, 22.0, 114.0, 10.0)
        source.set_field_origin(origin)
        source.pose = PoseSample(
            99.0,
            99.0,
            0.0,
            0.0,
            0.0,
            1.0,
            10.0,
        )
        source.global_position = GlobalPositionSample(
            22.0,
            114.00001,
            10.0,
            0,
            10.0,
        )

        state = source.build(vehicle_id=1, seq=0, timestamp_ms=0, now_mono=10.1)

        self.assertEqual(state.origin_id, 42)
        self.assertTrue(state.health & int(HealthFlag.ORIGIN_VALID))
        self.assertTrue(state.health & int(HealthFlag.POSITION_VALID))
        self.assertGreater(state.x_m, 0.9)
        self.assertLess(abs(state.y_m), 0.1)
        self.assertTrue(source.shared_field_ready(42, 10.1))

    def test_quaternion_yaw_normalizes_and_rejects_invalid_quaternion(self) -> None:
        self.assertAlmostEqual(quaternion_yaw(0.0, 0.0, 1.0, 1.0), math.pi / 2.0)
        self.assertIsNone(quaternion_yaw(0.0, 0.0, 0.0, 0.0))
        self.assertIsNone(quaternion_yaw(math.nan, 0.0, 0.0, 1.0))

    def test_invalid_timeout_is_rejected(self) -> None:
        for value in (0.0, -1.0, math.nan, math.inf):
            with self.subTest(value=value), self.assertRaises(ValueError):
                MiniStateAccumulator(value)

    def test_execution_readiness_is_fresh_and_mode_bounded(self) -> None:
        source = MiniStateAccumulator(sample_timeout_sec=2.0)
        source.state = StateSample(True, False, "MANUAL", True, 10.0)
        self.assertTrue(source.safe_execution_prestate(10.1))
        self.assertTrue(source.execution_session_ready(10.1))

        source.state = StateSample(True, False, "OFFBOARD", False, 10.2)
        self.assertFalse(source.safe_execution_prestate(10.3))
        self.assertTrue(source.execution_session_ready(10.3))

        source.state = StateSample(True, True, "OFFBOARD", False, 10.4)
        self.assertFalse(source.safe_execution_prestate(10.5))
        self.assertTrue(source.execution_session_ready(10.5))

        for sample in (
            StateSample(True, True, "MANUAL", True, 10.6),
            StateSample(True, False, "POSCTL", True, 10.6),
            StateSample(True, False, "MANUAL", False, 10.6),
            StateSample(False, False, "MANUAL", True, 10.6),
        ):
            source.state = sample
            self.assertFalse(source.execution_session_ready(10.7))
        source.state = StateSample(True, False, "MANUAL", True, 1.0)
        self.assertFalse(source.safe_execution_prestate(10.7))


if __name__ == "__main__":
    unittest.main()
