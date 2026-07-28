#!/usr/bin/env python3

from __future__ import annotations

import dataclasses
import math
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace


REPO_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_DIR / "src"))

from easydocking_pairb_adapter import (
    PairBAdapterError,
    QuantizationTolerance,
    adapt_ground_corridor_plan,
    adapt_ground_plan_command,
)
from lr24_compact_protocol import Phase, PlanFlag, Role


def make_plan(**changes: object) -> SimpleNamespace:
    tangent_point = (0.8883757492037598, -4.411438374071053)
    values: dict[str, object] = {
        "schema_version": 2,
        "plan_id": 7,
        "sequence": 9,
        "frame_id": "field_enu",
        "origin_id": 3,
        "sender_monotonic_ms": 1000,
        "valid_until_sender_monotonic_ms": 33000,
        "validity_ms": 32000,
        "requested_validity_ms": 32000,
        "required_validity_ms": 28400,
        "validity_margin_ms": 3600,
        "post_tangent_reserve_ms": 3350,
        "terminal_completion_budget_ms": 2000,
        "completion_hold_ms": 500,
        "plan_timing_guard_ms": 100,
        "validity_policy": "reject",
        "validity_extended": False,
        "tangent_point": tangent_point,
        "tangent_direction": (0.9803196386824563, 0.1974168331563911),
        "tangent_phase_rad": math.atan2(tangent_point[1], tangent_point[0])
        % (2.0 * math.pi),
        "mini_arrival_delay_ms": 25000,
        "terminal_length_m": 8.0,
        "target_front_gap_m": 0.35,
        "mini_speed_mps": 0.9,
        "carrier_max_speed_mps": 0.7,
        "command_ttl_ms": 500,
        "local_command_watchdog_ms": 750,
    }
    values.update(changes)
    return SimpleNamespace(**values)


def make_command(
    phase: str,
    role: str,
    **changes: object,
) -> SimpleNamespace:
    safe_phase = phase in {"HOLD", "STOP", "ABORT"}
    values: dict[str, object] = {
        "schema_version": 1,
        "plan_id": 7,
        "sequence": 4,
        "target_role": role,
        "phase": phase,
        "sender_monotonic_ms": 0xFFFFFF00,
        "valid_until_sender_monotonic_ms": 0x000000F4,
        "ttl_ms": 500,
        "body_speed_mps": 0.0 if safe_phase else 0.6,
        "yaw_rate_radps": 0.0 if safe_phase else 0.2,
        "max_speed_mps": 0.9,
        "max_accel_mps2": 0.5,
        "frame_id": "field_enu",
        "origin_id": 3,
    }
    values.update(changes)
    return SimpleNamespace(**values)


class GroundCorridorAdapterTest(unittest.TestCase):
    def test_schema_shaped_plan_maps_without_recomputing(self) -> None:
        source = make_plan()
        compact = adapt_ground_corridor_plan(
            source,
            expected_frame_id="field_enu",
            expected_origin_id=3,
            one_orbit_complete=True,
        )
        decoded = type(compact).decode(compact.encode())
        self.assertEqual(decoded.plan_schema_version, 2)
        self.assertEqual(decoded.plan_id, source.plan_id)
        self.assertEqual(decoded.seq, source.sequence)
        self.assertEqual(decoded.timestamp_ms, source.sender_monotonic_ms)
        self.assertEqual(
            decoded.valid_until_ms, source.valid_until_sender_monotonic_ms
        )
        self.assertAlmostEqual(decoded.rendezvous_x_m, source.tangent_point[0], delta=0.005)
        self.assertAlmostEqual(decoded.rendezvous_y_m, source.tangent_point[1], delta=0.005)
        self.assertAlmostEqual(decoded.tangent_dir_x, source.tangent_direction[0], delta=0.00005)
        self.assertAlmostEqual(decoded.tangent_dir_y, source.tangent_direction[1], delta=0.00005)
        self.assertEqual(decoded.corridor_length_m, source.terminal_length_m)
        self.assertEqual(decoded.target_front_gap_m, source.target_front_gap_m)
        self.assertEqual(decoded.ahead_distance_m, source.target_front_gap_m)
        self.assertEqual(decoded.required_validity_ms, source.required_validity_ms)
        self.assertTrue(decoded.flags & int(PlanFlag.ONE_ORBIT_COMPLETE))

    def test_plan_identity_schema_range_and_timing_are_rejected(self) -> None:
        cases = (
            (make_plan(schema_version=1), "unsupported_plan_schema"),
            (make_plan(frame_id="map"), "frame_id_mismatch"),
            (make_plan(origin_id=4), "origin_id_mismatch"),
            (make_plan(plan_id=70000), "wire_range:plan_id"),
            (make_plan(validity_policy="extend"), "unsupported_validity_policy"),
            (make_plan(validity_extended=True), "silently_extended_plan"),
            (
                make_plan(tangent_phase_rad=2.0 * math.pi),
                "tangent_phase_not_normalized",
            ),
            (make_plan(required_validity_ms=28300), "validity_margin_mismatch"),
            (make_plan(post_tangent_reserve_ms=3349), "invalid_pairb_plan"),
            (make_plan(terminal_length_m=700.0), "wire_range:terminal_length_m"),
        )
        for source, reason in cases:
            with self.subTest(reason=reason), self.assertRaisesRegex(
                PairBAdapterError, reason
            ):
                adapt_ground_corridor_plan(
                    source,
                    expected_frame_id="field_enu",
                    expected_origin_id=3,
                )

        with self.assertRaisesRegex(PairBAdapterError, "invalid_one_orbit_complete"):
            adapt_ground_corridor_plan(
                make_plan(),
                expected_frame_id="field_enu",
                expected_origin_id=3,
                one_orbit_complete=1,  # type: ignore[arg-type]
            )

    def test_explicit_quantization_tolerance_can_fail_closed(self) -> None:
        strict = dataclasses.replace(
            QuantizationTolerance(),
            linear_m=0.001,
        )
        with self.assertRaisesRegex(PairBAdapterError, "quantization_tolerance"):
            adapt_ground_corridor_plan(
                make_plan(),
                expected_frame_id="field_enu",
                expected_origin_id=3,
                tolerance=strict,
            )

    def test_real_easydocking_ground_plan_integration(self) -> None:
        easy_src = Path("/home/jetson/easydocking/src")
        if not easy_src.is_dir():
            self.skipTest("real EasyDocking source checkout is not available")
        sys.path.insert(0, str(easy_src))
        try:
            from ground_corridor_geometry import compute_ground_corridor_plan
            from ground_docking_leader import CommandPhase, GroundPlanCommand

            source = compute_ground_corridor_plan(
                plan_id=11,
                sequence=5,
                frame_id="field_enu",
                origin_id=3,
                sender_monotonic_ms=0xFFFFFF00,
                carrier_position=(-7.0, -6.0),
                mini_phase_rad=0.0,
                orbit_center=(0.0, 0.0),
                orbit_radius_m=4.5,
                turn_direction="ccw",
                mini_speed_mps=0.9,
                mini_max_accel_mps2=0.5,
                carrier_max_speed_mps=0.7,
                carrier_max_accel_mps2=0.3,
                validity_ms=120000,
            )
            compact = adapt_ground_corridor_plan(
                source,
                expected_frame_id="field_enu",
                expected_origin_id=3,
            )
            self.assertEqual(len(compact.encode()), 59)
            self.assertAlmostEqual(compact.rendezvous_x_m, source.tangent_point[0])
            self.assertAlmostEqual(compact.rendezvous_y_m, source.tangent_point[1])

            real_command = GroundPlanCommand(
                schema_version=1,
                plan_id=11,
                sequence=6,
                target_role="mini",
                phase=CommandPhase.HOLD,
                sender_monotonic_ms=0xFFFFFF00,
                valid_until_sender_monotonic_ms=0x000000F4,
                ttl_ms=500,
                body_speed_mps=0.0,
                yaw_rate_radps=0.0,
                max_speed_mps=0.9,
                max_accel_mps2=0.5,
                frame_id="field_enu",
                origin_id=3,
            )
            compact_command = adapt_ground_plan_command(
                real_command,
                expected_frame_id="field_enu",
                expected_origin_id=3,
                expected_role=Role.MINI,
            )
            self.assertEqual(compact_command.phase, Phase.HOLD)
            self.assertEqual(compact_command.validity_ms, 500)
        finally:
            sys.path.remove(str(easy_src))


class GroundCommandAdapterTest(unittest.TestCase):
    def test_all_phases_and_both_roles_map(self) -> None:
        expected_phases = {
            "HOLD": Phase.HOLD,
            "ORBIT": Phase.ORBIT,
            "ARC": Phase.ARC_TO_CORRIDOR,
            "TERMINAL": Phase.TERMINAL,
            "STOP": Phase.STOP,
            "ABORT": Phase.ABORT,
        }
        for role_name, role in (("mini", Role.MINI), ("carrier", Role.CARRIER)):
            for phase_name, phase in expected_phases.items():
                with self.subTest(role=role_name, phase=phase_name):
                    compact = adapt_ground_plan_command(
                        make_command(phase_name, role_name),
                        expected_frame_id="field_enu",
                        expected_origin_id=3,
                        expected_role=role,
                    )
                    self.assertEqual(compact.role, role)
                    self.assertEqual(compact.phase, phase)
                    self.assertEqual(compact.validity_ms, 500)
                    self.assertEqual(compact.duration_ms, 500)

    def test_command_identity_ttl_range_and_safe_phase_are_rejected(self) -> None:
        cases = (
            (make_command("HOLD", "mini", schema_version=2), "unsupported_command_schema"),
            (make_command("HOLD", "mini", frame_id="map"), "frame_id_mismatch"),
            (make_command("HOLD", "mini", origin_id=4), "origin_id_mismatch"),
            (make_command("HOLD", "unknown"), "unsupported_target_role"),
            (make_command("UNKNOWN", "mini"), "unsupported_command_phase"),
            (
                make_command("HOLD", "mini", ttl_ms=499),
                "command_ttl_mismatch",
            ),
            (
                make_command("HOLD", "mini", ttl_ms=65536),
                "wire_range:ttl_ms",
            ),
            (
                make_command("HOLD", "mini", body_speed_mps=0.1),
                "nonzero_safe_phase",
            ),
            (
                make_command("ORBIT", "mini", body_speed_mps=700.0, max_speed_mps=700.0),
                "wire_range:body_speed_mps",
            ),
        )
        for source, reason in cases:
            with self.subTest(reason=reason), self.assertRaisesRegex(
                PairBAdapterError, reason
            ):
                adapt_ground_plan_command(
                    source,
                    expected_frame_id="field_enu",
                    expected_origin_id=3,
                )


if __name__ == "__main__":
    unittest.main()
