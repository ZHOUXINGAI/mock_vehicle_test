from __future__ import annotations

import importlib.util
import contextlib
import io
from pathlib import Path
import sys
import unittest


MODULE_PATH = Path(__file__).parents[1] / "src" / "orin2_wheels_lifted_offboard_test.py"
SPEC = importlib.util.spec_from_file_location("orin2_wheels_lifted", MODULE_PATH)
assert SPEC and SPEC.loader
module = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = module
SPEC.loader.exec_module(module)


class WheelsLiftedTest(unittest.TestCase):
    def test_first_plan_is_tightly_bounded(self) -> None:
        plan = module.build_plan()
        module.validate_plan(plan)
        self.assertEqual(plan[0].name, "forward")
        motion = [step for step in plan if step.linear_x_mps or step.linear_y_mps]
        self.assertEqual([step.name for step in motion], ["forward", "left", "right"])
        self.assertTrue(all(step.duration_sec <= 1.0 for step in motion))
        self.assertEqual(max(abs(step.linear_x_mps) for step in plan), 0.05)
        self.assertEqual(max(abs(step.linear_y_mps) for step in plan), 0.05)

    def test_plan_has_no_post_offboard_zero_dwell(self) -> None:
        plan = module.build_plan(forward_only=True)
        self.assertEqual([step.name for step in plan], [
            "forward", "stop_after_forward"
        ])
        self.assertNotEqual(
            (plan[0].linear_x_mps, plan[0].linear_y_mps),
            (0.0, 0.0),
        )

    def test_live_mode_requires_exact_confirmation(self) -> None:
        args = module.parse_args(["--execute"])
        with self.assertRaisesRegex(ValueError, "live output refused"):
            module.require_live_confirmation(args)

        args = module.parse_args(
            ["--execute", "--confirm", module.EXECUTE_PHRASE]
        )
        module.require_live_confirmation(args)

        ground_args = module.parse_args(
            [
                "--execute",
                "--surface",
                "ground",
                "--confirm",
                module.GROUND_EXECUTE_PHRASE,
            ]
        )
        module.require_live_confirmation(ground_args)

        with self.assertRaisesRegex(ValueError, "surface=ground"):
            module.require_live_confirmation(
                module.parse_args(
                    [
                        "--execute",
                        "--surface",
                        "ground",
                        "--confirm",
                        module.EXECUTE_PHRASE,
                    ]
                )
            )

    def test_dry_run_needs_no_confirmation(self) -> None:
        module.require_live_confirmation(module.parse_args([]))

    def test_invalid_motion_is_rejected(self) -> None:
        invalid_steps = (
            module.Step("too_fast", 1.0, 0.051, 0.0),
            module.Step("too_much_yaw", 1.0, 0.0, 0.101),
            module.Step("too_long", 5.01, 0.01, 0.0),
            module.Step("nan", 1.0, float("nan"), 0.0),
        )
        for step in invalid_steps:
            with self.subTest(step=step), self.assertRaises(ValueError):
                module.validate_plan([step])

    def test_authorized_five_second_forward_only_plan(self) -> None:
        plan = module.build_plan(forward_sec=5.0, forward_only=True)
        module.validate_plan(plan)
        self.assertEqual([step.name for step in plan], [
            "forward", "stop_after_forward"
        ])
        self.assertEqual(plan[0].duration_sec, 5.0)

    def test_motion_state_failure_reasons_are_distinct(self) -> None:
        healthy = {
            "state_present": True,
            "state_age_sec": 0.01,
            "safe_prestate_seen": True,
            "connected": True,
            "armed": True,
            "mode": "OFFBOARD",
        }
        self.assertIsNone(module.classify_motion_state(**healthy))

        cases = (
            ("state_missing", {"state_present": False}),
            ("state_stale", {"state_age_sec": module.STATE_TIMEOUT_SEC + 0.01}),
            ("disconnected", {"connected": False}),
            ("safe_manual_prestate_missing", {"safe_prestate_seen": False}),
            ("unexpected_disarm", {"armed": False}),
            ("offboard_exit", {"mode": "MANUAL"}),
        )
        for expected, changed in cases:
            with self.subTest(expected=expected):
                state = {**healthy, **changed}
                self.assertEqual(
                    module.classify_motion_state(**state),
                    expected,
                )

    def test_zero_only_duration_is_bounded_and_finite(self) -> None:
        for value in (-1.0, 0.0, 0.5, 60.01, float("nan"), float("inf")):
            with self.subTest(value=value), self.assertRaises(ValueError):
                module.validate_zero_only_hold_sec(value)
        self.assertEqual(module.validate_zero_only_hold_sec(1.0), 1.0)
        self.assertEqual(module.validate_zero_only_hold_sec(60.0), 60.0)

    def test_zero_only_selects_empty_action_plan(self) -> None:
        args = module.parse_args(["--zero-only-hold-sec", "12.5"])
        diagnostic = module.select_diagnostic(args)
        self.assertIsInstance(diagnostic, module.ZeroOnlyDiagnostic)
        self.assertEqual(diagnostic.hold_sec, 12.5)
        self.assertEqual(diagnostic.plan, ())

    def test_zero_only_setpoint_has_no_nonzero_axis(self) -> None:
        self.assertTrue(module.setpoint_is_zero(module.ZERO_SETPOINT))
        self.assertEqual(
            module.ZERO_SETPOINT,
            module.VelocitySetpoint(
                linear_x=0.0,
                linear_y=0.0,
                linear_z=0.0,
                angular_x=0.0,
                angular_y=0.0,
                angular_z=0.0,
            ),
        )

    def test_zero_only_rejects_motion_options(self) -> None:
        cases = (
            ["--zero-only-hold-sec", "5", "--forward-only"],
            ["--zero-only-hold-sec", "5", "--forward-sec", "2"],
        )
        for argv in cases:
            with self.subTest(argv=argv), self.assertRaisesRegex(
                ValueError, "cannot be combined"
            ):
                module.select_diagnostic(module.parse_args(argv))

    def test_zero_only_dry_run_is_explicit(self) -> None:
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            result = module.main(["--zero-only-hold-sec", "5"])
        self.assertEqual(result, 0)
        rendered = output.getvalue()
        self.assertIn("ZERO-ONLY DIAGNOSTIC", rendered)
        self.assertIn("hold_sec=5.000", rendered)
        self.assertIn("no nonzero setpoint", rendered)
        self.assertIn("DRY RUN ONLY", rendered)

    def test_forward_only_default_remains_one_second_at_point_zero_five(self) -> None:
        diagnostic = module.select_diagnostic(module.parse_args(["--forward-only"]))
        self.assertIsInstance(diagnostic, module.MotionDiagnostic)
        self.assertEqual(diagnostic.plan[0].name, "forward")
        self.assertEqual(diagnostic.plan[0].duration_sec, 1.0)
        self.assertEqual(diagnostic.plan[0].linear_x_mps, 0.05)


if __name__ == "__main__":
    unittest.main()
