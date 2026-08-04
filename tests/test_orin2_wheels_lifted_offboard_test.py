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
    @staticmethod
    def observation(
        *,
        mode: str = "MANUAL",
        armed: bool = False,
        connected: bool = True,
        fresh: bool = True,
        manual_input: bool = True,
    ):
        return module.VehicleObservation(
            state_present=True,
            state_age_sec=0.01 if fresh else module.STATE_TIMEOUT_SEC + 0.01,
            connected=connected,
            armed=armed,
            mode=mode,
            manual_input=manual_input,
        )

    @staticmethod
    def missing_observation():
        return module.VehicleObservation(
            state_present=False,
            state_age_sec=float("inf"),
            connected=False,
            armed=False,
            mode="",
            manual_input=False,
        )

    def advance_auto_to_arm_request(self, machine):
        manual = self.observation()
        offboard = self.observation(mode="OFFBOARD")
        machine.start(0.0)
        self.assertIsNone(machine.tick(0.0, manual))
        self.assertEqual(machine.phase, module.AutoPhase.ZERO_PRESTREAM)
        self.assertIsNone(machine.tick(2.0, manual))
        self.assertEqual(machine.phase, module.AutoPhase.REQUEST_OFFBOARD)
        action = machine.tick(2.01, manual)
        self.assertEqual(action, module.ServiceAction.REQUEST_OFFBOARD)
        self.assertEqual(machine.phase, module.AutoPhase.VERIFY_OFFBOARD)
        self.assertIsNone(machine.tick(2.10, offboard))
        self.assertEqual(machine.phase, module.AutoPhase.REQUEST_ARM_ONCE)
        action = machine.tick(2.11, offboard)
        self.assertEqual(action, module.ServiceAction.REQUEST_ARM)
        self.assertEqual(machine.phase, module.AutoPhase.VERIFY_ARMED)
        return offboard

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
        with self.assertRaises(ValueError):
            module.AutoZeroOnlyStateMachine(float("nan"))

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

    def test_auto_entry_flag_requires_zero_only_and_preserves_manual_entry(self) -> None:
        with self.assertRaisesRegex(ValueError, "requires --zero-only-hold-sec"):
            module.select_diagnostic(
                module.parse_args(["--auto-enter-offboard"])
            )

        automatic = module.select_diagnostic(
            module.parse_args(
                ["--zero-only-hold-sec", "5", "--auto-enter-offboard"]
            )
        )
        manual = module.select_diagnostic(
            module.parse_args(["--zero-only-hold-sec", "5"])
        )
        self.assertIsInstance(automatic, module.AutoZeroOnlyDiagnostic)
        self.assertIsInstance(manual, module.ZeroOnlyDiagnostic)
        self.assertEqual(automatic.plan, ())
        self.assertEqual(manual.plan, ())

    def test_auto_entry_still_requires_exact_live_confirmation(self) -> None:
        args = module.parse_args(
            ["--execute", "--zero-only-hold-sec", "5", "--auto-enter-offboard"]
        )
        with self.assertRaisesRegex(ValueError, "live output refused"):
            module.require_live_confirmation(args)

    def test_cmode_then_manual_starts_precheck_only_at_safe_prestate(self) -> None:
        wait = module.InitialStateWait(started_sec=10.0)
        machine = module.AutoZeroOnlyStateMachine(hold_sec=5.0)
        transitional = self.observation(mode="CMODE(65536)")

        self.assertEqual(
            wait.evaluate(10.01, transitional),
            module.InitialStateWaitResult.WAITING,
        )
        self.assertFalse(machine.started)
        self.assertEqual(machine.offboard_requests, 0)
        self.assertEqual(machine.arm_requests, 0)
        self.assertTrue(
            module.setpoint_is_zero(
                module.auto_setpoint_for_phase(module.AutoPhase.PRECHECK)
            )
        )

        fresh = self.observation()
        self.assertEqual(
            wait.evaluate(11.02, fresh),
            module.InitialStateWaitResult.SAFE_PRESTATE_READY,
        )
        machine.start(11.02)
        self.assertIsNone(machine.tick(11.02, fresh))
        self.assertEqual(machine.phase, module.AutoPhase.ZERO_PRESTREAM)
        self.assertEqual(machine.offboard_requests, 0)
        self.assertEqual(machine.arm_requests, 0)

    def test_initial_state_timeout_enters_recovery_without_entry_requests(self) -> None:
        wait = module.InitialStateWait(started_sec=20.0)
        machine = module.AutoZeroOnlyStateMachine(hold_sec=5.0)
        missing = self.missing_observation()

        self.assertEqual(
            wait.evaluate(24.99, missing),
            module.InitialStateWaitResult.WAITING,
        )
        self.assertEqual(
            wait.evaluate(25.0, missing),
            module.InitialStateWaitResult.TIMED_OUT,
        )
        self.assertFalse(machine.started)
        machine.start_recovery("initial_safe_prestate_timeout", 25.0)
        self.assertEqual(machine.phase, module.AutoPhase.ZERO_EXIT_BURST)
        self.assertEqual(
            machine.primary_failure_reason,
            "initial_safe_prestate_timeout",
        )
        self.assertEqual(machine.offboard_requests, 0)
        self.assertEqual(machine.arm_requests, 0)
        self.assertTrue(
            module.setpoint_is_zero(
                module.auto_setpoint_for_phase(machine.phase)
            )
        )

    def test_state_just_below_two_seconds_is_fresh(self) -> None:
        self.assertEqual(module.STATE_TIMEOUT_SEC, 2.0)
        observation = module.VehicleObservation(
            state_present=True,
            state_age_sec=module.STATE_TIMEOUT_SEC - 1e-6,
            connected=True,
            armed=False,
            mode="MANUAL",
            manual_input=True,
        )
        wait = module.InitialStateWait(started_sec=10.0)
        self.assertEqual(
            wait.evaluate(10.1, observation),
            module.InitialStateWaitResult.SAFE_PRESTATE_READY,
        )

        machine = module.AutoZeroOnlyStateMachine(hold_sec=5.0)
        machine.start(10.1)
        self.assertIsNone(machine.tick(10.1, observation))
        self.assertEqual(machine.phase, module.AutoPhase.ZERO_PRESTREAM)

    def test_state_above_two_seconds_enters_zero_recovery(self) -> None:
        observation = module.VehicleObservation(
            state_present=True,
            state_age_sec=module.STATE_TIMEOUT_SEC + 1e-6,
            connected=True,
            armed=False,
            mode="MANUAL",
            manual_input=True,
        )
        machine = module.AutoZeroOnlyStateMachine(hold_sec=5.0)
        machine.start(20.0)
        self.assertIsNone(machine.tick(20.0, observation))
        self.assertEqual(machine.phase, module.AutoPhase.ZERO_EXIT_BURST)
        self.assertEqual(machine.primary_failure_reason, "state_stale")
        self.assertEqual(machine.offboard_requests, 0)
        self.assertEqual(machine.arm_requests, 0)
        self.assertTrue(
            module.setpoint_is_zero(
                module.auto_setpoint_for_phase(machine.phase)
            )
        )

    def test_persistent_cmode_times_out_without_entry_requests(self) -> None:
        wait = module.InitialStateWait(started_sec=30.0)
        machine = module.AutoZeroOnlyStateMachine(hold_sec=5.0)
        transitional = self.observation(mode="CMODE(65536)")

        self.assertEqual(
            wait.evaluate(30.01, transitional),
            module.InitialStateWaitResult.WAITING,
        )
        self.assertEqual(
            wait.evaluate(35.0, transitional),
            module.InitialStateWaitResult.TIMED_OUT,
        )
        self.assertFalse(machine.started)
        machine.start_recovery("initial_safe_prestate_timeout", 35.0)
        self.assertEqual(machine.phase, module.AutoPhase.ZERO_EXIT_BURST)
        self.assertEqual(machine.offboard_requests, 0)
        self.assertEqual(machine.arm_requests, 0)

    def test_initial_armed_state_enters_recovery_without_entry_requests(self) -> None:
        wait = module.InitialStateWait(started_sec=40.0)
        machine = module.AutoZeroOnlyStateMachine(hold_sec=5.0)
        armed = self.observation(armed=True)

        self.assertEqual(
            wait.evaluate(40.01, armed),
            module.InitialStateWaitResult.UNSAFE_ARMED,
        )
        machine.start_recovery("initial_unsafe_armed", 40.01)
        self.assertEqual(machine.phase, module.AutoPhase.ZERO_EXIT_BURST)
        self.assertEqual(machine.primary_failure_reason, "initial_unsafe_armed")
        self.assertEqual(machine.offboard_requests, 0)
        self.assertEqual(machine.arm_requests, 0)
        self.assertTrue(
            module.setpoint_is_zero(
                module.auto_setpoint_for_phase(machine.phase)
            )
        )

    def test_auto_sequence_requests_offboard_before_single_arm(self) -> None:
        machine = module.AutoZeroOnlyStateMachine(hold_sec=5.0)
        offboard = self.advance_auto_to_arm_request(machine)
        self.assertEqual(machine.offboard_requests, 1)
        self.assertEqual(machine.arm_requests, 1)
        machine.service_result(module.ServiceAction.REQUEST_ARM, True, 2.12)
        machine.tick(2.20, self.observation(mode="OFFBOARD", armed=True))
        self.assertEqual(machine.phase, module.AutoPhase.ZERO_HOLD)
        self.assertEqual(machine.arm_requests, 1)
        self.assertEqual(offboard.mode, "OFFBOARD")

    def test_arm_rejection_or_timeout_never_rearms(self) -> None:
        rejected = module.AutoZeroOnlyStateMachine(hold_sec=5.0)
        self.advance_auto_to_arm_request(rejected)
        rejected.service_result(module.ServiceAction.REQUEST_ARM, False, 2.12)
        self.assertEqual(rejected.phase, module.AutoPhase.ZERO_EXIT_BURST)
        self.assertEqual(rejected.arm_requests, 1)

        timed_out = module.AutoZeroOnlyStateMachine(hold_sec=5.0)
        offboard = self.advance_auto_to_arm_request(timed_out)
        timed_out.tick(4.11, offboard)
        self.assertEqual(timed_out.phase, module.AutoPhase.ZERO_EXIT_BURST)
        self.assertEqual(timed_out.primary_failure_reason, "arm_not_observed_timeout")
        self.assertEqual(timed_out.arm_requests, 1)

    def test_offboard_requests_are_bounded_and_timeout_to_recovery(self) -> None:
        machine = module.AutoZeroOnlyStateMachine(hold_sec=5.0)
        manual = self.observation()
        machine.start(0.0)
        machine.tick(0.0, manual)
        machine.tick(2.0, manual)
        actions = []
        for now in (2.01, 4.02, 4.03, 6.04, 6.05, 8.06):
            action = machine.tick(now, manual)
            if action is not None:
                actions.append(action)
        self.assertEqual(
            actions,
            [module.ServiceAction.REQUEST_OFFBOARD] * 3,
        )
        self.assertEqual(machine.offboard_requests, 3)
        self.assertEqual(machine.phase, module.AutoPhase.ZERO_EXIT_BURST)
        self.assertEqual(machine.primary_failure_reason, "offboard_not_observed")

    def test_unexpected_disarm_during_hold_does_not_rearm(self) -> None:
        machine = module.AutoZeroOnlyStateMachine(hold_sec=5.0)
        self.advance_auto_to_arm_request(machine)
        machine.service_result(module.ServiceAction.REQUEST_ARM, True, 2.12)
        machine.tick(2.20, self.observation(mode="OFFBOARD", armed=True))
        machine.tick(2.30, self.observation(mode="OFFBOARD", armed=False))
        self.assertEqual(machine.phase, module.AutoPhase.ZERO_EXIT_BURST)
        self.assertEqual(machine.primary_failure_reason, "unexpected_disarm")
        self.assertEqual(machine.arm_requests, 1)
        self.assertNotEqual(
            machine.tick(3.30, self.observation(mode="OFFBOARD", armed=False)),
            module.ServiceAction.REQUEST_ARM,
        )
        self.assertEqual(machine.arm_requests, 1)

    def test_recovery_requires_disarmed_and_manual(self) -> None:
        machine = module.AutoZeroOnlyStateMachine(hold_sec=1.0)
        self.advance_auto_to_arm_request(machine)
        machine.service_result(module.ServiceAction.REQUEST_ARM, True, 2.12)
        machine.tick(2.20, self.observation(mode="OFFBOARD", armed=True))
        machine.tick(3.20, self.observation(mode="OFFBOARD", armed=True))
        self.assertEqual(machine.phase, module.AutoPhase.ZERO_EXIT_BURST)
        machine.tick(4.20, self.observation(mode="OFFBOARD", armed=True))
        self.assertEqual(machine.phase, module.AutoPhase.REQUEST_DISARM)
        action = machine.tick(4.21, self.observation(mode="OFFBOARD", armed=True))
        self.assertEqual(action, module.ServiceAction.REQUEST_DISARM)
        machine.tick(4.30, self.observation(mode="OFFBOARD", armed=False))
        self.assertEqual(machine.phase, module.AutoPhase.REQUEST_MANUAL)
        action = machine.tick(4.31, self.observation(mode="OFFBOARD", armed=False))
        self.assertEqual(action, module.ServiceAction.REQUEST_MANUAL)
        machine.tick(4.40, self.observation(mode="MANUAL", armed=False))
        self.assertEqual(machine.phase, module.AutoPhase.DONE)
        self.assertTrue(machine.successful)

        self.assertFalse(
            module.recovery_is_confirmed(
                self.observation(mode="MANUAL", armed=True)
            )
        )
        self.assertFalse(
            module.recovery_is_confirmed(
                self.observation(mode="OFFBOARD", armed=False)
            )
        )

    def test_auto_zero_only_uses_only_the_six_axis_zero_setpoint(self) -> None:
        diagnostic = module.select_diagnostic(
            module.parse_args(
                ["--zero-only-hold-sec", "5", "--auto-enter-offboard"]
            )
        )
        self.assertEqual(diagnostic.plan, ())
        for phase in module.AutoPhase:
            with self.subTest(phase=phase):
                setpoint = module.auto_setpoint_for_phase(phase)
                self.assertIs(setpoint, module.ZERO_SETPOINT)
                self.assertTrue(module.setpoint_is_zero(setpoint))

    def test_statustext_uses_sensor_data_qos_path(self) -> None:
        sensor_data_qos = object()
        self.assertIs(
            module.statustext_subscription_qos(sensor_data_qos),
            sensor_data_qos,
        )

    def test_auto_zero_only_dry_run_is_explicit(self) -> None:
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            result = module.main(
                ["--zero-only-hold-sec", "5", "--auto-enter-offboard"]
            )
        self.assertEqual(result, 0)
        rendered = output.getvalue()
        self.assertIn("ZERO-ONLY DIAGNOSTIC", rendered)
        self.assertIn("automatic OFFBOARD request -> one Arm request", rendered)
        self.assertIn("verified Disarm -> MANUAL", rendered)
        self.assertIn("no nonzero setpoint", rendered)


if __name__ == "__main__":
    unittest.main()
