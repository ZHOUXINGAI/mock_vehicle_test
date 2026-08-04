#!/usr/bin/env python3

"""Fail-closed, wheels-lifted-only Offboard setpoint smoke test for Orin2.

Dry-run is the default and does not import ROS.  Manual-entry and motion modes
never call arming or mode services.  The explicit auto zero-only mode may
request OFFBOARD, one Arm, Disarm, and MANUAL only after the execution flag and
exact physical-safety confirmation; it never writes parameters or publishes a
nonzero setpoint.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
from enum import Enum
import math
import sys
import time
from dataclasses import dataclass
from typing import Iterable, Sequence


EXECUTE_PHRASE = "WHEELS_LIFTED_AND_RESTRAINED"
GROUND_EXECUTE_PHRASE = "GROUND_AREA_CLEAR_RC_KILL_READY"
MAX_LINEAR_MPS = 0.05
MAX_LATERAL_MPS = 0.05
MAX_MOTION_SEC = 5.0
PUBLISH_RATE_HZ = 20.0
# C2 MAVROS State is measured near 1 Hz; more than two missed periods fails
# closed without rejecting normal gaps of about 1.04 seconds.
STATE_TIMEOUT_SEC = 2.0
ZERO_BURST_SEC = 1.0
MIN_ZERO_ONLY_HOLD_SEC = 1.0
MAX_ZERO_ONLY_HOLD_SEC = 60.0
AUTO_ZERO_PRESTREAM_SEC = 2.0
AUTO_OFFBOARD_RETRY_SEC = 2.0
AUTO_OFFBOARD_MAX_ATTEMPTS = 3
AUTO_ENTRY_TIMEOUT_SEC = 10.0
AUTO_ARM_VERIFY_TIMEOUT_SEC = 2.0
AUTO_EXIT_BURST_SEC = 1.0
AUTO_RECOVERY_RETRY_SEC = 1.0
AUTO_RECOVERY_MAX_ATTEMPTS = 3
AUTO_RECOVERY_TIMEOUT_SEC = 6.0
AUTO_SERVICE_TIMEOUT_SEC = 2.0
INITIAL_STATE_WAIT_TIMEOUT_SEC = 5.0


@dataclass(frozen=True)
class Step:
    name: str
    duration_sec: float
    linear_x_mps: float
    linear_y_mps: float


@dataclass(frozen=True)
class VelocitySetpoint:
    linear_x: float = 0.0
    linear_y: float = 0.0
    linear_z: float = 0.0
    angular_x: float = 0.0
    angular_y: float = 0.0
    angular_z: float = 0.0


ZERO_SETPOINT = VelocitySetpoint()


@dataclass(frozen=True)
class MotionDiagnostic:
    plan: tuple[Step, ...]


@dataclass(frozen=True)
class ZeroOnlyDiagnostic:
    hold_sec: float

    @property
    def plan(self) -> tuple[Step, ...]:
        return ()


@dataclass(frozen=True)
class AutoZeroOnlyDiagnostic:
    hold_sec: float

    @property
    def plan(self) -> tuple[Step, ...]:
        return ()


Diagnostic = MotionDiagnostic | ZeroOnlyDiagnostic | AutoZeroOnlyDiagnostic


@dataclass(frozen=True)
class VehicleObservation:
    state_present: bool
    state_age_sec: float
    connected: bool
    armed: bool
    mode: str
    manual_input: bool


class InitialStateWaitResult(str, Enum):
    WAITING = "WAITING"
    SAFE_PRESTATE_READY = "SAFE_PRESTATE_READY"
    UNSAFE_ARMED = "UNSAFE_ARMED"
    TIMED_OUT = "TIMED_OUT"


def initial_safe_prestate_ready(observation: VehicleObservation) -> bool:
    return bool(
        observation.state_present
        and observation.state_age_sec <= STATE_TIMEOUT_SEC
        and observation.connected
        and not observation.armed
        and observation.mode.upper() == "MANUAL"
        and observation.manual_input
    )


@dataclass(frozen=True)
class InitialStateWait:
    started_sec: float
    timeout_sec: float = INITIAL_STATE_WAIT_TIMEOUT_SEC

    def evaluate(
        self,
        now_sec: float,
        observation: VehicleObservation,
    ) -> InitialStateWaitResult:
        if (
            observation.state_present
            and observation.state_age_sec <= STATE_TIMEOUT_SEC
            and observation.armed
        ):
            return InitialStateWaitResult.UNSAFE_ARMED
        if initial_safe_prestate_ready(observation):
            return InitialStateWaitResult.SAFE_PRESTATE_READY
        if now_sec - self.started_sec >= self.timeout_sec:
            return InitialStateWaitResult.TIMED_OUT
        return InitialStateWaitResult.WAITING


class AutoPhase(str, Enum):
    PRECHECK = "PRECHECK"
    ZERO_PRESTREAM = "ZERO_PRESTREAM"
    REQUEST_OFFBOARD = "REQUEST_OFFBOARD"
    VERIFY_OFFBOARD = "VERIFY_OFFBOARD"
    REQUEST_ARM_ONCE = "REQUEST_ARM_ONCE"
    VERIFY_ARMED = "VERIFY_ARMED"
    ZERO_HOLD = "ZERO_HOLD"
    ZERO_EXIT_BURST = "ZERO_EXIT_BURST"
    REQUEST_DISARM = "REQUEST_DISARM"
    VERIFY_DISARMED = "VERIFY_DISARMED"
    REQUEST_MANUAL = "REQUEST_MANUAL"
    VERIFY_MANUAL = "VERIFY_MANUAL"
    DONE = "DONE"
    RECOVERY_FAILED = "RECOVERY_FAILED"


class ServiceAction(str, Enum):
    REQUEST_OFFBOARD = "request_offboard"
    REQUEST_ARM = "request_arm"
    REQUEST_DISARM = "request_disarm"
    REQUEST_MANUAL = "request_manual"


def auto_setpoint_for_phase(phase: AutoPhase) -> VelocitySetpoint:
    """Auto zero-only has no phase that is allowed to emit motion."""
    if not isinstance(phase, AutoPhase):
        raise ValueError("unknown auto zero-only phase")
    return ZERO_SETPOINT


def auto_precheck_failure(observation: VehicleObservation) -> str | None:
    if not observation.state_present:
        return "state_missing"
    if observation.state_age_sec > STATE_TIMEOUT_SEC:
        return "state_stale"
    if not observation.connected:
        return "disconnected"
    if observation.armed:
        return "precheck_already_armed"
    if observation.mode.upper() != "MANUAL":
        return "precheck_not_manual"
    if not observation.manual_input:
        return "precheck_manual_input_false"
    return None


def active_control_failure(observation: VehicleObservation) -> str | None:
    if not observation.state_present:
        return "state_missing"
    if observation.state_age_sec > STATE_TIMEOUT_SEC:
        return "state_stale"
    if not observation.connected:
        return "disconnected"
    if not observation.armed:
        return "unexpected_disarm"
    if observation.mode.upper() != "OFFBOARD":
        return "offboard_exit"
    return None


def recovery_is_confirmed(observation: VehicleObservation) -> bool:
    return bool(
        observation.state_present
        and observation.state_age_sec <= STATE_TIMEOUT_SEC
        and observation.connected
        and not observation.armed
        and observation.mode.upper() == "MANUAL"
    )


def statustext_subscription_qos(sensor_data_qos: object) -> object:
    """Keep STATUSTEXT compatible with MAVROS's best-effort publisher."""
    return sensor_data_qos


@dataclass
class AutoZeroOnlyStateMachine:
    hold_sec: float
    phase: AutoPhase = AutoPhase.PRECHECK
    phase_started_sec: float = 0.0
    entry_started_sec: float | None = None
    last_offboard_request_sec: float = -math.inf
    offboard_requests: int = 0
    arm_requests: int = 0
    disarm_requests: int = 0
    manual_requests: int = 0
    primary_failure_reason: str | None = None
    recovery_failure_reason: str | None = None
    started: bool = False

    def __post_init__(self) -> None:
        self.hold_sec = validate_zero_only_hold_sec(self.hold_sec)

    def start(self, now_sec: float) -> None:
        if self.started:
            raise RuntimeError("auto zero-only state machine already started")
        self.started = True
        self.phase = AutoPhase.PRECHECK
        self.phase_started_sec = now_sec

    def start_recovery(self, reason: str, now_sec: float) -> None:
        if self.started:
            self.begin_recovery(reason, now_sec)
            return
        self.started = True
        self.primary_failure_reason = reason
        self.phase = AutoPhase.ZERO_EXIT_BURST
        self.phase_started_sec = now_sec

    def _enter(self, phase: AutoPhase, now_sec: float) -> None:
        self.phase = phase
        self.phase_started_sec = now_sec

    def begin_recovery(self, reason: str | None, now_sec: float) -> None:
        if reason and self.primary_failure_reason is None:
            self.primary_failure_reason = reason
        if self.phase not in {
            AutoPhase.ZERO_EXIT_BURST,
            AutoPhase.REQUEST_DISARM,
            AutoPhase.VERIFY_DISARMED,
            AutoPhase.REQUEST_MANUAL,
            AutoPhase.VERIFY_MANUAL,
            AutoPhase.DONE,
            AutoPhase.RECOVERY_FAILED,
        }:
            self._enter(AutoPhase.ZERO_EXIT_BURST, now_sec)

    def _recovery_failed(self, reason: str, now_sec: float) -> None:
        self.recovery_failure_reason = reason
        self._enter(AutoPhase.RECOVERY_FAILED, now_sec)

    def service_result(
        self,
        action: ServiceAction,
        accepted: bool,
        now_sec: float,
    ) -> None:
        if action is ServiceAction.REQUEST_ARM and not accepted:
            self.begin_recovery("arm_request_rejected_or_failed", now_sec)

    def tick(
        self,
        now_sec: float,
        observation: VehicleObservation,
    ) -> ServiceAction | None:
        if not self.started:
            raise RuntimeError("auto zero-only state machine was not started")

        if self.phase is AutoPhase.PRECHECK:
            failure = auto_precheck_failure(observation)
            if failure is not None:
                self.begin_recovery(failure, now_sec)
                return None
            self._enter(AutoPhase.ZERO_PRESTREAM, now_sec)
            return None

        if self.phase is AutoPhase.ZERO_PRESTREAM:
            failure = auto_precheck_failure(observation)
            if failure is not None:
                self.begin_recovery(failure, now_sec)
            elif now_sec - self.phase_started_sec >= AUTO_ZERO_PRESTREAM_SEC:
                self.entry_started_sec = now_sec
                self._enter(AutoPhase.REQUEST_OFFBOARD, now_sec)
            return None

        if self.phase is AutoPhase.REQUEST_OFFBOARD:
            failure = auto_precheck_failure(observation)
            if failure is not None:
                self.begin_recovery(failure, now_sec)
                return None
            if self.offboard_requests >= AUTO_OFFBOARD_MAX_ATTEMPTS:
                self.begin_recovery("offboard_not_observed", now_sec)
                return None
            self.offboard_requests += 1
            self.last_offboard_request_sec = now_sec
            self._enter(AutoPhase.VERIFY_OFFBOARD, now_sec)
            return ServiceAction.REQUEST_OFFBOARD

        if self.phase is AutoPhase.VERIFY_OFFBOARD:
            if not observation.state_present:
                self.begin_recovery("state_missing", now_sec)
            elif observation.state_age_sec > STATE_TIMEOUT_SEC:
                self.begin_recovery("state_stale", now_sec)
            elif not observation.connected:
                self.begin_recovery("disconnected", now_sec)
            elif observation.armed:
                self.begin_recovery("unexpected_arm_before_request", now_sec)
            elif observation.mode.upper() == "OFFBOARD":
                self._enter(AutoPhase.REQUEST_ARM_ONCE, now_sec)
            elif observation.mode.upper() != "MANUAL":
                self.begin_recovery("unexpected_mode_during_entry", now_sec)
            elif (
                self.entry_started_sec is not None
                and now_sec - self.entry_started_sec >= AUTO_ENTRY_TIMEOUT_SEC
            ):
                self.begin_recovery("offboard_entry_timeout", now_sec)
            elif now_sec - self.last_offboard_request_sec >= AUTO_OFFBOARD_RETRY_SEC:
                if self.offboard_requests < AUTO_OFFBOARD_MAX_ATTEMPTS:
                    self._enter(AutoPhase.REQUEST_OFFBOARD, now_sec)
                else:
                    self.begin_recovery("offboard_not_observed", now_sec)
            return None

        if self.phase is AutoPhase.REQUEST_ARM_ONCE:
            if (
                not observation.state_present
                or observation.state_age_sec > STATE_TIMEOUT_SEC
                or not observation.connected
                or observation.mode.upper() != "OFFBOARD"
                or observation.armed
            ):
                self.begin_recovery("arm_precondition_lost", now_sec)
                return None
            if self.arm_requests != 0:
                self.begin_recovery("arm_request_invariant_violation", now_sec)
                return None
            self.arm_requests = 1
            self._enter(AutoPhase.VERIFY_ARMED, now_sec)
            return ServiceAction.REQUEST_ARM

        if self.phase is AutoPhase.VERIFY_ARMED:
            if not observation.state_present:
                self.begin_recovery("state_missing", now_sec)
            elif observation.state_age_sec > STATE_TIMEOUT_SEC:
                self.begin_recovery("state_stale", now_sec)
            elif not observation.connected:
                self.begin_recovery("disconnected", now_sec)
            elif observation.mode.upper() != "OFFBOARD":
                self.begin_recovery("offboard_exit", now_sec)
            elif observation.armed:
                self._enter(AutoPhase.ZERO_HOLD, now_sec)
            elif now_sec - self.phase_started_sec >= AUTO_ARM_VERIFY_TIMEOUT_SEC:
                self.begin_recovery("arm_not_observed_timeout", now_sec)
            return None

        if self.phase is AutoPhase.ZERO_HOLD:
            failure = active_control_failure(observation)
            if failure is not None:
                self.begin_recovery(failure, now_sec)
            elif now_sec - self.phase_started_sec >= self.hold_sec:
                self.begin_recovery(None, now_sec)
            return None

        if self.phase is AutoPhase.ZERO_EXIT_BURST:
            if now_sec - self.phase_started_sec >= AUTO_EXIT_BURST_SEC:
                self._enter(AutoPhase.REQUEST_DISARM, now_sec)
            return None

        if self.phase is AutoPhase.REQUEST_DISARM:
            if not observation.state_present or not observation.connected:
                self._recovery_failed("cannot_confirm_connection_for_disarm", now_sec)
            elif observation.state_age_sec > STATE_TIMEOUT_SEC:
                self._recovery_failed("state_stale_during_disarm", now_sec)
            elif not observation.armed:
                self._enter(AutoPhase.REQUEST_MANUAL, now_sec)
            elif self.disarm_requests >= AUTO_RECOVERY_MAX_ATTEMPTS:
                self._recovery_failed("disarm_not_confirmed", now_sec)
            else:
                self.disarm_requests += 1
                self._enter(AutoPhase.VERIFY_DISARMED, now_sec)
                return ServiceAction.REQUEST_DISARM
            return None

        if self.phase is AutoPhase.VERIFY_DISARMED:
            if (
                observation.state_present
                and observation.state_age_sec <= STATE_TIMEOUT_SEC
                and observation.connected
                and not observation.armed
            ):
                self._enter(AutoPhase.REQUEST_MANUAL, now_sec)
            elif now_sec - self.phase_started_sec >= AUTO_RECOVERY_TIMEOUT_SEC:
                self._recovery_failed("disarm_not_confirmed", now_sec)
            elif now_sec - self.phase_started_sec >= AUTO_RECOVERY_RETRY_SEC:
                self._enter(AutoPhase.REQUEST_DISARM, now_sec)
            return None

        if self.phase is AutoPhase.REQUEST_MANUAL:
            if not observation.state_present or not observation.connected:
                self._recovery_failed("cannot_confirm_connection_for_manual", now_sec)
            elif observation.state_age_sec > STATE_TIMEOUT_SEC:
                self._recovery_failed("state_stale_during_manual", now_sec)
            elif observation.armed:
                self._enter(AutoPhase.REQUEST_DISARM, now_sec)
            elif self.manual_requests >= AUTO_RECOVERY_MAX_ATTEMPTS:
                self._recovery_failed("manual_not_confirmed", now_sec)
            else:
                self.manual_requests += 1
                self._enter(AutoPhase.VERIFY_MANUAL, now_sec)
                return ServiceAction.REQUEST_MANUAL
            return None

        if self.phase is AutoPhase.VERIFY_MANUAL:
            if recovery_is_confirmed(observation):
                self._enter(AutoPhase.DONE, now_sec)
            elif observation.armed:
                self._enter(AutoPhase.REQUEST_DISARM, now_sec)
            elif now_sec - self.phase_started_sec >= AUTO_RECOVERY_TIMEOUT_SEC:
                self._recovery_failed("manual_not_confirmed", now_sec)
            elif now_sec - self.phase_started_sec >= AUTO_RECOVERY_RETRY_SEC:
                self._enter(AutoPhase.REQUEST_MANUAL, now_sec)
            return None

        return None

    @property
    def terminal(self) -> bool:
        return self.phase in {AutoPhase.DONE, AutoPhase.RECOVERY_FAILED}

    @property
    def successful(self) -> bool:
        return self.phase is AutoPhase.DONE and self.primary_failure_reason is None


def build_plan(
    *, forward_sec: float = 1.0, forward_only: bool = False
) -> tuple[Step, ...]:
    """Return the immutable post-authorization motion sequence.

    The zero-setpoint wait in ``run_live`` is the Offboard prestream.  Once a
    fresh armed+OFFBOARD state is observed, this plan starts bounded motion
    immediately instead of adding another zero dwell.
    """
    forward_steps = (
        Step("forward", forward_sec, 0.05, 0.0),
        Step("stop_after_forward", 3.0, 0.0, 0.0),
    )
    if forward_only:
        return forward_steps
    return forward_steps + (
        Step("left", 1.0, 0.0, -0.05),
        Step("stop_after_left", 3.0, 0.0, 0.0),
        Step("right", 1.0, 0.0, 0.05),
        Step("final_stop", 3.0, 0.0, 0.0),
    )


def validate_plan(plan: Iterable[Step]) -> None:
    for step in plan:
        values = (step.duration_sec, step.linear_x_mps, step.linear_y_mps)
        if not all(math.isfinite(value) for value in values):
            raise ValueError(f"{step.name}: non-finite value")
        if step.duration_sec <= 0.0:
            raise ValueError(f"{step.name}: duration must be positive")
        if abs(step.linear_x_mps) > MAX_LINEAR_MPS:
            raise ValueError(f"{step.name}: linear speed exceeds {MAX_LINEAR_MPS}")
        if abs(step.linear_y_mps) > MAX_LATERAL_MPS:
            raise ValueError(f"{step.name}: lateral speed exceeds {MAX_LATERAL_MPS}")
        if (step.linear_x_mps or step.linear_y_mps) and step.duration_sec > MAX_MOTION_SEC:
            raise ValueError(f"{step.name}: motion duration exceeds {MAX_MOTION_SEC}")


def setpoint_for_step(step: Step) -> VelocitySetpoint:
    return VelocitySetpoint(linear_x=step.linear_x_mps, linear_y=step.linear_y_mps)


def setpoint_is_zero(setpoint: VelocitySetpoint) -> bool:
    return all(
        value == 0.0
        for value in (
            setpoint.linear_x,
            setpoint.linear_y,
            setpoint.linear_z,
            setpoint.angular_x,
            setpoint.angular_y,
            setpoint.angular_z,
        )
    )


def validate_zero_only_hold_sec(value: float) -> float:
    if not math.isfinite(value):
        raise ValueError("zero-only hold duration must be finite")
    if not MIN_ZERO_ONLY_HOLD_SEC <= value <= MAX_ZERO_ONLY_HOLD_SEC:
        raise ValueError(
            "zero-only hold duration must be within "
            f"{MIN_ZERO_ONLY_HOLD_SEC:.0f}-{MAX_ZERO_ONLY_HOLD_SEC:.0f} seconds"
        )
    return value


def classify_motion_state(
    *,
    state_present: bool,
    state_age_sec: float,
    safe_prestate_seen: bool,
    connected: bool,
    armed: bool,
    mode: str,
) -> str | None:
    """Return a stable, operator-visible reason when motion is not authorized."""
    if not state_present:
        return "state_missing"
    if state_age_sec > STATE_TIMEOUT_SEC:
        return "state_stale"
    if not connected:
        return "disconnected"
    if not safe_prestate_seen:
        return "safe_manual_prestate_missing"
    if not armed:
        return "unexpected_disarm"
    if mode.upper() != "OFFBOARD":
        return "offboard_exit"
    return None


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--execute",
        action="store_true",
        help="enable ROS publishing; omitted means print-only dry-run",
    )
    parser.add_argument(
        "--confirm",
        default="",
        help=f"required with --execute: {EXECUTE_PHRASE}",
    )
    parser.add_argument("--namespace", default="/mavros")
    parser.add_argument(
        "--surface", choices=("wheels_lifted", "ground"), default="wheels_lifted"
    )
    parser.add_argument("--forward-sec", type=float, default=1.0)
    parser.add_argument(
        "--zero-only-hold-sec",
        type=float,
        default=None,
        metavar="N",
        help="zero-only Offboard diagnostic hold, in seconds (1-60)",
    )
    parser.add_argument(
        "--auto-enter-offboard",
        action="store_true",
        help=(
            "only with --zero-only-hold-sec: request OFFBOARD, then issue one "
            "Arm request, and perform verified Disarm+MANUAL recovery"
        ),
    )
    parser.add_argument(
        "--forward-only",
        action="store_true",
        help="run forward/stop only; omit all turn steps",
    )
    return parser.parse_args(argv)


def select_diagnostic(args: argparse.Namespace) -> Diagnostic:
    if args.auto_enter_offboard and args.zero_only_hold_sec is None:
        raise ValueError(
            "--auto-enter-offboard requires --zero-only-hold-sec"
        )
    if args.zero_only_hold_sec is not None:
        hold_sec = validate_zero_only_hold_sec(args.zero_only_hold_sec)
        if args.forward_only or args.forward_sec != 1.0:
            raise ValueError(
                "zero-only diagnostic cannot be combined with motion options"
            )
        if args.auto_enter_offboard:
            return AutoZeroOnlyDiagnostic(hold_sec=hold_sec)
        return ZeroOnlyDiagnostic(hold_sec=hold_sec)

    plan = build_plan(
        forward_sec=args.forward_sec,
        forward_only=args.forward_only,
    )
    validate_plan(plan)
    return MotionDiagnostic(plan=plan)


def require_live_confirmation(args: argparse.Namespace) -> None:
    expected = (
        GROUND_EXECUTE_PHRASE if args.surface == "ground" else EXECUTE_PHRASE
    )
    if args.execute and args.confirm != expected:
        raise ValueError(
            f"live output refused for surface={args.surface}; "
            f"required confirmation: --confirm {expected}"
        )


def print_plan(plan: Iterable[Step]) -> None:
    print("Orin2 wheels-lifted Offboard smoke plan")
    print("No automatic arm, disarm, mode change, or parameter write.")
    for step in plan:
        print(
            f"  {step.name:20s} {step.duration_sec:4.1f}s "
            f"vx={step.linear_x_mps:+.3f}m/s vy={step.linear_y_mps:+.3f}m/s"
        )


def print_diagnostic(diagnostic: Diagnostic) -> None:
    if isinstance(diagnostic, (ZeroOnlyDiagnostic, AutoZeroOnlyDiagnostic)):
        print("ZERO-ONLY DIAGNOSTIC")
        print(f"hold_sec={diagnostic.hold_sec:.3f}")
        print("Action plan is empty; no nonzero setpoint can be published.")
        if isinstance(diagnostic, AutoZeroOnlyDiagnostic):
            print(
                "entry=automatic OFFBOARD request -> one Arm request; "
                "exit=verified Disarm -> MANUAL"
            )
        else:
            print("entry=human-controlled Arm + OFFBOARD")
        return
    print_plan(diagnostic.plan)


def utc_timestamp() -> str:
    return (
        datetime.now(timezone.utc)
        .isoformat(timespec="milliseconds")
        .replace("+00:00", "Z")
    )


def format_event(
    event: str, details: str, *, utc: str, monotonic_sec: float
) -> str:
    suffix = f" {details}" if details else ""
    return (
        f"EVENT utc={utc} monotonic={monotonic_sec:.6f} "
        f"event={event}{suffix}"
    )


def run_live(diagnostic: Diagnostic, namespace: str) -> int:
    try:
        import rclpy
        from geometry_msgs.msg import TwistStamped
        from mavros_msgs.msg import ExtendedState, State, StatusText
        from mavros_msgs.srv import CommandBool, SetMode
        from rcl_interfaces.srv import GetParameters
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from rclpy.signals import SignalHandlerOptions
    except ImportError as exc:
        print(f"ROS/MAVROS imports unavailable: {exc}", file=sys.stderr)
        return 3

    namespace = "/" + namespace.strip("/")

    run_start_monotonic = time.monotonic()

    def log_event(event: str, details: str = "") -> None:
        print(
            format_event(
                event,
                details,
                utc=utc_timestamp(),
                monotonic_sec=time.monotonic(),
            )
        )

    class WheelsLiftedNode(Node):
        def __init__(self) -> None:
            super().__init__("orin2_wheels_lifted_offboard_test")
            self.state = None
            self.state_rx_monotonic = 0.0
            self.safe_prestate_seen = False
            self.latest_landed_state = None
            self.latest_statustext = None
            self.last_state_fields = None
            self.publisher = self.create_publisher(
                TwistStamped, f"{namespace}/setpoint_velocity/cmd_vel", 10
            )
            self.create_subscription(State, f"{namespace}/state", self.on_state, 10)
            self.create_subscription(
                ExtendedState,
                f"{namespace}/extended_state",
                self.on_extended_state,
                10,
            )
            self.create_subscription(
                StatusText,
                f"{namespace}/statustext/recv",
                self.on_statustext,
                statustext_subscription_qos(qos_profile_sensor_data),
            )
            self.frame_client = self.create_client(
                GetParameters, f"{namespace}/setpoint_velocity/get_parameters"
            )
            self.arming_client = None
            self.set_mode_client = None
            if isinstance(diagnostic, AutoZeroOnlyDiagnostic):
                self.arming_client = self.create_client(
                    CommandBool, f"{namespace}/cmd/arming"
                )
                self.set_mode_client = self.create_client(
                    SetMode, f"{namespace}/set_mode"
                )

        def on_state(self, message: State) -> None:
            previous = self.last_state_fields
            self.state = message
            self.state_rx_monotonic = time.monotonic()
            if (
                message.connected
                and not message.armed
                and message.manual_input
                and message.mode.upper() == "MANUAL"
            ):
                self.safe_prestate_seen = True

            current = (
                bool(message.connected),
                bool(message.armed),
                message.mode,
                bool(message.manual_input),
            )
            if current != previous:
                log_event(
                    "state_change",
                    f"connected={current[0]} armed={current[1]} "
                    f"mode={current[2]!r} manual_input={current[3]}",
                )
            if message.armed and (previous is None or not previous[1]):
                log_event("t_arm_observed", f"mode={message.mode!r}")
            if message.mode.upper() == "OFFBOARD" and (
                previous is None or previous[2].upper() != "OFFBOARD"
            ):
                log_event("t_offboard_observed", f"armed={message.armed}")
            self.last_state_fields = current

        def on_extended_state(self, message: ExtendedState) -> None:
            landed_state = int(message.landed_state)
            if landed_state != self.latest_landed_state:
                self.latest_landed_state = landed_state
                log_event("landed_state_change", f"landed_state={landed_state}")

        def on_statustext(self, message: StatusText) -> None:
            current = (int(message.severity), message.text)
            if current != self.latest_statustext:
                self.latest_statustext = current
                log_event(
                    "statustext_change",
                    f"severity={current[0]} text={current[1]!r}",
                )

        def motion_failure_reason(self) -> str | None:
            state_present = self.state is not None
            state_age_sec = (
                time.monotonic() - self.state_rx_monotonic
                if state_present
                else math.inf
            )
            return classify_motion_state(
                state_present=state_present,
                state_age_sec=state_age_sec,
                safe_prestate_seen=self.safe_prestate_seen,
                connected=bool(state_present and self.state.connected),
                armed=bool(state_present and self.state.armed),
                mode=self.state.mode if state_present else "",
            )

        def state_diagnostics(self) -> str:
            if self.state is None:
                return (
                    f"state=missing landed_state={self.latest_landed_state!r} "
                    f"latest_statustext={self.latest_statustext!r}"
                )
            age_sec = time.monotonic() - self.state_rx_monotonic
            return (
                f"state_age={age_sec:.3f}s connected={self.state.connected} "
                f"armed={self.state.armed} mode={self.state.mode!r} "
                f"safe_manual_prestate_seen={self.safe_prestate_seen} "
                f"landed_state={self.latest_landed_state!r} "
                f"latest_statustext={self.latest_statustext!r}"
            )

        def ready_for_motion(self) -> bool:
            return self.motion_failure_reason() is None

        def observation(self) -> VehicleObservation:
            state_present = self.state is not None
            return VehicleObservation(
                state_present=state_present,
                state_age_sec=(
                    time.monotonic() - self.state_rx_monotonic
                    if state_present
                    else math.inf
                ),
                connected=bool(state_present and self.state.connected),
                armed=bool(state_present and self.state.armed),
                mode=self.state.mode if state_present else "",
                manual_input=bool(state_present and self.state.manual_input),
            )

        def publish(self, setpoint: VelocitySetpoint) -> None:
            message = TwistStamped()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = "base_link"
            message.twist.linear.x = setpoint.linear_x
            message.twist.linear.y = setpoint.linear_y
            message.twist.linear.z = setpoint.linear_z
            message.twist.angular.x = setpoint.angular_x
            message.twist.angular.y = setpoint.angular_y
            message.twist.angular.z = setpoint.angular_z
            self.publisher.publish(message)

    # Keep SIGINT under this program's control so the final zero burst can be
    # published before the ROS context is shut down.
    rclpy.init(args=None, signal_handler_options=SignalHandlerOptions.NO)
    node = WheelsLiftedNode()
    period = 1.0 / PUBLISH_RATE_HZ
    exit_code = 1
    outcome = "unhandled_exception"
    diagnostic_name = (
        "auto_zero_only"
        if isinstance(diagnostic, AutoZeroOnlyDiagnostic)
        else "zero_only"
        if isinstance(diagnostic, ZeroOnlyDiagnostic)
        else "motion"
    )
    log_event(
        "t_start",
        f"diagnostic={diagnostic_name} "
        f"run_start_monotonic={run_start_monotonic:.6f}",
    )

    def body_ned_is_configured() -> bool:
        if not node.frame_client.wait_for_service(timeout_sec=5.0):
            print("Refused: MAVROS setpoint_velocity parameter service unavailable.", file=sys.stderr)
            return False
        last_error = "no response"
        for attempt in range(1, 4):
            request = GetParameters.Request()
            request.names = ["mav_frame"]
            future = node.frame_client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
            if not future.done():
                last_error = f"attempt {attempt}: timeout"
                future.cancel()
                continue
            exception = future.exception()
            if exception is not None:
                last_error = f"attempt {attempt}: {exception}"
                continue
            values = future.result().values
            if not values:
                last_error = f"attempt {attempt}: parameter absent"
                continue
            parameter_value = values[0]
            value = parameter_value.string_value or parameter_value.integer_value
            if str(value).upper() not in {"BODY_NED", "8"}:
                print(
                    f"Refused: mav_frame={value!r}, expected BODY_NED (8).",
                    file=sys.stderr,
                )
                return False
            print(f"Verified MAVROS setpoint_velocity.mav_frame={value!r}.")
            return True
        print(
            f"Refused: cannot read MAVROS setpoint_velocity.mav_frame ({last_error}).",
            file=sys.stderr,
        )
        return False

    def publish_zero_burst() -> None:
        deadline = time.monotonic() + ZERO_BURST_SEC
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.0)
            node.publish(ZERO_SETPOINT)
            time.sleep(period)

    auto_machine = (
        AutoZeroOnlyStateMachine(diagnostic.hold_sec)
        if isinstance(diagnostic, AutoZeroOnlyDiagnostic)
        else None
    )
    pending_action = None
    pending_future = None
    pending_started_sec = 0.0
    last_logged_phase = None

    def log_auto_phase(machine: AutoZeroOnlyStateMachine) -> None:
        nonlocal last_logged_phase
        if machine.phase is last_logged_phase:
            return
        last_logged_phase = machine.phase
        log_event(
            "auto_phase",
            f"phase={machine.phase.value} "
            f"offboard_requests={machine.offboard_requests} "
            f"arm_requests={machine.arm_requests} "
            f"disarm_requests={machine.disarm_requests} "
            f"manual_requests={machine.manual_requests}",
        )

    def service_attempt_number(
        machine: AutoZeroOnlyStateMachine, action: ServiceAction
    ) -> int:
        if action is ServiceAction.REQUEST_OFFBOARD:
            return machine.offboard_requests
        if action is ServiceAction.REQUEST_ARM:
            return machine.arm_requests
        if action is ServiceAction.REQUEST_DISARM:
            return machine.disarm_requests
        return machine.manual_requests

    def start_service_request(
        machine: AutoZeroOnlyStateMachine,
        action: ServiceAction,
    ) -> None:
        nonlocal pending_action, pending_future, pending_started_sec
        now_sec = time.monotonic()
        if pending_future is not None:
            pending_future.cancel()
            log_event(
                "service_request_superseded",
                f"old_action={pending_action.value} new_action={action.value}",
            )
            pending_action = None
            pending_future = None

        if action in {
            ServiceAction.REQUEST_OFFBOARD,
            ServiceAction.REQUEST_MANUAL,
        }:
            client = node.set_mode_client
            request = SetMode.Request()
            request.base_mode = 0
            request.custom_mode = (
                "OFFBOARD"
                if action is ServiceAction.REQUEST_OFFBOARD
                else "MANUAL"
            )
        else:
            client = node.arming_client
            request = CommandBool.Request()
            request.value = action is ServiceAction.REQUEST_ARM

        if client is None or not client.service_is_ready():
            log_event(
                "service_request_failed",
                f"action={action.value} reason=service_not_ready "
                f"attempt={service_attempt_number(machine, action)}",
            )
            machine.service_result(action, False, now_sec)
            return

        pending_action = action
        pending_future = client.call_async(request)
        pending_started_sec = now_sec
        log_event(
            "service_request",
            f"action={action.value} "
            f"attempt={service_attempt_number(machine, action)}",
        )

    def poll_service_result(machine: AutoZeroOnlyStateMachine) -> None:
        nonlocal pending_action, pending_future, pending_started_sec
        if pending_future is None or pending_action is None:
            return
        now_sec = time.monotonic()
        if not pending_future.done():
            if now_sec - pending_started_sec < AUTO_SERVICE_TIMEOUT_SEC:
                return
            pending_future.cancel()
            action = pending_action
            pending_action = None
            pending_future = None
            log_event(
                "service_response",
                f"action={action.value} accepted=False reason=timeout",
            )
            machine.service_result(action, False, now_sec)
            return


        action = pending_action
        future = pending_future
        pending_action = None
        pending_future = None
        try:
            response = future.result()
            accepted = bool(
                response.mode_sent
                if action
                in {
                    ServiceAction.REQUEST_OFFBOARD,
                    ServiceAction.REQUEST_MANUAL,
                }
                else response.success
            )
            result_code = getattr(response, "result", None)
            details = f"result={result_code!r}"
        except Exception as exc:
            accepted = False
            details = f"exception={exc!r}"
        log_event(
            "service_response",
            f"action={action.value} accepted={accepted} {details}",
        )
        machine.service_result(action, accepted, now_sec)

    def wait_for_initial_state(machine: AutoZeroOnlyStateMachine) -> None:
        wait = InitialStateWait(started_sec=time.monotonic())
        last_transition_fields: tuple[bool, bool, bool, str, bool] | None = None
        log_event(
            "initial_safe_prestate_wait_start",
            f"timeout_sec={wait.timeout_sec:.3f}",
        )
        while rclpy.ok():
            node.publish(ZERO_SETPOINT)
            rclpy.spin_once(node, timeout_sec=0.0)
            now_sec = time.monotonic()
            observation = node.observation()
            result = wait.evaluate(now_sec, observation)
            transition_fields = (
                observation.state_present,
                observation.connected,
                observation.armed,
                observation.mode,
                observation.manual_input,
            )
            if (
                result is InitialStateWaitResult.WAITING
                and transition_fields != last_transition_fields
            ):
                log_event(
                    "initial_safe_prestate_transition",
                    node.state_diagnostics(),
                )
                last_transition_fields = transition_fields
            if result is InitialStateWaitResult.SAFE_PRESTATE_READY:
                log_event(
                    "initial_safe_prestate_ready",
                    node.state_diagnostics(),
                )
                machine.start(now_sec)
                return
            if result is InitialStateWaitResult.UNSAFE_ARMED:
                log_event(
                    "initial_unsafe_armed_detected",
                    node.state_diagnostics(),
                )
                machine.start_recovery("initial_unsafe_armed", now_sec)
                return
            if result is InitialStateWaitResult.TIMED_OUT:
                log_event(
                    "initial_safe_prestate_wait_timeout",
                    f"timeout_sec={wait.timeout_sec:.3f} "
                    f"{node.state_diagnostics()}",
                )
                machine.start_recovery("initial_safe_prestate_timeout", now_sec)
                return
            time.sleep(period)

        log_event(
            "initial_safe_prestate_wait_aborted",
            "reason=ros_context_shutdown",
        )
        machine.start_recovery("ros_context_shutdown", time.monotonic())

    def drive_auto_machine(
        machine: AutoZeroOnlyStateMachine,
    ) -> tuple[int, str]:
        if not machine.started:
            wait_for_initial_state(machine)
        log_auto_phase(machine)
        while rclpy.ok() and not machine.terminal:
            node.publish(auto_setpoint_for_phase(machine.phase))
            rclpy.spin_once(node, timeout_sec=0.0)
            poll_service_result(machine)
            log_auto_phase(machine)
            action = machine.tick(time.monotonic(), node.observation())
            log_auto_phase(machine)
            if action is not None:
                start_service_request(machine, action)
                log_auto_phase(machine)
            time.sleep(period)

        if pending_future is not None:
            pending_future.cancel()
        if not rclpy.ok():
            print(
                "AUTO RECOVERY NOT CONFIRMED: ROS context shut down; "
                "use RC Kill/physical cutoff and verify MANUAL+disarmed.",
                file=sys.stderr,
            )
            return 8, "ros_context_shutdown_recovery_unconfirmed"
        if machine.phase is AutoPhase.RECOVERY_FAILED:
            print(
                "AUTO RECOVERY NOT CONFIRMED: "
                f"{machine.recovery_failure_reason}; use RC Kill/physical cutoff "
                "and verify MANUAL+disarmed.",
                file=sys.stderr,
            )
            log_event(
                "recovery_failed",
                f"reason={machine.recovery_failure_reason} "
                f"primary_failure={machine.primary_failure_reason!r} "
                f"{node.state_diagnostics()}",
            )
            return 8, "auto_recovery_unconfirmed"
        if machine.primary_failure_reason is not None:
            log_event(
                "auto_zero_only_failed_recovered",
                f"reason={machine.primary_failure_reason} "
                f"{node.state_diagnostics()}",
            )
            return 5, machine.primary_failure_reason
        log_event("auto_zero_only_complete", node.state_diagnostics())
        return 0, "auto_zero_only_complete"

    def recover_auto_after_exception(
        machine: AutoZeroOnlyStateMachine,
        reason: str,
    ) -> tuple[int, str]:
        machine.start_recovery(reason, time.monotonic())
        log_event("auto_recovery_started", f"reason={reason}")
        return drive_auto_machine(machine)

    try:
        if not body_ned_is_configured():
            outcome = "body_ned_check_failed"
            exit_code = 6
            log_event("diagnostic_failed", f"reason={outcome}")
            if auto_machine is not None:
                recovery_code, recovery_outcome = recover_auto_after_exception(
                    auto_machine, outcome
                )
                if recovery_code == 8:
                    exit_code = recovery_code
                    outcome = recovery_outcome
            return exit_code

        if auto_machine is not None:
            print(
                "AUTO ZERO-ONLY DIAGNOSTIC: PRECHECK -> 2s zero prestream -> "
                "OFFBOARD request -> one Arm request -> zero hold -> verified "
                "Disarm + MANUAL recovery."
            )
            exit_code, outcome = drive_auto_machine(auto_machine)
            return exit_code

        print(
            "Publishing zero prestream only; first require "
            "connected+MANUAL+disarmed+manual_input, then wait for "
            "human-controlled Arm + OFFBOARD."
        )
        ready_deadline = time.monotonic() + 60.0
        while rclpy.ok() and time.monotonic() < ready_deadline:
            rclpy.spin_once(node, timeout_sec=0.0)
            node.publish(ZERO_SETPOINT)
            if node.ready_for_motion():
                break
            time.sleep(period)
        else:
            print(
                "Refused: safe MANUAL prestate followed by fresh connected+armed+OFFBOARD "
                "was not observed.",
                file=sys.stderr,
            )
            outcome = node.motion_failure_reason() or "authorization_timeout"
            exit_code = 4
            log_event(
                "diagnostic_failed",
                f"reason={outcome} {node.state_diagnostics()}",
            )
            return exit_code

        log_event("authorization_observed", node.state_diagnostics())

        if isinstance(diagnostic, ZeroOnlyDiagnostic):
            print(
                "ZERO-ONLY DIAGNOSTIC active: publishing no nonzero setpoint "
                f"for {diagnostic.hold_sec:.3f}s."
            )
            log_event(
                "zero_only_hold_start",
                f"hold_sec={diagnostic.hold_sec:.3f}",
            )
            deadline = time.monotonic() + diagnostic.hold_sec
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.0)
                failure_reason = node.motion_failure_reason()
                if failure_reason is not None:
                    print(
                        f"ABORT: {failure_reason}; {node.state_diagnostics()}",
                        file=sys.stderr,
                    )
                    outcome = failure_reason
                    exit_code = 5
                    log_event(
                        "zero_only_hold_failed",
                        f"reason={failure_reason} {node.state_diagnostics()}",
                    )
                    return exit_code
                node.publish(ZERO_SETPOINT)
                time.sleep(period)
            if not rclpy.ok():
                outcome = "ros_context_shutdown"
                exit_code = 5
                log_event("zero_only_hold_failed", f"reason={outcome}")
                return exit_code
            outcome = "zero_only_complete"
            exit_code = 0
            log_event(
                "zero_only_hold_complete",
                f"hold_sec={diagnostic.hold_sec:.3f} {node.state_diagnostics()}",
            )
            return exit_code

        print(
            "Fresh armed+OFFBOARD observed; starting the first bounded motion "
            "step immediately (no post-authorization zero dwell)."
        )
        for step in diagnostic.plan:
            deadline = time.monotonic() + step.duration_sec
            print(f"step={step.name}")
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.0)
                failure_reason = node.motion_failure_reason()
                if failure_reason is not None:
                    print(
                        f"ABORT: {failure_reason}; {node.state_diagnostics()}",
                        file=sys.stderr,
                    )
                    outcome = failure_reason
                    exit_code = 5
                    log_event(
                        "motion_failed",
                        f"reason={failure_reason} {node.state_diagnostics()}",
                    )
                    return exit_code
                node.publish(setpoint_for_step(step))
                time.sleep(period)
        outcome = "motion_complete"
        exit_code = 0
        log_event("motion_complete", node.state_diagnostics())
        return exit_code
    except KeyboardInterrupt:
        print("Interrupted: final zero burst will be sent.", file=sys.stderr)
        outcome = "keyboard_interrupt"
        exit_code = 130
        log_event("diagnostic_interrupted", f"reason={outcome}")
        if auto_machine is not None:
            recovery_code, recovery_outcome = recover_auto_after_exception(
                auto_machine, outcome
            )
            if recovery_code == 8:
                exit_code = recovery_code
                outcome = recovery_outcome
        return exit_code
    except Exception as exc:
        outcome = "runtime_exception"
        exit_code = 7
        log_event("diagnostic_failed", f"reason={outcome} exception={exc!r}")
        if auto_machine is not None:
            recovery_code, recovery_outcome = recover_auto_after_exception(
                auto_machine, outcome
            )
            if recovery_code == 8:
                exit_code = recovery_code
                outcome = recovery_outcome
        return exit_code
    finally:
        if rclpy.ok():
            log_event("final_zero_burst_start")
            publish_zero_burst()
            log_event("final_zero_burst_complete")
        else:
            log_event("final_zero_burst_unavailable", "reason=ros_context_shutdown")
        log_event("t_end", f"outcome={outcome} exit_code={exit_code}")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        diagnostic = select_diagnostic(args)
        require_live_confirmation(args)
    except ValueError as exc:
        print(f"REFUSED: {exc}", file=sys.stderr)
        return 2
    print_diagnostic(diagnostic)
    print(f"surface={args.surface}")
    if not args.execute:
        print("DRY RUN ONLY: no ROS import and no setpoint publication.")
        return 0
    return run_live(diagnostic, args.namespace)


if __name__ == "__main__":
    raise SystemExit(main())
