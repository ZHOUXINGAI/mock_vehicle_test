#!/usr/bin/env python3

"""Fail-closed, wheels-lifted-only Offboard setpoint smoke test for Orin2.

The program never arms, disarms, changes flight mode, or writes parameters.
Dry-run is the default and does not import ROS.  Live publishing requires an
explicit execution flag and an exact physical-safety confirmation phrase.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
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
STATE_TIMEOUT_SEC = 0.50
ZERO_BURST_SEC = 1.0
MIN_ZERO_ONLY_HOLD_SEC = 1.0
MAX_ZERO_ONLY_HOLD_SEC = 60.0


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


Diagnostic = MotionDiagnostic | ZeroOnlyDiagnostic


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
        "--forward-only",
        action="store_true",
        help="run forward/stop only; omit all turn steps",
    )
    return parser.parse_args(argv)


def select_diagnostic(args: argparse.Namespace) -> Diagnostic:
    if args.zero_only_hold_sec is not None:
        hold_sec = validate_zero_only_hold_sec(args.zero_only_hold_sec)
        if args.forward_only or args.forward_sec != 1.0:
            raise ValueError(
                "zero-only diagnostic cannot be combined with motion options"
            )
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
    if isinstance(diagnostic, ZeroOnlyDiagnostic):
        print("ZERO-ONLY DIAGNOSTIC")
        print(f"hold_sec={diagnostic.hold_sec:.3f}")
        print("Action plan is empty; no nonzero setpoint can be published.")
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
        from rcl_interfaces.srv import GetParameters
        from rclpy.node import Node
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
                10,
            )
            self.frame_client = self.create_client(
                GetParameters, f"{namespace}/setpoint_velocity/get_parameters"
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
        "zero_only" if isinstance(diagnostic, ZeroOnlyDiagnostic) else "motion"
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

    try:
        if not body_ned_is_configured():
            outcome = "body_ned_check_failed"
            exit_code = 6
            log_event("diagnostic_failed", f"reason={outcome}")
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
