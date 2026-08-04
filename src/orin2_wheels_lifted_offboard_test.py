#!/usr/bin/env python3

"""Fail-closed, wheels-lifted-only Offboard setpoint smoke test for Orin2.

The program never arms, disarms, changes flight mode, or writes parameters.
Dry-run is the default and does not import ROS.  Live publishing requires an
explicit execution flag and an exact physical-safety confirmation phrase.
"""

from __future__ import annotations

import argparse
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


@dataclass(frozen=True)
class Step:
    name: str
    duration_sec: float
    linear_x_mps: float
    linear_y_mps: float


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
        "--forward-only",
        action="store_true",
        help="run stop/forward/stop only; omit all turn steps",
    )
    return parser.parse_args(argv)


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


def run_live(plan: tuple[Step, ...], namespace: str) -> int:
    try:
        import rclpy
        from geometry_msgs.msg import TwistStamped
        from mavros_msgs.msg import State
        from rcl_interfaces.srv import GetParameters
        from rclpy.node import Node
        from rclpy.signals import SignalHandlerOptions
    except ImportError as exc:
        print(f"ROS/MAVROS imports unavailable: {exc}", file=sys.stderr)
        return 3

    namespace = "/" + namespace.strip("/")

    class WheelsLiftedNode(Node):
        def __init__(self) -> None:
            super().__init__("orin2_wheels_lifted_offboard_test")
            self.state = None
            self.state_rx_monotonic = 0.0
            self.safe_prestate_seen = False
            self.publisher = self.create_publisher(
                TwistStamped, f"{namespace}/setpoint_velocity/cmd_vel", 10
            )
            self.create_subscription(State, f"{namespace}/state", self.on_state, 10)
            self.frame_client = self.create_client(
                GetParameters, f"{namespace}/setpoint_velocity/get_parameters"
            )

        def on_state(self, message: State) -> None:
            self.state = message
            self.state_rx_monotonic = time.monotonic()
            if (
                message.connected
                and not message.armed
                and message.manual_input
                and message.mode.upper() == "MANUAL"
            ):
                self.safe_prestate_seen = True

        def state_is_fresh(self) -> bool:
            return (
                self.state is not None
                and time.monotonic() - self.state_rx_monotonic <= STATE_TIMEOUT_SEC
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
                return "state=missing"
            age_sec = time.monotonic() - self.state_rx_monotonic
            return (
                f"state_age={age_sec:.3f}s connected={self.state.connected} "
                f"armed={self.state.armed} mode={self.state.mode!r} "
                f"safe_manual_prestate_seen={self.safe_prestate_seen}"
            )

        def ready_for_motion(self) -> bool:
            return self.motion_failure_reason() is None

        def publish(self, linear_x: float, linear_y: float) -> None:
            message = TwistStamped()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = "base_link"
            message.twist.linear.x = linear_x
            message.twist.linear.y = linear_y
            message.twist.angular.z = 0.0
            self.publisher.publish(message)

    # Keep SIGINT under this program's control so the final zero burst can be
    # published before the ROS context is shut down.
    rclpy.init(args=None, signal_handler_options=SignalHandlerOptions.NO)
    node = WheelsLiftedNode()
    period = 1.0 / PUBLISH_RATE_HZ

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
            node.publish(0.0, 0.0)
            time.sleep(period)

    try:
        if not body_ned_is_configured():
            return 6
        print(
            "Publishing zero prestream only; first require "
            "connected+MANUAL+disarmed+manual_input, then wait for "
            "human-controlled Arm + OFFBOARD."
        )
        ready_deadline = time.monotonic() + 60.0
        while rclpy.ok() and time.monotonic() < ready_deadline:
            rclpy.spin_once(node, timeout_sec=0.0)
            node.publish(0.0, 0.0)
            if node.ready_for_motion():
                break
            time.sleep(period)
        else:
            print(
                "Refused: safe MANUAL prestate followed by fresh connected+armed+OFFBOARD "
                "was not observed.",
                file=sys.stderr,
            )
            return 4

        print(
            "Fresh armed+OFFBOARD observed; starting the first bounded motion "
            "step immediately (no post-authorization zero dwell)."
        )
        for step in plan:
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
                    publish_zero_burst()
                    return 5
                node.publish(step.linear_x_mps, step.linear_y_mps)
                time.sleep(period)
        publish_zero_burst()
        return 0
    except KeyboardInterrupt:
        print("Interrupted: sending final zero burst.", file=sys.stderr)
        publish_zero_burst()
        return 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    plan = build_plan(
        forward_sec=args.forward_sec,
        forward_only=args.forward_only,
    )
    try:
        validate_plan(plan)
        require_live_confirmation(args)
    except ValueError as exc:
        print(f"REFUSED: {exc}", file=sys.stderr)
        return 2
    print_plan(plan)
    print(f"surface={args.surface}")
    if not args.execute:
        print("DRY RUN ONLY: no ROS import and no setpoint publication.")
        return 0
    return run_live(plan, args.namespace)


if __name__ == "__main__":
    raise SystemExit(main())
