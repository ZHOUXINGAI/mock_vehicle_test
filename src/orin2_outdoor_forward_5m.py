#!/usr/bin/env python3

"""Guarded C2 outdoor MAVROS Offboard mission: forward exactly one 5 m leg."""

from __future__ import annotations

import argparse
import math
import time
from dataclasses import dataclass
from typing import Callable, Sequence


EXECUTE_PHRASE = "OUTDOOR_FORWARD_5M_AREA_CLEAR_RC_KILL_READY"
STATE_MAX_AGE_SEC = 2.0
POSE_MAX_AGE_SEC = 1.0
GPS_MAX_AGE_SEC = 2.0
GPS_RAW_MAX_AGE_SEC = 2.0
MIN_GPS_FIX_TYPE = 3
MIN_SATELLITES_VISIBLE = 6


@dataclass(frozen=True)
class Observation:
    state_present: bool = False
    state_age_sec: float = math.inf
    connected: bool = False
    armed: bool = False
    mode: str = ""
    manual_input: bool = False
    pose_present: bool = False
    pose_age_sec: float = math.inf
    x_m: float = math.nan
    y_m: float = math.nan
    yaw_rad: float = math.nan
    gps_present: bool = False
    gps_age_sec: float = math.inf
    gps_status: int = -1
    latitude_deg: float = math.nan
    longitude_deg: float = math.nan
    gps_raw_present: bool = False
    gps_raw_age_sec: float = math.inf
    gps_fix_type: int = 0
    satellites_visible: int = 0


@dataclass(frozen=True)
class MissionConfig:
    distance_m: float = 5.0
    tolerance_m: float = 0.15
    speed_mps: float = 0.12
    max_speed_mps: float = 0.15
    max_cross_track_m: float = 0.75
    max_heading_error_deg: float = 35.0
    max_motion_sec: float = 75.0
    stall_window_sec: float = 8.0
    stall_min_progress_m: float = 0.08


def validate_config(config: MissionConfig) -> None:
    values = vars(config).values()
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError("mission configuration must contain only finite values")
    if not 0.5 <= config.distance_m <= 10.0:
        raise ValueError("distance_m must be within [0.5, 10.0]")
    if not 0.0 <= config.tolerance_m < config.distance_m:
        raise ValueError("tolerance_m must be nonnegative and below distance_m")
    if not 0.0 < config.speed_mps <= config.max_speed_mps <= 0.25:
        raise ValueError("speed must be positive and no greater than the 0.25 m/s rover limit")
    if not 0.1 <= config.max_cross_track_m <= 2.0:
        raise ValueError("max_cross_track_m must be within [0.1, 2.0]")
    if not 5.0 <= config.max_heading_error_deg <= 60.0:
        raise ValueError("max_heading_error_deg must be within [5, 60]")
    if not 10.0 <= config.max_motion_sec <= 180.0:
        raise ValueError("max_motion_sec must be within [10, 180]")
    if not 2.0 <= config.stall_window_sec <= 30.0:
        raise ValueError("stall_window_sec must be within [2, 30]")
    if not 0.01 <= config.stall_min_progress_m <= 1.0:
        raise ValueError("stall_min_progress_m must be within [0.01, 1.0]")


def _finite_pose(observation: Observation) -> bool:
    return all(
        math.isfinite(value)
        for value in (observation.x_m, observation.y_m, observation.yaw_rad)
    )


def _finite_gps(observation: Observation) -> bool:
    return all(
        math.isfinite(value)
        for value in (observation.latitude_deg, observation.longitude_deg)
    )


def navigation_ready(observation: Observation) -> bool:
    return bool(
        observation.state_present
        and observation.state_age_sec <= STATE_MAX_AGE_SEC
        and observation.connected
        and observation.pose_present
        and observation.pose_age_sec <= POSE_MAX_AGE_SEC
        and _finite_pose(observation)
        and observation.gps_present
        and observation.gps_age_sec <= GPS_MAX_AGE_SEC
        and observation.gps_status >= 0
        and _finite_gps(observation)
        and observation.gps_raw_present
        and observation.gps_raw_age_sec <= GPS_RAW_MAX_AGE_SEC
        and observation.gps_fix_type >= MIN_GPS_FIX_TYPE
        and observation.satellites_visible >= MIN_SATELLITES_VISIBLE
    )


def safe_manual_prestate(observation: Observation) -> bool:
    return bool(
        navigation_ready(observation)
        and not observation.armed
        and observation.mode.upper() == "MANUAL"
        and observation.manual_input
    )


def manual_arm_ready(observation: Observation) -> bool:
    return bool(
        navigation_ready(observation)
        and observation.armed
        and observation.mode.upper() == "MANUAL"
    )


def motion_fault(observation: Observation) -> str | None:
    if not observation.state_present:
        return "state_missing"
    if observation.state_age_sec > STATE_MAX_AGE_SEC:
        return "state_stale"
    if not observation.connected:
        return "mavros_disconnected"
    if not observation.pose_present:
        return "local_pose_missing"
    if observation.pose_age_sec > POSE_MAX_AGE_SEC:
        return "local_pose_stale"
    if not _finite_pose(observation):
        return "local_pose_nonfinite"
    if not observation.gps_present:
        return "gps_missing"
    if observation.gps_age_sec > GPS_MAX_AGE_SEC:
        return "gps_stale"
    if observation.gps_status < 0 or not _finite_gps(observation):
        return "gps_no_fix"
    if not observation.gps_raw_present:
        return "gps_raw_missing"
    if observation.gps_raw_age_sec > GPS_RAW_MAX_AGE_SEC:
        return "gps_raw_stale"
    if observation.gps_fix_type < MIN_GPS_FIX_TYPE:
        return "gps_not_3d"
    if observation.satellites_visible < MIN_SATELLITES_VISIBLE:
        return "gps_satellites_low"
    if not observation.armed:
        return "unexpected_disarm"
    if observation.mode.upper() != "OFFBOARD":
        return "offboard_exit"
    return None


def wrap_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def track_metrics(
    start_x_m: float,
    start_y_m: float,
    start_yaw_rad: float,
    current_x_m: float,
    current_y_m: float,
    current_yaw_rad: float,
) -> tuple[float, float, float, float]:
    dx = current_x_m - start_x_m
    dy = current_y_m - start_y_m
    heading_x = math.cos(start_yaw_rad)
    heading_y = math.sin(start_yaw_rad)
    along_track_m = dx * heading_x + dy * heading_y
    cross_track_m = -dx * heading_y + dy * heading_x
    displacement_m = math.hypot(dx, dy)
    heading_error_rad = wrap_pi(current_yaw_rad - start_yaw_rad)
    return along_track_m, cross_track_m, displacement_m, heading_error_rad


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--confirm", default="")
    parser.add_argument("--namespace", default="/mavros")
    parser.add_argument("--distance-m", type=float, default=5.0)
    parser.add_argument("--tolerance-m", type=float, default=0.15)
    parser.add_argument("--speed-mps", type=float, default=0.12)
    parser.add_argument("--max-speed-mps", type=float, default=0.15)
    parser.add_argument("--max-cross-track-m", type=float, default=0.75)
    parser.add_argument("--max-heading-error-deg", type=float, default=35.0)
    parser.add_argument("--max-motion-sec", type=float, default=75.0)
    parser.add_argument("--stall-window-sec", type=float, default=8.0)
    parser.add_argument("--stall-min-progress-m", type=float, default=0.08)
    parser.add_argument("--recover-only", action="store_true")
    parser.add_argument("--verify-only", action="store_true")
    return parser


def config_from_args(args: argparse.Namespace) -> MissionConfig:
    config = MissionConfig(
        distance_m=args.distance_m,
        tolerance_m=args.tolerance_m,
        speed_mps=args.speed_mps,
        max_speed_mps=args.max_speed_mps,
        max_cross_track_m=args.max_cross_track_m,
        max_heading_error_deg=args.max_heading_error_deg,
        max_motion_sec=args.max_motion_sec,
        stall_window_sec=args.stall_window_sec,
        stall_min_progress_m=args.stall_min_progress_m,
    )
    validate_config(config)
    return config


def print_plan(config: MissionConfig) -> None:
    print("C2 OUTDOOR FORWARD-ONLY OFFBOARD PLAN")
    print(f"distance={config.distance_m:.2f}m speed={config.speed_mps:.3f}m/s")
    print("navigation=real GNSS + real MAVROS local position; fake EV/GPS forbidden")
    print("entry=human Arm in MANUAL, then program requests OFFBOARD")
    print("motion=BODY_NED forward only; no turn, reverse, lateral or yaw command")
    print("exit=zero burst -> Disarm -> MANUAL -> verified safe state")


def run_live(args: argparse.Namespace, config: MissionConfig) -> int:
    try:
        import rclpy
        from geometry_msgs.msg import PoseStamped, TwistStamped
        from mavros_msgs.msg import GPSRAW, State, StatusText
        from mavros_msgs.srv import CommandBool, SetMode
        from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
        from rcl_interfaces.srv import GetParameters, SetParameters
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from rclpy.signals import SignalHandlerOptions
        from sensor_msgs.msg import NavSatFix
    except ImportError as exc:
        print(f"REFUSED: ROS/MAVROS dependency unavailable: {exc}")
        return 2

    namespace = args.namespace.rstrip("/")

    class ForwardNode(Node):
        def __init__(self) -> None:
            super().__init__("orin2_outdoor_forward_5m")
            self.state = None
            self.state_rx = 0.0
            self.pose = None
            self.pose_rx = 0.0
            self.gps = None
            self.gps_rx = 0.0
            self.gps_raw = None
            self.gps_raw_rx = 0.0
            self.last_status = None
            self.publisher = self.create_publisher(
                TwistStamped, f"{namespace}/setpoint_velocity/cmd_vel", 10
            )
            self.create_subscription(State, f"{namespace}/state", self._state_cb, 10)
            self.create_subscription(
                PoseStamped,
                f"{namespace}/local_position/pose",
                self._pose_cb,
                qos_profile_sensor_data,
            )
            self.create_subscription(
                NavSatFix,
                f"{namespace}/global_position/raw/fix",
                self._gps_cb,
                qos_profile_sensor_data,
            )
            self.create_subscription(
                GPSRAW,
                f"{namespace}/gpsstatus/gps1/raw",
                self._gps_raw_cb,
                qos_profile_sensor_data,
            )
            self.create_subscription(
                StatusText,
                f"{namespace}/statustext/recv",
                self._status_cb,
                qos_profile_sensor_data,
            )
            self.get_frame = self.create_client(
                GetParameters, f"{namespace}/setpoint_velocity/get_parameters"
            )
            self.set_frame = self.create_client(
                SetParameters, f"{namespace}/setpoint_velocity/set_parameters"
            )
            self.arming = self.create_client(CommandBool, f"{namespace}/cmd/arming")
            self.set_mode = self.create_client(SetMode, f"{namespace}/set_mode")

        def _state_cb(self, message) -> None:
            self.state = message
            self.state_rx = time.monotonic()

        def _pose_cb(self, message) -> None:
            self.pose = message
            self.pose_rx = time.monotonic()

        def _gps_cb(self, message) -> None:
            self.gps = message
            self.gps_rx = time.monotonic()

        def _gps_raw_cb(self, message) -> None:
            self.gps_raw = message
            self.gps_raw_rx = time.monotonic()

        def _status_cb(self, message) -> None:
            item = (int(message.severity), str(message.text))
            if item != self.last_status:
                self.last_status = item
                print(f"PX4_STATUS severity={item[0]} text={item[1]!r}", flush=True)

        def observation(self) -> Observation:
            now = time.monotonic()
            pose = self.pose.pose if self.pose is not None else None
            yaw = math.nan
            if pose is not None:
                q = pose.orientation
                yaw = math.atan2(
                    2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z),
                )
            return Observation(
                state_present=self.state is not None,
                state_age_sec=now - self.state_rx if self.state is not None else math.inf,
                connected=bool(self.state and self.state.connected),
                armed=bool(self.state and self.state.armed),
                mode=str(self.state.mode) if self.state is not None else "",
                manual_input=bool(self.state and self.state.manual_input),
                pose_present=pose is not None,
                pose_age_sec=now - self.pose_rx if pose is not None else math.inf,
                x_m=float(pose.position.x) if pose is not None else math.nan,
                y_m=float(pose.position.y) if pose is not None else math.nan,
                yaw_rad=yaw,
                gps_present=self.gps is not None,
                gps_age_sec=now - self.gps_rx if self.gps is not None else math.inf,
                gps_status=int(self.gps.status.status) if self.gps is not None else -1,
                latitude_deg=float(self.gps.latitude) if self.gps is not None else math.nan,
                longitude_deg=float(self.gps.longitude) if self.gps is not None else math.nan,
                gps_raw_present=self.gps_raw is not None,
                gps_raw_age_sec=(
                    now - self.gps_raw_rx if self.gps_raw is not None else math.inf
                ),
                gps_fix_type=(
                    int(self.gps_raw.fix_type) if self.gps_raw is not None else 0
                ),
                satellites_visible=(
                    int(self.gps_raw.satellites_visible)
                    if self.gps_raw is not None
                    else 0
                ),
            )

        def publish(self, speed_mps: float) -> None:
            message = TwistStamped()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = "base_link"
            message.twist.linear.x = float(speed_mps)
            message.twist.linear.y = 0.0
            message.twist.linear.z = 0.0
            message.twist.angular.x = 0.0
            message.twist.angular.y = 0.0
            message.twist.angular.z = 0.0
            self.publisher.publish(message)

        def spin_publish(self, duration_sec: float, speed_mps: float = 0.0) -> None:
            deadline = time.monotonic() + max(0.0, duration_sec)
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.0)
                self.publish(speed_mps)
                time.sleep(0.05)

        def wait_for(
            self,
            predicate: Callable[[Observation], bool],
            timeout_sec: float,
            label: str,
            *,
            publish_zero: bool = True,
        ) -> Observation | None:
            deadline = time.monotonic() + timeout_sec
            last_log = 0.0
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.0)
                observation = self.observation()
                if publish_zero:
                    self.publish(0.0)
                if predicate(observation):
                    print(f"GATE_PASS {label} {observation}", flush=True)
                    return observation
                if time.monotonic() - last_log >= 2.0:
                    last_log = time.monotonic()
                    print(f"GATE_WAIT {label} {observation}", flush=True)
                time.sleep(0.05)
            print(f"GATE_FAIL {label} {self.observation()}", flush=True)
            return None

        def _call(self, client, request, label: str, timeout_sec: float = 5.0):
            if not client.wait_for_service(timeout_sec=timeout_sec):
                raise RuntimeError(f"service unavailable: {label}")
            future = client.call_async(request)
            deadline = time.monotonic() + timeout_sec
            while rclpy.ok() and time.monotonic() < deadline and not future.done():
                rclpy.spin_once(self, timeout_sec=0.0)
                self.publish(0.0)
                time.sleep(0.05)
            if not future.done():
                future.cancel()
                raise RuntimeError(f"service timeout: {label}")
            if future.exception() is not None:
                raise RuntimeError(f"service error {label}: {future.exception()}")
            return future.result()

        def configure_body_ned(self) -> None:
            request = SetParameters.Request()
            request.parameters = [
                Parameter(
                    name="mav_frame",
                    value=ParameterValue(
                        type=ParameterType.PARAMETER_STRING,
                        string_value="BODY_NED",
                    ),
                )
            ]
            result = self._call(self.set_frame, request, "set BODY_NED")
            if not result.results or not result.results[0].successful:
                raise RuntimeError("MAVROS rejected BODY_NED")
            verify = GetParameters.Request()
            verify.names = ["mav_frame"]
            values = self._call(self.get_frame, verify, "verify BODY_NED").values
            value = values[0].string_value if values else ""
            if value.upper() not in {"BODY_NED", "8"}:
                raise RuntimeError(f"BODY_NED verification failed: {value!r}")
            print(f"BODY_NED_VERIFIED value={value!r}", flush=True)

        def request_offboard(self) -> None:
            request = SetMode.Request()
            request.custom_mode = "OFFBOARD"
            response = self._call(self.set_mode, request, "request OFFBOARD")
            if not response.mode_sent:
                raise RuntimeError("OFFBOARD request rejected")
            print("OFFBOARD_REQUEST_ACCEPTED", flush=True)

        def recover(self) -> bool:
            self.spin_publish(0.8)
            observation = self.observation()
            if observation.state_present and observation.connected and observation.armed:
                request = CommandBool.Request()
                request.value = False
                response = self._call(self.arming, request, "request Disarm")
                print(
                    f"DISARM_RESPONSE success={response.success} result={response.result}",
                    flush=True,
                )
            if self.wait_for(
                lambda item: item.state_present and item.connected and not item.armed,
                8.0,
                "disarmed",
            ) is None:
                return False
            request = SetMode.Request()
            request.custom_mode = "MANUAL"
            response = self._call(self.set_mode, request, "request MANUAL")
            print(f"MANUAL_RESPONSE mode_sent={response.mode_sent}", flush=True)
            return self.wait_for(
                lambda item: bool(
                    item.state_present
                    and item.state_age_sec <= STATE_MAX_AGE_SEC
                    and item.connected
                    and not item.armed
                    and item.mode.upper() == "MANUAL"
                    and item.manual_input
                ),
                10.0,
                "final MANUAL/disarmed",
            ) is not None

    rclpy.init(args=None, signal_handler_options=SignalHandlerOptions.NO)
    node = ForwardNode()
    primary_error: str | None = None
    result_code = 7
    recovery_required = not args.verify_only and not args.recover_only
    try:
        if args.recover_only:
            result_code = 0 if node.recover() else 8
            return result_code
        if args.verify_only:
            result = node.wait_for(
                safe_manual_prestate,
                15.0,
                "verify safe final state",
                publish_zero=False,
            )
            result_code = 0 if result is not None else 8
            return result_code

        if node.wait_for(safe_manual_prestate, 30.0, "real-GPS MANUAL/disarmed preflight") is None:
            result_code = 3
            return result_code
        node.configure_body_ned()
        print("ZERO_PRESTREAM 2.0s", flush=True)
        node.spin_publish(2.0)
        print("READY_FOR_MANUAL_ARM: Arm once with RC while remaining in MANUAL.", flush=True)
        armed_observation = node.wait_for(manual_arm_ready, 120.0, "manual Arm in MANUAL")
        if armed_observation is None:
            primary_error = "manual_arm_timeout_or_navigation_not_ready"
            result_code = 4
            return result_code

        for attempt in range(1, 4):
            node.request_offboard()
            entered = node.wait_for(
                lambda item: bool(
                    navigation_ready(item)
                    and item.armed
                    and item.mode.upper() == "OFFBOARD"
                ),
                3.0,
                f"OFFBOARD observed attempt={attempt}",
            )
            if entered is not None:
                break
        else:
            primary_error = "offboard_not_observed"
            result_code = 5
            return result_code

        node.spin_publish(0.2)
        start = node.observation()
        start_x, start_y, start_yaw = start.x_m, start.y_m, start.yaw_rad
        print(
            f"FORWARD_START x={start_x:.3f} y={start_y:.3f} "
            f"yaw_deg={math.degrees(start_yaw):.2f}",
            flush=True,
        )
        motion_start = time.monotonic()
        stall_start = motion_start
        stall_along = 0.0
        last_log = 0.0
        completed = False
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            observation = node.observation()
            fault = motion_fault(observation)
            if fault is not None:
                primary_error = fault
                break
            along, cross, displacement, heading_error = track_metrics(
                start_x,
                start_y,
                start_yaw,
                observation.x_m,
                observation.y_m,
                observation.yaw_rad,
            )
            elapsed = time.monotonic() - motion_start
            if abs(cross) > config.max_cross_track_m:
                primary_error = f"cross_track_limit:{cross:.3f}"
                break
            if abs(math.degrees(heading_error)) > config.max_heading_error_deg:
                primary_error = f"heading_error_limit:{math.degrees(heading_error):.2f}"
                break
            if elapsed > config.max_motion_sec:
                primary_error = "motion_timeout"
                break
            if time.monotonic() - stall_start >= config.stall_window_sec:
                if along - stall_along < config.stall_min_progress_m:
                    primary_error = (
                        f"progress_stall:{along - stall_along:.3f}m/"
                        f"{config.stall_window_sec:.1f}s"
                    )
                    break
                stall_start = time.monotonic()
                stall_along = along
            if along >= config.distance_m - config.tolerance_m:
                completed = True
                print(
                    f"FORWARD_TARGET_REACHED along={along:.3f} cross={cross:.3f} "
                    f"displacement={displacement:.3f}",
                    flush=True,
                )
                break
            node.publish(config.speed_mps)
            if time.monotonic() - last_log >= 1.0:
                last_log = time.monotonic()
                print(
                    f"FORWARD_PROGRESS elapsed={elapsed:.1f}s along={along:.3f} "
                    f"cross={cross:.3f} displacement={displacement:.3f} "
                    f"heading_error_deg={math.degrees(heading_error):.2f}",
                    flush=True,
                )
            time.sleep(0.05)

        node.spin_publish(1.0)
        if not completed:
            print(f"FORWARD_ABORT reason={primary_error}", flush=True)
            result_code = 6
            return result_code
        print("FORWARD_COMPLETE; starting safe recovery", flush=True)
        result_code = 0
        return result_code
    except KeyboardInterrupt:
        primary_error = "keyboard_interrupt"
        print("FORWARD_ABORT reason=keyboard_interrupt", flush=True)
        result_code = 130
        return result_code
    except Exception as exc:
        primary_error = f"exception:{exc}"
        print(f"FORWARD_ABORT reason={primary_error}", flush=True)
        result_code = 7
        return result_code
    finally:
        recovery_override: int | None = None
        if recovery_required:
            try:
                safe = node.recover()
                print(f"FINAL_RECOVERY_SAFE={safe} primary_error={primary_error!r}", flush=True)
                if not safe and result_code == 0:
                    recovery_override = 8
            except Exception as exc:
                print(f"FINAL_RECOVERY_SAFE=False recovery_exception={exc!r}", flush=True)
                if result_code == 0:
                    recovery_override = 8
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if recovery_override is not None:
            return recovery_override


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        config = config_from_args(args)
    except ValueError as exc:
        print(f"REFUSED: {exc}")
        return 2
    print_plan(config)
    if not args.execute and not args.recover_only and not args.verify_only:
        print("DRY RUN ONLY: no ROS import, no process start, no setpoint publication.")
        return 0
    if args.execute and args.confirm != EXECUTE_PHRASE:
        print(f"REFUSED: required --confirm {EXECUTE_PHRASE}")
        return 2
    return run_live(args, config)


if __name__ == "__main__":
    raise SystemExit(main())
