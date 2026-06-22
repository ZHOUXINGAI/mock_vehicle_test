#!/usr/bin/env python3

"""Conservative MAVROS Offboard smoke test for the real rover."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

try:
    import rclpy
    from geometry_msgs.msg import Twist
    from geometry_msgs.msg import TwistStamped
    from mavros_msgs.msg import State
    from mavros_msgs.msg import StatusText
    from mavros_msgs.srv import CommandBool
    from mavros_msgs.srv import SetMode
    from rcl_interfaces.msg import ParameterDescriptor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
except ImportError:  # pragma: no cover
    rclpy = None
    Twist = None
    TwistStamped = None
    State = None
    StatusText = None
    CommandBool = None
    SetMode = None
    ParameterDescriptor = None
    Node = object
    qos_profile_sensor_data = 10


@dataclass(frozen=True)
class SmokeStep:
    name: str
    duration_sec: float
    linear_x_mps: float
    linear_y_mps: float
    yaw_rate_radps: float


class RealRoverMavrosOffboardSmoke(Node):
    def __init__(self) -> None:
        super().__init__("real_rover_mavros_offboard_smoke")

        dynamic_param = ParameterDescriptor(dynamic_typing=True)

        def declare_param(name: str, default: object) -> None:
            self.declare_parameter(name, default, dynamic_param)

        declare_param("mavros_namespace", "/mavros")
        declare_param("command_rate_hz", 20.0)
        declare_param("publish_unstamped_cmd_vel", True)
        declare_param("warmup_sec", 2.0)
        declare_param("initial_stop_sec", -1.0)
        declare_param("stop_sec", 1.0)
        declare_param("forward_sec", 1.0)
        declare_param("backward_sec", 1.0)
        declare_param("turn_sec", 0.5)
        declare_param("turn_left_sec", -1.0)
        declare_param("turn_right_sec", -1.0)
        declare_param("final_stop_sec", 2.0)
        declare_param("linear_speed_mps", 0.12)
        declare_param("linear_direction_sign", -1.0)
        declare_param("turn_linear_speed_mps", 0.0)
        declare_param("turn_linear_direction_sign", 1.0)
        declare_param("turn_lateral_speed_mps", 0.0)
        declare_param("turn_yaw_rate_radps", 0.25)
        declare_param("turn_sign", 1.0)
        declare_param("max_linear_speed_mps", 0.30)
        declare_param("max_yaw_rate_radps", 0.70)
        declare_param("mode_change_on_start", False)
        declare_param("arm_on_start", False)
        declare_param("require_armed_before_mode_change", False)
        declare_param("mode_request_retry_sec", 2.0)
        declare_param("arm_request_retry_sec", 2.0)
        declare_param("disarm_on_finish", False)
        declare_param("require_connected", True)
        declare_param("require_offboard_mode", True)
        declare_param("require_armed", True)
        declare_param("abort_on_mode_exit", True)
        declare_param("abort_on_disarm", True)
        declare_param("abort_on_arm_rejected", True)
        declare_param("max_wait_for_ready_sec", 60.0)
        declare_param("stop_burst_sec", 0.8)
        declare_param("test_surface", "wheels_lifted")
        declare_param("confirm_wheels_lifted", False)
        declare_param("confirm_ground_area_clear", False)
        declare_param("confirm_low_speed_ground_test", False)
        declare_param("confirm_rc_ready", False)
        declare_param("confirm_param_backup", False)

        self.mavros_namespace = str(
            self.get_parameter("mavros_namespace").value
        ).rstrip("/")
        self.command_rate_hz = max(2.0, float(self.get_parameter("command_rate_hz").value))
        self.period_sec = 1.0 / self.command_rate_hz
        self.publish_unstamped_cmd_vel = self._as_bool(
            self.get_parameter("publish_unstamped_cmd_vel").value
        )
        self.warmup_sec = max(0.5, float(self.get_parameter("warmup_sec").value))
        initial_stop_sec = float(self.get_parameter("initial_stop_sec").value)
        self.stop_sec = max(0.2, float(self.get_parameter("stop_sec").value))
        self.initial_stop_sec = (
            self.stop_sec if initial_stop_sec < 0.0 else max(0.0, initial_stop_sec)
        )
        self.forward_sec = max(0.0, float(self.get_parameter("forward_sec").value))
        self.backward_sec = max(0.0, float(self.get_parameter("backward_sec").value))
        self.turn_sec = max(0.0, float(self.get_parameter("turn_sec").value))
        turn_left_sec = float(self.get_parameter("turn_left_sec").value)
        turn_right_sec = float(self.get_parameter("turn_right_sec").value)
        self.turn_left_sec = self.turn_sec if turn_left_sec < 0.0 else max(0.0, turn_left_sec)
        self.turn_right_sec = (
            self.turn_sec if turn_right_sec < 0.0 else max(0.0, turn_right_sec)
        )
        self.final_stop_sec = max(0.5, float(self.get_parameter("final_stop_sec").value))
        self.linear_speed_mps = abs(float(self.get_parameter("linear_speed_mps").value))
        self.linear_direction_sign = (
            1.0
            if float(self.get_parameter("linear_direction_sign").value) >= 0.0
            else -1.0
        )
        self.turn_linear_speed_mps = abs(
            float(self.get_parameter("turn_linear_speed_mps").value)
        )
        self.turn_linear_direction_sign = (
            1.0
            if float(self.get_parameter("turn_linear_direction_sign").value) >= 0.0
            else -1.0
        )
        self.turn_lateral_speed_mps = abs(
            float(self.get_parameter("turn_lateral_speed_mps").value)
        )
        self.turn_yaw_rate_radps = abs(
            float(self.get_parameter("turn_yaw_rate_radps").value)
        )
        self.turn_sign = 1.0 if float(self.get_parameter("turn_sign").value) >= 0.0 else -1.0
        self.max_linear_speed_mps = abs(
            float(self.get_parameter("max_linear_speed_mps").value)
        )
        self.max_yaw_rate_radps = abs(
            float(self.get_parameter("max_yaw_rate_radps").value)
        )
        self.mode_change_on_start = self._as_bool(
            self.get_parameter("mode_change_on_start").value
        )
        self.arm_on_start = self._as_bool(self.get_parameter("arm_on_start").value)
        self.require_armed_before_mode_change = self._as_bool(
            self.get_parameter("require_armed_before_mode_change").value
        )
        self.mode_request_retry_sec = max(
            0.5, float(self.get_parameter("mode_request_retry_sec").value)
        )
        self.arm_request_retry_sec = max(
            0.5, float(self.get_parameter("arm_request_retry_sec").value)
        )
        self.disarm_on_finish = self._as_bool(
            self.get_parameter("disarm_on_finish").value
        )
        self.require_connected = self._as_bool(
            self.get_parameter("require_connected").value
        )
        self.require_offboard_mode = self._as_bool(
            self.get_parameter("require_offboard_mode").value
        )
        self.require_armed = self._as_bool(self.get_parameter("require_armed").value)
        self.abort_on_mode_exit = self._as_bool(
            self.get_parameter("abort_on_mode_exit").value
        )
        self.abort_on_disarm = self._as_bool(
            self.get_parameter("abort_on_disarm").value
        )
        self.abort_on_arm_rejected = self._as_bool(
            self.get_parameter("abort_on_arm_rejected").value
        )
        self.max_wait_for_ready_sec = max(
            0.0, float(self.get_parameter("max_wait_for_ready_sec").value)
        )
        self.stop_burst_sec = max(0.2, float(self.get_parameter("stop_burst_sec").value))
        self.test_surface = str(self.get_parameter("test_surface").value).strip().lower()
        self.confirm_wheels_lifted = self._as_bool(
            self.get_parameter("confirm_wheels_lifted").value
        )
        self.confirm_ground_area_clear = self._as_bool(
            self.get_parameter("confirm_ground_area_clear").value
        )
        self.confirm_low_speed_ground_test = self._as_bool(
            self.get_parameter("confirm_low_speed_ground_test").value
        )
        self.confirm_rc_ready = self._as_bool(self.get_parameter("confirm_rc_ready").value)
        self.confirm_param_backup = self._as_bool(
            self.get_parameter("confirm_param_backup").value
        )

        self._validate()

        self.state: Optional[State] = None
        self.sequence = self._build_sequence()
        self.step_index = 0
        self.start_time_sec = self._now_sec()
        self.step_start_sec: Optional[float] = None
        self.last_wait_log_sec = 0.0
        self.mode_request_sent = False
        self.arm_request_sent = False
        self.last_mode_request_sec = -1.0e9
        self.last_arm_request_sec = -1.0e9
        self.disarm_request_sent = False
        self.done = False
        self.last_state_summary: Optional[tuple[bool, bool, str, bool]] = None
        self.offboard_seen = False
        self.armed_seen = False

        self.cmd_vel_pub = self.create_publisher(
            TwistStamped,
            f"{self.mavros_namespace}/setpoint_velocity/cmd_vel",
            10,
        )
        self.cmd_vel_unstamped_pub = self.create_publisher(
            Twist,
            f"{self.mavros_namespace}/setpoint_velocity/cmd_vel_unstamped",
            10,
        )
        self.create_subscription(
            State,
            f"{self.mavros_namespace}/state",
            self._state_cb,
            10,
        )
        self.create_subscription(
            StatusText,
            f"{self.mavros_namespace}/statustext/recv",
            self._statustext_cb,
            qos_profile_sensor_data,
        )
        self.arming_client = self.create_client(
            CommandBool,
            f"{self.mavros_namespace}/cmd/arming",
        )
        self.set_mode_client = self.create_client(
            SetMode,
            f"{self.mavros_namespace}/set_mode",
        )
        self.timer = self.create_timer(self.period_sec, self._timer_cb)

        self.get_logger().warn(
            f"MAVROS rover smoke test loaded: "
            f"ns={self.mavros_namespace} rate={self.command_rate_hz:.1f}Hz "
            f"surface={self.test_surface}"
        )
        self.get_logger().warn(
            f"control flags: mode_change_on_start={self.mode_change_on_start} "
            f"require_armed_before_mode_change={self.require_armed_before_mode_change} "
            f"arm_on_start={self.arm_on_start} "
            f"require_offboard_mode={self.require_offboard_mode} "
            f"require_armed={self.require_armed} "
            f"mode_request_retry_sec={self.mode_request_retry_sec:.1f} "
            f"publish_unstamped_cmd_vel={self.publish_unstamped_cmd_vel} "
            f"linear_direction_sign={self.linear_direction_sign:.0f} "
            f"turn_vector=({self.turn_linear_speed_mps * self.turn_linear_direction_sign:.3f}, "
            f"{self.turn_lateral_speed_mps:.3f})"
        )
        self._log_sequence()

    def _validate(self) -> None:
        missing = []
        if self.test_surface not in {"wheels_lifted", "ground"}:
            raise ValueError("test_surface must be 'wheels_lifted' or 'ground'")
        if self.test_surface == "wheels_lifted" and not self.confirm_wheels_lifted:
            missing.append("CONFIRM_WHEELS_LIFTED=true")
        if self.test_surface == "ground":
            if not self.confirm_ground_area_clear:
                missing.append("CONFIRM_GROUND_AREA_CLEAR=true")
            if not self.confirm_low_speed_ground_test:
                missing.append("CONFIRM_LOW_SPEED_GROUND_TEST=true")
        if not self.confirm_rc_ready:
            missing.append("CONFIRM_RC_READY=true")
        if not self.confirm_param_backup:
            missing.append("CONFIRM_PARAM_BACKUP=true")
        if missing:
            raise RuntimeError(
                "Refusing to run real-hardware motion test until safety is confirmed: "
                + ", ".join(missing)
            )
        if self.linear_speed_mps > self.max_linear_speed_mps:
            raise ValueError("linear_speed_mps exceeds max_linear_speed_mps")
        if self.turn_linear_speed_mps > self.max_linear_speed_mps:
            raise ValueError("turn_linear_speed_mps exceeds max_linear_speed_mps")
        if self.turn_lateral_speed_mps > self.max_linear_speed_mps:
            raise ValueError("turn_lateral_speed_mps exceeds max_linear_speed_mps")
        if self.turn_yaw_rate_radps > self.max_yaw_rate_radps:
            raise ValueError("turn_yaw_rate_radps exceeds max_yaw_rate_radps")

    def _build_sequence(self) -> list[SmokeStep]:
        forward_x = self.linear_speed_mps * self.linear_direction_sign
        turn_x = self.turn_linear_speed_mps * self.turn_linear_direction_sign
        turn = self.turn_yaw_rate_radps * self.turn_sign
        turn_y = self.turn_lateral_speed_mps * self.turn_sign
        sequence = []
        if self.initial_stop_sec > 0.0:
            sequence.append(SmokeStep("initial_stop", self.initial_stop_sec, 0.0, 0.0, 0.0))
        if self.forward_sec > 0.0:
            sequence.append(SmokeStep("forward", self.forward_sec, forward_x, 0.0, 0.0))
            sequence.append(SmokeStep("stop_after_forward", self.stop_sec, 0.0, 0.0, 0.0))
        if self.backward_sec > 0.0:
            sequence.append(SmokeStep("backward", self.backward_sec, -forward_x, 0.0, 0.0))
            sequence.append(SmokeStep("stop_after_backward", self.stop_sec, 0.0, 0.0, 0.0))
        if self.turn_left_sec > 0.0:
            sequence.append(SmokeStep("turn_left", self.turn_left_sec, turn_x, turn_y, turn))
            sequence.append(SmokeStep("stop_after_left", self.stop_sec, 0.0, 0.0, 0.0))
        if self.turn_right_sec > 0.0:
            sequence.append(SmokeStep("turn_right", self.turn_right_sec, turn_x, -turn_y, -turn))
            sequence.append(SmokeStep("stop_after_right", self.stop_sec, 0.0, 0.0, 0.0))
        sequence.append(SmokeStep("final_stop", self.final_stop_sec, 0.0, 0.0, 0.0))
        return sequence

    def _timer_cb(self) -> None:
        if self.done:
            return

        now = self._now_sec()
        self._maybe_request_mode_and_arm(now)

        if self.step_start_sec is None:
            self._publish_velocity(0.0, 0.0, 0.0)
            if self._abort_if_control_lost():
                return
            if self._ready_to_start(now):
                self.step_start_sec = now
                self.step_index = 0
                self.get_logger().warn("starting MAVROS smoke sequence")
            else:
                self._log_waiting(now)
            return

        if self._abort_if_control_lost():
            return

        step = self.sequence[self.step_index]
        self._publish_velocity(
            step.linear_x_mps, step.linear_y_mps, step.yaw_rate_radps
        )
        if now - self.step_start_sec < step.duration_sec:
            return

        self.step_index += 1
        if self.step_index >= len(self.sequence):
            self._publish_velocity(0.0, 0.0, 0.0)
            self._maybe_disarm_on_finish()
            self.get_logger().warn("MAVROS smoke sequence complete; stop command sent")
            self.done = True
            return

        self.step_start_sec = now
        self.get_logger().info(f"step -> {self.sequence[self.step_index].name}")

    def _ready_to_start(self, now: float) -> bool:
        if self.require_connected and not self._is_connected():
            return self._check_timeout(now, "MAVROS connection")
        if self.require_offboard_mode and not self._is_offboard_mode():
            return self._check_timeout(now, "OFFBOARD mode")
        if self.require_armed and not self._is_armed():
            return self._check_timeout(now, "armed state")
        if now - self.start_time_sec < self.warmup_sec:
            return False
        return True

    def _check_timeout(self, now: float, waiting_for: str) -> bool:
        if self.max_wait_for_ready_sec <= 0.0:
            return False
        if now - self.start_time_sec <= self.max_wait_for_ready_sec:
            return False
        self.get_logger().error(f"timed out waiting for {waiting_for}")
        self._publish_velocity(0.0, 0.0, 0.0)
        self.done = True
        return False

    def _maybe_request_mode_and_arm(self, now: float) -> None:
        if now - self.start_time_sec < self.warmup_sec:
            return
        if self.mode_change_on_start and not self._is_offboard_mode():
            if self.require_armed_before_mode_change and not self._is_armed():
                return
            if now - self.last_mode_request_sec >= self.mode_request_retry_sec:
                if self._request_offboard():
                    self.mode_request_sent = True
                    self.last_mode_request_sec = now
        if self.arm_on_start and not self._is_armed():
            if self.require_offboard_mode and not self._is_offboard_mode():
                return
            if now - self.last_arm_request_sec >= self.arm_request_retry_sec:
                if self._request_arm(True):
                    self.arm_request_sent = True
                    self.last_arm_request_sec = now

    def _request_offboard(self) -> bool:
        if not self.set_mode_client.service_is_ready():
            self.get_logger().warn("set_mode service is not ready yet")
            return False
        request = SetMode.Request()
        request.base_mode = 0
        request.custom_mode = "OFFBOARD"
        future = self.set_mode_client.call_async(request)
        future.add_done_callback(self._log_set_mode_result)
        self.get_logger().warn("requested OFFBOARD mode")
        return True

    def _request_arm(self, arm: bool) -> bool:
        if not self.arming_client.service_is_ready():
            self.get_logger().warn("arming service is not ready yet")
            return False
        request = CommandBool.Request()
        request.value = bool(arm)
        future = self.arming_client.call_async(request)
        future.add_done_callback(
            lambda done_future, requested_arm=arm: self._log_arming_result(
                done_future, requested_arm
            )
        )
        self.get_logger().warn("requested ARM" if arm else "requested DISARM")
        return True

    def _log_set_mode_result(self, future: object) -> None:
        try:
            result = future.result()
        except Exception as exc:  # pragma: no cover - depends on ROS runtime
            self.get_logger().error(f"OFFBOARD mode request failed: {exc}")
            return
        self.get_logger().warn(
            f"OFFBOARD mode request response: mode_sent={result.mode_sent}"
        )

    def _log_arming_result(self, future: object, requested_arm: bool) -> None:
        action = "ARM" if requested_arm else "DISARM"
        try:
            result = future.result()
        except Exception as exc:  # pragma: no cover - depends on ROS runtime
            self.get_logger().error(f"{action} request failed: {exc}")
            return
        self.get_logger().warn(
            f"{action} request response: "
            f"success={getattr(result, 'success', 'UNKNOWN')} "
            f"result={getattr(result, 'result', 'UNKNOWN')}"
        )
        if requested_arm and self.abort_on_arm_rejected:
            if not bool(getattr(result, "success", False)):
                self._abort_sequence(
                    f"ARM request rejected: result={getattr(result, 'result', 'UNKNOWN')}"
                )

    def _abort_if_control_lost(self) -> bool:
        if self.state is None:
            return False
        if (
            self.abort_on_mode_exit
            and self.require_offboard_mode
            and self.offboard_seen
            and not self._is_offboard_mode()
        ):
            self._abort_sequence(
                f"mode changed away from OFFBOARD: {self.state.mode}"
            )
            return True
        if (
            self.abort_on_disarm
            and self.require_armed
            and self.armed_seen
            and not self._is_armed()
        ):
            self._abort_sequence("vehicle disarmed")
            return True
        return False

    def _abort_sequence(self, reason: str) -> None:
        self._publish_velocity(0.0, 0.0, 0.0)
        self.get_logger().error(
            f"aborting smoke sequence: {reason}; stop command sent"
        )
        self.done = True

    def _maybe_disarm_on_finish(self) -> None:
        if self.disarm_on_finish and not self.disarm_request_sent:
            self._request_arm(False)
            self.disarm_request_sent = True

    def _publish_velocity(
        self, linear_x_mps: float, linear_y_mps: float, yaw_rate_radps: float
    ) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x = float(linear_x_mps)
        msg.twist.linear.y = float(linear_y_mps)
        msg.twist.linear.z = 0.0
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = float(yaw_rate_radps)
        self.cmd_vel_pub.publish(msg)
        if self.publish_unstamped_cmd_vel:
            unstamped_msg = Twist()
            unstamped_msg.linear.x = float(linear_x_mps)
            unstamped_msg.linear.y = float(linear_y_mps)
            unstamped_msg.linear.z = 0.0
            unstamped_msg.angular.x = 0.0
            unstamped_msg.angular.y = 0.0
            unstamped_msg.angular.z = float(yaw_rate_radps)
            self.cmd_vel_unstamped_pub.publish(unstamped_msg)

    def publish_stop_burst(self, duration_sec: float) -> None:
        end_sec = self._now_sec() + max(0.2, duration_sec)
        while rclpy.ok() and self._now_sec() < end_sec:
            self._publish_velocity(0.0, 0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=self.period_sec)

    def _state_cb(self, msg: State) -> None:
        self.state = msg
        if msg.mode.upper() == "OFFBOARD":
            self.offboard_seen = True
        if msg.armed:
            self.armed_seen = True
        summary = (bool(msg.connected), bool(msg.armed), str(msg.mode), bool(msg.guided))
        if summary != self.last_state_summary:
            self.last_state_summary = summary
            self.get_logger().warn(
                f"state changed: connected={msg.connected} "
                f"mode={msg.mode} armed={msg.armed} guided={msg.guided}"
            )

    def _statustext_cb(self, msg: StatusText) -> None:
        text = str(msg.text).strip()
        if not text:
            return
        self.get_logger().warn(
            f"PX4 STATUSTEXT severity={msg.severity}: {text}"
        )

    def _is_connected(self) -> bool:
        return bool(self.state and self.state.connected)

    def _is_offboard_mode(self) -> bool:
        return bool(self.state and self.state.mode.upper() == "OFFBOARD")

    def _is_armed(self) -> bool:
        return bool(self.state and self.state.armed)

    def _log_waiting(self, now: float) -> None:
        if now - self.last_wait_log_sec < 1.0:
            return
        self.last_wait_log_sec = now
        mode = self.state.mode if self.state else "UNKNOWN"
        stamped_subs = self.cmd_vel_pub.get_subscription_count()
        unstamped_subs = self.cmd_vel_unstamped_pub.get_subscription_count()
        self.get_logger().info(
            f"holding stop; connected={self._is_connected()} "
            f"mode={mode} armed={self._is_armed()} "
            f"setpoint_subs=stamped:{stamped_subs} unstamped:{unstamped_subs}"
        )

    def _log_sequence(self) -> None:
        self.get_logger().info("sequence:")
        for index, step in enumerate(self.sequence, start=1):
            self.get_logger().info(
                f"  {index:02d} {step.name}: {step.duration_sec:.2f}s "
                f"vx={step.linear_x_mps:.3f} "
                f"vy={step.linear_y_mps:.3f} "
                f"yaw_rate={step.yaw_rate_radps:.3f}"
            )

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    @staticmethod
    def _as_bool(value: object) -> bool:
        if isinstance(value, bool):
            return value
        if isinstance(value, (int, float)):
            return bool(value)
        if isinstance(value, str):
            normalized = value.strip().lower()
            if normalized in {"true", "1", "yes", "y", "on"}:
                return True
            if normalized in {"false", "0", "no", "n", "off", ""}:
                return False
        raise ValueError(f"Cannot parse boolean parameter value: {value!r}")


def main() -> None:
    if (
        rclpy is None
        or Twist is None
        or TwistStamped is None
        or State is None
        or StatusText is None
    ):
        raise SystemExit(
            "MAVROS dependencies are not importable. Install/source ros-humble-mavros."
        )

    rclpy.init()
    node = RealRoverMavrosOffboardSmoke()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().warn("interrupted; sending stop burst")
    finally:
        if rclpy.ok():
            node.publish_stop_burst(node.stop_burst_sec)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
