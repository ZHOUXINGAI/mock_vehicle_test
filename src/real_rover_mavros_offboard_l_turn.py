#!/usr/bin/env python3

"""Body-frame MAVROS Offboard L-turn task for the real differential rover."""

from __future__ import annotations

import math
from typing import Optional

try:
    import rclpy
    from geometry_msgs.msg import PoseStamped
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
    PoseStamped = None
    Twist = None
    TwistStamped = None
    State = None
    StatusText = None
    CommandBool = None
    SetMode = None
    ParameterDescriptor = None
    Node = object
    qos_profile_sensor_data = 10


class BodyLTurnOffboard(Node):
    def __init__(self) -> None:
        super().__init__("real_rover_mavros_offboard_body_l_turn")

        dynamic_param = ParameterDescriptor(dynamic_typing=True)

        def declare_param(name: str, default: object) -> None:
            self.declare_parameter(name, default, dynamic_param)

        declare_param("mavros_namespace", "/mavros")
        declare_param("command_rate_hz", 20.0)
        declare_param("publish_unstamped_cmd_vel", True)
        declare_param("warmup_sec", 2.0)
        declare_param("initial_stop_sec", 0.5)
        declare_param("stop_after_first_sec", 0.4)
        declare_param("stop_after_turn_sec", 0.4)
        declare_param("final_stop_sec", 1.0)
        declare_param("first_distance_m", 3.0)
        declare_param("second_distance_m", 3.0)
        declare_param("linear_speed_mps", 0.12)
        declare_param("turn_angle_deg", 90.0)
        declare_param("turn_direction_sign", -1.0)
        declare_param("turn_lateral_speed_mps", 0.10)
        declare_param("turn_forward_speed_mps", 0.0)
        declare_param("yaw_tolerance_deg", 12.0)
        declare_param("distance_tolerance_m", 0.12)
        declare_param("first_leg_max_sec", 35.0)
        declare_param("turn_max_sec", 12.0)
        declare_param("second_leg_max_sec", 35.0)
        declare_param("max_linear_speed_mps", 0.20)
        declare_param("max_pose_age_sec", 1.0)
        declare_param("mode_change_on_start", True)
        declare_param("arm_on_start", True)
        declare_param("require_armed_before_mode_change", False)
        declare_param("mode_request_retry_sec", 2.0)
        declare_param("arm_request_retry_sec", 2.0)
        declare_param("disarm_on_finish", True)
        declare_param("require_connected", True)
        declare_param("require_offboard_mode", True)
        declare_param("require_armed", True)
        declare_param("abort_on_mode_exit", True)
        declare_param("abort_on_disarm", True)
        declare_param("abort_on_arm_rejected", True)
        declare_param("max_wait_for_ready_sec", 60.0)
        declare_param("stop_burst_sec", 0.8)
        declare_param("test_surface", "ground")
        declare_param("confirm_ground_area_clear", False)
        declare_param("confirm_low_speed_ground_test", False)
        declare_param("confirm_rc_ready", False)
        declare_param("confirm_param_backup", False)
        declare_param("confirm_real_local_position", False)

        self.mavros_namespace = str(self.get_parameter("mavros_namespace").value).rstrip("/")
        self.command_rate_hz = max(2.0, float(self.get_parameter("command_rate_hz").value))
        self.period_sec = 1.0 / self.command_rate_hz
        self.publish_unstamped_cmd_vel = self._as_bool(
            self.get_parameter("publish_unstamped_cmd_vel").value
        )
        self.warmup_sec = max(0.5, float(self.get_parameter("warmup_sec").value))
        self.initial_stop_sec = max(0.0, float(self.get_parameter("initial_stop_sec").value))
        self.stop_after_first_sec = max(0.0, float(self.get_parameter("stop_after_first_sec").value))
        self.stop_after_turn_sec = max(0.0, float(self.get_parameter("stop_after_turn_sec").value))
        self.final_stop_sec = max(0.2, float(self.get_parameter("final_stop_sec").value))
        self.first_distance_m = max(0.05, float(self.get_parameter("first_distance_m").value))
        self.second_distance_m = max(0.05, float(self.get_parameter("second_distance_m").value))
        self.linear_speed_mps = abs(float(self.get_parameter("linear_speed_mps").value))
        self.turn_angle_rad = math.radians(abs(float(self.get_parameter("turn_angle_deg").value)))
        self.turn_direction_sign = (
            1.0
            if float(self.get_parameter("turn_direction_sign").value) >= 0.0
            else -1.0
        )
        self.turn_lateral_speed_mps = abs(
            float(self.get_parameter("turn_lateral_speed_mps").value)
        )
        self.turn_forward_speed_mps = max(
            0.0, float(self.get_parameter("turn_forward_speed_mps").value)
        )
        self.yaw_tolerance_rad = math.radians(
            max(0.0, float(self.get_parameter("yaw_tolerance_deg").value))
        )
        self.distance_tolerance_m = max(
            0.0, float(self.get_parameter("distance_tolerance_m").value)
        )
        self.first_leg_max_sec = max(0.5, float(self.get_parameter("first_leg_max_sec").value))
        self.turn_max_sec = max(0.5, float(self.get_parameter("turn_max_sec").value))
        self.second_leg_max_sec = max(0.5, float(self.get_parameter("second_leg_max_sec").value))
        self.max_linear_speed_mps = abs(
            float(self.get_parameter("max_linear_speed_mps").value)
        )
        self.max_pose_age_sec = max(
            0.1, float(self.get_parameter("max_pose_age_sec").value)
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
        self.abort_on_disarm = self._as_bool(self.get_parameter("abort_on_disarm").value)
        self.abort_on_arm_rejected = self._as_bool(
            self.get_parameter("abort_on_arm_rejected").value
        )
        self.max_wait_for_ready_sec = max(
            0.0, float(self.get_parameter("max_wait_for_ready_sec").value)
        )
        self.stop_burst_sec = max(0.2, float(self.get_parameter("stop_burst_sec").value))
        self.test_surface = str(self.get_parameter("test_surface").value).strip().lower()
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
        self.confirm_real_local_position = self._as_bool(
            self.get_parameter("confirm_real_local_position").value
        )

        self._validate()

        self.state: Optional[State] = None
        self.pose: Optional[PoseStamped] = None
        self.pose_time_sec = -1.0e9
        self.current_xy: Optional[tuple[float, float]] = None
        self.current_yaw: Optional[float] = None
        self.initial_yaw: Optional[float] = None
        self.turn_start_yaw: Optional[float] = None

        self.stage = "waiting"
        self.stage_start_sec: Optional[float] = None
        self.leg_start_xy: Optional[tuple[float, float]] = None
        self.start_time_sec = self._now_sec()
        self.last_wait_log_sec = 0.0
        self.last_progress_log_sec = 0.0
        self.last_mode_request_sec = -1.0e9
        self.last_arm_request_sec = -1.0e9
        self.disarm_request_sent = False
        self.done = False
        self.offboard_seen = False
        self.armed_seen = False
        self.last_state_summary: Optional[tuple[bool, bool, str, bool]] = None

        self.cmd_vel_pub = self.create_publisher(
            TwistStamped, f"{self.mavros_namespace}/setpoint_velocity/cmd_vel", 10
        )
        self.cmd_vel_unstamped_pub = self.create_publisher(
            Twist, f"{self.mavros_namespace}/setpoint_velocity/cmd_vel_unstamped", 10
        )
        self.create_subscription(
            State, f"{self.mavros_namespace}/state", self._state_cb, 10
        )
        self.create_subscription(
            StatusText,
            f"{self.mavros_namespace}/statustext/recv",
            self._statustext_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PoseStamped,
            f"{self.mavros_namespace}/local_position/pose",
            self._pose_cb,
            qos_profile_sensor_data,
        )
        self.arming_client = self.create_client(
            CommandBool, f"{self.mavros_namespace}/cmd/arming"
        )
        self.set_mode_client = self.create_client(
            SetMode, f"{self.mavros_namespace}/set_mode"
        )
        self.timer = self.create_timer(self.period_sec, self._timer_cb)

        turn_name = "left" if self.turn_direction_sign < 0.0 else "right"
        self.get_logger().warn(
            "body L-turn task loaded: "
            f"first={self.first_distance_m:.2f}m turn={turn_name} "
            f"{math.degrees(self.turn_angle_rad):.1f}deg "
            f"second={self.second_distance_m:.2f}m "
            f"forward_speed={self.linear_speed_mps:.2f}m/s "
            f"turn_body=({self.turn_forward_speed_mps:.2f}, "
            f"{self.turn_direction_sign * self.turn_lateral_speed_mps:.2f})m/s"
        )

    def _validate(self) -> None:
        missing = []
        if self.test_surface != "ground":
            raise ValueError("body L-turn task requires test_surface='ground'")
        if not self.confirm_ground_area_clear:
            missing.append("CONFIRM_GROUND_AREA_CLEAR=true")
        if not self.confirm_low_speed_ground_test:
            missing.append("CONFIRM_LOW_SPEED_GROUND_TEST=true")
        if not self.confirm_rc_ready:
            missing.append("CONFIRM_RC_READY=true")
        if not self.confirm_param_backup:
            missing.append("CONFIRM_PARAM_BACKUP=true")
        if not self.confirm_real_local_position:
            missing.append("CONFIRM_REAL_LOCAL_POSITION=true")
        if missing:
            raise RuntimeError(
                "Refusing to run real body L-turn Offboard task until safety is confirmed: "
                + ", ".join(missing)
            )

        max_command_speed = max(
            self.linear_speed_mps,
            math.hypot(self.turn_forward_speed_mps, self.turn_lateral_speed_mps),
        )
        if max_command_speed > self.max_linear_speed_mps:
            raise ValueError("commanded body speed exceeds max_linear_speed_mps")
        if self.turn_angle_rad <= 0.0:
            raise ValueError("turn_angle_deg must be positive")

    def _timer_cb(self) -> None:
        if self.done:
            return

        now = self._now_sec()
        self._maybe_request_mode_and_arm(now)

        if self.stage == "waiting":
            self._publish_body_velocity(0.0, 0.0)
            if self._abort_if_control_lost():
                return
            if self._ready_to_start(now):
                self.initial_yaw = self.current_yaw
                self.get_logger().warn(
                    f"captured initial heading={math.degrees(self.initial_yaw):.1f}deg"
                )
                self._enter_stage("initial_stop", now)
            else:
                self._log_waiting(now)
            return

        if self._abort_if_control_lost():
            return
        if not self._pose_is_fresh(now):
            self._publish_body_velocity(0.0, 0.0)
            self._log_progress(now, "holding stop; local pose is stale")
            return

        if self.stage == "initial_stop":
            self._publish_body_velocity(0.0, 0.0)
            if self._stage_elapsed(now) >= self.initial_stop_sec:
                self._start_leg("leg1_body_forward", now)
            return

        if self.stage == "leg1_body_forward":
            self._publish_body_velocity(self.linear_speed_mps, 0.0)
            distance = self._leg_distance_m()
            self._log_progress(
                now,
                f"leg1 distance={distance:.2f}m elapsed={self._stage_elapsed(now):.1f}s",
            )
            if self._distance_done(distance, self.first_distance_m):
                self._enter_stage("stop_after_leg1", now)
            elif self._stage_elapsed(now) >= self.first_leg_max_sec:
                self.get_logger().warn("leg1 max time reached; continuing to turn stage")
                self._enter_stage("stop_after_leg1", now)
            return

        if self.stage == "stop_after_leg1":
            self._publish_body_velocity(0.0, 0.0)
            if self._stage_elapsed(now) >= self.stop_after_first_sec:
                self.turn_start_yaw = self.current_yaw
                self._enter_stage("body_left_turn_arc", now)
            return

        if self.stage == "body_left_turn_arc":
            self._publish_body_velocity(
                self.turn_forward_speed_mps,
                self.turn_direction_sign * self.turn_lateral_speed_mps,
            )
            yaw_delta = self._signed_turn_delta_rad()
            self._log_progress(
                now,
                f"turn yaw_delta={math.degrees(yaw_delta):.1f}deg "
                f"elapsed={self._stage_elapsed(now):.1f}s",
            )
            if yaw_delta >= max(0.0, self.turn_angle_rad - self.yaw_tolerance_rad):
                self._enter_stage("stop_after_turn", now)
            elif self._stage_elapsed(now) >= self.turn_max_sec:
                self.get_logger().warn("turn max time reached; continuing to second leg")
                self._enter_stage("stop_after_turn", now)
            return

        if self.stage == "stop_after_turn":
            self._publish_body_velocity(0.0, 0.0)
            if self._stage_elapsed(now) >= self.stop_after_turn_sec:
                self._start_leg("leg2_body_forward", now)
            return

        if self.stage == "leg2_body_forward":
            self._publish_body_velocity(self.linear_speed_mps, 0.0)
            distance = self._leg_distance_m()
            self._log_progress(
                now,
                f"leg2 distance={distance:.2f}m elapsed={self._stage_elapsed(now):.1f}s",
            )
            if self._distance_done(distance, self.second_distance_m):
                self._enter_stage("final_stop", now)
            elif self._stage_elapsed(now) >= self.second_leg_max_sec:
                self.get_logger().warn("leg2 max time reached; stopping")
                self._enter_stage("final_stop", now)
            return

        if self.stage == "final_stop":
            self._publish_body_velocity(0.0, 0.0)
            if self._stage_elapsed(now) >= self.final_stop_sec:
                self._maybe_disarm_on_finish()
                self.get_logger().warn("body L-turn task complete; stop command sent")
                self.done = True
            return

    def _ready_to_start(self, now: float) -> bool:
        if self.require_connected and not self._is_connected():
            return self._check_timeout(now, "MAVROS connection")
        if not self._pose_is_fresh(now):
            return self._check_timeout(now, "fresh local position")
        if self.current_yaw is None or self.current_xy is None:
            return self._check_timeout(now, "valid local pose yaw/position")
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
        self._publish_body_velocity(0.0, 0.0)
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
                    self.last_mode_request_sec = now
        if self.arm_on_start and not self._is_armed():
            if self.require_offboard_mode and not self._is_offboard_mode():
                return
            if now - self.last_arm_request_sec >= self.arm_request_retry_sec:
                if self._request_arm(True):
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
        except Exception as exc:
            self.get_logger().error(f"OFFBOARD mode request failed: {exc}")
            return
        self.get_logger().warn(
            f"OFFBOARD mode request response: mode_sent={result.mode_sent}"
        )

    def _log_arming_result(self, future: object, requested_arm: bool) -> None:
        action = "ARM" if requested_arm else "DISARM"
        try:
            result = future.result()
        except Exception as exc:
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
            self._abort_sequence(f"mode changed away from OFFBOARD: {self.state.mode}")
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
        self._publish_body_velocity(0.0, 0.0)
        self.get_logger().error(f"aborting body L-turn task: {reason}; stop sent")
        self.done = True

    def _maybe_disarm_on_finish(self) -> None:
        if self.disarm_on_finish and not self.disarm_request_sent:
            self._request_arm(False)
            self.disarm_request_sent = True

    def _start_leg(self, stage: str, now: float) -> None:
        self.leg_start_xy = self.current_xy
        self._enter_stage(stage, now)

    def _publish_body_velocity(self, x_mps: float, y_mps: float) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x = float(x_mps)
        msg.twist.linear.y = float(y_mps)
        msg.twist.linear.z = 0.0
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(msg)
        if self.publish_unstamped_cmd_vel:
            unstamped = Twist()
            unstamped.linear.x = float(x_mps)
            unstamped.linear.y = float(y_mps)
            unstamped.linear.z = 0.0
            unstamped.angular.x = 0.0
            unstamped.angular.y = 0.0
            unstamped.angular.z = 0.0
            self.cmd_vel_unstamped_pub.publish(unstamped)

    def publish_stop_burst(self, duration_sec: float) -> None:
        end_sec = self._now_sec() + max(0.2, duration_sec)
        while rclpy.ok() and self._now_sec() < end_sec:
            self._publish_body_velocity(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=self.period_sec)

    def _enter_stage(self, stage: str, now: float) -> None:
        self.stage = stage
        self.stage_start_sec = now
        self.last_progress_log_sec = 0.0
        self.get_logger().warn(f"stage -> {stage}")

    def _stage_elapsed(self, now: float) -> float:
        if self.stage_start_sec is None:
            return 0.0
        return now - self.stage_start_sec

    def _leg_distance_m(self) -> float:
        if self.leg_start_xy is None or self.current_xy is None:
            return 0.0
        dx = self.current_xy[0] - self.leg_start_xy[0]
        dy = self.current_xy[1] - self.leg_start_xy[1]
        return math.hypot(dx, dy)

    def _distance_done(self, distance_m: float, target_m: float) -> bool:
        return distance_m >= max(0.0, target_m - self.distance_tolerance_m)

    def _signed_turn_delta_rad(self) -> float:
        if self.turn_start_yaw is None or self.current_yaw is None:
            return 0.0
        return -self.turn_direction_sign * self._wrap_pi(
            self.current_yaw - self.turn_start_yaw
        )

    def _pose_is_fresh(self, now: float) -> bool:
        return self.pose is not None and now - self.pose_time_sec <= self.max_pose_age_sec

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

    def _pose_cb(self, msg: PoseStamped) -> None:
        self.pose = msg
        self.pose_time_sec = self._now_sec()
        self.current_xy = (float(msg.pose.position.x), float(msg.pose.position.y))
        self.current_yaw = self._yaw_from_quaternion(msg.pose.orientation)

    def _statustext_cb(self, msg: StatusText) -> None:
        text = str(msg.text).strip()
        if text:
            self.get_logger().warn(f"PX4 STATUSTEXT severity={msg.severity}: {text}")

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
        pose_age = now - self.pose_time_sec if self.pose is not None else float("inf")
        self.get_logger().info(
            f"holding stop; connected={self._is_connected()} mode={mode} "
            f"armed={self._is_armed()} pose_fresh={self._pose_is_fresh(now)} "
            f"pose_age={pose_age:.2f}s"
        )

    def _log_progress(self, now: float, text: str) -> None:
        if now - self.last_progress_log_sec < 1.0:
            return
        self.last_progress_log_sec = now
        self.get_logger().info(text)

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    @staticmethod
    def _yaw_from_quaternion(q: object) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _wrap_pi(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

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
        or PoseStamped is None
        or Twist is None
        or TwistStamped is None
        or State is None
        or StatusText is None
    ):
        raise SystemExit(
            "MAVROS dependencies are not importable. Install/source ros-humble-mavros."
        )

    rclpy.init()
    node = BodyLTurnOffboard()
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
