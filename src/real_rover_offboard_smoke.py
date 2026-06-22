#!/usr/bin/env python3

"""Conservative PX4 Offboard smoke test for the real rover hardware."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Optional

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

try:
    from px4_msgs.msg import ActuatorMotors
except ImportError:  # pragma: no cover
    ActuatorMotors = None

try:
    from px4_msgs.msg import ActuatorServos
except ImportError:  # pragma: no cover
    ActuatorServos = None

try:
    from px4_msgs.msg import OffboardControlMode
    from px4_msgs.msg import TrajectorySetpoint
    from px4_msgs.msg import VehicleCommand
    from px4_msgs.msg import VehicleStatus
except ImportError:  # pragma: no cover
    OffboardControlMode = None
    TrajectorySetpoint = None
    VehicleCommand = None
    VehicleStatus = None


@dataclass(frozen=True)
class SmokeStep:
    name: str
    duration_sec: float
    linear: float
    turn: float


class RealRoverOffboardSmoke(Node):
    """Publish a short, low-power Offboard sequence for wheels-up validation."""

    def __init__(self) -> None:
        super().__init__("real_rover_offboard_smoke")

        dynamic_param = ParameterDescriptor(dynamic_typing=True)

        def declare_param(name: str, default: object) -> None:
            self.declare_parameter(name, default, dynamic_param)

        declare_param("px4_namespace", "/px4_1")
        declare_param("vehicle_id", 1)
        declare_param("command_mode", "velocity")
        declare_param("direct_actuator_topic", "motors")
        declare_param("command_rate_hz", 20.0)
        declare_param("warmup_sec", 1.0)
        declare_param("stop_sec", 1.0)
        declare_param("forward_sec", 1.0)
        declare_param("backward_sec", 1.0)
        declare_param("turn_sec", 0.5)
        declare_param("final_stop_sec", 2.0)
        declare_param("linear_speed_mps", 0.12)
        declare_param("turn_yaw_rate_radps", 0.25)
        declare_param("turn_with_linear_mps", 0.0)
        declare_param("linear_actuator", 0.12)
        declare_param("turn_actuator", 0.10)
        declare_param("max_linear_speed_mps", 0.30)
        declare_param("max_yaw_rate_radps", 0.70)
        declare_param("max_actuator_abs", 0.25)
        declare_param("turn_sign", 1.0)
        declare_param("mode_change_on_start", False)
        declare_param("arm_on_start", False)
        declare_param("disarm_on_finish", False)
        declare_param("require_offboard_mode", True)
        declare_param("require_armed", True)
        declare_param("max_wait_for_ready_sec", 60.0)
        declare_param("stop_burst_sec", 0.8)
        declare_param("confirm_wheels_lifted", False)
        declare_param("confirm_rc_ready", False)
        declare_param("confirm_param_backup", False)

        if OffboardControlMode is None or TrajectorySetpoint is None or VehicleCommand is None:
            raise RuntimeError(
                "px4_msgs is not importable. Source ROS 2 and a workspace that "
                "contains px4_msgs before running this script."
            )

        self.px4_namespace = str(self.get_parameter("px4_namespace").value).rstrip("/")
        self.vehicle_id = int(self.get_parameter("vehicle_id").value)
        self.command_mode = str(self.get_parameter("command_mode").value).lower()
        self.direct_actuator_topic = str(
            self.get_parameter("direct_actuator_topic").value
        ).lower()
        self.command_rate_hz = max(2.0, float(self.get_parameter("command_rate_hz").value))
        self.warmup_sec = max(0.5, float(self.get_parameter("warmup_sec").value))
        self.stop_sec = max(0.2, float(self.get_parameter("stop_sec").value))
        self.forward_sec = max(0.0, float(self.get_parameter("forward_sec").value))
        self.backward_sec = max(0.0, float(self.get_parameter("backward_sec").value))
        self.turn_sec = max(0.0, float(self.get_parameter("turn_sec").value))
        self.final_stop_sec = max(0.5, float(self.get_parameter("final_stop_sec").value))
        self.linear_speed_mps = abs(float(self.get_parameter("linear_speed_mps").value))
        self.turn_yaw_rate_radps = abs(
            float(self.get_parameter("turn_yaw_rate_radps").value)
        )
        self.turn_with_linear_mps = float(self.get_parameter("turn_with_linear_mps").value)
        self.linear_actuator = abs(float(self.get_parameter("linear_actuator").value))
        self.turn_actuator = abs(float(self.get_parameter("turn_actuator").value))
        self.max_linear_speed_mps = abs(
            float(self.get_parameter("max_linear_speed_mps").value)
        )
        self.max_yaw_rate_radps = abs(
            float(self.get_parameter("max_yaw_rate_radps").value)
        )
        self.max_actuator_abs = abs(float(self.get_parameter("max_actuator_abs").value))
        self.turn_sign = 1.0 if float(self.get_parameter("turn_sign").value) >= 0.0 else -1.0
        self.mode_change_on_start = self._as_bool(
            self.get_parameter("mode_change_on_start").value
        )
        self.arm_on_start = self._as_bool(self.get_parameter("arm_on_start").value)
        self.disarm_on_finish = self._as_bool(
            self.get_parameter("disarm_on_finish").value
        )
        self.require_offboard_mode = self._as_bool(
            self.get_parameter("require_offboard_mode").value
        )
        self.require_armed = self._as_bool(self.get_parameter("require_armed").value)
        self.max_wait_for_ready_sec = max(
            0.0, float(self.get_parameter("max_wait_for_ready_sec").value)
        )
        self.stop_burst_sec = max(0.2, float(self.get_parameter("stop_burst_sec").value))
        self.confirm_wheels_lifted = self._as_bool(
            self.get_parameter("confirm_wheels_lifted").value
        )
        self.confirm_rc_ready = self._as_bool(self.get_parameter("confirm_rc_ready").value)
        self.confirm_param_backup = self._as_bool(
            self.get_parameter("confirm_param_backup").value
        )

        self._validate_safety_confirmations()
        self._validate_command_mode()
        self._validate_command_limits()

        px4_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        command_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode,
            f"{self.px4_namespace}/fmu/in/offboard_control_mode",
            px4_qos,
        )
        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint,
            f"{self.px4_namespace}/fmu/in/trajectory_setpoint",
            px4_qos,
        )
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand,
            f"{self.px4_namespace}/fmu/in/vehicle_command",
            command_qos,
        )
        self.actuator_motors_pub = None
        self.actuator_servos_pub = None
        if self.command_mode == "direct_actuator":
            if self.direct_actuator_topic in {"motors", "both"}:
                self.actuator_motors_pub = self.create_publisher(
                    ActuatorMotors,
                    f"{self.px4_namespace}/fmu/in/actuator_motors",
                    px4_qos,
                )
            if self.direct_actuator_topic in {"servos", "both"}:
                self.actuator_servos_pub = self.create_publisher(
                    ActuatorServos,
                    f"{self.px4_namespace}/fmu/in/actuator_servos",
                    px4_qos,
                )

        self.vehicle_status: Optional[VehicleStatus] = None
        if VehicleStatus is not None:
            self.create_subscription(
                VehicleStatus,
                f"{self.px4_namespace}/fmu/out/vehicle_status_v2",
                self._vehicle_status_cb,
                px4_qos,
            )
            self.create_subscription(
                VehicleStatus,
                f"{self.px4_namespace}/fmu/out/vehicle_status",
                self._vehicle_status_cb,
                px4_qos,
            )

        self.sequence = self._build_sequence()
        self.timer = self.create_timer(1.0 / self.command_rate_hz, self._timer_cb)
        self.start_time_sec = self._now_sec()
        self.sequence_start_time_sec: Optional[float] = None
        self.step_index = 0
        self.mode_request_sent = False
        self.arm_request_sent = False
        self.disarm_request_sent = False
        self.done = False
        self.last_wait_log_sec = 0.0

        self.get_logger().warn(
            "real hardware smoke test loaded: mode=%s namespace=%s rate=%.1fHz",
            self.command_mode,
            self.px4_namespace,
            self.command_rate_hz,
        )
        self._log_sequence()

    def _validate_safety_confirmations(self) -> None:
        missing = []
        if not self.confirm_wheels_lifted:
            missing.append("CONFIRM_WHEELS_LIFTED=true")
        if not self.confirm_rc_ready:
            missing.append("CONFIRM_RC_READY=true")
        if not self.confirm_param_backup:
            missing.append("CONFIRM_PARAM_BACKUP=true")
        if missing:
            raise RuntimeError(
                "Refusing to run real-hardware motion test until safety is confirmed: "
                + ", ".join(missing)
            )

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

    def _validate_command_mode(self) -> None:
        if self.command_mode not in {"velocity", "direct_actuator"}:
            raise ValueError("command_mode must be 'velocity' or 'direct_actuator'")
        if self.direct_actuator_topic not in {"motors", "servos", "both"}:
            raise ValueError("direct_actuator_topic must be 'motors', 'servos', or 'both'")
        if self.command_mode != "direct_actuator":
            return
        if not hasattr(OffboardControlMode(), "direct_actuator"):
            raise RuntimeError("This px4_msgs OffboardControlMode has no direct_actuator field")
        if self.direct_actuator_topic in {"motors", "both"} and ActuatorMotors is None:
            raise RuntimeError("px4_msgs.msg.ActuatorMotors is not available")
        if self.direct_actuator_topic in {"servos", "both"} and ActuatorServos is None:
            raise RuntimeError("px4_msgs.msg.ActuatorServos is not available")

    def _validate_command_limits(self) -> None:
        if self.linear_speed_mps > self.max_linear_speed_mps:
            raise ValueError(
                f"linear_speed_mps={self.linear_speed_mps:.3f} exceeds "
                f"max_linear_speed_mps={self.max_linear_speed_mps:.3f}"
            )
        if abs(self.turn_with_linear_mps) > self.max_linear_speed_mps:
            raise ValueError(
                f"turn_with_linear_mps={self.turn_with_linear_mps:.3f} exceeds "
                f"max_linear_speed_mps={self.max_linear_speed_mps:.3f}"
            )
        if self.turn_yaw_rate_radps > self.max_yaw_rate_radps:
            raise ValueError(
                f"turn_yaw_rate_radps={self.turn_yaw_rate_radps:.3f} exceeds "
                f"max_yaw_rate_radps={self.max_yaw_rate_radps:.3f}"
            )
        if self.linear_actuator > self.max_actuator_abs:
            raise ValueError(
                f"linear_actuator={self.linear_actuator:.3f} exceeds "
                f"max_actuator_abs={self.max_actuator_abs:.3f}"
            )
        if self.turn_actuator > self.max_actuator_abs:
            raise ValueError(
                f"turn_actuator={self.turn_actuator:.3f} exceeds "
                f"max_actuator_abs={self.max_actuator_abs:.3f}"
            )

    def _build_sequence(self) -> list[SmokeStep]:
        if self.command_mode == "velocity":
            linear = self.linear_speed_mps
            turn = self.turn_yaw_rate_radps
            turn_linear = self.turn_with_linear_mps
        else:
            linear = self.linear_actuator
            turn = self.turn_actuator
            turn_linear = 0.0

        turn *= self.turn_sign

        return [
            SmokeStep("initial_stop", self.stop_sec, 0.0, 0.0),
            SmokeStep("forward", self.forward_sec, linear, 0.0),
            SmokeStep("stop_after_forward", self.stop_sec, 0.0, 0.0),
            SmokeStep("backward", self.backward_sec, -linear, 0.0),
            SmokeStep("stop_after_backward", self.stop_sec, 0.0, 0.0),
            SmokeStep("turn_left", self.turn_sec, turn_linear, turn),
            SmokeStep("stop_after_left", self.stop_sec, 0.0, 0.0),
            SmokeStep("turn_right", self.turn_sec, turn_linear, -turn),
            SmokeStep("final_stop", self.final_stop_sec, 0.0, 0.0),
        ]

    def _log_sequence(self) -> None:
        self.get_logger().info("sequence:")
        for index, step in enumerate(self.sequence, start=1):
            self.get_logger().info(
                "  %02d %s: %.2fs linear=%.3f turn=%.3f",
                index,
                step.name,
                step.duration_sec,
                step.linear,
                step.turn,
            )

    def _vehicle_status_cb(self, msg: VehicleStatus) -> None:
        self.vehicle_status = msg

    def _timer_cb(self) -> None:
        if self.done:
            return

        now = self._now_sec()
        self._maybe_request_mode_and_arm(now)

        if self.sequence_start_time_sec is None:
            self._publish_command(0.0, 0.0)
            if self._ready_to_start(now):
                self.sequence_start_time_sec = now
                self.step_index = 0
                self.get_logger().warn("starting smoke sequence")
            else:
                self._log_waiting(now)
            return

        step = self.sequence[self.step_index]
        self._publish_command(step.linear, step.turn)

        elapsed = now - self.sequence_start_time_sec
        if elapsed < step.duration_sec:
            return

        self.step_index += 1
        if self.step_index >= len(self.sequence):
            self._publish_command(0.0, 0.0)
            self._maybe_disarm_on_finish()
            self.get_logger().warn("smoke sequence complete; stop command sent")
            self.done = True
            return

        self.sequence_start_time_sec = now
        next_step = self.sequence[self.step_index]
        self.get_logger().info("step -> %s", next_step.name)

    def _ready_to_start(self, now: float) -> bool:
        elapsed = now - self.start_time_sec
        if elapsed < self.warmup_sec:
            return False
        if self.require_offboard_mode and not self._is_offboard_mode():
            return self._check_wait_timeout(now, "OFFBOARD mode")
        if self.require_armed and not self._is_armed():
            return self._check_wait_timeout(now, "armed state")
        return True

    def _check_wait_timeout(self, now: float, waiting_for: str) -> bool:
        if self.max_wait_for_ready_sec <= 0.0:
            return False
        elapsed = now - self.start_time_sec
        if elapsed <= self.max_wait_for_ready_sec:
            return False
        self.get_logger().error(
            "timed out waiting for %s after %.1fs; exiting with stop command",
            waiting_for,
            elapsed,
        )
        self._publish_command(0.0, 0.0)
        self.done = True
        return False

    def _maybe_request_mode_and_arm(self, now: float) -> None:
        if now - self.start_time_sec < self.warmup_sec:
            return
        if self.mode_change_on_start and not self.mode_request_sent:
            self._publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                param1=1.0,
                param2=6.0,
            )
            self.mode_request_sent = True
            self.get_logger().warn("requested OFFBOARD mode")
        if self.arm_on_start and not self.arm_request_sent:
            if self.require_offboard_mode and not self._is_offboard_mode():
                return
            self._publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                param1=1.0,
            )
            self.arm_request_sent = True
            self.get_logger().warn("requested ARM")

    def _maybe_disarm_on_finish(self) -> None:
        if not self.disarm_on_finish or self.disarm_request_sent:
            return
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=0.0,
        )
        self.disarm_request_sent = True
        self.get_logger().warn("requested DISARM")

    def _log_waiting(self, now: float) -> None:
        if now - self.last_wait_log_sec < 1.0:
            return
        self.last_wait_log_sec = now
        self.get_logger().info(
            "holding stop; waiting warmup/offboard/armed: warmup=%s offboard=%s armed=%s",
            "yes" if now - self.start_time_sec >= self.warmup_sec else "no",
            "yes" if self._is_offboard_mode() else "no",
            "yes" if self._is_armed() else "no",
        )

    def _publish_command(self, linear: float, turn: float) -> None:
        timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self._publish_offboard_mode(timestamp)
        if self.command_mode == "velocity":
            self._publish_velocity_setpoint(timestamp, linear, turn)
        else:
            self._publish_direct_actuator_setpoint(timestamp, linear, turn)

    def _publish_offboard_mode(self, timestamp: int) -> None:
        msg = OffboardControlMode()
        msg.timestamp = timestamp
        msg.position = False
        msg.velocity = self.command_mode == "velocity"
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        if hasattr(msg, "thrust_and_torque"):
            msg.thrust_and_torque = False
        if hasattr(msg, "direct_actuator"):
            msg.direct_actuator = self.command_mode == "direct_actuator"
        self.offboard_mode_pub.publish(msg)

    def _publish_velocity_setpoint(self, timestamp: int, linear: float, turn: float) -> None:
        msg = TrajectorySetpoint()
        msg.timestamp = timestamp
        msg.position = [math.nan, math.nan, math.nan]
        msg.velocity = [float(linear), 0.0, math.nan]
        msg.acceleration = [math.nan, math.nan, math.nan]
        if hasattr(msg, "jerk"):
            msg.jerk = [math.nan, math.nan, math.nan]
        msg.yaw = math.nan
        msg.yawspeed = float(turn)
        self.trajectory_pub.publish(msg)

    def _publish_direct_actuator_setpoint(
        self, timestamp: int, linear: float, turn: float
    ) -> None:
        if self.actuator_motors_pub is not None:
            msg = ActuatorMotors()
            self._fill_actuator_message(msg, timestamp, linear, turn)
            if hasattr(msg, "reversible_flags"):
                msg.reversible_flags = 0b11
            self.actuator_motors_pub.publish(msg)
        if self.actuator_servos_pub is not None:
            msg = ActuatorServos()
            self._fill_actuator_message(msg, timestamp, linear, turn)
            self.actuator_servos_pub.publish(msg)

    @staticmethod
    def _fill_actuator_message(msg: object, timestamp: int, linear: float, turn: float) -> None:
        msg.timestamp = timestamp
        if hasattr(msg, "timestamp_sample"):
            msg.timestamp_sample = timestamp
        control_count = len(msg.control)
        controls = [math.nan] * control_count
        if control_count >= 1:
            controls[0] = float(linear)
        if control_count >= 2:
            controls[1] = float(turn)
        msg.control = controls

    def _publish_vehicle_command(
        self,
        command: int,
        param1: float = 0.0,
        param2: float = 0.0,
    ) -> None:
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.command = int(command)
        msg.target_system = self.vehicle_id
        msg.target_component = 1
        msg.source_system = self.vehicle_id
        msg.source_component = 1
        msg.from_external = True
        self.vehicle_command_pub.publish(msg)

    def publish_stop_burst(self, duration_sec: float) -> None:
        count = max(1, int(duration_sec * self.command_rate_hz))
        sleep_sec = 1.0 / self.command_rate_hz
        for _ in range(count):
            self._publish_command(0.0, 0.0)
            time.sleep(sleep_sec)

    def _is_offboard_mode(self) -> bool:
        if self.vehicle_status is None or VehicleStatus is None:
            return False
        offboard_state = getattr(VehicleStatus, "NAVIGATION_STATE_OFFBOARD", 14)
        return int(self.vehicle_status.nav_state) == int(offboard_state)

    def _is_armed(self) -> bool:
        if self.vehicle_status is None or VehicleStatus is None:
            return False
        armed_state = getattr(VehicleStatus, "ARMING_STATE_ARMED", 2)
        return int(self.vehicle_status.arming_state) == int(armed_state)

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9


def main() -> None:
    if OffboardControlMode is None or TrajectorySetpoint is None or VehicleCommand is None:
        raise SystemExit(
            "px4_msgs is not importable. Source ROS 2 and a workspace that "
            "contains px4_msgs before running this script."
        )

    rclpy.init()
    node = RealRoverOffboardSmoke()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().warn("interrupted; sending stop burst")
    finally:
        if rclpy.ok():
            node.publish_stop_burst(node.stop_burst_sec)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
