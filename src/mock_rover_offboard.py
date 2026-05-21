#!/usr/bin/env python3

"""PX4 rover offboard trainer for a minimal ground vehicle workflow."""

from __future__ import annotations

import math
from enum import Enum
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

try:
    from px4_msgs.msg import OffboardControlMode
    from px4_msgs.msg import TrajectorySetpoint
    from px4_msgs.msg import VehicleCommand
    from px4_msgs.msg import VehicleLocalPosition
    from px4_msgs.msg import VehicleStatus
except ImportError:  # pragma: no cover
    OffboardControlMode = None
    TrajectorySetpoint = None
    VehicleCommand = None
    VehicleLocalPosition = None
    VehicleStatus = None


class MissionState(Enum):
    WAIT_FOR_POSITION = "WAIT_FOR_POSITION"
    STREAM_SETPOINTS = "STREAM_SETPOINTS"
    GOTO_FORWARD = "GOTO_FORWARD"
    RETURN_HOME = "RETURN_HOME"
    FENCE_RETURN = "FENCE_RETURN"
    COMPLETE = "COMPLETE"


class MockRoverOffboard(Node):
    def __init__(self) -> None:
        super().__init__("mock_rover_offboard")

        self.declare_parameter("px4_namespace", "/px4_1")
        self.declare_parameter("vehicle_id", 1)
        self.declare_parameter("arm_on_start", True)
        self.declare_parameter("mission_mode", "position")
        self.declare_parameter("travel_distance_m", 3.0)
        self.declare_parameter("fence_radius_m", 10.0)
        self.declare_parameter("acceptance_radius_m", 0.25)
        self.declare_parameter("setpoint_rate_hz", 20.0)
        self.declare_parameter("forward_speed_mps", 0.7)
        self.declare_parameter("return_speed_mps", 0.5)
        self.declare_parameter("stream_warmup_sec", 1.2)
        self.declare_parameter("auto_start", True)

        self.px4_namespace = str(self.get_parameter("px4_namespace").value).rstrip("/")
        self.vehicle_id = int(self.get_parameter("vehicle_id").value)
        self.arm_on_start = bool(self.get_parameter("arm_on_start").value)
        self.mission_mode = str(self.get_parameter("mission_mode").value).lower()
        self.travel_distance_m = abs(float(self.get_parameter("travel_distance_m").value))
        self.fence_radius_m = abs(float(self.get_parameter("fence_radius_m").value))
        self.acceptance_radius_m = max(0.05, float(self.get_parameter("acceptance_radius_m").value))
        self.forward_speed_mps = abs(float(self.get_parameter("forward_speed_mps").value))
        self.return_speed_mps = abs(float(self.get_parameter("return_speed_mps").value))
        self.stream_warmup_sec = max(0.1, float(self.get_parameter("stream_warmup_sec").value))
        self.auto_start = bool(self.get_parameter("auto_start").value)
        setpoint_rate_hz = max(2.0, float(self.get_parameter("setpoint_rate_hz").value))

        if self.mission_mode not in {"position", "velocity"}:
            raise ValueError("mission_mode must be 'position' or 'velocity'")
        if OffboardControlMode is None:
            self.get_logger().error(
                "px4_msgs is not importable. Source ROS and the px4_msgs workspace first."
            )
            raise RuntimeError("px4_msgs is not importable")

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
        self.create_subscription(
            VehicleLocalPosition,
            f"{self.px4_namespace}/fmu/out/vehicle_local_position",
            self._local_position_cb,
            px4_qos,
        )
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

        self.local_position: Optional[VehicleLocalPosition] = None
        self.vehicle_status: Optional[VehicleStatus] = None
        self.home_xy: Optional[tuple[float, float]] = None
        self.forward_xy: Optional[tuple[float, float]] = None
        self.state = MissionState.WAIT_FOR_POSITION
        self.state_start_time_sec = self._now_sec()
        self.mode_request_sent = False
        self.arm_request_sent = False
        self.completed_logged = False

        self.timer = self.create_timer(1.0 / setpoint_rate_hz, self._timer_cb)
        self.get_logger().info(
            "ready: mode=%s distance=%.2fm fence=%.2fm namespace=%s",
            self.mission_mode,
            self.travel_distance_m,
            self.fence_radius_m,
            self.px4_namespace,
        )

    def _local_position_cb(self, msg: VehicleLocalPosition) -> None:
        self.local_position = msg
        if self.home_xy is None and bool(msg.xy_valid):
            self.home_xy = (float(msg.x), float(msg.y))
            self.forward_xy = (self.home_xy[0] + self.travel_distance_m, self.home_xy[1])
            self.get_logger().info(
                "home locked: x=%.2f y=%.2f, forward target x=%.2f y=%.2f",
                self.home_xy[0],
                self.home_xy[1],
                self.forward_xy[0],
                self.forward_xy[1],
            )

    def _vehicle_status_cb(self, msg: VehicleStatus) -> None:
        self.vehicle_status = msg

    def _timer_cb(self) -> None:
        if self.local_position is None or self.home_xy is None or self.forward_xy is None:
            return
        if not bool(self.local_position.xy_valid):
            return

        current_xy = self._current_xy()
        distance_from_home = self._distance_xy(current_xy, self.home_xy)
        if distance_from_home > self.fence_radius_m and self.state != MissionState.FENCE_RETURN:
            self._set_state(MissionState.FENCE_RETURN)
            self.get_logger().warn(
                "software fence exceeded: %.2fm > %.2fm, returning home",
                distance_from_home,
                self.fence_radius_m,
            )

        if self.state == MissionState.WAIT_FOR_POSITION:
            if self.auto_start:
                self._set_state(MissionState.STREAM_SETPOINTS)
            else:
                self._publish_target(self.home_xy, speed_mps=0.0)
            return

        if self.state == MissionState.STREAM_SETPOINTS:
            self._publish_target(self.home_xy, speed_mps=0.0)
            if self._state_elapsed_sec() >= self.stream_warmup_sec:
                self._request_offboard_and_arm()
                self._set_state(MissionState.GOTO_FORWARD)
            return

        if self.state == MissionState.GOTO_FORWARD:
            self._publish_target(self.forward_xy, speed_mps=self.forward_speed_mps)
            if self._distance_xy(current_xy, self.forward_xy) <= self.acceptance_radius_m:
                self._set_state(MissionState.RETURN_HOME)
            return

        if self.state == MissionState.RETURN_HOME:
            self._publish_target(self.home_xy, speed_mps=-self.return_speed_mps)
            if self._distance_xy(current_xy, self.home_xy) <= self.acceptance_radius_m:
                self._set_state(MissionState.COMPLETE)
            return

        if self.state == MissionState.FENCE_RETURN:
            self._publish_target(self.home_xy, speed_mps=self.return_speed_mps)
            if self._distance_xy(current_xy, self.home_xy) <= self.acceptance_radius_m:
                self._set_state(MissionState.COMPLETE)
            return

        if self.state == MissionState.COMPLETE:
            self._publish_target(self.home_xy, speed_mps=0.0)
            if not self.completed_logged:
                self.completed_logged = True
                self.get_logger().info("mission complete: holding home setpoint")

    def _publish_target(self, target_xy: tuple[float, float], speed_mps: float) -> None:
        timestamp = int(self.get_clock().now().nanoseconds / 1000)

        offboard_mode = OffboardControlMode()
        offboard_mode.timestamp = timestamp
        offboard_mode.position = self.mission_mode == "position"
        offboard_mode.velocity = self.mission_mode == "velocity"
        offboard_mode.acceleration = False
        offboard_mode.attitude = False
        offboard_mode.body_rate = False
        if hasattr(offboard_mode, "thrust_and_torque"):
            offboard_mode.thrust_and_torque = False
        if hasattr(offboard_mode, "direct_actuator"):
            offboard_mode.direct_actuator = False
        self.offboard_mode_pub.publish(offboard_mode)

        setpoint = TrajectorySetpoint()
        setpoint.timestamp = timestamp
        setpoint.position = [math.nan, math.nan, math.nan]
        setpoint.velocity = [math.nan, math.nan, math.nan]
        setpoint.acceleration = [math.nan, math.nan, math.nan]
        if hasattr(setpoint, "jerk"):
            setpoint.jerk = [math.nan, math.nan, math.nan]
        setpoint.yaw = 0.0
        setpoint.yawspeed = math.nan

        if self.mission_mode == "position":
            setpoint.position = [float(target_xy[0]), float(target_xy[1]), math.nan]
        else:
            setpoint.velocity = [float(speed_mps), 0.0, math.nan]

        self.trajectory_pub.publish(setpoint)

    def _request_offboard_and_arm(self) -> None:
        if not self.mode_request_sent:
            self._publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                param1=1.0,
                param2=6.0,
            )
            self.mode_request_sent = True
            self.get_logger().info("requested OFFBOARD mode")
        if self.arm_on_start and not self.arm_request_sent:
            self._publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                param1=1.0,
            )
            self.arm_request_sent = True
            self.get_logger().info("requested ARM")

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

    def _set_state(self, state: MissionState) -> None:
        if self.state == state:
            return
        self.state = state
        self.state_start_time_sec = self._now_sec()
        self.get_logger().info("mission state -> %s", state.value)

    def _current_xy(self) -> tuple[float, float]:
        assert self.local_position is not None
        return (float(self.local_position.x), float(self.local_position.y))

    @staticmethod
    def _distance_xy(a: tuple[float, float], b: tuple[float, float]) -> float:
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def _state_elapsed_sec(self) -> float:
        return self._now_sec() - self.state_start_time_sec

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9


def main() -> None:
    rclpy.init()
    node = MockRoverOffboard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
