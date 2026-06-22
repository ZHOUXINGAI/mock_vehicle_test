#!/usr/bin/env python3

"""Publish fixed external-vision pose/odometry into MAVROS."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

try:
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from mavros_msgs.msg import State
    from mavros_msgs.msg import StatusText
    from nav_msgs.msg import Odometry
    from rcl_interfaces.msg import ParameterDescriptor
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
except ImportError:  # pragma: no cover
    rclpy = None
    PoseStamped = None
    PoseWithCovarianceStamped = None
    Odometry = None
    State = None
    StatusText = None
    ParameterDescriptor = None
    ExternalShutdownException = KeyboardInterrupt
    Node = object
    qos_profile_sensor_data = 10


@dataclass
class PoseSeed:
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float


class MavrosFakeExternalVision(Node):
    def __init__(self) -> None:
        super().__init__("mavros_fake_external_vision")

        dynamic_param = ParameterDescriptor(dynamic_typing=True)

        def declare_param(name: str, default: object) -> None:
            self.declare_parameter(name, default, dynamic_param)

        declare_param("mavros_namespace", "/mavros")
        declare_param("rate_hz", 30.0)
        declare_param("duration_sec", 0.0)
        declare_param("use_current_local_pose", True)
        declare_param("current_pose_wait_sec", 5.0)
        declare_param("x_m", 0.0)
        declare_param("y_m", 0.0)
        declare_param("z_m", 0.0)
        declare_param("qx", 0.0)
        declare_param("qy", 0.0)
        declare_param("qz", 0.0)
        declare_param("qw", 1.0)
        declare_param("publish_vision_pose", True)
        declare_param("publish_vision_pose_cov", True)
        declare_param("publish_odometry", True)
        declare_param("pose_covariance", 0.02)
        declare_param("orientation_covariance", 0.02)
        declare_param("velocity_covariance", 0.05)
        declare_param("frame_id", "map")
        declare_param("child_frame_id", "base_link")
        declare_param("log_interval_sec", 1.0)

        self.mavros_namespace = str(
            self.get_parameter("mavros_namespace").value
        ).rstrip("/")
        self.rate_hz = max(1.0, float(self.get_parameter("rate_hz").value))
        self.period_sec = 1.0 / self.rate_hz
        self.duration_sec = max(0.0, float(self.get_parameter("duration_sec").value))
        self.use_current_local_pose = self._as_bool(
            self.get_parameter("use_current_local_pose").value
        )
        self.current_pose_wait_sec = max(
            0.0, float(self.get_parameter("current_pose_wait_sec").value)
        )
        self.seed = PoseSeed(
            x=float(self.get_parameter("x_m").value),
            y=float(self.get_parameter("y_m").value),
            z=float(self.get_parameter("z_m").value),
            qx=float(self.get_parameter("qx").value),
            qy=float(self.get_parameter("qy").value),
            qz=float(self.get_parameter("qz").value),
            qw=float(self.get_parameter("qw").value),
        )
        self.publish_vision_pose = self._as_bool(
            self.get_parameter("publish_vision_pose").value
        )
        self.publish_vision_pose_cov = self._as_bool(
            self.get_parameter("publish_vision_pose_cov").value
        )
        self.publish_odometry = self._as_bool(
            self.get_parameter("publish_odometry").value
        )
        self.pose_covariance = max(0.0, float(self.get_parameter("pose_covariance").value))
        self.orientation_covariance = max(
            0.0, float(self.get_parameter("orientation_covariance").value)
        )
        self.velocity_covariance = max(
            0.0, float(self.get_parameter("velocity_covariance").value)
        )
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.child_frame_id = str(self.get_parameter("child_frame_id").value)
        self.log_interval_sec = max(
            0.5, float(self.get_parameter("log_interval_sec").value)
        )

        if not (
            self.publish_vision_pose
            or self.publish_vision_pose_cov
            or self.publish_odometry
        ):
            raise ValueError("at least one output stream must be enabled")

        self.start_time_sec = self._now_sec()
        self.last_log_sec = -1.0e9
        self.done = False
        self.state: Optional[State] = None
        self.current_local_pose: Optional[PoseStamped] = None
        self.publish_seed: Optional[PoseSeed] = None
        self.last_state_summary: Optional[tuple[bool, bool, str, bool, bool]] = None

        self.vision_pose_pub = self.create_publisher(
            PoseStamped,
            f"{self.mavros_namespace}/vision_pose/pose",
            10,
        )
        self.vision_pose_cov_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            f"{self.mavros_namespace}/vision_pose/pose_cov",
            10,
        )
        self.odometry_pub = self.create_publisher(
            Odometry,
            f"{self.mavros_namespace}/odometry/out",
            10,
        )
        self.create_subscription(
            PoseStamped,
            f"{self.mavros_namespace}/local_position/pose",
            self._local_pose_cb,
            qos_profile_sensor_data,
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
        self.timer = self.create_timer(self.period_sec, self._timer_cb)

        self.get_logger().warn(
            "fake external vision publisher loaded: "
            f"rate={self.rate_hz:.1f}Hz use_current_local_pose={self.use_current_local_pose} "
            f"vision_pose={self.publish_vision_pose} "
            f"vision_pose_cov={self.publish_vision_pose_cov} "
            f"odometry={self.publish_odometry}"
        )

    def _timer_cb(self) -> None:
        if self.done:
            return

        now = self._now_sec()
        seed = self._get_publish_seed(now)
        if seed is None:
            self._log_waiting_for_seed(now)
            return

        self._publish(seed)
        self._log_snapshot(now, seed)

        if self.duration_sec > 0.0 and now - self.start_time_sec >= self.duration_sec:
            self.get_logger().warn("fake external vision duration complete")
            self.done = True

    def _get_publish_seed(self, now: float) -> Optional[PoseSeed]:
        if self.publish_seed is not None:
            return self.publish_seed

        if self.use_current_local_pose and self.current_local_pose is not None:
            p = self.current_local_pose.pose.position
            q = self.current_local_pose.pose.orientation
            self.publish_seed = PoseSeed(
                x=float(p.x),
                y=float(p.y),
                z=float(p.z),
                qx=float(q.x),
                qy=float(q.y),
                qz=float(q.z),
                qw=float(q.w),
            )
            self.get_logger().warn(
                "seeded fake external vision from current local pose: "
                f"x={p.x:.2f} y={p.y:.2f} z={p.z:.2f}"
            )
            return self.publish_seed

        if self.use_current_local_pose and now - self.start_time_sec < self.current_pose_wait_sec:
            return None

        self.publish_seed = self.seed
        self.get_logger().warn(
            "seeded fake external vision from configured pose: "
            f"x={self.seed.x:.2f} y={self.seed.y:.2f} z={self.seed.z:.2f}"
        )
        return self.publish_seed

    def _publish(self, seed: PoseSeed) -> None:
        stamp = self.get_clock().now().to_msg()
        if self.publish_vision_pose:
            msg = PoseStamped()
            msg.header.stamp = stamp
            msg.header.frame_id = self.frame_id
            self._fill_pose(msg.pose, seed)
            self.vision_pose_pub.publish(msg)

        if self.publish_vision_pose_cov:
            msg_cov = PoseWithCovarianceStamped()
            msg_cov.header.stamp = stamp
            msg_cov.header.frame_id = self.frame_id
            self._fill_pose(msg_cov.pose.pose, seed)
            self._fill_pose_covariance(msg_cov.pose.covariance)
            self.vision_pose_cov_pub.publish(msg_cov)

        if self.publish_odometry:
            odom = Odometry()
            odom.header.stamp = stamp
            odom.header.frame_id = self.frame_id
            odom.child_frame_id = self.child_frame_id
            self._fill_pose(odom.pose.pose, seed)
            self._fill_pose_covariance(odom.pose.covariance)
            odom.twist.twist.linear.x = 0.0
            odom.twist.twist.linear.y = 0.0
            odom.twist.twist.linear.z = 0.0
            odom.twist.twist.angular.x = 0.0
            odom.twist.twist.angular.y = 0.0
            odom.twist.twist.angular.z = 0.0
            self._fill_twist_covariance(odom.twist.covariance)
            self.odometry_pub.publish(odom)

    def _fill_pose(self, pose: object, seed: PoseSeed) -> None:
        pose.position.x = seed.x
        pose.position.y = seed.y
        pose.position.z = seed.z
        pose.orientation.x = seed.qx
        pose.orientation.y = seed.qy
        pose.orientation.z = seed.qz
        pose.orientation.w = seed.qw

    def _fill_pose_covariance(self, covariance: list[float]) -> None:
        for i in range(36):
            covariance[i] = 0.0
        covariance[0] = self.pose_covariance
        covariance[7] = self.pose_covariance
        covariance[14] = self.pose_covariance
        covariance[21] = self.orientation_covariance
        covariance[28] = self.orientation_covariance
        covariance[35] = self.orientation_covariance

    def _fill_twist_covariance(self, covariance: list[float]) -> None:
        for i in range(36):
            covariance[i] = 0.0
        covariance[0] = self.velocity_covariance
        covariance[7] = self.velocity_covariance
        covariance[14] = self.velocity_covariance
        covariance[21] = self.velocity_covariance
        covariance[28] = self.velocity_covariance
        covariance[35] = self.velocity_covariance

    def _log_waiting_for_seed(self, now: float) -> None:
        if now - self.last_log_sec < self.log_interval_sec:
            return
        self.last_log_sec = now
        self.get_logger().info(
            "waiting for current local pose seed; "
            f"pose_seen={self.current_local_pose is not None}"
        )

    def _log_snapshot(self, now: float, seed: PoseSeed) -> None:
        if now - self.last_log_sec < self.log_interval_sec:
            return
        self.last_log_sec = now
        mode = self.state.mode if self.state else "UNKNOWN"
        local_text = "none"
        if self.current_local_pose is not None:
            p = self.current_local_pose.pose.position
            local_text = f"x={p.x:.2f} y={p.y:.2f} z={p.z:.2f}"
        self.get_logger().info(
            f"snapshot mode={mode} armed={self._is_armed()} "
            f"manual_input={self._manual_input()} "
            f"subs=vision:{self.vision_pose_pub.get_subscription_count()} "
            f"vision_cov:{self.vision_pose_cov_pub.get_subscription_count()} "
            f"odom:{self.odometry_pub.get_subscription_count()} "
            f"published=x={seed.x:.2f} y={seed.y:.2f} z={seed.z:.2f} "
            f"local={local_text}"
        )

    def _local_pose_cb(self, msg: PoseStamped) -> None:
        self.current_local_pose = msg

    def _state_cb(self, msg: State) -> None:
        self.state = msg
        summary = (
            bool(msg.connected),
            bool(msg.armed),
            str(msg.mode),
            bool(msg.guided),
            bool(msg.manual_input),
        )
        if summary != self.last_state_summary:
            self.last_state_summary = summary
            self.get_logger().warn(
                f"state connected={msg.connected} armed={msg.armed} "
                f"mode={msg.mode} guided={msg.guided} "
                f"manual_input={msg.manual_input}"
            )
            if msg.armed:
                self.get_logger().error(
                    "vehicle is armed while fake external vision is running; keep wheels lifted"
                )

    def _statustext_cb(self, msg: StatusText) -> None:
        text = str(msg.text).strip()
        if text:
            self.get_logger().warn(f"PX4 STATUSTEXT severity={msg.severity}: {text}")

    def _is_armed(self) -> bool:
        return bool(self.state and self.state.armed)

    def _manual_input(self) -> bool:
        return bool(self.state and self.state.manual_input)

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
        or PoseStamped is None
        or PoseWithCovarianceStamped is None
        or Odometry is None
        or State is None
        or StatusText is None
    ):
        raise SystemExit(
            "MAVROS dependencies are not importable. Install/source ros-humble-mavros."
        )

    rclpy.init()
    node = MavrosFakeExternalVision()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        node.get_logger().warn("interrupted; stopping fake external vision")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
