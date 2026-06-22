#!/usr/bin/env python3

"""Publish a fixed MAVROS GPS_INPUT stream and watch PX4 position outputs."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Optional

try:
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from mavros_msgs.msg import GPSINPUT
    from mavros_msgs.msg import State
    from mavros_msgs.msg import StatusText
    from rcl_interfaces.msg import ParameterDescriptor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import NavSatFix
except ImportError:  # pragma: no cover
    rclpy = None
    GPSINPUT = None
    State = None
    StatusText = None
    ParameterDescriptor = None
    Node = object
    qos_profile_sensor_data = 10
    NavSatFix = None
    PoseStamped = None


GPS_EPOCH_UNIX_SEC = 315964800
SECONDS_PER_WEEK = 7 * 24 * 60 * 60


@dataclass(frozen=True)
class GpsWeekTime:
    week: int
    week_ms: int


class MavrosFakeGpsInput(Node):
    def __init__(self) -> None:
        super().__init__("mavros_fake_gps_input")

        dynamic_param = ParameterDescriptor(dynamic_typing=True)

        def declare_param(name: str, default: object) -> None:
            self.declare_parameter(name, default, dynamic_param)

        declare_param("mavros_namespace", "/mavros")
        declare_param("gps_input_topic", "gps_input/gps_input")
        declare_param("rate_hz", 10.0)
        declare_param("duration_sec", 0.0)
        declare_param("lat_deg", 22.41674)
        declare_param("lon_deg", 114.04280)
        declare_param("alt_m", 20.0)
        declare_param("fix_type", 3)
        declare_param("satellites_visible", 14)
        declare_param("hdop", 0.7)
        declare_param("vdop", 0.9)
        declare_param("speed_accuracy", 0.1)
        declare_param("horiz_accuracy", 0.8)
        declare_param("vert_accuracy", 1.2)
        declare_param("yaw_cdeg", 0)
        declare_param("gps_id", 0)
        declare_param("frame_id", "fake_gps")
        declare_param("log_interval_sec", 1.0)

        self.mavros_namespace = str(
            self.get_parameter("mavros_namespace").value
        ).rstrip("/")
        raw_topic = str(self.get_parameter("gps_input_topic").value).strip()
        self.gps_input_topic = self._resolve_topic(raw_topic)
        self.rate_hz = max(1.0, float(self.get_parameter("rate_hz").value))
        self.period_sec = 1.0 / self.rate_hz
        self.duration_sec = max(0.0, float(self.get_parameter("duration_sec").value))
        self.lat_deg = float(self.get_parameter("lat_deg").value)
        self.lon_deg = float(self.get_parameter("lon_deg").value)
        self.alt_m = float(self.get_parameter("alt_m").value)
        self.fix_type = int(self.get_parameter("fix_type").value)
        self.satellites_visible = int(
            self.get_parameter("satellites_visible").value
        )
        self.hdop = float(self.get_parameter("hdop").value)
        self.vdop = float(self.get_parameter("vdop").value)
        self.speed_accuracy = float(self.get_parameter("speed_accuracy").value)
        self.horiz_accuracy = float(self.get_parameter("horiz_accuracy").value)
        self.vert_accuracy = float(self.get_parameter("vert_accuracy").value)
        self.yaw_cdeg = int(self.get_parameter("yaw_cdeg").value)
        self.gps_id = int(self.get_parameter("gps_id").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.log_interval_sec = max(
            0.5, float(self.get_parameter("log_interval_sec").value)
        )

        self._validate()

        self.start_time_sec = self._now_sec()
        self.last_log_sec = -1.0e9
        self.done = False
        self.state: Optional[State] = None
        self.global_fix: Optional[NavSatFix] = None
        self.local_pose: Optional[PoseStamped] = None
        self.last_state_summary: Optional[tuple[bool, bool, str, bool, bool]] = None

        self.gps_pub = self.create_publisher(GPSINPUT, self.gps_input_topic, 10)
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
        self.create_subscription(
            NavSatFix,
            f"{self.mavros_namespace}/global_position/global",
            self._global_fix_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PoseStamped,
            f"{self.mavros_namespace}/local_position/pose",
            self._local_pose_cb,
            qos_profile_sensor_data,
        )
        self.timer = self.create_timer(self.period_sec, self._timer_cb)

        self.get_logger().warn(
            "fake GPS_INPUT publisher loaded: "
            f"topic={self.gps_input_topic} rate={self.rate_hz:.1f}Hz "
            f"lat={self.lat_deg:.7f} lon={self.lon_deg:.7f} alt={self.alt_m:.1f}m "
            f"duration={'infinite' if self.duration_sec <= 0.0 else self.duration_sec}"
        )

    def _validate(self) -> None:
        if not -90.0 <= self.lat_deg <= 90.0:
            raise ValueError("lat_deg must be in [-90, 90]")
        if not -180.0 <= self.lon_deg <= 180.0:
            raise ValueError("lon_deg must be in [-180, 180]")
        if not 0 <= self.fix_type <= 8:
            raise ValueError("fix_type must be in [0, 8]")
        if not 0 <= self.satellites_visible <= 255:
            raise ValueError("satellites_visible must be in [0, 255]")
        if not 0 <= self.gps_id <= 255:
            raise ValueError("gps_id must be in [0, 255]")
        if not 0 <= self.yaw_cdeg <= 36000:
            raise ValueError("yaw_cdeg must be in [0, 36000]")
        for name, value in {
            "hdop": self.hdop,
            "vdop": self.vdop,
            "speed_accuracy": self.speed_accuracy,
            "horiz_accuracy": self.horiz_accuracy,
            "vert_accuracy": self.vert_accuracy,
        }.items():
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f"{name} must be finite and non-negative")

    def _timer_cb(self) -> None:
        if self.done:
            return

        now = self._now_sec()
        self._publish_gps_input()
        self._log_snapshot(now)

        if self.duration_sec > 0.0 and now - self.start_time_sec >= self.duration_sec:
            self.get_logger().warn("fake GPS_INPUT duration complete")
            self.done = True

    def _publish_gps_input(self) -> None:
        msg = GPSINPUT()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.fix_type = int(self.fix_type)
        msg.gps_id = int(self.gps_id)
        msg.ignore_flags = 0
        gps_time = self._gps_week_time(time.time())
        msg.time_week = int(gps_time.week)
        msg.time_week_ms = int(gps_time.week_ms)
        msg.lat = int(round(self.lat_deg * 1e7))
        msg.lon = int(round(self.lon_deg * 1e7))
        msg.alt = float(self.alt_m)
        msg.hdop = float(self.hdop)
        msg.vdop = float(self.vdop)
        msg.vn = 0.0
        msg.ve = 0.0
        msg.vd = 0.0
        msg.speed_accuracy = float(self.speed_accuracy)
        msg.horiz_accuracy = float(self.horiz_accuracy)
        msg.vert_accuracy = float(self.vert_accuracy)
        msg.satellites_visible = int(self.satellites_visible)
        msg.yaw = int(self.yaw_cdeg)
        self.gps_pub.publish(msg)

    def _log_snapshot(self, now: float) -> None:
        if now - self.last_log_sec < self.log_interval_sec:
            return
        self.last_log_sec = now
        mode = self.state.mode if self.state else "UNKNOWN"
        global_text = "none"
        if self.global_fix is not None:
            global_text = (
                f"{self.global_fix.latitude:.7f},"
                f"{self.global_fix.longitude:.7f},"
                f"{self.global_fix.altitude:.1f}"
            )
        local_text = "none"
        if self.local_pose is not None:
            p = self.local_pose.pose.position
            local_text = f"x={p.x:.2f} y={p.y:.2f} z={p.z:.2f}"
        self.get_logger().info(
            f"snapshot mode={mode} armed={self._is_armed()} "
            f"manual_input={self._manual_input()} "
            f"gps_input_subs={self.gps_pub.get_subscription_count()} "
            f"global={global_text} local={local_text}"
        )

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
                    "vehicle is armed while fake GPS is running; keep wheels lifted"
                )

    def _statustext_cb(self, msg: StatusText) -> None:
        text = str(msg.text).strip()
        if text:
            self.get_logger().warn(f"PX4 STATUSTEXT severity={msg.severity}: {text}")

    def _global_fix_cb(self, msg: NavSatFix) -> None:
        self.global_fix = msg

    def _local_pose_cb(self, msg: PoseStamped) -> None:
        self.local_pose = msg

    def _resolve_topic(self, topic: str) -> str:
        if topic.startswith("/"):
            return topic
        return f"{self.mavros_namespace}/{topic.lstrip('/')}"

    def _is_armed(self) -> bool:
        return bool(self.state and self.state.armed)

    def _manual_input(self) -> bool:
        return bool(self.state and self.state.manual_input)

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    @staticmethod
    def _gps_week_time(unix_sec: float) -> GpsWeekTime:
        gps_elapsed = max(0.0, unix_sec - GPS_EPOCH_UNIX_SEC)
        week = int(gps_elapsed // SECONDS_PER_WEEK)
        week_sec = gps_elapsed - week * SECONDS_PER_WEEK
        return GpsWeekTime(week=week, week_ms=int(week_sec * 1000.0))


def main() -> None:
    if (
        rclpy is None
        or GPSINPUT is None
        or State is None
        or StatusText is None
        or NavSatFix is None
        or PoseStamped is None
    ):
        raise SystemExit(
            "MAVROS dependencies are not importable. Install/source ros-humble-mavros."
        )

    rclpy.init()
    node = MavrosFakeGpsInput()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().warn("interrupted; stopping fake GPS_INPUT")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
