#!/usr/bin/env python3

"""ROS-independent trace storage and optional RViz trajectory publishers."""

from __future__ import annotations

import math
import csv
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable


@dataclass(frozen=True)
class TracePoint:
    x_m: float
    y_m: float
    yaw_rad: float


def quaternion_zw_from_yaw(yaw_rad: float) -> tuple[float, float]:
    if not math.isfinite(yaw_rad):
        raise ValueError("yaw must be finite")
    return math.sin(0.5 * yaw_rad), math.cos(0.5 * yaw_rad)


def path_trace_from_xy(points: Iterable[object]) -> tuple[TracePoint, ...]:
    """Convert objects with x_m/y_m fields to a finite oriented path."""
    coordinates: list[tuple[float, float]] = []
    for point in points:
        if hasattr(point, "x_m") and hasattr(point, "y_m"):
            x_m = float(getattr(point, "x_m"))
            y_m = float(getattr(point, "y_m"))
        else:
            x_m = float(point[0])
            y_m = float(point[1])
        if not math.isfinite(x_m) or not math.isfinite(y_m):
            raise ValueError("path coordinates must be finite")
        coordinates.append((x_m, y_m))
    if len(coordinates) < 2:
        raise ValueError("path requires at least two points")

    result: list[TracePoint] = []
    previous_yaw = 0.0
    for index, (x_m, y_m) in enumerate(coordinates):
        if index + 1 < len(coordinates):
            following = coordinates[index + 1]
            previous_yaw = math.atan2(following[1] - y_m, following[0] - x_m)
        result.append(TracePoint(x_m, y_m, previous_yaw))
    return tuple(result)


def write_trace_csv(path: Path, points: Iterable[TracePoint]) -> None:
    """Persist one finite trace without depending on ROS or plotting tools."""
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="ascii") as stream:
        writer = csv.writer(stream)
        writer.writerow(("x_m", "y_m", "yaw_rad"))
        for point in points:
            if not all(
                math.isfinite(value)
                for value in (point.x_m, point.y_m, point.yaw_rad)
            ):
                raise ValueError("trajectory artifact points must be finite")
            writer.writerow((f"{point.x_m:.9f}", f"{point.y_m:.9f}", f"{point.yaw_rad:.9f}"))


def trace_to_plan_frame(
    points: Iterable[TracePoint], origin: TracePoint, plan_yaw_rad: float
) -> tuple[TracePoint, ...]:
    """Translate and rotate map-frame samples into plan-aligned XY."""
    if not all(
        math.isfinite(value)
        for value in (origin.x_m, origin.y_m, plan_yaw_rad)
    ):
        raise ValueError("plan-frame transform must be finite")
    cosine = math.cos(plan_yaw_rad)
    sine = math.sin(plan_yaw_rad)
    transformed = []
    for point in points:
        dx = point.x_m - origin.x_m
        dy = point.y_m - origin.y_m
        transformed.append(
            TracePoint(
                cosine * dx + sine * dy,
                -sine * dx + cosine * dy,
                math.atan2(
                    math.sin(point.yaw_rad - plan_yaw_rad),
                    math.cos(point.yaw_rad - plan_yaw_rad),
                ),
            )
        )
    return tuple(transformed)


class ActualTrajectoryTrace:
    def __init__(self, max_points: int = 5000, min_spacing_m: float = 0.02) -> None:
        if not isinstance(max_points, int) or max_points < 2:
            raise ValueError("max_points must be an integer >= 2")
        if not math.isfinite(min_spacing_m) or min_spacing_m <= 0.0:
            raise ValueError("min_spacing_m must be positive and finite")
        self.max_points = max_points
        self.min_spacing_m = min_spacing_m
        self._points: list[TracePoint] = []

    @property
    def points(self) -> tuple[TracePoint, ...]:
        return tuple(self._points)

    def reset(self) -> None:
        self._points.clear()

    def append(self, x_m: float, y_m: float, yaw_rad: float) -> bool:
        if not all(math.isfinite(value) for value in (x_m, y_m, yaw_rad)):
            raise ValueError("actual trajectory sample must be finite")
        point = TracePoint(float(x_m), float(y_m), float(yaw_rad))
        if self._points:
            previous = self._points[-1]
            distance_m = math.hypot(
                point.x_m - previous.x_m, point.y_m - previous.y_m
            )
            if distance_m + 1.0e-12 < self.min_spacing_m:
                return False
        self._points.append(point)
        if len(self._points) > self.max_points:
            del self._points[: len(self._points) - self.max_points]
        return True


class RvizTrajectoryPublisher:
    """Publish planned/actual paths and live poses without owning a ROS node."""

    def __init__(self, node, topic_prefix: str = "/orin2/offboard") -> None:
        from geometry_msgs.msg import PoseStamped
        from nav_msgs.msg import Path
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

        prefix = topic_prefix.rstrip("/")
        if not prefix.startswith("/") or prefix == "":
            raise ValueError("RViz topic prefix must be an absolute ROS name")
        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        live_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self._node = node
        self._pose_type = PoseStamped
        self._path_type = Path
        self._planned_pub = node.create_publisher(
            Path, f"{prefix}/planned_path", latched_qos
        )
        self._actual_pub = node.create_publisher(
            Path, f"{prefix}/actual_path", live_qos
        )
        self._vehicle_pub = node.create_publisher(
            PoseStamped, f"{prefix}/vehicle_pose", live_qos
        )
        self._target_pub = node.create_publisher(
            PoseStamped, f"{prefix}/lookahead_target", live_qos
        )
        self._actual = ActualTrajectoryTrace()
        self._planned: tuple[TracePoint, ...] = ()
        self._frame_id = "map"

    @staticmethod
    def _set_pose(pose, point: TracePoint) -> None:
        pose.position.x = point.x_m
        pose.position.y = point.y_m
        pose.position.z = 0.0
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z, pose.orientation.w = quaternion_zw_from_yaw(
            point.yaw_rad
        )

    def _pose_message(self, point: TracePoint, stamp):
        message = self._pose_type()
        message.header.frame_id = self._frame_id
        message.header.stamp = stamp
        self._set_pose(message.pose, point)
        return message

    def _path_message(self, points: Iterable[TracePoint], stamp):
        message = self._path_type()
        message.header.frame_id = self._frame_id
        message.header.stamp = stamp
        message.poses = [self._pose_message(point, stamp) for point in points]
        return message

    def set_plan(self, points: Iterable[object], frame_id: str) -> None:
        self._frame_id = frame_id or "map"
        self._actual.reset()
        self._planned = path_trace_from_xy(points)
        stamp = self._node.get_clock().now().to_msg()
        self._planned_pub.publish(
            self._path_message(self._planned, stamp)
        )
        self._actual_pub.publish(self._path_message((), stamp))

    def start_actual(self, frame_id: str) -> None:
        self._frame_id = frame_id or "map"
        self._actual.reset()
        stamp = self._node.get_clock().now().to_msg()
        self._actual_pub.publish(self._path_message((), stamp))

    def seed_actual(self, points: Iterable[object]) -> None:
        trace = path_trace_from_xy(points)
        self._actual.reset()
        for point in trace:
            self._actual.append(point.x_m, point.y_m, point.yaw_rad)
        stamp = self._node.get_clock().now().to_msg()
        self._actual_pub.publish(self._path_message(self._actual.points, stamp))

    def update_actual(self, x_m: float, y_m: float, yaw_rad: float) -> None:
        stamp = self._node.get_clock().now().to_msg()
        vehicle = TracePoint(x_m, y_m, yaw_rad)
        self._vehicle_pub.publish(self._pose_message(vehicle, stamp))
        if self._actual.append(x_m, y_m, yaw_rad):
            self._actual_pub.publish(self._path_message(self._actual.points, stamp))

    def update(
        self,
        x_m: float,
        y_m: float,
        yaw_rad: float,
        target_x_m: float,
        target_y_m: float,
    ) -> None:
        self.update_actual(x_m, y_m, yaw_rad)
        stamp = self._node.get_clock().now().to_msg()
        target_yaw = math.atan2(target_y_m - y_m, target_x_m - x_m)
        self._target_pub.publish(
            self._pose_message(TracePoint(target_x_m, target_y_m, target_yaw), stamp)
        )

    def write_artifacts(self, directory: Path) -> tuple[Path, Path]:
        if len(self._planned) < 2:
            raise RuntimeError("planned trajectory is unavailable")
        planned_path = directory / "planned_trajectory.csv"
        actual_path = directory / "actual_trajectory.csv"
        write_trace_csv(planned_path, self._planned)
        write_trace_csv(actual_path, self._actual.points)
        return planned_path, actual_path
