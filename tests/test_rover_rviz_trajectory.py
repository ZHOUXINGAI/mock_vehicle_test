import math
import unittest
from dataclasses import dataclass

from src.rover_rviz_trajectory import (
    ActualTrajectoryTrace,
    path_trace_from_xy,
    quaternion_zw_from_yaw,
    trace_to_plan_frame,
    write_trace_csv,
)


@dataclass(frozen=True)
class Point:
    x_m: float
    y_m: float


class RvizTrajectoryPureTests(unittest.TestCase):
    def test_planned_path_gets_segment_orientations(self):
        trace = path_trace_from_xy(
            [Point(0.0, 0.0), Point(1.0, 0.0), Point(1.0, 1.0)]
        )
        self.assertAlmostEqual(trace[0].yaw_rad, 0.0)
        self.assertAlmostEqual(trace[1].yaw_rad, math.pi / 2.0)
        self.assertAlmostEqual(trace[2].yaw_rad, math.pi / 2.0)

    def test_actual_trace_decimates_stationary_noise_and_bounds_memory(self):
        trace = ActualTrajectoryTrace(max_points=3, min_spacing_m=0.10)
        self.assertTrue(trace.append(0.0, 0.0, 0.0))
        self.assertFalse(trace.append(0.05, 0.0, 0.1))
        self.assertTrue(trace.append(0.10, 0.0, 0.0))
        self.assertTrue(trace.append(0.20, 0.0, 0.0))
        self.assertTrue(trace.append(0.30, 0.0, 0.0))
        self.assertEqual(len(trace.points), 3)
        self.assertAlmostEqual(trace.points[0].x_m, 0.10)

    def test_invalid_samples_fail_closed(self):
        with self.assertRaises(ValueError):
            path_trace_from_xy([Point(0.0, 0.0)])
        with self.assertRaises(ValueError):
            ActualTrajectoryTrace().append(math.nan, 0.0, 0.0)

    def test_yaw_quaternion_is_unit_length(self):
        z, w = quaternion_zw_from_yaw(math.radians(123.0))
        self.assertAlmostEqual(z * z + w * w, 1.0)

    def test_plan_frame_transform_makes_plan_x_axis_explicit(self):
        trace = path_trace_from_xy([Point(2.0, 3.0), Point(3.0, 4.0)])
        transformed = trace_to_plan_frame(trace, trace[0], math.pi / 4.0)
        self.assertAlmostEqual(transformed[0].x_m, 0.0)
        self.assertAlmostEqual(transformed[0].y_m, 0.0)
        self.assertAlmostEqual(transformed[1].x_m, math.sqrt(2.0))
        self.assertAlmostEqual(transformed[1].y_m, 0.0)

    def test_trace_csv_has_stable_header_and_precision(self):
        import tempfile
        from pathlib import Path

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "trace.csv"
            trace = path_trace_from_xy([Point(0.0, 0.0), Point(1.0, 0.0)])
            write_trace_csv(path, trace)
            lines = path.read_text(encoding="ascii").splitlines()
        self.assertEqual(lines[0], "x_m,y_m,yaw_rad")
        self.assertEqual(lines[1], "0.000000000,0.000000000,0.000000000")

    def test_ros_publisher_emits_plan_actual_pose_and_target(self):
        try:
            from builtin_interfaces.msg import Time
        except ImportError:
            self.skipTest("ROS messages are unavailable")

        from src.rover_rviz_trajectory import RvizTrajectoryPublisher

        class Publisher:
            def __init__(self):
                self.messages = []

            def publish(self, message):
                self.messages.append(message)

        class ClockNow:
            @staticmethod
            def to_msg():
                return Time(sec=12, nanosec=34)

        class Clock:
            @staticmethod
            def now():
                return ClockNow()

        class Node:
            def __init__(self):
                self.publishers = {}

            def create_publisher(self, _message_type, topic, _qos):
                publisher = Publisher()
                self.publishers[topic] = publisher
                return publisher

            @staticmethod
            def get_clock():
                return Clock()

        node = Node()
        publisher = RvizTrajectoryPublisher(node, "/test/offboard")
        publisher.start_actual("map")
        publisher.update_actual(0.0, 0.0, 0.0)
        publisher.set_plan([Point(0.0, 0.0), Point(1.0, 0.0)], "map")
        publisher.seed_actual([(0.0, 0.0), (0.1, 0.0)])
        publisher.update(0.2, 0.1, 0.0, 0.8, 0.0)
        self.assertEqual(
            len(node.publishers["/test/offboard/planned_path"].messages[-1].poses),
            2,
        )
        self.assertEqual(
            len(node.publishers["/test/offboard/actual_path"].messages[-1].poses),
            3,
        )
        self.assertEqual(
            node.publishers["/test/offboard/vehicle_pose"].messages[-1].header.frame_id,
            "map",
        )
        self.assertAlmostEqual(
            node.publishers["/test/offboard/lookahead_target"]
            .messages[-1]
            .pose.position.x,
            0.8,
        )

    def test_ros_publisher_rejects_relative_topic_prefix(self):
        try:
            import nav_msgs.msg  # noqa: F401
        except ImportError:
            self.skipTest("ROS messages are unavailable")

        from src.rover_rviz_trajectory import RvizTrajectoryPublisher

        class Node:
            @staticmethod
            def create_publisher(*_args):
                raise AssertionError("invalid prefix must fail before publisher creation")

        with self.assertRaisesRegex(ValueError, "absolute ROS name"):
            RvizTrajectoryPublisher(Node(), "relative/topic")


if __name__ == "__main__":
    unittest.main()
