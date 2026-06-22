#!/usr/bin/env python3

"""Watch MAVROS RC input/output channels for hardware debugging."""

from __future__ import annotations

from typing import Optional

try:
    import rclpy
    from mavros_msgs.msg import RCIn
    from mavros_msgs.msg import RCOut
    from mavros_msgs.msg import State
    from mavros_msgs.msg import StatusText
    from rcl_interfaces.msg import ParameterDescriptor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
except ImportError:  # pragma: no cover
    rclpy = None
    RCIn = None
    RCOut = None
    State = None
    StatusText = None
    ParameterDescriptor = None
    Node = object
    qos_profile_sensor_data = 10


class MavrosRcIoWatch(Node):
    def __init__(self) -> None:
        super().__init__("mavros_rc_io_watch")

        dynamic_param = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameter("mavros_namespace", "/mavros", dynamic_param)
        self.declare_parameter("duration_sec", 40.0, dynamic_param)
        self.declare_parameter("channels_to_print", 8, dynamic_param)
        self.declare_parameter("print_period_sec", 1.0, dynamic_param)
        self.declare_parameter("change_threshold_us", 15, dynamic_param)

        self.mavros_namespace = str(
            self.get_parameter("mavros_namespace").value
        ).rstrip("/")
        self.duration_sec = max(1.0, float(self.get_parameter("duration_sec").value))
        self.channels_to_print = max(
            1, min(18, int(self.get_parameter("channels_to_print").value))
        )
        self.print_period_sec = max(
            0.2, float(self.get_parameter("print_period_sec").value)
        )
        self.change_threshold_us = max(
            1, int(self.get_parameter("change_threshold_us").value)
        )

        self.start_sec = self._now_sec()
        self.last_print_sec = 0.0
        self.done = False
        self.state: Optional[State] = None
        self.rc_in: Optional[list[int]] = None
        self.rc_out: Optional[list[int]] = None
        self.last_logged_in: Optional[list[int]] = None
        self.last_logged_out: Optional[list[int]] = None
        self.in_min: Optional[list[int]] = None
        self.in_max: Optional[list[int]] = None
        self.out_min: Optional[list[int]] = None
        self.out_max: Optional[list[int]] = None

        self.create_subscription(RCIn, f"{self.mavros_namespace}/rc/in", self._rc_in_cb, 10)
        self.create_subscription(RCOut, f"{self.mavros_namespace}/rc/out", self._rc_out_cb, 10)
        self.create_subscription(State, f"{self.mavros_namespace}/state", self._state_cb, 10)
        self.create_subscription(
            StatusText,
            f"{self.mavros_namespace}/statustext/recv",
            self._statustext_cb,
            qos_profile_sensor_data,
        )
        self.timer = self.create_timer(0.1, self._timer_cb)

        self.get_logger().warn(
            f"watching {self.mavros_namespace}/rc/in and /rc/out for "
            f"{self.duration_sec:.1f}s; printing first {self.channels_to_print} channels"
        )

    def _rc_in_cb(self, msg: RCIn) -> None:
        values = [int(value) for value in msg.channels]
        self.rc_in = values
        self.in_min, self.in_max = self._update_range(self.in_min, self.in_max, values)
        if self._changed_enough(self.last_logged_in, values):
            self.last_logged_in = values[:]
            self.get_logger().info(f"rc/in  {self._format_channels(values)}")

    def _rc_out_cb(self, msg: RCOut) -> None:
        values = [int(value) for value in msg.channels]
        self.rc_out = values
        self.out_min, self.out_max = self._update_range(self.out_min, self.out_max, values)
        if self._changed_enough(self.last_logged_out, values):
            self.last_logged_out = values[:]
            self.get_logger().info(f"rc/out {self._format_channels(values)}")

    def _state_cb(self, msg: State) -> None:
        previous = self.state
        self.state = msg
        if (
            previous is None
            or previous.connected != msg.connected
            or previous.armed != msg.armed
            or previous.mode != msg.mode
            or previous.manual_input != msg.manual_input
        ):
            self.get_logger().warn(
                f"state connected={msg.connected} mode={msg.mode} "
                f"armed={msg.armed} manual_input={msg.manual_input}"
            )

    def _statustext_cb(self, msg: StatusText) -> None:
        text = str(msg.text).strip()
        if text:
            self.get_logger().warn(f"PX4 STATUSTEXT severity={msg.severity}: {text}")

    def _timer_cb(self) -> None:
        now = self._now_sec()
        if now - self.start_sec >= self.duration_sec:
            self._print_summary()
            self.done = True
            return
        if now - self.last_print_sec < self.print_period_sec:
            return
        self.last_print_sec = now
        mode = self.state.mode if self.state else "UNKNOWN"
        armed = self.state.armed if self.state else False
        self.get_logger().info(
            f"snapshot mode={mode} armed={armed} "
            f"rc/in={self._format_channels(self.rc_in)} "
            f"rc/out={self._format_channels(self.rc_out)}"
        )

    def _print_summary(self) -> None:
        self.get_logger().warn("RC watch summary:")
        self.get_logger().warn(f"  rc/in  min={self._format_channels(self.in_min)}")
        self.get_logger().warn(f"  rc/in  max={self._format_channels(self.in_max)}")
        self.get_logger().warn(f"  rc/out min={self._format_channels(self.out_min)}")
        self.get_logger().warn(f"  rc/out max={self._format_channels(self.out_max)}")

    def _changed_enough(
        self, previous: Optional[list[int]], current: Optional[list[int]]
    ) -> bool:
        if current is None:
            return False
        if previous is None:
            return True
        count = min(len(previous), len(current), self.channels_to_print)
        for index in range(count):
            if abs(current[index] - previous[index]) >= self.change_threshold_us:
                return True
        return len(previous) != len(current)

    def _update_range(
        self,
        current_min: Optional[list[int]],
        current_max: Optional[list[int]],
        values: list[int],
    ) -> tuple[list[int], list[int]]:
        if current_min is None or current_max is None:
            return values[:], values[:]
        count = min(len(current_min), len(current_max), len(values))
        for index in range(count):
            current_min[index] = min(current_min[index], values[index])
            current_max[index] = max(current_max[index], values[index])
        if len(values) > count:
            current_min.extend(values[count:])
            current_max.extend(values[count:])
        return current_min, current_max

    def _format_channels(self, values: Optional[list[int]]) -> str:
        if values is None:
            return "none"
        pairs = []
        for index, value in enumerate(values[: self.channels_to_print], start=1):
            pairs.append(f"{index}:{value}")
        return " ".join(pairs)

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9


def main() -> None:
    if rclpy is None or RCIn is None or RCOut is None or State is None:
        raise SystemExit(
            "MAVROS dependencies are not importable. Install/source ros-humble-mavros."
        )

    rclpy.init()
    node = MavrosRcIoWatch()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
