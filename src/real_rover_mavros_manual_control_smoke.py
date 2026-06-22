#!/usr/bin/env python3

"""Conservative MAVROS MANUAL_CONTROL smoke test for the real rover."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

try:
    import rclpy
    from mavros_msgs.msg import ManualControl
    from mavros_msgs.msg import State
    from mavros_msgs.msg import StatusText
    from rcl_interfaces.msg import ParameterDescriptor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
except ImportError:  # pragma: no cover
    rclpy = None
    ManualControl = None
    State = None
    StatusText = None
    ParameterDescriptor = None
    Node = object
    qos_profile_sensor_data = 10


@dataclass(frozen=True)
class ManualStep:
    name: str
    duration_sec: float
    x: float
    y: float
    z: float
    r: float


class RealRoverMavrosManualControlSmoke(Node):
    def __init__(self) -> None:
        super().__init__("real_rover_mavros_manual_control_smoke")

        dynamic_param = ParameterDescriptor(dynamic_typing=True)

        def declare_param(name: str, default: object) -> None:
            self.declare_parameter(name, default, dynamic_param)

        declare_param("mavros_namespace", "/mavros")
        declare_param("manual_control_topic", "manual_control/send")
        declare_param("command_rate_hz", 20.0)
        declare_param("warmup_sec", 2.0)
        declare_param("stop_sec", 1.0)
        declare_param("forward_sec", 1.0)
        declare_param("backward_sec", 1.0)
        declare_param("turn_sec", 0.5)
        declare_param("final_stop_sec", 2.0)
        declare_param("forward_axis", "x")
        declare_param("turn_axis", "y")
        declare_param("forward_value_raw", 120.0)
        declare_param("turn_value_raw", 120.0)
        declare_param("forward_sign", 1.0)
        declare_param("turn_sign", 1.0)
        declare_param("neutral_z_raw", 0.0)
        declare_param("max_abs_xy_r_raw", 250.0)
        declare_param("min_z_raw", 0.0)
        declare_param("max_z_raw", 1000.0)
        declare_param("allowed_modes", "MANUAL")
        declare_param("require_connected", True)
        declare_param("require_armed", True)
        declare_param("abort_on_mode_exit", True)
        declare_param("abort_on_disarm", True)
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
        raw_topic = str(self.get_parameter("manual_control_topic").value).strip()
        self.manual_control_topic = self._resolve_topic(raw_topic)
        self.command_rate_hz = max(2.0, float(self.get_parameter("command_rate_hz").value))
        self.period_sec = 1.0 / self.command_rate_hz
        self.warmup_sec = max(0.5, float(self.get_parameter("warmup_sec").value))
        self.stop_sec = max(0.2, float(self.get_parameter("stop_sec").value))
        self.forward_sec = max(0.0, float(self.get_parameter("forward_sec").value))
        self.backward_sec = max(0.0, float(self.get_parameter("backward_sec").value))
        self.turn_sec = max(0.0, float(self.get_parameter("turn_sec").value))
        self.final_stop_sec = max(0.5, float(self.get_parameter("final_stop_sec").value))
        self.forward_axis = str(self.get_parameter("forward_axis").value).strip().lower()
        self.turn_axis = str(self.get_parameter("turn_axis").value).strip().lower()
        self.forward_value_raw = abs(
            float(self.get_parameter("forward_value_raw").value)
        )
        self.turn_value_raw = abs(float(self.get_parameter("turn_value_raw").value))
        self.forward_sign = (
            1.0 if float(self.get_parameter("forward_sign").value) >= 0.0 else -1.0
        )
        self.turn_sign = (
            1.0 if float(self.get_parameter("turn_sign").value) >= 0.0 else -1.0
        )
        self.neutral_z_raw = float(self.get_parameter("neutral_z_raw").value)
        self.max_abs_xy_r_raw = abs(
            float(self.get_parameter("max_abs_xy_r_raw").value)
        )
        self.min_z_raw = float(self.get_parameter("min_z_raw").value)
        self.max_z_raw = float(self.get_parameter("max_z_raw").value)
        self.allowed_modes = {
            mode.strip().upper()
            for mode in str(self.get_parameter("allowed_modes").value).split(",")
            if mode.strip()
        }
        self.require_connected = self._as_bool(
            self.get_parameter("require_connected").value
        )
        self.require_armed = self._as_bool(self.get_parameter("require_armed").value)
        self.abort_on_mode_exit = self._as_bool(
            self.get_parameter("abort_on_mode_exit").value
        )
        self.abort_on_disarm = self._as_bool(
            self.get_parameter("abort_on_disarm").value
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
        self.done = False
        self.last_state_summary: Optional[tuple[bool, bool, str, bool]] = None
        self.armed_seen = False
        self.started = False

        self.manual_pub = self.create_publisher(ManualControl, self.manual_control_topic, 10)
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
            f"MAVROS manual-control smoke test loaded: "
            f"ns={self.mavros_namespace} topic={self.manual_control_topic} "
            f"rate={self.command_rate_hz:.1f}Hz surface={self.test_surface}"
        )
        self.get_logger().warn(
            f"control mapping: forward_axis={self.forward_axis} "
            f"turn_axis={self.turn_axis} forward_raw={self.forward_value_raw:.1f} "
            f"turn_raw={self.turn_value_raw:.1f} neutral_z={self.neutral_z_raw:.1f} "
            f"allowed_modes={','.join(sorted(self.allowed_modes))}"
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
                "Refusing to run real-hardware manual-control test until safety "
                "is confirmed: " + ", ".join(missing)
            )
        if self.forward_axis not in {"x", "y", "z", "r"}:
            raise ValueError("forward_axis must be one of x, y, z, r")
        if self.turn_axis not in {"x", "y", "z", "r"}:
            raise ValueError("turn_axis must be one of x, y, z, r")
        if self.forward_axis == self.turn_axis:
            raise ValueError("forward_axis and turn_axis must be different")
        if self.forward_value_raw > self.max_abs_xy_r_raw:
            raise ValueError("forward_value_raw exceeds max_abs_xy_r_raw")
        if self.turn_value_raw > self.max_abs_xy_r_raw:
            raise ValueError("turn_value_raw exceeds max_abs_xy_r_raw")
        if not self.allowed_modes:
            raise ValueError("allowed_modes must not be empty")
        self._validate_manual_values(0.0, 0.0, self.neutral_z_raw, 0.0)

    def _build_sequence(self) -> list[ManualStep]:
        forward = self.forward_value_raw * self.forward_sign
        turn = self.turn_value_raw * self.turn_sign
        return [
            self._step("initial_stop", self.stop_sec, 0.0, 0.0),
            self._step("forward", self.forward_sec, forward, 0.0),
            self._step("stop_after_forward", self.stop_sec, 0.0, 0.0),
            self._step("backward", self.backward_sec, -forward, 0.0),
            self._step("stop_after_backward", self.stop_sec, 0.0, 0.0),
            self._step("turn_left", self.turn_sec, 0.0, turn),
            self._step("stop_after_left", self.stop_sec, 0.0, 0.0),
            self._step("turn_right", self.turn_sec, 0.0, -turn),
            self._step("final_stop", self.final_stop_sec, 0.0, 0.0),
        ]

    def _step(
        self, name: str, duration_sec: float, forward_raw: float, turn_raw: float
    ) -> ManualStep:
        values = {"x": 0.0, "y": 0.0, "z": self.neutral_z_raw, "r": 0.0}
        values[self.forward_axis] = forward_raw
        values[self.turn_axis] = turn_raw
        self._validate_manual_values(values["x"], values["y"], values["z"], values["r"])
        return ManualStep(name, duration_sec, values["x"], values["y"], values["z"], values["r"])

    def _timer_cb(self) -> None:
        if self.done:
            return

        now = self._now_sec()
        if self.step_start_sec is None:
            self._publish_manual(0.0, 0.0, self.neutral_z_raw, 0.0)
            if self._abort_if_control_lost():
                return
            if self._ready_to_start(now):
                self.step_start_sec = now
                self.step_index = 0
                self.started = True
                self.get_logger().warn("starting MAVROS manual-control sequence")
            else:
                self._log_waiting(now)
            return

        if self._abort_if_control_lost():
            return

        step = self.sequence[self.step_index]
        self._publish_manual(step.x, step.y, step.z, step.r)
        if now - self.step_start_sec < step.duration_sec:
            return

        self.step_index += 1
        if self.step_index >= len(self.sequence):
            self._publish_manual(0.0, 0.0, self.neutral_z_raw, 0.0)
            self.get_logger().warn(
                "MAVROS manual-control sequence complete; neutral command sent"
            )
            self.done = True
            return

        self.step_start_sec = now
        self.get_logger().info(f"step -> {self.sequence[self.step_index].name}")

    def _ready_to_start(self, now: float) -> bool:
        if self.manual_pub.get_subscription_count() <= 0:
            return self._check_timeout(now, "MAVROS manual_control subscriber")
        if self.require_connected and not self._is_connected():
            return self._check_timeout(now, "MAVROS connection")
        if not self._is_allowed_mode():
            return self._check_timeout(now, "allowed manual mode")
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
        self._publish_manual(0.0, 0.0, self.neutral_z_raw, 0.0)
        self.done = True
        return False

    def _abort_if_control_lost(self) -> bool:
        if self.state is None:
            return False
        if self.abort_on_mode_exit and self.started and not self._is_allowed_mode():
            self._abort_sequence(f"mode changed away from allowed modes: {self.state.mode}")
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
        self._publish_manual(0.0, 0.0, self.neutral_z_raw, 0.0)
        self.get_logger().error(
            f"aborting manual-control sequence: {reason}; neutral command sent"
        )
        self.done = True

    def _publish_manual(self, x: float, y: float, z: float, r: float) -> None:
        msg = ManualControl()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.x = float(x)
        msg.y = float(y)
        msg.z = float(z)
        msg.r = float(r)
        msg.buttons = 0
        msg.buttons2 = 0
        msg.enabled_extensions = 0
        self.manual_pub.publish(msg)

    def publish_stop_burst(self, duration_sec: float) -> None:
        end_sec = self._now_sec() + max(0.2, duration_sec)
        while rclpy.ok() and self._now_sec() < end_sec:
            self._publish_manual(0.0, 0.0, self.neutral_z_raw, 0.0)
            rclpy.spin_once(self, timeout_sec=self.period_sec)

    def _state_cb(self, msg: State) -> None:
        self.state = msg
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
        if text:
            self.get_logger().warn(f"PX4 STATUSTEXT severity={msg.severity}: {text}")

    def _is_connected(self) -> bool:
        return bool(self.state and self.state.connected)

    def _is_armed(self) -> bool:
        return bool(self.state and self.state.armed)

    def _is_allowed_mode(self) -> bool:
        return bool(self.state and self.state.mode.upper() in self.allowed_modes)

    def _log_waiting(self, now: float) -> None:
        if now - self.last_wait_log_sec < 1.0:
            return
        self.last_wait_log_sec = now
        mode = self.state.mode if self.state else "UNKNOWN"
        self.get_logger().info(
            f"holding neutral; connected={self._is_connected()} "
            f"mode={mode} armed={self._is_armed()} "
            f"manual_subs={self.manual_pub.get_subscription_count()}"
        )

    def _log_sequence(self) -> None:
        self.get_logger().info("sequence:")
        for index, step in enumerate(self.sequence, start=1):
            self.get_logger().info(
                f"  {index:02d} {step.name}: {step.duration_sec:.2f}s "
                f"x={step.x:.1f} y={step.y:.1f} z={step.z:.1f} r={step.r:.1f}"
            )

    def _resolve_topic(self, topic: str) -> str:
        if topic.startswith("/"):
            return topic
        return f"{self.mavros_namespace}/{topic.lstrip('/')}"

    def _validate_manual_values(self, x: float, y: float, z: float, r: float) -> None:
        for axis, value in {"x": x, "y": y, "r": r}.items():
            if abs(value) > self.max_abs_xy_r_raw:
                raise ValueError(f"{axis}={value} exceeds max_abs_xy_r_raw")
        if not self.min_z_raw <= z <= self.max_z_raw:
            raise ValueError("z value outside min_z_raw/max_z_raw")

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
    if rclpy is None or ManualControl is None or State is None or StatusText is None:
        raise SystemExit(
            "MAVROS dependencies are not importable. Install/source ros-humble-mavros."
        )

    rclpy.init()
    node = RealRoverMavrosManualControlSmoke()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().warn("interrupted; sending neutral burst")
    finally:
        if rclpy.ok():
            node.publish_stop_burst(node.stop_burst_sec)
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
