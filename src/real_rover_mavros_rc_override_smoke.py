#!/usr/bin/env python3

"""Conservative MAVROS RC override smoke test for the real rover."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

try:
    import rclpy
    from mavros_msgs.msg import OverrideRCIn
    from mavros_msgs.msg import State
    from mavros_msgs.msg import StatusText
    from rcl_interfaces.msg import ParameterDescriptor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
except ImportError:  # pragma: no cover
    rclpy = None
    OverrideRCIn = None
    State = None
    StatusText = None
    ParameterDescriptor = None
    Node = object
    qos_profile_sensor_data = 10


@dataclass(frozen=True)
class RcStep:
    name: str
    duration_sec: float
    throttle_us: int
    steering_us: int


class RealRoverMavrosRcOverrideSmoke(Node):
    def __init__(self) -> None:
        super().__init__("real_rover_mavros_rc_override_smoke")

        dynamic_param = ParameterDescriptor(dynamic_typing=True)

        def declare_param(name: str, default: object) -> None:
            self.declare_parameter(name, default, dynamic_param)

        declare_param("mavros_namespace", "/mavros")
        declare_param("override_topic", "rc/override")
        declare_param("command_rate_hz", 20.0)
        declare_param("warmup_sec", 1.0)
        declare_param("stop_sec", 1.0)
        declare_param("forward_sec", 1.0)
        declare_param("backward_sec", 1.0)
        declare_param("turn_sec", 0.5)
        declare_param("final_stop_sec", 2.0)
        declare_param("throttle_channel", 2)
        declare_param("steering_channel", 4)
        declare_param("neutral_pwm_us", 1500)
        declare_param("forward_delta_us", 150)
        declare_param("turn_delta_us", 150)
        declare_param("forward_sign", 1)
        declare_param("turn_sign", 1)
        declare_param("max_delta_us", 180)
        declare_param("min_pwm_us", 1100)
        declare_param("max_pwm_us", 1900)
        declare_param("allowed_modes", "MANUAL")
        declare_param("require_connected", True)
        declare_param("require_armed", True)
        declare_param("abort_on_mode_exit", True)
        declare_param("abort_on_disarm", True)
        declare_param("max_wait_for_ready_sec", 60.0)
        declare_param("release_burst_sec", 1.0)
        declare_param("test_surface", "wheels_lifted")
        declare_param("confirm_wheels_lifted", False)
        declare_param("confirm_ground_area_clear", False)
        declare_param("confirm_low_speed_ground_test", False)
        declare_param("confirm_rc_ready", False)
        declare_param("confirm_rc_sticks_centered", False)
        declare_param("confirm_param_backup", False)

        self.mavros_namespace = str(
            self.get_parameter("mavros_namespace").value
        ).rstrip("/")
        self.override_topic = self._resolve_topic(
            str(self.get_parameter("override_topic").value).strip()
        )
        self.command_rate_hz = max(2.0, float(self.get_parameter("command_rate_hz").value))
        self.period_sec = 1.0 / self.command_rate_hz
        self.warmup_sec = max(0.2, float(self.get_parameter("warmup_sec").value))
        self.stop_sec = max(0.2, float(self.get_parameter("stop_sec").value))
        self.forward_sec = max(0.0, float(self.get_parameter("forward_sec").value))
        self.backward_sec = max(0.0, float(self.get_parameter("backward_sec").value))
        self.turn_sec = max(0.0, float(self.get_parameter("turn_sec").value))
        self.final_stop_sec = max(0.5, float(self.get_parameter("final_stop_sec").value))
        self.throttle_channel = int(self.get_parameter("throttle_channel").value)
        self.steering_channel = int(self.get_parameter("steering_channel").value)
        self.neutral_pwm_us = int(self.get_parameter("neutral_pwm_us").value)
        self.forward_delta_us = abs(int(self.get_parameter("forward_delta_us").value))
        self.turn_delta_us = abs(int(self.get_parameter("turn_delta_us").value))
        self.forward_sign = 1 if int(self.get_parameter("forward_sign").value) >= 0 else -1
        self.turn_sign = 1 if int(self.get_parameter("turn_sign").value) >= 0 else -1
        self.max_delta_us = abs(int(self.get_parameter("max_delta_us").value))
        self.min_pwm_us = int(self.get_parameter("min_pwm_us").value)
        self.max_pwm_us = int(self.get_parameter("max_pwm_us").value)
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
        self.release_burst_sec = max(0.2, float(self.get_parameter("release_burst_sec").value))
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
        self.confirm_rc_sticks_centered = self._as_bool(
            self.get_parameter("confirm_rc_sticks_centered").value
        )
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
        self.started = False
        self.armed_seen = False
        self.last_state_summary: Optional[tuple[bool, bool, str, bool]] = None

        self.override_pub = self.create_publisher(OverrideRCIn, self.override_topic, 10)
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
            f"MAVROS RC override smoke test loaded: ns={self.mavros_namespace} "
            f"topic={self.override_topic} rate={self.command_rate_hz:.1f}Hz "
            f"surface={self.test_surface}"
        )
        self.get_logger().warn(
            f"RC override mapping: throttle_ch={self.throttle_channel} "
            f"steering_ch={self.steering_channel} neutral={self.neutral_pwm_us} "
            f"forward_delta={self.forward_delta_us} turn_delta={self.turn_delta_us} "
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
        if not self.confirm_rc_sticks_centered:
            missing.append("CONFIRM_RC_STICKS_CENTERED=true")
        if not self.confirm_param_backup:
            missing.append("CONFIRM_PARAM_BACKUP=true")
        if missing:
            raise RuntimeError(
                "Refusing to run real-hardware RC override test until safety "
                "is confirmed: " + ", ".join(missing)
            )
        if not 1 <= self.throttle_channel <= 18:
            raise ValueError("throttle_channel must be in 1..18")
        if not 1 <= self.steering_channel <= 18:
            raise ValueError("steering_channel must be in 1..18")
        if self.throttle_channel == self.steering_channel:
            raise ValueError("throttle_channel and steering_channel must differ")
        if self.forward_delta_us > self.max_delta_us:
            raise ValueError("forward_delta_us exceeds max_delta_us")
        if self.turn_delta_us > self.max_delta_us:
            raise ValueError("turn_delta_us exceeds max_delta_us")
        if not self.allowed_modes:
            raise ValueError("allowed_modes must not be empty")
        for value in (
            self.neutral_pwm_us,
            self.neutral_pwm_us + self.forward_delta_us,
            self.neutral_pwm_us - self.forward_delta_us,
            self.neutral_pwm_us + self.turn_delta_us,
            self.neutral_pwm_us - self.turn_delta_us,
        ):
            self._validate_pwm(value)

    def _build_sequence(self) -> list[RcStep]:
        forward_delta = self.forward_delta_us * self.forward_sign
        turn_delta = self.turn_delta_us * self.turn_sign
        neutral = self.neutral_pwm_us
        return [
            RcStep("initial_stop", self.stop_sec, neutral, neutral),
            RcStep("forward", self.forward_sec, neutral + forward_delta, neutral),
            RcStep("stop_after_forward", self.stop_sec, neutral, neutral),
            RcStep("backward", self.backward_sec, neutral - forward_delta, neutral),
            RcStep("stop_after_backward", self.stop_sec, neutral, neutral),
            RcStep("turn_left", self.turn_sec, neutral, neutral + turn_delta),
            RcStep("stop_after_left", self.stop_sec, neutral, neutral),
            RcStep("turn_right", self.turn_sec, neutral, neutral - turn_delta),
            RcStep("final_stop", self.final_stop_sec, neutral, neutral),
        ]

    def _timer_cb(self) -> None:
        if self.done:
            return

        now = self._now_sec()
        if self.step_start_sec is None:
            self._publish_release()
            if self._abort_if_control_lost():
                return
            if self._ready_to_start(now):
                self.step_start_sec = now
                self.step_index = 0
                self.started = True
                self.get_logger().warn("starting MAVROS RC override sequence")
            else:
                self._log_waiting(now)
            return

        if self._abort_if_control_lost():
            return

        step = self.sequence[self.step_index]
        self._publish_override(step.throttle_us, step.steering_us)
        if now - self.step_start_sec < step.duration_sec:
            return

        self.step_index += 1
        if self.step_index >= len(self.sequence):
            self._publish_override(self.neutral_pwm_us, self.neutral_pwm_us)
            self.get_logger().warn(
                "MAVROS RC override sequence complete; neutral sent, releasing override"
            )
            self.publish_release_burst(self.release_burst_sec)
            self.done = True
            return

        self.step_start_sec = now
        self.get_logger().info(f"step -> {self.sequence[self.step_index].name}")

    def _ready_to_start(self, now: float) -> bool:
        if self.override_pub.get_subscription_count() <= 0:
            return self._check_timeout(now, "MAVROS rc override subscriber")
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
        self.publish_release_burst(self.release_burst_sec)
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
        self._publish_override(self.neutral_pwm_us, self.neutral_pwm_us)
        self.publish_release_burst(self.release_burst_sec)
        self.get_logger().error(
            f"aborting RC override sequence: {reason}; neutral sent and override released"
        )
        self.done = True

    def _publish_override(self, throttle_us: int, steering_us: int) -> None:
        self._validate_pwm(throttle_us)
        self._validate_pwm(steering_us)
        msg = OverrideRCIn()
        msg.channels = [OverrideRCIn.CHAN_NOCHANGE] * 18
        msg.channels[self.throttle_channel - 1] = int(throttle_us)
        msg.channels[self.steering_channel - 1] = int(steering_us)
        self.override_pub.publish(msg)

    def _publish_release(self) -> None:
        msg = OverrideRCIn()
        msg.channels = [OverrideRCIn.CHAN_NOCHANGE] * 18
        msg.channels[self.throttle_channel - 1] = OverrideRCIn.CHAN_RELEASE
        msg.channels[self.steering_channel - 1] = OverrideRCIn.CHAN_RELEASE
        self.override_pub.publish(msg)

    def publish_release_burst(self, duration_sec: float) -> None:
        end_sec = self._now_sec() + max(0.2, duration_sec)
        while rclpy.ok() and self._now_sec() < end_sec:
            self._publish_release()
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
            f"holding release; connected={self._is_connected()} "
            f"mode={mode} armed={self._is_armed()} "
            f"override_subs={self.override_pub.get_subscription_count()}"
        )

    def _log_sequence(self) -> None:
        self.get_logger().info("sequence:")
        for index, step in enumerate(self.sequence, start=1):
            self.get_logger().info(
                f"  {index:02d} {step.name}: {step.duration_sec:.2f}s "
                f"throttle={step.throttle_us} steering={step.steering_us}"
            )

    def _resolve_topic(self, topic: str) -> str:
        if topic.startswith("/"):
            return topic
        return f"{self.mavros_namespace}/{topic.lstrip('/')}"

    def _validate_pwm(self, value: int) -> None:
        if not self.min_pwm_us <= int(value) <= self.max_pwm_us:
            raise ValueError(f"PWM value {value} outside min_pwm_us/max_pwm_us")

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
    if rclpy is None or OverrideRCIn is None or State is None or StatusText is None:
        raise SystemExit(
            "MAVROS dependencies are not importable. Install/source ros-humble-mavros."
        )

    rclpy.init()
    node = RealRoverMavrosRcOverrideSmoke()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().warn("interrupted; sending RC override release burst")
    finally:
        if rclpy.ok():
            node.publish_release_burst(node.release_burst_sec)
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
