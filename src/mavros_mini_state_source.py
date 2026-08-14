#!/usr/bin/env python3

"""Fail-closed MAVROS telemetry source for compact Pair B MiniState frames."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass

try:
    from lr24_compact_protocol import FieldOrigin, HealthFlag, MiniState
    from lr24_field_frame import FieldFrame
except ImportError:
    from src.lr24_compact_protocol import FieldOrigin, HealthFlag, MiniState
    from src.lr24_field_frame import FieldFrame


DEFAULT_SAMPLE_TIMEOUT_SEC = 2.0


@dataclass
class StateSample:
    connected: bool = False
    armed: bool = False
    mode: str = ""
    manual_input: bool = False
    received_mono: float | None = None


@dataclass
class PoseSample:
    x_m: float = 0.0
    y_m: float = 0.0
    qx: float = 0.0
    qy: float = 0.0
    qz: float = 0.0
    qw: float = 1.0
    received_mono: float | None = None


@dataclass
class VelocitySample:
    vx_mps: float = 0.0
    vy_mps: float = 0.0
    received_mono: float | None = None


@dataclass
class AngularRateSample:
    omega_radps: float = 0.0
    received_mono: float | None = None


@dataclass
class GlobalPositionSample:
    latitude_deg: float = 0.0
    longitude_deg: float = 0.0
    altitude_m: float = 0.0
    status: int = -1
    received_mono: float | None = None


def _fresh(sample_mono: float | None, now_mono: float, timeout_sec: float) -> bool:
    return (
        sample_mono is not None
        and math.isfinite(sample_mono)
        and 0.0 <= now_mono - sample_mono <= timeout_sec
    )


def quaternion_yaw(qx: float, qy: float, qz: float, qw: float) -> float | None:
    values = (qx, qy, qz, qw)
    if not all(math.isfinite(value) for value in values):
        return None
    norm_sq = sum(value * value for value in values)
    if norm_sq < 1.0e-8:
        return None
    scale = 1.0 / math.sqrt(norm_sq)
    qx, qy, qz, qw = (value * scale for value in values)
    sin_yaw = 2.0 * (qw * qz + qx * qy)
    cos_yaw = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(sin_yaw, cos_yaw)


class MiniStateAccumulator:
    """Builds health-qualified state without granting execution readiness."""

    def __init__(self, sample_timeout_sec: float = DEFAULT_SAMPLE_TIMEOUT_SEC) -> None:
        if not math.isfinite(sample_timeout_sec) or sample_timeout_sec <= 0.0:
            raise ValueError("sample_timeout_sec must be finite and positive")
        self.sample_timeout_sec = sample_timeout_sec
        self.state = StateSample()
        self.pose = PoseSample()
        self.velocity = VelocitySample()
        self.angular_rate = AngularRateSample()
        self.global_position = GlobalPositionSample()
        self.field_frame: FieldFrame | None = None

    def build(
        self,
        *,
        vehicle_id: int,
        seq: int,
        timestamp_ms: int,
        now_mono: float,
    ) -> MiniState:
        health = HealthFlag(0)
        x_m = y_m = vx_mps = vy_mps = yaw_rad = omega_radps = 0.0
        origin_id = 0

        if (
            _fresh(self.state.received_mono, now_mono, self.sample_timeout_sec)
            and self.state.connected
        ):
            health |= HealthFlag.PX4_CONNECTED
            if not self.state.armed:
                health |= HealthFlag.DISARMED
            if self.state.manual_input:
                health |= HealthFlag.MANUAL_INPUT

        pose_fresh = _fresh(
            self.pose.received_mono, now_mono, self.sample_timeout_sec
        )
        if self.field_frame is None:
            if (
                pose_fresh
                and math.isfinite(self.pose.x_m)
                and math.isfinite(self.pose.y_m)
            ):
                x_m, y_m = self.pose.x_m, self.pose.y_m
                health |= HealthFlag.POSITION_VALID
        elif (
            _fresh(
                self.global_position.received_mono,
                now_mono,
                self.sample_timeout_sec,
            )
            and self.global_position.status >= 0
            and all(
                math.isfinite(value)
                for value in (
                    self.global_position.latitude_deg,
                    self.global_position.longitude_deg,
                    self.global_position.altitude_m,
                )
            )
        ):
            x_m, y_m, _ = self.field_frame.to_enu(
                self.global_position.latitude_deg,
                self.global_position.longitude_deg,
                self.global_position.altitude_m,
            )
            origin_id = self.field_frame.origin_id
            health |= HealthFlag.POSITION_VALID | HealthFlag.ORIGIN_VALID
        yaw = quaternion_yaw(
            self.pose.qx, self.pose.qy, self.pose.qz, self.pose.qw
        )
        if pose_fresh and yaw is not None:
            yaw_rad = yaw
            health |= HealthFlag.YAW_VALID

        if (
            _fresh(self.velocity.received_mono, now_mono, self.sample_timeout_sec)
            and math.isfinite(self.velocity.vx_mps)
            and math.isfinite(self.velocity.vy_mps)
        ):
            vx_mps, vy_mps = self.velocity.vx_mps, self.velocity.vy_mps
            health |= HealthFlag.VELOCITY_VALID

        if (
            _fresh(
                self.angular_rate.received_mono,
                now_mono,
                self.sample_timeout_sec,
            )
            and math.isfinite(self.angular_rate.omega_radps)
        ):
            omega_radps = self.angular_rate.omega_radps

        return MiniState(
            vehicle_id=vehicle_id,
            seq=seq,
            timestamp_ms=timestamp_ms,
            x_m=x_m,
            y_m=y_m,
            vx_mps=vx_mps,
            vy_mps=vy_mps,
            yaw_rad=yaw_rad,
            omega_radps=omega_radps,
            health=int(health),
            origin_id=origin_id,
        )

    def set_field_origin(self, origin: FieldOrigin) -> None:
        self.field_frame = FieldFrame(
            origin_id=origin.origin_id,
            latitude_deg=origin.latitude_deg,
            longitude_deg=origin.longitude_deg,
            altitude_m=origin.altitude_m,
        )

    def field_origin_candidate(
        self,
        *,
        origin_id: int,
        seq: int,
        timestamp_ms: int,
        now_mono: float,
    ) -> FieldOrigin | None:
        sample = self.global_position
        if not (
            _fresh(sample.received_mono, now_mono, self.sample_timeout_sec)
            and sample.status >= 0
            and all(
                math.isfinite(value)
                for value in (
                    sample.latitude_deg,
                    sample.longitude_deg,
                    sample.altitude_m,
                )
            )
        ):
            return None
        return FieldOrigin(
            origin_id=origin_id,
            seq=seq,
            timestamp_ms=timestamp_ms,
            latitude_deg=sample.latitude_deg,
            longitude_deg=sample.longitude_deg,
            altitude_m=sample.altitude_m,
        )

    def shared_field_ready(self, expected_origin_id: int, now_mono: float) -> bool:
        return bool(
            self.field_frame is not None
            and self.field_frame.origin_id == expected_origin_id
            and self.build(
                vehicle_id=0,
                seq=0,
                timestamp_ms=0,
                now_mono=now_mono,
            ).health
            & int(HealthFlag.ORIGIN_VALID | HealthFlag.POSITION_VALID)
            == int(HealthFlag.ORIGIN_VALID | HealthFlag.POSITION_VALID)
        )

    def safe_execution_prestate(self, now_mono: float) -> bool:
        return bool(
            _fresh(
                self.state.received_mono,
                now_mono,
                self.sample_timeout_sec,
            )
            and self.state.connected
            and not self.state.armed
            and self.state.mode.upper() == "MANUAL"
            and self.state.manual_input
        )

    def execution_session_ready(self, now_mono: float) -> bool:
        if not (
            _fresh(
                self.state.received_mono,
                now_mono,
                self.sample_timeout_sec,
            )
            and self.state.connected
        ):
            return False
        mode = self.state.mode.upper()
        if mode == "MANUAL":
            return bool(not self.state.armed and self.state.manual_input)
        # MAVROS reports manual_input=False while PX4 is in OFFBOARD. RC/Kill
        # availability is checked before entry; this flag is not an OFFBOARD
        # link-health signal.
        return mode == "OFFBOARD"

    def status_text(self, now_mono: float) -> str:
        def age(sample_mono: float | None) -> str:
            if sample_mono is None:
                return "missing"
            return f"{max(0.0, now_mono - sample_mono):.3f}s"

        return (
            f"connected={self.state.connected} armed={self.state.armed} "
            f"mode={self.state.mode or 'UNKNOWN'} manual_input={self.state.manual_input} "
            f"ages(state={age(self.state.received_mono)},"
            f"pose={age(self.pose.received_mono)},"
            f"velocity={age(self.velocity.received_mono)},"
            f"imu={age(self.angular_rate.received_mono)},"
            f"gps={age(self.global_position.received_mono)}) "
            f"origin={self.field_frame.origin_id if self.field_frame else 0}"
        )


class MavrosMiniStateSource:
    """ROS adapter imported only when explicit live telemetry mode is selected."""

    def __init__(
        self,
        namespace: str = "/mavros",
        sample_timeout_sec: float = DEFAULT_SAMPLE_TIMEOUT_SEC,
    ) -> None:
        import rclpy
        from geometry_msgs.msg import PoseStamped, TwistStamped
        from mavros_msgs.msg import State
        from rclpy.qos import (
            DurabilityPolicy,
            QoSProfile,
            ReliabilityPolicy,
            qos_profile_sensor_data,
        )
        from sensor_msgs.msg import Imu, NavSatFix

        self._rclpy = rclpy
        self._owns_context = not rclpy.ok()
        if self._owns_context:
            rclpy.init(args=None)
        self.accumulator = MiniStateAccumulator(sample_timeout_sec)
        self.node = rclpy.create_node("pairb_live_mini_state_source")
        prefix = "/" + namespace.strip("/")
        qos = qos_profile_sensor_data
        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.node.create_subscription(
            State,
            f"{prefix}/state",
            self._state_cb,
            state_qos,
        )
        self.node.create_subscription(
            PoseStamped,
            f"{prefix}/local_position/pose",
            self._pose_cb,
            qos,
        )
        self.node.create_subscription(
            TwistStamped,
            f"{prefix}/local_position/velocity_local",
            self._velocity_cb,
            qos,
        )
        self.node.create_subscription(Imu, f"{prefix}/imu/data", self._imu_cb, qos)
        self.node.create_subscription(
            NavSatFix,
            f"{prefix}/global_position/global",
            self._global_position_cb,
            qos,
        )

    def _state_cb(self, message: object) -> None:
        self.accumulator.state = StateSample(
            connected=bool(message.connected),
            armed=bool(message.armed),
            mode=str(message.mode),
            manual_input=bool(message.manual_input),
            received_mono=time.monotonic(),
        )

    def _pose_cb(self, message: object) -> None:
        pose = message.pose
        self.accumulator.pose = PoseSample(
            x_m=float(pose.position.x),
            y_m=float(pose.position.y),
            qx=float(pose.orientation.x),
            qy=float(pose.orientation.y),
            qz=float(pose.orientation.z),
            qw=float(pose.orientation.w),
            received_mono=time.monotonic(),
        )

    def _velocity_cb(self, message: object) -> None:
        self.accumulator.velocity = VelocitySample(
            vx_mps=float(message.twist.linear.x),
            vy_mps=float(message.twist.linear.y),
            received_mono=time.monotonic(),
        )

    def _imu_cb(self, message: object) -> None:
        self.accumulator.angular_rate = AngularRateSample(
            omega_radps=float(message.angular_velocity.z),
            received_mono=time.monotonic(),
        )

    def _global_position_cb(self, message: object) -> None:
        self.accumulator.global_position = GlobalPositionSample(
            latitude_deg=float(message.latitude),
            longitude_deg=float(message.longitude),
            altitude_m=float(message.altitude),
            status=int(message.status.status),
            received_mono=time.monotonic(),
        )

    def spin_once(self, timeout_sec: float = 0.0) -> None:
        self._rclpy.spin_once(self.node, timeout_sec=timeout_sec)

    def build(self, vehicle_id: int, seq: int, timestamp_ms: int) -> MiniState:
        return self.accumulator.build(
            vehicle_id=vehicle_id,
            seq=seq,
            timestamp_ms=timestamp_ms,
            now_mono=time.monotonic(),
        )

    def safe_execution_prestate(self) -> bool:
        return self.accumulator.safe_execution_prestate(time.monotonic())

    def execution_session_ready(self) -> bool:
        return self.accumulator.execution_session_ready(time.monotonic())

    def set_field_origin(self, origin: FieldOrigin) -> None:
        self.accumulator.set_field_origin(origin)

    def field_origin_candidate(
        self,
        *,
        origin_id: int,
        seq: int,
        timestamp_ms: int,
    ) -> FieldOrigin | None:
        return self.accumulator.field_origin_candidate(
            origin_id=origin_id,
            seq=seq,
            timestamp_ms=timestamp_ms,
            now_mono=time.monotonic(),
        )

    def shared_field_ready(self, expected_origin_id: int) -> bool:
        return self.accumulator.shared_field_ready(
            expected_origin_id,
            time.monotonic(),
        )

    def status_text(self) -> str:
        return self.accumulator.status_text(time.monotonic())

    def close(self) -> None:
        self.node.destroy_node()
        if self._owns_context and self._rclpy.ok():
            self._rclpy.shutdown()
