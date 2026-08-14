#!/usr/bin/env python3

"""Read-only Carrier HIL plus virtual-Mini cooperative mission RViz replay.

The ROS side subscribes only to MAVROS State and local pose. It publishes only
RViz Path/Pose topics and never creates an arming, mode, or setpoint interface.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import socket
import sys
import time
from dataclasses import dataclass
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from src.rover_rviz_trajectory import TracePoint  # noqa: E402
from src.lr24_compact_protocol import (  # noqa: E402
    FrameReader,
    HealthFlag,
    MessageType,
    MiniState,
    sequence_is_newer,
)
from src.lr24_mavlink_tunnel import make_transport  # noqa: E402
from scripts.run_pairb_hitl_state_relay import decode_packet  # noqa: E402


PAIRB_HITL_REQUIRED_HEALTH = int(
    HealthFlag.POSITION_VALID
    | HealthFlag.YAW_VALID
    | HealthFlag.PX4_CONNECTED
    | HealthFlag.DISARMED
)


@dataclass(frozen=True)
class ReplayRow:
    time_s: float
    mission_phase: str
    coordination_mode: str
    carrier_x_m: float
    carrier_y_m: float
    mini_x_m: float
    mini_y_m: float
    carrier_speed_mps: float
    mini_speed_mps: float
    carrier_speed_limit_mps: float
    mini_speed_limit_mps: float
    rendezvous_speed_mps: float
    relative_speed_mps: float
    terminal_capture_duration_s: float
    terminal_capture_qualified: bool
    front_gap_m: float
    reason: str


@dataclass(frozen=True)
class ReplayBundle:
    rows: tuple[ReplayRow, ...]
    carrier_plan: tuple[TracePoint, ...]
    mini_plan: tuple[TracePoint, ...]
    plan_time_s: float
    duration_s: float
    carrier_speed_range_mps: tuple[float, float]
    mini_speed_range_mps: tuple[float, float]
    terminal_speed_range_mps: tuple[float, float]
    rendezvous_speed_mps: float
    terminal_capture_required_s: float


def oriented_trace(coordinates: list[tuple[float, float]]) -> tuple[TracePoint, ...]:
    result = []
    yaw = 0.0
    for index, coordinate in enumerate(coordinates):
        if index + 1 < len(coordinates):
            following = coordinates[index + 1]
            yaw = math.atan2(following[1] - coordinate[1], following[0] - coordinate[0])
        result.append(TracePoint(coordinate[0], coordinate[1], yaw))
    return tuple(result)


def load_replay_bundle(replay_dir: Path) -> ReplayBundle:
    timeline_path = replay_dir / "timeline.csv"
    plan_path = replay_dir / "plan.json"
    if not timeline_path.is_file() or not plan_path.is_file():
        raise FileNotFoundError(f"replay needs timeline.csv and plan.json: {replay_dir}")
    with timeline_path.open(newline="", encoding="ascii") as stream:
        rows = tuple(
            ReplayRow(
                time_s=float(row["time_s"]),
                mission_phase=row["mission_phase"],
                coordination_mode=row["coordination_mode"],
                carrier_x_m=float(row["carrier_x_m"]),
                carrier_y_m=float(row["carrier_y_m"]),
                mini_x_m=float(row["mini_x_m"]),
                mini_y_m=float(row["mini_y_m"]),
                carrier_speed_mps=float(row["carrier_speed_mps"]),
                mini_speed_mps=float(row["mini_speed_mps"]),
                carrier_speed_limit_mps=float(row["carrier_speed_limit_mps"]),
                mini_speed_limit_mps=float(
                    row.get("mini_speed_limit_mps", row["mini_speed_mps"])
                ),
                rendezvous_speed_mps=float(row.get("rendezvous_speed_mps", "nan")),
                relative_speed_mps=float(
                    row.get(
                        "relative_speed_mps",
                        str(
                            float(row["carrier_speed_mps"])
                            - float(row["mini_speed_mps"])
                        ),
                    )
                ),
                terminal_capture_duration_s=float(
                    row.get("terminal_capture_duration_s", "0")
                ),
                terminal_capture_qualified=(
                    row.get("terminal_capture_qualified", "0").strip().lower()
                    in {"1", "true", "yes"}
                ),
                front_gap_m=float(row["front_gap_m"]),
                reason=row["reason"],
            )
            for row in csv.DictReader(stream)
        )
    if len(rows) < 2 or rows[0].mission_phase != "ORBIT_QUALIFICATION":
        raise ValueError("replay does not start with Mini orbit qualification")
    first_plan_index = next(
        (index for index, row in enumerate(rows) if row.mission_phase != "ORBIT_QUALIFICATION"),
        None,
    )
    if first_plan_index is None or first_plan_index < 2:
        raise ValueError("replay has no post-orbit cooperative phase")
    plan = json.loads(plan_path.read_text(encoding="ascii"))
    carrier_speed_range = tuple(
        float(value) for value in plan.get("carrier_speed_range_mps", (0.0, 0.16))
    )
    mini_speed_range = tuple(
        float(value) for value in plan.get("mini_speed_range_mps", (0.12, 0.20))
    )
    terminal_speed_range = tuple(
        float(value) for value in plan.get("terminal_speed_range_mps", (0.12, 0.16))
    )
    if not all(
        len(speed_range) == 2
        for speed_range in (
            carrier_speed_range,
            mini_speed_range,
            terminal_speed_range,
        )
    ):
        raise ValueError("replay speed envelopes must contain [min, max]")
    rendezvous_speed = float(
        plan.get(
            "rendezvous_speed_mps",
            0.5 * (terminal_speed_range[0] + terminal_speed_range[1]),
        )
    )
    carrier_coordinates = [
        (float(point[0]), float(point[1])) for point in plan["carrier_path"]
    ]
    terminal_coordinates = [
        (float(point[0]), float(point[1])) for point in plan["mini_terminal_path"]
    ]
    center = tuple(float(value) for value in plan["orbit_center"])
    radius = float(plan["orbit_radius_m"])
    direction = str(plan["turn_direction"])
    sign = 1.0 if direction == "ccw" else -1.0
    initial_phase = math.atan2(
        rows[0].mini_y_m - center[1], rows[0].mini_x_m - center[0]
    )
    qualification_orbit = [
        (
            center[0] + radius * math.cos(initial_phase + sign * 2.0 * math.pi * index / 240),
            center[1] + radius * math.sin(initial_phase + sign * 2.0 * math.pi * index / 240),
        )
        for index in range(241)
    ]
    plan_phase = float(plan["mini_phase_at_plan_rad"])
    exit_delta = float(plan["mini_exit_delta_rad"])
    exit_count = max(2, math.ceil(exit_delta * radius / 0.08))
    exit_orbit = [
        (
            center[0] + radius * math.cos(plan_phase + sign * exit_delta * index / exit_count),
            center[1] + radius * math.sin(plan_phase + sign * exit_delta * index / exit_count),
        )
        for index in range(exit_count + 1)
    ]
    mini_coordinates = qualification_orbit + exit_orbit + terminal_coordinates
    return ReplayBundle(
        rows=rows,
        carrier_plan=oriented_trace(carrier_coordinates),
        mini_plan=oriented_trace(mini_coordinates),
        plan_time_s=rows[first_plan_index].time_s,
        duration_s=rows[-1].time_s,
        carrier_speed_range_mps=carrier_speed_range,
        mini_speed_range_mps=mini_speed_range,
        terminal_speed_range_mps=terminal_speed_range,
        rendezvous_speed_mps=rendezvous_speed,
        terminal_capture_required_s=float(
            plan.get("terminal_capture_required_s", 2.0)
        ),
    )


def transform_point(point: TracePoint, origin: TracePoint) -> TracePoint:
    cosine = math.cos(origin.yaw_rad)
    sine = math.sin(origin.yaw_rad)
    return TracePoint(
        origin.x_m + cosine * point.x_m - sine * point.y_m,
        origin.y_m + sine * point.x_m + cosine * point.y_m,
        math.atan2(
            math.sin(origin.yaw_rad + point.yaw_rad),
            math.cos(origin.yaw_rad + point.yaw_rad),
        ),
    )


def align_relative_trace(
    current: TracePoint,
    sensor_anchor: TracePoint,
    display_anchor: TracePoint,
) -> TracePoint:
    """Map one local sensor frame into its assigned HIL display start pose."""

    dx = current.x_m - sensor_anchor.x_m
    dy = current.y_m - sensor_anchor.y_m
    rotation = display_anchor.yaw_rad - sensor_anchor.yaw_rad
    cosine = math.cos(rotation)
    sine = math.sin(rotation)
    return TracePoint(
        display_anchor.x_m + cosine * dx - sine * dy,
        display_anchor.y_m + sine * dx + cosine * dy,
        math.atan2(
            math.sin(current.yaw_rad + rotation),
            math.cos(current.yaw_rad + rotation),
        ),
    )


def mavros_topic(namespace: str, suffix: str) -> str:
    prefix = "/" + namespace.strip("/")
    if prefix == "/" or not suffix.startswith("/"):
        raise ValueError("MAVROS namespace and topic suffix are invalid")
    return prefix + suffix


def pairb_mini_state_safe(state: MiniState | None) -> tuple[bool, str]:
    if state is None:
        return False, "mini_pairb_state_missing"
    missing = PAIRB_HITL_REQUIRED_HEALTH & ~int(state.health)
    if missing:
        return False, f"mini_pairb_health_missing:0x{missing:04x}"
    return True, "ready"


def yaw_from_quaternion(quaternion) -> float:
    siny = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cosy = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    return math.atan2(siny, cosy)


def replay_trace_point(
    rows: tuple[ReplayRow, ...], index: int, *, carrier: bool
) -> TracePoint:
    row = rows[index]
    x_m = row.carrier_x_m if carrier else row.mini_x_m
    y_m = row.carrier_y_m if carrier else row.mini_y_m
    neighbor_index = index + 1 if index + 1 < len(rows) else index - 1
    neighbor = rows[neighbor_index]
    neighbor_x = neighbor.carrier_x_m if carrier else neighbor.mini_x_m
    neighbor_y = neighbor.carrier_y_m if carrier else neighbor.mini_y_m
    if neighbor_index > index:
        yaw = math.atan2(neighbor_y - y_m, neighbor_x - x_m)
    else:
        yaw = math.atan2(y_m - neighbor_y, x_m - neighbor_x)
    return TracePoint(x_m, y_m, yaw)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Read-only real Carrier state plus virtual Mini cooperative RViz replay."
    )
    parser.add_argument("--replay-dir", required=True)
    parser.add_argument("--time-scale", type=float, default=4.0)
    parser.add_argument("--state-timeout-sec", type=float, default=2.0)
    parser.add_argument("--carrier-namespace", default="/mavros")
    parser.add_argument("--mini-namespace", default="/mini/mavros")
    parser.add_argument(
        "--mini-relay-port",
        type=int,
        default=0,
        help="HITL-only UDP port for Mini state when cross-host ROS topics are unavailable.",
    )
    parser.add_argument(
        "--mini-relay-host",
        default="",
        help="Expected Mini relay source IP; empty accepts any source on the bound port.",
    )
    parser.add_argument(
        "--require-real-mini",
        action="store_true",
        help="Gate replay on a second real, disarmed Mini MAVROS state and pose.",
    )
    parser.add_argument(
        "--mini-pairb-port",
        default="",
        help="Carrier-side Pair B Ground CP2102 used for real MiniState HIL input.",
    )
    parser.add_argument("--mini-pairb-baud", type=int, default=115200)
    parser.add_argument(
        "--allowed-disarmed-mode",
        action="append",
        default=["MANUAL"],
        help="Additional observed mode accepted only while the vehicle remains disarmed.",
    )
    parser.add_argument("--artifact-dir", default="results/pairb_virtual_mini_hil")
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not math.isfinite(args.time_scale) or not 0.1 <= args.time_scale <= 10.0:
        raise SystemExit("--time-scale must be within [0.1, 10]")
    bundle = load_replay_bundle(Path(args.replay_dir))
    mode_label = (
        "PAIRB-DUAL-PIXHAWK"
        if args.require_real_mini and args.mini_pairb_port
        else "DUAL-PIXHAWK"
        if args.require_real_mini
        else "VIRTUAL_MINI"
    )
    print(f"READ-ONLY HIL: {mode_label} health gates + shadow cooperative mission")
    print("NO ARM / NO OFFBOARD / NO SETPOINT / NO ACTUATOR")
    print(
        f"replay rows={len(bundle.rows)} plan_after={bundle.plan_time_s:.2f}s "
        f"duration={bundle.duration_s:.2f}s time_scale={args.time_scale:.2f}"
    )
    print(
        f"speed_envelopes Carrier={bundle.carrier_speed_range_mps} "
        f"Mini={bundle.mini_speed_range_mps} "
        f"terminal={bundle.terminal_speed_range_mps} "
        f"rendezvous={bundle.rendezvous_speed_mps:.3f}m/s "
        f"capture={bundle.terminal_capture_required_s:.1f}s"
    )
    print(f"allowed_disarmed_modes={sorted(set(args.allowed_disarmed_mode))}")
    print(
        f"carrier_namespace={args.carrier_namespace} "
        f"mini_source="
        f"{'pairb:' + args.mini_pairb_port if args.mini_pairb_port else 'udp:' + str(args.mini_relay_port) if args.mini_relay_port else args.mini_namespace}"
    )
    if args.dry_run:
        return 0

    import rclpy
    from geometry_msgs.msg import PoseStamped
    from mavros_msgs.msg import State
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from visualization_msgs.msg import Marker

    from src.rover_rviz_trajectory import RvizTrajectoryPublisher

    class VirtualMiniHilNode(Node):
        def __init__(self) -> None:
            super().__init__("pairb_virtual_mini_read_only_hil")
            self.bundle = bundle
            self.carrier_state = None
            self.carrier_state_rx = None
            self.carrier_pose = None
            self.carrier_pose_rx = None
            self.mini_state = None
            self.mini_state_rx = None
            self.mini_pose = None
            self.mini_pose_rx = None
            self.mini_relay_socket = None
            self.mini_relay_last_seq = None
            self.mini_pairb_transport = None
            self.mini_pairb_reader = FrameReader()
            self.mini_pairb_state = None
            self.mini_pairb_state_rx = None
            self.mini_pairb_last_seq = None
            self.anchor = None
            self.mini_sensor_anchor = None
            self.mini_display_anchor = None
            self.replay_started = None
            self.index = 0
            self.stopped = False
            self.real_carrier = RvizTrajectoryPublisher(self, "/pairb/real_carrier")
            self.real_mini = RvizTrajectoryPublisher(self, "/pairb/real_mini")
            self.shadow_carrier = RvizTrajectoryPublisher(self, "/pairb/shadow_carrier")
            self.virtual_mini = RvizTrajectoryPublisher(self, "/pairb/mini")
            self.status_publisher = self.create_publisher(
                Marker, "/pairb/cooperation_status", 1
            )
            self.create_subscription(
                State,
                mavros_topic(args.carrier_namespace, "/state"),
                self.on_carrier_state,
                10,
            )
            self.create_subscription(
                PoseStamped,
                mavros_topic(args.carrier_namespace, "/local_position/pose"),
                self.on_carrier_pose,
                qos_profile_sensor_data,
            )
            if args.require_real_mini and args.mini_pairb_port:
                self.mini_pairb_transport = make_transport(
                    "mavlink-serial",
                    port=args.mini_pairb_port,
                    baud=args.mini_pairb_baud,
                    source_system=2,
                    target_system=1,
                    expected_source_system=1,
                )
            elif args.require_real_mini and args.mini_relay_port:
                if not 1024 <= args.mini_relay_port <= 65535:
                    raise ValueError("Mini relay port must be within [1024, 65535]")
                self.mini_relay_socket = socket.socket(
                    socket.AF_INET, socket.SOCK_DGRAM
                )
                self.mini_relay_socket.setblocking(False)
                self.mini_relay_socket.bind(("0.0.0.0", args.mini_relay_port))
            elif args.require_real_mini:
                self.create_subscription(
                    State,
                    mavros_topic(args.mini_namespace, "/state"),
                    self.on_mini_state,
                    10,
                )
                self.create_subscription(
                    PoseStamped,
                    mavros_topic(args.mini_namespace, "/local_position/pose"),
                    self.on_mini_pose,
                    qos_profile_sensor_data,
                )
            self.timer = self.create_timer(0.05, self.tick)
            self.get_logger().info(
                "waiting for fresh connected+disarmed Carrier state in an allowed mode; publishers are RViz-only"
            )

        def on_carrier_state(self, message) -> None:
            self.carrier_state = message
            self.carrier_state_rx = time.monotonic()

        def on_carrier_pose(self, message) -> None:
            position = message.pose.position
            self.carrier_pose = TracePoint(
                float(position.x),
                float(position.y),
                yaw_from_quaternion(message.pose.orientation),
            )
            self.carrier_pose_rx = time.monotonic()

        def on_mini_state(self, message) -> None:
            self.mini_state = message
            self.mini_state_rx = time.monotonic()

        def on_mini_pose(self, message) -> None:
            position = message.pose.position
            self.mini_pose = TracePoint(
                float(position.x),
                float(position.y),
                yaw_from_quaternion(message.pose.orientation),
            )
            self.mini_pose_rx = time.monotonic()

        def drain_mini_relay(self, now: float) -> None:
            if self.mini_relay_socket is None:
                return
            while True:
                try:
                    payload, source = self.mini_relay_socket.recvfrom(2048)
                except BlockingIOError:
                    return
                if args.mini_relay_host and source[0] != args.mini_relay_host:
                    continue
                try:
                    packet = decode_packet(payload)
                except ValueError as exc:
                    self.get_logger().warning(f"rejected Mini HITL relay packet: {exc}")
                    continue
                if (
                    self.mini_relay_last_seq is not None
                    and packet.seq <= self.mini_relay_last_seq
                ):
                    continue
                self.mini_relay_last_seq = packet.seq
                self.mini_state = packet
                self.mini_state_rx = now
                self.mini_pose = TracePoint(
                    packet.x_m, packet.y_m, packet.yaw_rad
                )
                self.mini_pose_rx = now

        def drain_mini_pairb(self, now: float) -> None:
            if self.mini_pairb_transport is None:
                return
            for payload in self.mini_pairb_transport.receive(0.0):
                for frame in self.mini_pairb_reader.feed(payload):
                    if frame.msg_type != MessageType.MINI_STATE:
                        continue
                    try:
                        state = MiniState.decode(frame.payload)
                    except ValueError as exc:
                        self.get_logger().warning(f"rejected Pair B MiniState: {exc}")
                        continue
                    if self.mini_pairb_last_seq is not None and not sequence_is_newer(
                        state.seq, self.mini_pairb_last_seq
                    ):
                        continue
                    self.mini_pairb_last_seq = state.seq
                    self.mini_pairb_state = state
                    self.mini_pairb_state_rx = now
                    self.mini_pose = TracePoint(state.x_m, state.y_m, state.yaw_rad)
                    self.mini_pose_rx = now

        def vehicle_safe(self, role: str, state, state_rx, pose, pose_rx, now: float) -> tuple[bool, str]:
            if state is None or pose is None:
                return False, f"{role}_state_or_pose_missing"
            if state_rx is None or pose_rx is None:
                return False, f"{role}_timestamp_missing"
            if max(now - state_rx, now - pose_rx) > args.state_timeout_sec:
                return False, f"{role}_state_or_pose_stale"
            if not state.connected:
                return False, f"{role}_mavros_disconnected"
            if state.armed:
                return False, f"{role}_vehicle_armed"
            if state.mode not in set(args.allowed_disarmed_mode):
                return False, f"{role}_disarmed_mode_not_allowed:{state.mode}"
            return True, "ready"

        def safe_prestate(self, now: float) -> tuple[bool, str]:
            carrier_safe = self.vehicle_safe(
                "carrier",
                self.carrier_state,
                self.carrier_state_rx,
                self.carrier_pose,
                self.carrier_pose_rx,
                now,
            )
            if not carrier_safe[0]:
                return carrier_safe
            if args.require_real_mini:
                if self.mini_pairb_transport is not None:
                    if (
                        self.mini_pairb_state_rx is None
                        or now - self.mini_pairb_state_rx > args.state_timeout_sec
                    ):
                        return False, "mini_pairb_state_stale"
                    return pairb_mini_state_safe(self.mini_pairb_state)
                return self.vehicle_safe(
                    "mini",
                    self.mini_state,
                    self.mini_state_rx,
                    self.mini_pose,
                    self.mini_pose_rx,
                    now,
                )
            return True, "ready"

        def initialize_replay(self, now: float) -> None:
            self.anchor = self.carrier_pose
            carrier_plan = tuple(
                transform_point(point, self.anchor) for point in self.bundle.carrier_plan
            )
            mini_plan = tuple(
                transform_point(point, self.anchor) for point in self.bundle.mini_plan
            )
            self.shadow_carrier.set_plan(carrier_plan, "map")
            self.virtual_mini.set_plan(mini_plan, "map")
            self.real_carrier.start_actual("map")
            if args.require_real_mini:
                self.mini_sensor_anchor = self.mini_pose
                self.mini_display_anchor = mini_plan[0]
                self.real_mini.start_actual("map")
            self.shadow_carrier.start_actual("map")
            self.virtual_mini.start_actual("map")
            self.replay_started = now
            self.get_logger().info(
                "HIL_READY dual_real_health="
                f"{args.require_real_mini} motion=shadow_only plan=cooperative"
            )

        def stop(self, reason: str) -> None:
            if self.stopped:
                return
            self.stopped = True
            self.get_logger().error(f"HIL_STOP reason={reason}; no vehicle command was sent")

        def publish_status(self, row: ReplayRow) -> None:
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "pairb_cooperation"
            marker.id = 1
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            assert self.anchor is not None
            marker.pose.position.x = self.anchor.x_m
            marker.pose.position.y = self.anchor.y_m
            marker.pose.position.z = 1.0
            marker.scale.z = 0.35
            marker.color.r = 0.95
            marker.color.g = 0.95
            marker.color.b = 0.95
            marker.color.a = 1.0
            gap_text = (
                "n/a" if not math.isfinite(row.front_gap_m) else f"{row.front_gap_m:.2f}m"
            )
            marker.text = (
                f"{mode_label} HIL | {row.mission_phase} | {row.coordination_mode}\n"
                f"Carrier [{self.bundle.carrier_speed_range_mps[0]:.2f},"
                f"{self.bundle.carrier_speed_range_mps[1]:.2f}] "
                f"actual/cmd {row.carrier_speed_mps:.2f}/{row.carrier_speed_limit_mps:.2f}\n"
                f"Mini [{self.bundle.mini_speed_range_mps[0]:.2f},"
                f"{self.bundle.mini_speed_range_mps[1]:.2f}] "
                f"actual/cmd {row.mini_speed_mps:.2f}/{row.mini_speed_limit_mps:.2f}\n"
                f"terminal [{self.bundle.terminal_speed_range_mps[0]:.2f},"
                f"{self.bundle.terminal_speed_range_mps[1]:.2f}] "
                f"v*={self.bundle.rendezvous_speed_mps:.2f} "
                f"dv={row.relative_speed_mps:+.3f} gap={gap_text}\n"
                f"capture {row.terminal_capture_duration_s:.1f}/"
                f"{self.bundle.terminal_capture_required_s:.1f}s "
                f"{'PASS' if row.terminal_capture_qualified else 'PENDING'}\n"
                f"{row.reason} | "
                f"{'REAL CARRIER+MINI DISARMED' if args.require_real_mini else 'REAL CARRIER DISARMED'}"
            )
            self.status_publisher.publish(marker)

        def tick(self) -> None:
            now = time.monotonic()
            self.drain_mini_relay(now)
            self.drain_mini_pairb(now)
            safe, reason = self.safe_prestate(now)
            if self.replay_started is None:
                if safe:
                    self.initialize_replay(now)
                return
            if not safe:
                self.stop(reason)
                return
            if self.stopped:
                return
            self.real_carrier.update_actual(
                self.carrier_pose.x_m,
                self.carrier_pose.y_m,
                self.carrier_pose.yaw_rad,
            )
            if args.require_real_mini:
                assert self.mini_sensor_anchor is not None
                assert self.mini_display_anchor is not None
                aligned_mini = align_relative_trace(
                    self.mini_pose,
                    self.mini_sensor_anchor,
                    self.mini_display_anchor,
                )
                self.real_mini.update_actual(
                    aligned_mini.x_m,
                    aligned_mini.y_m,
                    aligned_mini.yaw_rad,
                )
            replay_time = (now - self.replay_started) * args.time_scale
            while (
                self.index + 1 < len(self.bundle.rows)
                and self.bundle.rows[self.index + 1].time_s <= replay_time
            ):
                self.index += 1
            row = self.bundle.rows[self.index]
            assert self.anchor is not None
            shadow = transform_point(
                replay_trace_point(self.bundle.rows, self.index, carrier=True),
                self.anchor,
            )
            mini = transform_point(
                replay_trace_point(self.bundle.rows, self.index, carrier=False),
                self.anchor,
            )
            self.shadow_carrier.update_actual(shadow.x_m, shadow.y_m, shadow.yaw_rad)
            self.virtual_mini.update_actual(mini.x_m, mini.y_m, mini.yaw_rad)
            self.publish_status(row)
            if self.index + 1 >= len(self.bundle.rows):
                self.get_logger().info("HIL_REPLAY_COMPLETE; vehicle remained disarmed")
                self.stopped = True

        def destroy_node(self) -> bool:
            if self.mini_relay_socket is not None:
                self.mini_relay_socket.close()
            if self.mini_pairb_transport is not None:
                self.mini_pairb_transport.close()
            return super().destroy_node()

    rclpy.init()
    node = VirtualMiniHilNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
