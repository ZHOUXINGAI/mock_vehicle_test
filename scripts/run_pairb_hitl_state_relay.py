#!/usr/bin/env python3

"""Send one vehicle's read-only MAVROS state/pose to the HITL display host.

This is a development-only unidirectional UDP telemetry relay. It has no
receiver, service client, setpoint publisher, serial access, or actuator path.
Production two-vehicle coordination remains on Pair B.
"""

from __future__ import annotations

import argparse
import ipaddress
import json
import math
import socket
import time
from dataclasses import asdict, dataclass


SCHEMA_VERSION = 1
MAX_PACKET_BYTES = 1024


@dataclass(frozen=True)
class HitlStatePacket:
    schema: int
    role: str
    seq: int
    connected: bool
    armed: bool
    mode: str
    manual_input: bool
    x_m: float
    y_m: float
    z_m: float
    qx: float
    qy: float
    qz: float
    qw: float

    @property
    def yaw_rad(self) -> float:
        siny = 2.0 * (self.qw * self.qz + self.qx * self.qy)
        cosy = 1.0 - 2.0 * (self.qy * self.qy + self.qz * self.qz)
        return math.atan2(siny, cosy)


def encode_packet(packet: HitlStatePacket) -> bytes:
    validate_packet(packet)
    payload = json.dumps(
        asdict(packet), separators=(",", ":"), sort_keys=True
    ).encode("ascii")
    if len(payload) > MAX_PACKET_BYTES:
        raise ValueError("HITL relay packet exceeds size limit")
    return payload


def decode_packet(payload: bytes) -> HitlStatePacket:
    if not payload or len(payload) > MAX_PACKET_BYTES:
        raise ValueError("HITL relay packet size is invalid")
    try:
        raw = json.loads(payload.decode("ascii"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ValueError("HITL relay packet is not valid ASCII JSON") from exc
    if not isinstance(raw, dict) or set(raw) != set(HitlStatePacket.__dataclass_fields__):
        raise ValueError("HITL relay packet fields are invalid")
    packet = HitlStatePacket(**raw)
    validate_packet(packet)
    return packet


def validate_packet(packet: HitlStatePacket) -> None:
    if packet.schema != SCHEMA_VERSION or packet.role != "mini":
        raise ValueError("HITL relay schema or role is invalid")
    if not isinstance(packet.seq, int) or not 0 <= packet.seq <= 0xFFFFFFFF:
        raise ValueError("HITL relay sequence is invalid")
    if not packet.mode or len(packet.mode) > 32:
        raise ValueError("HITL relay mode is invalid")
    values = (
        packet.x_m,
        packet.y_m,
        packet.z_m,
        packet.qx,
        packet.qy,
        packet.qz,
        packet.qw,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("HITL relay pose must be finite")
    quaternion_norm = math.sqrt(
        packet.qx**2 + packet.qy**2 + packet.qz**2 + packet.qw**2
    )
    if not 0.8 <= quaternion_norm <= 1.2:
        raise ValueError("HITL relay quaternion norm is invalid")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--namespace", default="/mini/mavros")
    parser.add_argument("--destination-host", required=True)
    parser.add_argument("--destination-port", type=int, default=15120)
    parser.add_argument("--rate-hz", type=float, default=10.0)
    parser.add_argument("--input-timeout-sec", type=float, default=2.0)
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def topic(namespace: str, suffix: str) -> str:
    prefix = "/" + namespace.strip("/")
    if prefix == "/" or not suffix.startswith("/"):
        raise ValueError("MAVROS namespace is invalid")
    return prefix + suffix


def main() -> int:
    args = parse_args()
    destination = ipaddress.ip_address(args.destination_host)
    if destination.is_multicast or destination.is_unspecified:
        raise SystemExit("destination must be a unicast IP address")
    if not 1024 <= args.destination_port <= 65535:
        raise SystemExit("destination port must be within [1024, 65535]")
    if not math.isfinite(args.rate_hz) or not 1.0 <= args.rate_hz <= 20.0:
        raise SystemExit("rate must be within [1, 20] Hz")
    if not math.isfinite(args.input_timeout_sec) or not 0.5 <= args.input_timeout_sec <= 5.0:
        raise SystemExit("input timeout must be within [0.5, 5] seconds")
    print("HITL-ONLY MINI STATE RELAY: SEND-ONLY / NO VEHICLE COMMANDS")
    print(
        f"source={args.namespace} destination={destination}:{args.destination_port} "
        f"rate={args.rate_hz:.1f}Hz"
    )
    if args.dry_run:
        return 0

    import rclpy
    from geometry_msgs.msg import PoseStamped
    from mavros_msgs.msg import State
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data

    class RelayNode(Node):
        def __init__(self) -> None:
            super().__init__("pairb_hitl_mini_state_send_only_relay")
            self.state = None
            self.state_rx_s = None
            self.pose = None
            self.pose_rx_s = None
            self.seq = 0
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.destination = (str(destination), args.destination_port)
            self.create_subscription(
                State, topic(args.namespace, "/state"), self.on_state, 10
            )
            self.create_subscription(
                PoseStamped,
                topic(args.namespace, "/local_position/pose"),
                self.on_pose,
                qos_profile_sensor_data,
            )
            self.create_timer(1.0 / args.rate_hz, self.tick)

        def on_state(self, message) -> None:
            self.state = message
            self.state_rx_s = time.monotonic()

        def on_pose(self, message) -> None:
            self.pose = message
            self.pose_rx_s = time.monotonic()

        def tick(self) -> None:
            now = time.monotonic()
            if self.state is None or self.pose is None:
                return
            if self.state_rx_s is None or self.pose_rx_s is None:
                return
            if max(now - self.state_rx_s, now - self.pose_rx_s) > args.input_timeout_sec:
                return
            position = self.pose.pose.position
            orientation = self.pose.pose.orientation
            packet = HitlStatePacket(
                schema=SCHEMA_VERSION,
                role="mini",
                seq=self.seq,
                connected=bool(self.state.connected),
                armed=bool(self.state.armed),
                mode=str(self.state.mode),
                manual_input=bool(self.state.manual_input),
                x_m=float(position.x),
                y_m=float(position.y),
                z_m=float(position.z),
                qx=float(orientation.x),
                qy=float(orientation.y),
                qz=float(orientation.z),
                qw=float(orientation.w),
            )
            self.socket.sendto(encode_packet(packet), self.destination)
            self.seq = (self.seq + 1) & 0xFFFFFFFF

        def destroy_node(self) -> bool:
            self.socket.close()
            return super().destroy_node()

    rclpy.init()
    node = RelayNode()
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
