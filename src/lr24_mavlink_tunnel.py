"""MAVLink 2 TUNNEL transport for LR24 Pair B compact frames."""

from __future__ import annotations

import os
import select
import struct
import termios
import time
from collections import deque
from pathlib import Path
from typing import Protocol

from pymavlink.dialects.v20 import common as mavlink2


TUNNEL_COMPONENT_ID = int(mavlink2.MAV_COMP_ID_TUNNEL_NODE)
TUNNEL_PAYLOAD_TYPE = int(mavlink2.MAV_TUNNEL_PAYLOAD_TYPE_UNKNOWN)
TUNNEL_PAYLOAD_CAPACITY = 128
MAVLINK2_STX = 0xFD

BAUD = {
    9600: termios.B9600,
    19200: termios.B19200,
    38400: termios.B38400,
    57600: termios.B57600,
    115200: termios.B115200,
    230400: termios.B230400,
    460800: termios.B460800,
    921600: termios.B921600,
}


class CompactFrameTransport(Protocol):
    description: str

    def send(self, frame: bytes) -> None: ...

    def receive(self, timeout_sec: float) -> list[bytes]: ...

    def close(self) -> None: ...


class TunnelCodec:
    """Pack and validate one compact L2 frame per MAVLink TUNNEL message."""

    def __init__(
        self,
        source_system: int,
        source_component: int = TUNNEL_COMPONENT_ID,
        target_system: int = 0,
        target_component: int = TUNNEL_COMPONENT_ID,
    ) -> None:
        for name, value in (
            ("source_system", source_system),
            ("source_component", source_component),
            ("target_system", target_system),
            ("target_component", target_component),
        ):
            if not 0 <= int(value) <= 255:
                raise ValueError(f"{name} must be in [0, 255]")
        if source_system == 0 or source_component == 0:
            raise ValueError("MAVLink source system/component must be nonzero")

        self.source_system = int(source_system)
        self.source_component = int(source_component)
        self.target_system = int(target_system)
        self.target_component = int(target_component)
        self._encoder = mavlink2.MAVLink(
            None,
            srcSystem=self.source_system,
            srcComponent=self.source_component,
        )
        self._parser = mavlink2.MAVLink(None)
        self._parser.robust_parsing = True

    def tunnel_message(self, frame: bytes) -> mavlink2.MAVLink_tunnel_message:
        raw = bytes(frame)
        if not raw:
            raise ValueError("compact frame must not be empty")
        if len(raw) > TUNNEL_PAYLOAD_CAPACITY:
            raise ValueError(
                f"compact frame is {len(raw)} bytes; TUNNEL limit is "
                f"{TUNNEL_PAYLOAD_CAPACITY}"
            )
        payload = raw + bytes(TUNNEL_PAYLOAD_CAPACITY - len(raw))
        return self._encoder.tunnel_encode(
            self.target_system,
            self.target_component,
            TUNNEL_PAYLOAD_TYPE,
            len(raw),
            payload,
        )

    def heartbeat_message(self) -> mavlink2.MAVLink_heartbeat_message:
        return self._encoder.heartbeat_encode(
            mavlink2.MAV_TYPE_ONBOARD_CONTROLLER,
            mavlink2.MAV_AUTOPILOT_INVALID,
            0,
            0,
            mavlink2.MAV_STATE_ACTIVE,
        )

    def pack_message(self, message: mavlink2.MAVLink_message) -> bytes:
        return bytes(message.pack(self._encoder, force_mavlink1=False))

    def pack_frame(self, frame: bytes) -> bytes:
        return self.pack_message(self.tunnel_message(frame))

    def parse_bytes(self, data: bytes) -> list[mavlink2.MAVLink_message]:
        messages = self._parser.parse_buffer(bytes(data))
        return [] if messages is None else list(messages)

    def extract_frame(
        self,
        message: mavlink2.MAVLink_message,
        *,
        expected_source_system: int | None = None,
        expected_source_component: int | None = TUNNEL_COMPONENT_ID,
    ) -> bytes | None:
        if message.get_type() != "TUNNEL":
            return None
        if int(message.payload_type) != TUNNEL_PAYLOAD_TYPE:
            return None
        if int(message.target_system) not in (0, self.source_system):
            return None
        if int(message.target_component) not in (0, self.source_component):
            return None
        if expected_source_system is not None and message.get_srcSystem() != expected_source_system:
            return None
        if (
            expected_source_component is not None
            and message.get_srcComponent() != expected_source_component
        ):
            return None
        payload_length = int(message.payload_length)
        if not 0 < payload_length <= TUNNEL_PAYLOAD_CAPACITY:
            return None
        return bytes(message.payload[:payload_length])


class RawSerialTransport:
    def __init__(self, port: str, baud: int) -> None:
        self.fd = _open_serial(port, baud)
        self.description = f"raw-serial:{port}@{baud}"

    def send(self, frame: bytes) -> None:
        _write_all(self.fd, frame)

    def receive(self, timeout_sec: float) -> list[bytes]:
        readable, _, _ = select.select([self.fd], [], [], max(0.0, timeout_sec))
        if not readable:
            return []
        try:
            data = os.read(self.fd, 4096)
        except BlockingIOError:
            return []
        return [data] if data else []

    def close(self) -> None:
        if self.fd >= 0:
            os.close(self.fd)
            self.fd = -1


class MavlinkSerialTunnelTransport:
    def __init__(
        self,
        port: str,
        baud: int,
        source_system: int,
        target_system: int,
        source_component: int = TUNNEL_COMPONENT_ID,
        target_component: int = TUNNEL_COMPONENT_ID,
        expected_source_system: int | None = None,
        heartbeat_rate_hz: float = 1.0,
    ) -> None:
        self.fd = _open_serial(port, baud)
        self.codec = TunnelCodec(
            source_system,
            source_component,
            target_system,
            target_component,
        )
        self.expected_source_system = expected_source_system
        self.heartbeat_period = 1.0 / max(0.1, heartbeat_rate_hz)
        self.next_heartbeat = 0.0
        self.description = (
            f"mavlink-tunnel-serial:{port}@{baud} "
            f"{source_system}.{source_component}->{target_system}.{target_component}"
        )

    def _heartbeat_if_due(self) -> None:
        now = time.monotonic()
        if now < self.next_heartbeat:
            return
        _write_all(self.fd, self.codec.pack_message(self.codec.heartbeat_message()))
        self.next_heartbeat = now + self.heartbeat_period

    def send(self, frame: bytes) -> None:
        self._heartbeat_if_due()
        _write_all(self.fd, self.codec.pack_frame(frame))

    def receive(self, timeout_sec: float) -> list[bytes]:
        self._heartbeat_if_due()
        readable, _, _ = select.select([self.fd], [], [], max(0.0, timeout_sec))
        if not readable:
            return []
        try:
            data = os.read(self.fd, 4096)
        except BlockingIOError:
            return []
        frames: list[bytes] = []
        for message in self.codec.parse_bytes(data):
            frame = self.codec.extract_frame(
                message,
                expected_source_system=self.expected_source_system,
            )
            if frame is not None:
                frames.append(frame)
        return frames

    def close(self) -> None:
        if self.fd >= 0:
            os.close(self.fd)
            self.fd = -1


class MavrosRouterTunnelTransport:
    """ROS 2 endpoint attached to the MAVROS router on the Mini computer."""

    def __init__(
        self,
        source_system: int,
        target_system: int,
        source_component: int = TUNNEL_COMPONENT_ID,
        target_component: int = TUNNEL_COMPONENT_ID,
        expected_source_system: int | None = None,
        topic_prefix: str = "/pairb_tunnel",
        router_add_service: str = "auto",
        heartbeat_rate_hz: float = 1.0,
        startup_timeout_sec: float = 10.0,
    ) -> None:
        try:
            import rclpy
            from mavros_msgs.msg import Mavlink
            from mavros_msgs.srv import EndpointAdd, EndpointDel
            from rclpy.qos import (
                QoSDurabilityPolicy,
                QoSHistoryPolicy,
                QoSProfile,
                QoSReliabilityPolicy,
            )
        except ImportError as exc:
            raise RuntimeError("ROS 2 MAVROS Python packages are required") from exc

        self._rclpy = rclpy
        self._Mavlink = Mavlink
        self._EndpointAdd = EndpointAdd
        self._EndpointDel = EndpointDel
        self._owns_rclpy = not rclpy.ok()
        if self._owns_rclpy:
            rclpy.init(args=None)

        self.codec = TunnelCodec(
            source_system,
            source_component,
            target_system,
            target_component,
        )
        self.expected_source_system = expected_source_system
        self.topic_prefix = "/" + topic_prefix.strip("/")
        self.node = rclpy.create_node(f"lr24_pairb_tunnel_{source_system}")
        self._rx: deque[bytes] = deque()
        self.publisher = self.node.create_publisher(
            Mavlink, f"{self.topic_prefix}/mavlink_sink", 50
        )
        source_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=50,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.subscription = self.node.create_subscription(
            Mavlink,
            f"{self.topic_prefix}/mavlink_source",
            self._receive_ros_message,
            source_qos,
        )
        self.endpoint_id = 0
        self.endpoint_owned = False
        self.router_add_service = self._resolve_add_service(
            router_add_service, startup_timeout_sec
        )
        self._ensure_router_endpoint(startup_timeout_sec)
        self.heartbeat_period = 1.0 / max(0.1, heartbeat_rate_hz)
        self.next_heartbeat = 0.0
        self.description = (
            f"mavros-router-tunnel:{self.topic_prefix} "
            f"{source_system}.{source_component}->{target_system}.{target_component}"
        )

    def _resolve_add_service(self, requested: str, timeout_sec: float) -> str:
        if requested != "auto":
            return requested
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            matches = [
                name
                for name, types in self.node.get_service_names_and_types()
                if name.endswith("/add_endpoint")
                and "mavros_msgs/srv/EndpointAdd" in types
            ]
            if len(matches) == 1:
                return matches[0]
            if len(matches) > 1:
                raise RuntimeError(
                    "multiple MAVROS router add_endpoint services found; "
                    "pass --mavros-router-service explicitly"
                )
            self._rclpy.spin_once(self.node, timeout_sec=0.1)
        raise RuntimeError("MAVROS router add_endpoint service not found")

    def _router_topics_exist(self) -> bool:
        names = {name for name, _types in self.node.get_topic_names_and_types()}
        return {
            f"{self.topic_prefix}/mavlink_sink",
            f"{self.topic_prefix}/mavlink_source",
        }.issubset(names)

    def _ensure_router_endpoint(self, timeout_sec: float) -> None:
        # An endpoint can survive a bridge process restart inside MAVROS. Reuse
        # its topics instead of creating a duplicate route.
        if self._router_topics_exist() and self.publisher.get_subscription_count() > 0:
            return

        client = self.node.create_client(self._EndpointAdd, self.router_add_service)
        if not client.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError(f"router service unavailable: {self.router_add_service}")
        request = self._EndpointAdd.Request()
        request.url = self.topic_prefix
        request.type = self._EndpointAdd.Request.TYPE_UAS
        future = client.call_async(request)
        self._rclpy.spin_until_future_complete(
            self.node, future, timeout_sec=timeout_sec
        )
        response = future.result()
        if response is None or not response.successful:
            reason = "no response" if response is None else response.reason
            if self._router_topics_exist():
                return
            raise RuntimeError(f"failed to add MAVROS router endpoint: {reason}")
        self.endpoint_id = int(response.id)
        self.endpoint_owned = True

    def _receive_ros_message(self, ros_message: object) -> None:
        packet = _ros_mavlink_to_bytes(ros_message)
        for message in self.codec.parse_bytes(packet):
            frame = self.codec.extract_frame(
                message,
                expected_source_system=self.expected_source_system,
            )
            if frame is not None:
                self._rx.append(frame)

    def _publish_mavlink(self, message: mavlink2.MAVLink_message) -> None:
        # pymavlink fills the MAVLink 2 header, sequence and checksum only when
        # pack() is called. MAVROS transports the decoded ROS fields, so the
        # message must be packed before converting it to mavros_msgs/Mavlink.
        self.codec.pack_message(message)
        self.publisher.publish(_pymavlink_to_ros(message, self._Mavlink))

    def _heartbeat_if_due(self) -> None:
        now = time.monotonic()
        if now < self.next_heartbeat:
            return
        self._publish_mavlink(self.codec.heartbeat_message())
        self.next_heartbeat = now + self.heartbeat_period

    def send(self, frame: bytes) -> None:
        self._heartbeat_if_due()
        self._publish_mavlink(self.codec.tunnel_message(frame))

    def receive(self, timeout_sec: float) -> list[bytes]:
        self._heartbeat_if_due()
        deadline = time.monotonic() + max(0.0, timeout_sec)
        while not self._rx and time.monotonic() < deadline:
            self._rclpy.spin_once(
                self.node,
                timeout_sec=min(0.02, max(0.0, deadline - time.monotonic())),
            )
        frames = list(self._rx)
        self._rx.clear()
        return frames

    def close(self) -> None:
        if self.endpoint_owned and self.endpoint_id:
            service = self.router_add_service.rsplit("/", 1)[0] + "/del_endpoint"
            client = self.node.create_client(self._EndpointDel, service)
            if client.wait_for_service(timeout_sec=1.0):
                request = self._EndpointDel.Request()
                request.id = self.endpoint_id
                request.url = ""
                request.type = self._EndpointDel.Request.TYPE_UAS
                future = client.call_async(request)
                self._rclpy.spin_until_future_complete(self.node, future, timeout_sec=1.0)
        self.node.destroy_node()
        if self._owns_rclpy and self._rclpy.ok():
            self._rclpy.shutdown()


def make_transport(
    transport: str,
    *,
    port: str | None,
    baud: int,
    source_system: int,
    target_system: int,
    source_component: int = TUNNEL_COMPONENT_ID,
    target_component: int = TUNNEL_COMPONENT_ID,
    expected_source_system: int | None = None,
    topic_prefix: str = "/pairb_tunnel",
    router_add_service: str = "auto",
) -> CompactFrameTransport:
    if transport == "raw-serial":
        if not port:
            raise ValueError("--port is required for raw-serial transport")
        return RawSerialTransport(port, baud)
    if transport == "mavlink-serial":
        if not port:
            raise ValueError("--port is required for mavlink-serial transport")
        return MavlinkSerialTunnelTransport(
            port,
            baud,
            source_system,
            target_system,
            source_component,
            target_component,
            expected_source_system,
        )
    if transport == "mavros-router":
        return MavrosRouterTunnelTransport(
            source_system,
            target_system,
            source_component,
            target_component,
            expected_source_system,
            topic_prefix,
            router_add_service,
        )
    raise ValueError(f"unsupported transport: {transport}")


def mavlink_packet_size(frame_length: int) -> int:
    """Return the unsigned MAVLink 2 TUNNEL packet size after zero truncation."""

    codec = TunnelCodec(1, target_system=2)
    return len(codec.pack_frame(bytes([0xA5]) * frame_length))


def _open_serial(port: str, baud: int) -> int:
    if baud not in BAUD:
        raise ValueError(f"unsupported baud {baud}; supported: {sorted(BAUD)}")
    path = Path(port)
    if not path.exists():
        raise FileNotFoundError(f"serial port not found: {port}")
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0
    attrs[1] = 0
    attrs[2] = termios.CLOCAL | termios.CREAD | termios.CS8
    attrs[3] = 0
    # Python's tcsetattr() takes input/output speed from indexes 4 and 5.
    # Putting B57600 only in c_cflag leaves many USB UARTs at their previous
    # speed (observed as 9600 on the Pair B CP2102).
    attrs[4] = BAUD[baud]
    attrs[5] = BAUD[baud]
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 1
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    applied = termios.tcgetattr(fd)
    if applied[4] != BAUD[baud] or applied[5] != BAUD[baud]:
        os.close(fd)
        raise OSError(f"serial port rejected baud {baud}: {port}")
    termios.tcflush(fd, termios.TCIOFLUSH)
    return fd


def _write_all(fd: int, data: bytes) -> None:
    view = memoryview(data)
    while view:
        try:
            written = os.write(fd, view)
        except BlockingIOError:
            select.select([], [fd], [], 0.1)
            continue
        if written <= 0:
            raise OSError("serial write returned zero bytes")
        view = view[written:]


def _payload64(payload: bytes) -> list[int]:
    padded = payload + bytes((-len(payload)) % 8)
    if not padded:
        return []
    return list(struct.unpack(f"<{len(padded) // 8}Q", padded))


def _pymavlink_to_ros(message: mavlink2.MAVLink_message, ros_type: type) -> object:
    header = message.get_header()
    return ros_type(
        framing_status=ros_type.FRAMING_OK,
        magic=MAVLINK2_STX,
        len=header.mlen,
        incompat_flags=header.incompat_flags,
        compat_flags=header.compat_flags,
        seq=message.get_seq(),
        sysid=header.srcSystem,
        compid=header.srcComponent,
        msgid=header.msgId,
        checksum=message.get_crc(),
        payload64=_payload64(bytes(message.get_payload())),
        signature=[],
    )


def _ros_mavlink_to_bytes(message: object) -> bytes:
    payload = struct.pack(f"<{len(message.payload64)}Q", *message.payload64)
    payload = payload[: int(message.len)]
    if int(message.magic) != MAVLINK2_STX:
        return b""
    msg_id = int(message.msgid)
    packet = struct.pack(
        "<BBBBBBBBBB",
        MAVLINK2_STX,
        int(message.len),
        int(message.incompat_flags),
        int(message.compat_flags),
        int(message.seq),
        int(message.sysid),
        int(message.compid),
        msg_id & 0xFF,
        (msg_id >> 8) & 0xFF,
        (msg_id >> 16) & 0xFF,
    )
    packet += payload + struct.pack("<H", int(message.checksum))
    packet += bytes(message.signature)
    return packet
