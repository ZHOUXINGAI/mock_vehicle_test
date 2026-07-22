#!/usr/bin/env python3

import os
import pty
import select
import sys
import termios
import threading
import time
import unittest
from pathlib import Path


REPO_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_DIR / "src"))

from lr24_compact_protocol import MessageType, MiniState, encode_frame, frame_sizes
from lr24_mavlink_tunnel import (
    MAVLINK2_STX,
    MavlinkSerialTunnelTransport,
    TUNNEL_COMPONENT_ID,
    TUNNEL_PAYLOAD_CAPACITY,
    TunnelCodec,
    _pymavlink_to_ros,
    _ros_mavlink_to_bytes,
    mavlink_packet_size,
)


class FakeRosMavlink:
    FRAMING_OK = 1

    def __init__(self, **fields):
        self.__dict__.update(fields)


def sample_frame() -> bytes:
    state = MiniState(
        vehicle_id=2,
        seq=7,
        timestamp_ms=1234,
        x_m=1.0,
        y_m=-2.0,
        vx_mps=0.9,
        vy_mps=0.0,
        yaw_rad=1.2,
        omega_radps=0.2,
        health=0x47,
        origin_id=1,
    )
    return encode_frame(MessageType.MINI_STATE, state.encode())


class TunnelCodecTest(unittest.TestCase):
    def setUp(self):
        self.carrier = TunnelCodec(1, target_system=2)
        self.mini = TunnelCodec(2, target_system=1)

    def test_mavlink2_tunnel_round_trip(self):
        frame = sample_frame()
        packet = self.carrier.pack_frame(frame)
        self.assertEqual(packet[0], MAVLINK2_STX)

        messages = self.mini.parse_bytes(packet)
        self.assertEqual(len(messages), 1)
        message = messages[0]
        self.assertEqual(message.get_type(), "TUNNEL")
        self.assertEqual(message.get_srcSystem(), 1)
        self.assertEqual(message.get_srcComponent(), TUNNEL_COMPONENT_ID)
        self.assertEqual(message.target_system, 2)
        self.assertEqual(message.target_component, TUNNEL_COMPONENT_ID)
        self.assertEqual(self.mini.extract_frame(message, expected_source_system=1), frame)

    def test_wrong_source_or_target_is_rejected(self):
        message = self.carrier.tunnel_message(sample_frame())
        self.assertIsNone(self.mini.extract_frame(message, expected_source_system=2))

        wrong_target = TunnelCodec(1, target_system=3).tunnel_message(sample_frame())
        self.assertIsNone(self.mini.extract_frame(wrong_target, expected_source_system=1))

    def test_payload_capacity_is_enforced(self):
        self.carrier.tunnel_message(bytes(TUNNEL_PAYLOAD_CAPACITY))
        with self.assertRaises(ValueError):
            self.carrier.tunnel_message(bytes(TUNNEL_PAYLOAD_CAPACITY + 1))

    def test_ros_mavlink_conversion_preserves_packet(self):
        message = self.carrier.tunnel_message(sample_frame())
        message.pack(self.carrier._encoder, force_mavlink1=False)
        ros_message = _pymavlink_to_ros(message, FakeRosMavlink)
        rebuilt = _ros_mavlink_to_bytes(ros_message)
        parsed = self.mini.parse_bytes(rebuilt)
        self.assertEqual(len(parsed), 1)
        self.assertEqual(
            self.mini.extract_frame(parsed[0], expected_source_system=1),
            sample_frame(),
        )

    def test_ros_conversion_requires_a_packed_message(self):
        message = self.carrier.tunnel_message(sample_frame())
        self.assertEqual(message.get_header().mlen, 0)
        self.assertIsNone(message.get_crc())

        self.carrier.pack_message(message)
        ros_message = _pymavlink_to_ros(message, FakeRosMavlink)
        self.assertEqual(ros_message.magic, MAVLINK2_STX)
        self.assertEqual(ros_message.msgid, 385)
        self.assertGreater(ros_message.len, 0)
        self.assertIsNotNone(ros_message.checksum)

    def test_all_compact_frames_fit_low_bandwidth_tunnel(self):
        compact_sizes = {
            name: size for name, size in frame_sizes() if name.endswith("_frame")
        }
        self.assertLessEqual(max(compact_sizes.values()), TUNNEL_PAYLOAD_CAPACITY)
        for compact_size in compact_sizes.values():
            self.assertEqual(mavlink_packet_size(compact_size), compact_size + 17)


class SerialTunnelIntegrationTest(unittest.TestCase):
    def test_bidirectional_frames_over_linked_ptys(self):
        master_a, slave_a = pty.openpty()
        master_b, slave_b = pty.openpty()
        path_a = os.ttyname(slave_a)
        path_b = os.ttyname(slave_b)
        stop = threading.Event()

        def relay() -> None:
            while not stop.is_set():
                readable, _, _ = select.select([master_a, master_b], [], [], 0.02)
                for source, target in ((master_a, master_b), (master_b, master_a)):
                    if source not in readable:
                        continue
                    try:
                        data = os.read(source, 4096)
                        if data:
                            os.write(target, data)
                    except OSError:
                        return

        carrier = None
        mini = None
        thread = threading.Thread(target=relay, daemon=True)
        try:
            carrier = MavlinkSerialTunnelTransport(
                path_a, 57600, 1, 2, expected_source_system=2
            )
            mini = MavlinkSerialTunnelTransport(
                path_b, 57600, 2, 1, expected_source_system=1
            )
            self.assertEqual(termios.tcgetattr(carrier.fd)[4], termios.B57600)
            self.assertEqual(termios.tcgetattr(carrier.fd)[5], termios.B57600)
            self.assertEqual(termios.tcgetattr(mini.fd)[4], termios.B57600)
            self.assertEqual(termios.tcgetattr(mini.fd)[5], termios.B57600)
            os.close(slave_a)
            os.close(slave_b)
            slave_a = -1
            slave_b = -1
            thread.start()

            frame = sample_frame()
            carrier.send(frame)
            self.assertEqual(self._receive_one(mini), frame)

            mini.send(frame)
            self.assertEqual(self._receive_one(carrier), frame)
        finally:
            if carrier is not None:
                carrier.close()
            if mini is not None:
                mini.close()
            stop.set()
            if thread.is_alive():
                thread.join(timeout=1.0)
            for fd in (slave_a, slave_b, master_a, master_b):
                if fd >= 0:
                    try:
                        os.close(fd)
                    except OSError:
                        pass

    @staticmethod
    def _receive_one(transport: MavlinkSerialTunnelTransport) -> bytes | None:
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            frames = transport.receive(0.05)
            if frames:
                return frames[0]
        return None


if __name__ == "__main__":
    unittest.main()
