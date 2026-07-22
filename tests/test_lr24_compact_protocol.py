#!/usr/bin/env python3

import math
import sys
import unittest
from pathlib import Path


REPO_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_DIR / "src"))

from lr24_command_guard import CommandGuardPolicy, Decision, MiniCommandGate
from lr24_field_frame import (
    FieldFrame,
    field_enu_yaw_to_px4_ned,
    px4_ned_yaw_to_field_enu,
)
from lr24_compact_protocol import (
    Abort,
    AbortReason,
    CorridorPlanCompact,
    FieldOrigin,
    FrameReader,
    HealthFlag,
    MessageType,
    MiniState,
    Phase,
    PlanCommand,
    PlanFlag,
    Role,
    encode_frame,
    frame_sizes,
    sequence_is_newer,
    validity_window_ms,
)


def decoded_frame(msg_type: MessageType, payload: bytes):
    frames = FrameReader().feed(encode_frame(msg_type, payload))
    if len(frames) != 1:
        raise AssertionError(f"expected one frame, got {len(frames)}")
    return frames[0]


def sample_plan(seq: int = 1, plan_id: int = 7) -> CorridorPlanCompact:
    return CorridorPlanCompact(
        plan_id=plan_id,
        seq=seq,
        timestamp_ms=1000,
        valid_until_ms=31000,
        rendezvous_x_m=-1.553,
        rendezvous_y_m=-4.224,
        tangent_dir_x=0.9386,
        tangent_dir_y=-0.3450,
        corridor_length_m=8.2,
        ahead_distance_m=0.35,
        mini_arrival_delay_ms=25724,
        trigger_phase_rad=4.36,
        mini_speed_mps=0.9,
        carrier_max_speed_mps=0.7,
        target_front_gap_m=0.35,
        flags=int(PlanFlag.CORRIDOR_VALID | PlanFlag.ONE_ORBIT_COMPLETE),
        origin_id=1,
    )


def sample_command(seq: int = 1, plan_id: int = 7) -> PlanCommand:
    return PlanCommand(
        plan_id=plan_id,
        role=Role.MINI,
        phase=Phase.TERMINAL,
        seq=seq,
        timestamp_ms=2000,
        valid_until_ms=2500,
        v_mps=0.6,
        omega_radps=0.1,
        duration_ms=1000,
        distance_m=0.0,
        max_speed_mps=0.9,
        max_accel_mps2=0.3,
        flags=0,
    )


def sample_origin(seq: int = 1, origin_id: int = 1) -> FieldOrigin:
    return FieldOrigin(
        origin_id=origin_id,
        seq=seq,
        timestamp_ms=900,
        latitude_deg=31.2304,
        longitude_deg=121.4737,
        altitude_m=4.2,
    )


class CompactProtocolTest(unittest.TestCase):
    def test_golden_mini_state_frame(self):
        state = MiniState(
            vehicle_id=2,
            seq=1,
            timestamp_ms=1000,
            x_m=1.25,
            y_m=-2.5,
            vx_mps=0.9,
            vy_mps=-0.1,
            yaw_rad=math.pi / 2.0,
            omega_radps=0.2,
            health=int(
                HealthFlag.POSITION_VALID
                | HealthFlag.VELOCITY_VALID
                | HealthFlag.YAW_VALID
            ),
            origin_id=1,
        )
        raw = encode_frame(MessageType.MINI_STATE, state.encode())
        self.assertEqual(
            raw.hex(),
            "4c320101190201000000e80300007d0006ff5a00f6ff28237a0407000100637d",
        )
        decoded = MiniState.decode(FrameReader().feed(raw)[0].payload)
        self.assertEqual(decoded.seq, 1)
        self.assertAlmostEqual(decoded.yaw_rad, math.pi / 2.0, places=4)

    def test_frame_sizes_match_radio_budget(self):
        sizes = dict(frame_sizes())
        self.assertEqual(sizes["mini_state_frame"], 32)
        self.assertEqual(sizes["plan_command_frame"], 37)
        self.assertEqual(sizes["corridor_plan_frame"], 49)
        self.assertEqual(sizes["abort_frame"], 21)
        self.assertEqual(sizes["field_origin_frame"], 31)

    def test_reader_handles_fragmented_magic_and_frame(self):
        raw = encode_frame(MessageType.MINI_STATE, MiniState(2, 1, 2, 0, 0, 0, 0, 0, 0, 1).encode())
        reader = FrameReader()
        self.assertEqual(reader.feed(b"noiseL"), [])
        self.assertEqual(reader.feed(raw[1:8]), [])
        frames = reader.feed(raw[8:])
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0].msg_type, MessageType.MINI_STATE)

    def test_reader_rejects_crc_and_payload_length_errors(self):
        raw = bytearray(encode_frame(MessageType.MINI_STATE, MiniState(2, 1, 2, 0, 0, 0, 0, 0, 0, 1).encode()))
        raw[-1] ^= 0x80
        reader = FrameReader()
        self.assertEqual(reader.feed(raw), [])
        self.assertEqual(reader.crc_errors, 1)

        reader = FrameReader()
        self.assertEqual(reader.feed(encode_frame(MessageType.PING, b"short")), [])
        self.assertEqual(reader.length_errors, 1)

    def test_wrapping_sequence_and_ttl(self):
        self.assertTrue(sequence_is_newer(0, 0xFFFFFFFF))
        self.assertFalse(sequence_is_newer(10, 10))
        self.assertFalse(sequence_is_newer(9, 10))
        self.assertEqual(validity_window_ms(0xFFFFFFF0, 0x00000054), 100)


class CommandGuardTest(unittest.TestCase):
    def setUp(self):
        self.gate = MiniCommandGate(
            CommandGuardPolicy(
                max_linear_speed_mps=1.0,
                max_yaw_rate_radps=0.6,
                max_accel_mps2=0.5,
                command_watchdog_ms=750,
            )
        )

    def test_plan_then_command_and_watchdog_stop(self):
        self.gate.ingest(
            decoded_frame(MessageType.FIELD_ORIGIN, sample_origin().encode()), 9999
        )
        result = self.gate.ingest(
            decoded_frame(MessageType.CORRIDOR_PLAN, sample_plan().encode()), 10000
        )
        self.assertEqual(result.decision, Decision.ACCEPT)

        result = self.gate.ingest(
            decoded_frame(MessageType.PLAN_COMMAND, sample_command().encode()), 10100
        )
        self.assertEqual(result.decision, Decision.ACCEPT)
        self.assertEqual(self.gate.poll(10599).decision, Decision.ACCEPT)
        self.assertEqual(self.gate.poll(10600).decision, Decision.STOP)

    def test_duplicate_and_motion_without_plan_are_rejected(self):
        result = self.gate.ingest(
            decoded_frame(MessageType.PLAN_COMMAND, sample_command().encode()), 10000
        )
        self.assertEqual(result.reason, "motion_without_corridor_plan")

        self.gate.ingest(
            decoded_frame(MessageType.FIELD_ORIGIN, sample_origin().encode()), 9999
        )
        self.gate.ingest(
            decoded_frame(MessageType.CORRIDOR_PLAN, sample_plan().encode()), 10000
        )
        self.gate.ingest(
            decoded_frame(MessageType.PLAN_COMMAND, sample_command().encode()), 10000
        )
        result = self.gate.ingest(
            decoded_frame(MessageType.PLAN_COMMAND, sample_command().encode()), 10100
        )
        self.assertEqual(result.reason, "duplicate_or_old_command_seq")

    def test_limits_and_nonzero_hold_are_rejected(self):
        too_fast = sample_command()
        too_fast = PlanCommand(**{**too_fast.__dict__, "v_mps": 1.2})
        result = self.gate.ingest(
            decoded_frame(MessageType.PLAN_COMMAND, too_fast.encode()), 10000
        )
        self.assertEqual(result.reason, "local_linear_limit")

        hold = sample_command()
        hold = PlanCommand(**{**hold.__dict__, "phase": Phase.HOLD})
        result = self.gate.ingest(
            decoded_frame(MessageType.PLAN_COMMAND, hold.encode()), 10000
        )
        self.assertEqual(result.reason, "nonzero_hold_or_stop")

    def test_abort_is_latched_and_has_priority(self):
        abort = Abort(
            source_role=Role.CARRIER,
            reason=AbortReason.OPERATOR,
            plan_id=7,
            seq=4,
            timestamp_ms=1234,
        )
        result = self.gate.ingest(
            decoded_frame(MessageType.ABORT, abort.encode()), 10000
        )
        self.assertEqual(result.decision, Decision.ABORT)
        self.assertEqual(self.gate.poll(999999).decision, Decision.ABORT)
        result = self.gate.ingest(
            decoded_frame(MessageType.CORRIDOR_PLAN, sample_plan().encode()), 10001
        )
        self.assertEqual(result.reason, "abort_latched")

        self.gate.clear_abort_locally()
        self.assertEqual(self.gate.poll(10002).decision, Decision.HOLD)


class FieldFrameTest(unittest.TestCase):
    def test_origin_maps_to_zero(self):
        frame = FieldFrame(1, 31.2304, 121.4737, 4.2)
        east, north, up = frame.to_enu(31.2304, 121.4737, 4.2)
        self.assertAlmostEqual(east, 0.0, places=6)
        self.assertAlmostEqual(north, 0.0, places=6)
        self.assertAlmostEqual(up, 0.0, places=6)

    def test_small_east_and_north_offsets(self):
        frame = FieldFrame(1, 31.2304, 121.4737, 4.2)
        east, north, _up = frame.to_enu(31.2304, 121.473805, 4.2)
        self.assertAlmostEqual(east, 10.0, delta=0.2)
        self.assertAlmostEqual(north, 0.0, delta=0.1)

        east, north, _up = frame.to_enu(31.23049, 121.4737, 4.2)
        self.assertAlmostEqual(east, 0.0, delta=0.1)
        self.assertAlmostEqual(north, 10.0, delta=0.2)

    def test_px4_and_field_yaw_conversion_round_trip(self):
        self.assertAlmostEqual(px4_ned_yaw_to_field_enu(0.0), math.pi / 2.0)
        self.assertAlmostEqual(px4_ned_yaw_to_field_enu(math.pi / 2.0), 0.0)
        yaw = -2.4
        self.assertAlmostEqual(
            field_enu_yaw_to_px4_ned(px4_ned_yaw_to_field_enu(yaw)), yaw
        )


if __name__ == "__main__":
    unittest.main()
