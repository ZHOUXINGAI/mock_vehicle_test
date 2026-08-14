#!/usr/bin/env python3

"""Bidirectional no-motion LR24 Pair B dry run.

Carrier role:
  receive MiniState frames and transmit bounded PlanCommand frames.

Mini role:
  transmit simulated or read-only MAVROS MiniState frames and receive
  PlanCommand frames through a no-motion executor.

Physical transport and docking role are independent.  The parser defaults are
the original Orin1-Carrier/Orin2-Mini deployment; role-reversal launchers must
override system IDs and transport explicitly.  Neither role publishes motor
commands.
"""

from __future__ import annotations

import argparse
import csv
import math
import struct
import sys
import time
from pathlib import Path

REPO_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_DIR / "src"))

from lr24_compact_protocol import (  # noqa: E402
    CorridorPlanCompact,
    FieldOrigin,
    Frame,
    FrameReader,
    HealthFlag,
    MessageType,
    MiniState,
    Phase,
    PlanCommand,
    PlanFlag,
    PLAN_SCHEMA_VERSION,
    Role,
    StagedMissionFlag,
    StagedMissionPlan,
    corridor_plan_post_tangent_reserve_ms,
    corridor_plan_required_validity_ms,
    describe_frame,
    encode_frame,
    frame_sizes,
)
from lr24_command_guard import CommandGuardPolicy  # noqa: E402
from lr24_live_follower import (  # noqa: E402
    CarrierLocalFollower,
    ExecutorCounters,
    FollowerOutcome,
    MiniLiveFollower,
)
from lr24_mavlink_tunnel import (  # noqa: E402
    TUNNEL_COMPONENT_ID,
    CompactFrameTransport,
    make_transport,
)


def monotonic_ms() -> int:
    return int(time.monotonic() * 1000.0) & 0xFFFFFFFF


def require_no_motion(args: argparse.Namespace) -> None:
    if not args.confirm_no_motion:
        raise SystemExit(
            "Refusing to run: pass --confirm-no-motion after motors are disabled, "
            "wheels are lifted, or the endpoint is not wired to motor execution."
        )


def open_transport(args: argparse.Namespace) -> CompactFrameTransport:
    return make_transport(
        args.transport,
        port=args.port,
        baud=args.baud,
        source_system=args.source_system,
        target_system=args.target_system,
        source_component=args.source_component,
        target_component=args.target_component,
        expected_source_system=args.expected_source_system,
        topic_prefix=args.mavros_topic_prefix,
        router_add_service=args.mavros_router_service,
    )


def read_frames(
    transport: CompactFrameTransport,
    reader: FrameReader,
    timeout_sec: float,
) -> list[Frame]:
    frames: list[Frame] = []
    for data in transport.receive(timeout_sec):
        frames.extend(reader.feed(data))
    return frames


def write_frame(
    transport: CompactFrameTransport,
    msg_type: MessageType,
    payload: bytes,
) -> None:
    transport.send(encode_frame(msg_type, payload))


def simulated_mini_state(args: argparse.Namespace, seq: int, period: float) -> MiniState:
    t = seq * period
    if args.simulate_orbit:
        omega = args.speed_mps / max(0.01, args.radius_m)
        phase = args.phase_rad + omega * t
        x = args.radius_m * math.cos(phase)
        y = args.radius_m * math.sin(phase)
        vx = -args.speed_mps * math.sin(phase)
        vy = args.speed_mps * math.cos(phase)
        yaw = phase + math.pi / 2.0
    else:
        x, y, vx, vy, yaw, omega = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0

    return MiniState(
        vehicle_id=args.vehicle_id,
        seq=seq,
        timestamp_ms=monotonic_ms(),
        x_m=x,
        y_m=y,
        vx_mps=vx,
        vy_mps=vy,
        yaw_rad=yaw,
        omega_radps=omega,
        health=int(
            HealthFlag.POSITION_VALID
            | HealthFlag.VELOCITY_VALID
            | HealthFlag.YAW_VALID
            | HealthFlag.ORIGIN_VALID
        ),
        origin_id=args.origin_id,
    )


def make_corridor_plan(args: argparse.Namespace, seq: int) -> CorridorPlanCompact:
    stamp = monotonic_ms()
    reserve_ms = corridor_plan_post_tangent_reserve_ms(
        args.terminal_completion_budget_ms,
        args.completion_hold_ms,
        args.valid_for_ms,
        args.local_command_watchdog_ms,
        args.plan_timing_guard_ms,
    )
    required_validity_ms = corridor_plan_required_validity_ms(
        args.mini_arrival_delay_ms,
        reserve_ms,
    )
    return CorridorPlanCompact(
        plan_schema_version=PLAN_SCHEMA_VERSION,
        plan_id=args.plan_id,
        seq=seq,
        timestamp_ms=stamp,
        valid_until_ms=(stamp + args.corridor_plan_valid_for_ms) & 0xFFFFFFFF,
        rendezvous_x_m=args.rendezvous_x_m,
        rendezvous_y_m=args.rendezvous_y_m,
        tangent_dir_x=args.tangent_dir_x,
        tangent_dir_y=args.tangent_dir_y,
        corridor_length_m=args.corridor_length_m,
        ahead_distance_m=args.ahead_distance_m,
        mini_arrival_delay_ms=args.mini_arrival_delay_ms,
        trigger_phase_rad=args.trigger_phase_rad,
        mini_speed_mps=args.mini_speed_mps,
        carrier_max_speed_mps=args.carrier_max_speed_mps,
        target_front_gap_m=args.target_front_gap_m,
        required_validity_ms=required_validity_ms,
        post_tangent_reserve_ms=reserve_ms,
        terminal_completion_budget_ms=args.terminal_completion_budget_ms,
        completion_hold_ms=args.completion_hold_ms,
        plan_timing_guard_ms=args.plan_timing_guard_ms,
        command_ttl_ms=args.valid_for_ms,
        local_command_watchdog_ms=args.local_command_watchdog_ms,
        flags=int(PlanFlag.CORRIDOR_VALID | PlanFlag.ONE_ORBIT_COMPLETE),
        origin_id=args.origin_id,
    )


def make_field_origin(args: argparse.Namespace, seq: int) -> FieldOrigin:
    return FieldOrigin(
        origin_id=args.origin_id,
        seq=seq,
        timestamp_ms=monotonic_ms(),
        latitude_deg=args.field_origin_lat_deg,
        longitude_deg=args.field_origin_lon_deg,
        altitude_m=args.field_origin_alt_m,
        flags=0,
    )


def make_staged_mission_plan(args: argparse.Namespace, seq: int) -> StagedMissionPlan:
    stamp = monotonic_ms()
    return StagedMissionPlan(
        schema_version=1,
        plan_id=args.plan_id,
        seq=seq,
        timestamp_ms=stamp,
        valid_until_ms=(stamp + args.staged_plan_valid_for_ms) & 0xFFFFFFFF,
        lead_delay_ms=args.staged_lead_delay_ms,
        lead_distance_m=args.staged_lead_distance_m,
        lateral_offset_m=args.staged_lateral_offset_m,
        straight_distance_m=args.staged_straight_distance_m,
        turn_radius_m=args.staged_turn_radius_m,
        mini_speed_mps=args.staged_mini_speed_mps,
        carrier_speed_mps=args.staged_carrier_speed_mps,
        flags=int(StagedMissionFlag.S_BEND_RETURN),
    )


def print_frame_sizes() -> None:
    print("Compact LR24 frame sizes:")
    for name, size in frame_sizes():
        print(f"  {name}: {size} bytes")


def format_executor_summary(role: str, counters: ExecutorCounters) -> str:
    return (
        f"{role} executor_mode=no_motion "
        f"executor_decisions={counters.executor_decisions} "
        f"zero_output_count={counters.zero_output_count} "
        f"blocked_motion_count={counters.blocked_motion_count} "
        f"nonzero_output_count={counters.nonzero_output_count}"
    )


def dry_run_exit_code(*counters: ExecutorCounters) -> int:
    return 1 if any(item.nonzero_output_count != 0 for item in counters) else 0


def describe_follower_outcome(outcome: FollowerOutcome) -> str:
    output = outcome.executor_output
    return (
        f"gate={outcome.gate_result.decision.value}:{outcome.gate_result.reason} "
        f"executor={output.decision.value}:{output.reason} "
        f"output=({output.v_mps:.1f},{output.omega_radps:.1f})"
    )


def safe_describe_frame(frame: Frame) -> str:
    try:
        return describe_frame(frame)
    except (ValueError, struct.error):
        return f"{frame.msg_type.name} malformed_payload"


def make_carrier_local_hold(args: argparse.Namespace, seq: int = 1) -> PlanCommand:
    stamp = monotonic_ms()
    return PlanCommand(
        plan_id=args.plan_id,
        role=Role.CARRIER,
        phase=Phase.HOLD,
        seq=seq,
        timestamp_ms=stamp,
        valid_until_ms=(stamp + args.valid_for_ms) & 0xFFFFFFFF,
        v_mps=0.0,
        omega_radps=0.0,
        duration_ms=args.command_duration_ms,
        distance_m=0.0,
        max_speed_mps=args.max_speed_mps,
        max_accel_mps2=args.max_accel_mps2,
        flags=0,
    )


def open_csv(path: str | None, fieldnames: list[str]) -> tuple[csv.DictWriter, object] | None:
    if not path:
        return None
    handle = open(path, "w", newline="", encoding="utf-8")
    writer = csv.DictWriter(handle, fieldnames=fieldnames)
    writer.writeheader()
    return writer, handle


def carrier_role(args: argparse.Namespace) -> int:
    require_no_motion(args)
    desired_phase = Phase[args.phase.upper()]
    if desired_phase != Phase.HOLD and not args.allow_nonhold_command:
        raise SystemExit("Non-HOLD phase requires --allow-nonhold-command.")
    if (abs(args.v_mps) > 1.0e-6 or abs(args.omega_radps) > 1.0e-6) and not args.allow_nonhold_command:
        raise SystemExit("Nonzero command requires --allow-nonhold-command.")

    carrier_follower = CarrierLocalFollower()
    local_hold = make_carrier_local_hold(args)
    local_outcome = carrier_follower.apply_command(local_hold, monotonic_ms())
    print(f"carrier local HOLD {describe_follower_outcome(local_outcome)}")

    transport = open_transport(args)
    reader = FrameReader()
    end = time.monotonic() + args.duration_sec if args.duration_sec > 0 else None
    command_period = 1.0 / max(0.1, args.command_rate_hz)
    corridor_plan_period = (
        1.0 / max(0.1, args.corridor_plan_rate_hz)
        if args.send_corridor_plan
        else None
    )
    staged_plan_period = (
        1.0 / max(0.1, args.staged_plan_rate_hz)
        if args.send_staged_mission_plan
        else None
    )
    field_origin_period = 1.0 / max(0.05, args.field_origin_rate_hz)
    next_command = 0.0
    next_corridor_plan = 0.0
    next_staged_plan = 0.0
    next_field_origin = 0.0
    command_seq = 0
    corridor_plan_seq = 0
    staged_plan_seq = 0
    field_origin_seq = 0
    last_state_rx: float | None = None
    last_state_seq: int | None = None
    state_count = 0
    command_count = 0
    corridor_plan_count = 0
    staged_plan_count = 0
    field_origin_count = 0
    gaps = 0
    csv_bundle = open_csv(
        args.csv,
        [
            "role",
            "event",
            "mono_ms",
            "seq",
            "phase",
            "stale_ms",
            "x_m",
            "y_m",
            "v_mps",
            "omega_radps",
        ],
    )

    print(f"carrier dry-run transport={transport.description}")
    print(
        f"command target phase={desired_phase.name} v={args.v_mps:.2f} "
        f"omega={args.omega_radps:.3f} rate={args.command_rate_hz:.1f}Hz"
    )
    if args.send_corridor_plan:
        print(
            "corridor plan enabled "
            f"T=({args.rendezvous_x_m:.2f},{args.rendezvous_y_m:.2f}) "
            f"dir=({args.tangent_dir_x:.3f},{args.tangent_dir_y:.3f}) "
            f"rate={args.corridor_plan_rate_hz:.2f}Hz"
        )
    if args.send_staged_mission_plan:
        print(
            "staged mission plan enabled "
            f"straight={args.staged_straight_distance_m:.2f}m "
            f"radius={args.staged_turn_radius_m:.2f}m "
            f"lead={args.staged_lead_distance_m:.2f}m/"
            f"{args.staged_lead_delay_ms}ms"
        )
    try:
        while end is None or time.monotonic() < end:
            for frame in read_frames(transport, reader, 0.02):
                if frame.msg_type != MessageType.MINI_STATE:
                    print(f"rx {describe_frame(frame)}")
                    continue
                msg = MiniState.decode(frame.payload)
                now = time.monotonic()
                if last_state_seq is not None and msg.seq != last_state_seq + 1:
                    gaps += max(0, msg.seq - last_state_seq - 1)
                last_state_seq = msg.seq
                last_state_rx = now
                state_count += 1
                print(f"rx {describe_frame(frame)}")
                if csv_bundle:
                    writer, _handle = csv_bundle
                    writer.writerow(
                        {
                            "role": "carrier",
                            "event": "rx_state",
                            "mono_ms": monotonic_ms(),
                            "seq": msg.seq,
                            "phase": "",
                            "stale_ms": 0,
                            "x_m": f"{msg.x_m:.3f}",
                            "y_m": f"{msg.y_m:.3f}",
                            "v_mps": "",
                            "omega_radps": "",
                        }
                    )

            now = time.monotonic()
            if now >= next_field_origin:
                origin = make_field_origin(args, field_origin_seq)
                write_frame(transport, MessageType.FIELD_ORIGIN, origin.encode())
                field_origin_count += 1
                print(
                    f"tx FIELD_ORIGIN seq={field_origin_seq} id={origin.origin_id} "
                    f"lat={origin.latitude_deg:.7f} lon={origin.longitude_deg:.7f}"
                )
                field_origin_seq += 1
                next_field_origin = now + field_origin_period

            if (
                args.send_corridor_plan
                and corridor_plan_period is not None
                and now >= next_corridor_plan
            ):
                plan = make_corridor_plan(args, corridor_plan_seq)
                write_frame(transport, MessageType.CORRIDOR_PLAN, plan.encode())
                corridor_plan_count += 1
                print(f"tx CORRIDOR_PLAN seq={corridor_plan_seq}")
                if csv_bundle:
                    writer, _handle = csv_bundle
                    writer.writerow(
                        {
                            "role": "carrier",
                            "event": "tx_corridor_plan",
                            "mono_ms": plan.timestamp_ms,
                            "seq": corridor_plan_seq,
                            "phase": "CORRIDOR_PLAN",
                            "stale_ms": "",
                            "x_m": f"{plan.rendezvous_x_m:.3f}",
                            "y_m": f"{plan.rendezvous_y_m:.3f}",
                            "v_mps": f"{plan.mini_speed_mps:.3f}",
                            "omega_radps": "",
                        }
                    )
                corridor_plan_seq += 1
                next_corridor_plan = now + corridor_plan_period

            if (
                args.send_staged_mission_plan
                and staged_plan_period is not None
                and now >= next_staged_plan
            ):
                staged_plan = make_staged_mission_plan(args, staged_plan_seq)
                write_frame(
                    transport,
                    MessageType.STAGED_MISSION_PLAN,
                    staged_plan.encode(),
                )
                staged_plan_count += 1
                print(f"tx STAGED_MISSION_PLAN seq={staged_plan_seq}")
                staged_plan_seq += 1
                next_staged_plan = now + staged_plan_period

            if now < next_command:
                continue

            stale_ms: float | None
            if last_state_rx is None:
                stale_ms = None
                effective_phase = Phase.HOLD
                v_mps = 0.0
                omega_radps = 0.0
            else:
                stale_ms = (now - last_state_rx) * 1000.0
                if stale_ms > args.stale_ms:
                    effective_phase = Phase.HOLD
                    v_mps = 0.0
                    omega_radps = 0.0
                else:
                    effective_phase = desired_phase
                    v_mps = args.v_mps
                    omega_radps = args.omega_radps

            stamp = monotonic_ms()
            cmd = PlanCommand(
                plan_id=args.plan_id,
                role=Role.MINI,
                phase=effective_phase,
                seq=command_seq,
                timestamp_ms=stamp,
                valid_until_ms=(stamp + args.valid_for_ms) & 0xFFFFFFFF,
                v_mps=v_mps,
                omega_radps=omega_radps,
                duration_ms=args.command_duration_ms,
                distance_m=args.distance_m,
                max_speed_mps=args.max_speed_mps,
                max_accel_mps2=args.max_accel_mps2,
                flags=0,
            )
            write_frame(transport, MessageType.PLAN_COMMAND, cmd.encode())
            command_count += 1
            stale_text = "no_state" if stale_ms is None else f"{stale_ms:.1f}ms"
            print(f"tx PLAN_COMMAND seq={command_seq} phase={effective_phase.name} stale={stale_text}")
            if csv_bundle:
                writer, _handle = csv_bundle
                writer.writerow(
                    {
                        "role": "carrier",
                        "event": "tx_command",
                        "mono_ms": stamp,
                        "seq": command_seq,
                        "phase": effective_phase.name,
                        "stale_ms": "" if stale_ms is None else f"{stale_ms:.1f}",
                        "x_m": "",
                        "y_m": "",
                        "v_mps": f"{v_mps:.3f}",
                        "omega_radps": f"{omega_radps:.3f}",
                    }
                )
            command_seq += 1
            next_command = now + command_period
    finally:
        transport.close()
        if csv_bundle:
            _writer, handle = csv_bundle
            handle.close()
            print(f"wrote {args.csv}")

    print(
        f"carrier summary states_rx={state_count} state_seq_gaps={gaps} "
        f"commands_tx={command_count} corridor_plans_tx={corridor_plan_count} "
        f"staged_plans_tx={staged_plan_count} field_origins_tx={field_origin_count}"
    )
    print(format_executor_summary("carrier_local", carrier_follower.executor.counters))
    return dry_run_exit_code(carrier_follower.executor.counters)


def mini_role(args: argparse.Namespace) -> int:
    require_no_motion(args)
    if not 1 <= args.local_max_plan_ttl_ms <= 180000:
        raise SystemExit("--local-max-plan-ttl-ms must be within [1, 180000]")
    if args.state_source == "mavros-local" and args.simulate_orbit:
        raise SystemExit("--simulate-orbit cannot be used with --state-source mavros-local")
    live_state_source = None
    if args.state_source == "mavros-local":
        from mavros_mini_state_source import MavrosMiniStateSource

        live_state_source = MavrosMiniStateSource(
            namespace=args.mavros_namespace,
            sample_timeout_sec=args.state_sample_timeout_sec,
        )
    transport = open_transport(args)
    reader = FrameReader()
    end = time.monotonic() + args.duration_sec if args.duration_sec > 0 else None
    state_period = 1.0 / max(0.1, args.state_rate_hz)
    next_state = time.monotonic()
    state_seq = 0
    command_count = 0
    corridor_plan_count = 0
    staged_plan_count = 0
    state_count = 0
    last_command_seq: int | None = None
    last_corridor_plan_seq: int | None = None
    last_staged_plan_seq: int | None = None
    command_gaps = 0
    corridor_plan_gaps = 0
    staged_plan_gaps = 0
    rejected_count = 0
    abort_count = 0
    follower = MiniLiveFollower(
        CommandGuardPolicy(
            max_linear_speed_mps=args.local_max_speed_mps,
            max_yaw_rate_radps=args.local_max_yaw_rate_radps,
            max_accel_mps2=args.local_max_accel_mps2,
            max_plan_ttl_ms=args.local_max_plan_ttl_ms,
            command_watchdog_ms=args.command_watchdog_ms,
        )
    )
    follower_poll_period_sec = 0.05
    next_follower_poll = 0.0
    last_poll_signature: tuple[str, str] | None = None
    next_source_status = 0.0
    csv_bundle = open_csv(
        args.csv,
        ["role", "event", "mono_ms", "seq", "phase", "x_m", "y_m", "v_mps", "omega_radps"],
    )

    print(
        f"mini dry-run transport={transport.description} "
        f"state_rate={args.state_rate_hz:.1f}Hz state_source={args.state_source} "
        f"simulate_orbit={args.simulate_orbit}"
    )
    try:
        while end is None or time.monotonic() < end:
            if live_state_source is not None:
                live_state_source.spin_once(0.0)
            now = time.monotonic()
            if now >= next_state:
                if live_state_source is None:
                    msg = simulated_mini_state(args, state_seq, state_period)
                else:
                    msg = live_state_source.build(
                        args.vehicle_id,
                        state_seq,
                        monotonic_ms(),
                    )
                write_frame(transport, MessageType.MINI_STATE, msg.encode())
                source_status = ""
                if live_state_source is not None and now >= next_source_status:
                    source_status = f" {live_state_source.status_text()}"
                    next_source_status = now + 1.0
                print(
                    f"tx MINI_STATE seq={state_seq} health=0x{msg.health:04x} "
                    f"origin={msg.origin_id}{source_status}"
                )
                if csv_bundle:
                    writer, _handle = csv_bundle
                    writer.writerow(
                        {
                            "role": "mini",
                            "event": "tx_state",
                            "mono_ms": msg.timestamp_ms,
                            "seq": state_seq,
                            "phase": "",
                            "x_m": f"{msg.x_m:.3f}",
                            "y_m": f"{msg.y_m:.3f}",
                            "v_mps": "",
                            "omega_radps": "",
                        }
                    )
                state_seq += 1
                state_count += 1
                candidate = next_state + state_period
                next_state = candidate if candidate > now else now + state_period

            for frame in read_frames(transport, reader, 0.002):
                if frame.msg_type == MessageType.FIELD_ORIGIN:
                    outcome = follower.ingest(frame, monotonic_ms())
                    result = outcome.gate_result
                    print(
                        f"rx {safe_describe_frame(frame)} "
                        f"{describe_follower_outcome(outcome)}"
                    )
                    if result.decision.value == "reject":
                        rejected_count += 1
                    continue
                if frame.msg_type == MessageType.ABORT:
                    outcome = follower.ingest(frame, monotonic_ms())
                    result = outcome.gate_result
                    abort_count += 1
                    print(f"rx {safe_describe_frame(frame)} {describe_follower_outcome(outcome)}")
                    continue
                if frame.msg_type == MessageType.CORRIDOR_PLAN:
                    outcome = follower.ingest(frame, monotonic_ms())
                    result = outcome.gate_result
                    try:
                        plan = CorridorPlanCompact.decode(frame.payload)
                    except (ValueError, struct.error):
                        print(
                            f"rx {safe_describe_frame(frame)} "
                            f"{describe_follower_outcome(outcome)}"
                        )
                        rejected_count += 1
                        continue
                    if (
                        last_corridor_plan_seq is not None
                        and plan.seq != last_corridor_plan_seq + 1
                    ):
                        corridor_plan_gaps += max(
                            0, plan.seq - last_corridor_plan_seq - 1
                        )
                    last_corridor_plan_seq = plan.seq
                    corridor_plan_count += 1
                    print(
                        f"rx {safe_describe_frame(frame)} "
                        f"{describe_follower_outcome(outcome)}"
                    )
                    if result.decision.value == "reject":
                        rejected_count += 1
                    if csv_bundle:
                        writer, _handle = csv_bundle
                        writer.writerow(
                            {
                                "role": "mini",
                                "event": "rx_corridor_plan",
                                "mono_ms": monotonic_ms(),
                                "seq": plan.seq,
                                "phase": "CORRIDOR_PLAN",
                                "x_m": f"{plan.rendezvous_x_m:.3f}",
                                "y_m": f"{plan.rendezvous_y_m:.3f}",
                                "v_mps": f"{plan.mini_speed_mps:.3f}",
                                "omega_radps": "",
                            }
                        )
                    continue
                if frame.msg_type == MessageType.STAGED_MISSION_PLAN:
                    outcome = follower.ingest(frame, monotonic_ms())
                    result = outcome.gate_result
                    try:
                        plan = StagedMissionPlan.decode(frame.payload)
                    except (ValueError, struct.error):
                        print(
                            f"rx {safe_describe_frame(frame)} "
                            f"{describe_follower_outcome(outcome)}"
                        )
                        rejected_count += 1
                        continue
                    if (
                        last_staged_plan_seq is not None
                        and plan.seq != last_staged_plan_seq + 1
                    ):
                        staged_plan_gaps += max(
                            0, plan.seq - last_staged_plan_seq - 1
                        )
                    last_staged_plan_seq = plan.seq
                    staged_plan_count += 1
                    print(
                        f"rx {safe_describe_frame(frame)} "
                        f"{describe_follower_outcome(outcome)}"
                    )
                    if result.decision.value == "reject":
                        rejected_count += 1
                    if csv_bundle:
                        writer, _handle = csv_bundle
                        writer.writerow(
                            {
                                "role": "mini",
                                "event": "rx_staged_mission_plan",
                                "mono_ms": monotonic_ms(),
                                "seq": plan.seq,
                                "phase": "STAGED_MISSION_PLAN",
                                "x_m": "",
                                "y_m": "",
                                "v_mps": f"{plan.mini_speed_mps:.3f}",
                                "omega_radps": "",
                            }
                        )
                    continue
                if frame.msg_type != MessageType.PLAN_COMMAND:
                    print(f"rx {safe_describe_frame(frame)}")
                    continue
                outcome = follower.ingest(frame, monotonic_ms())
                result = outcome.gate_result
                try:
                    cmd = PlanCommand.decode(frame.payload)
                except (ValueError, struct.error):
                    print(
                        f"rx {safe_describe_frame(frame)} "
                        f"{describe_follower_outcome(outcome)}"
                    )
                    rejected_count += 1
                    continue
                if last_command_seq is not None and cmd.seq != last_command_seq + 1:
                    command_gaps += max(0, cmd.seq - last_command_seq - 1)
                last_command_seq = cmd.seq
                command_count += 1
                print(
                    f"rx {safe_describe_frame(frame)} "
                    f"{describe_follower_outcome(outcome)}"
                )
                if result.decision.value == "reject":
                    rejected_count += 1
                if csv_bundle:
                    writer, _handle = csv_bundle
                    writer.writerow(
                        {
                            "role": "mini",
                            "event": "rx_command",
                            "mono_ms": monotonic_ms(),
                            "seq": cmd.seq,
                            "phase": cmd.phase.name,
                            "x_m": "",
                            "y_m": "",
                            "v_mps": f"{cmd.v_mps:.3f}",
                            "omega_radps": f"{cmd.omega_radps:.3f}",
                        }
                    )

            if now >= next_follower_poll:
                poll_outcome = follower.poll(monotonic_ms())
                signature = (
                    poll_outcome.gate_result.decision.value,
                    poll_outcome.gate_result.reason,
                )
                if signature != last_poll_signature:
                    print(f"mini follower poll {describe_follower_outcome(poll_outcome)}")
                    last_poll_signature = signature
                next_follower_poll = now + follower_poll_period_sec
    finally:
        transport.close()
        if live_state_source is not None:
            live_state_source.close()
        if csv_bundle:
            _writer, handle = csv_bundle
            handle.close()
            print(f"wrote {args.csv}")

    print(
        f"mini summary states_tx={state_count} commands_rx={command_count} "
        f"command_seq_gaps={command_gaps} corridor_plans_rx={corridor_plan_count} "
        f"corridor_plan_seq_gaps={corridor_plan_gaps} "
        f"staged_plans_rx={staged_plan_count} "
        f"staged_plan_seq_gaps={staged_plan_gaps} rejected={rejected_count} "
        f"aborts_rx={abort_count}"
    )
    print(format_executor_summary("mini", follower.executor.counters))
    return dry_run_exit_code(follower.executor.counters)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--print-frame-sizes", action="store_true")
    sub = parser.add_subparsers(dest="role", required=True)

    def add_common(p: argparse.ArgumentParser) -> None:
        p.add_argument(
            "--transport",
            choices=["raw-serial", "mavlink-serial", "mavros-router"],
            default="raw-serial",
        )
        p.add_argument("--port")
        p.add_argument("--baud", type=int, default=115200)
        p.add_argument("--source-system", type=int)
        p.add_argument("--target-system", type=int)
        p.add_argument("--source-component", type=int, default=TUNNEL_COMPONENT_ID)
        p.add_argument("--target-component", type=int, default=TUNNEL_COMPONENT_ID)
        p.add_argument("--expected-source-system", type=int)
        p.add_argument("--mavros-topic-prefix", default="/pairb_tunnel")
        p.add_argument("--mavros-router-service", default="auto")
        p.add_argument("--duration-sec", type=float, default=60.0)
        p.add_argument("--confirm-no-motion", action="store_true")
        p.add_argument("--csv")

    p = sub.add_parser("carrier", help="Carrier leader dry run.")
    add_common(p)
    p.add_argument("--command-rate-hz", type=float, default=10.0)
    p.add_argument("--plan-id", type=int, default=1)
    p.add_argument(
        "--phase",
        choices=[
            "hold",
            "orbit",
            "arc_to_corridor",
            "terminal",
            "stop",
            "abort",
            "trajectory",
        ],
        default="hold",
    )
    p.add_argument("--v-mps", type=float, default=0.0)
    p.add_argument("--omega-radps", type=float, default=0.0)
    p.add_argument("--command-duration-ms", type=int, default=1000)
    p.add_argument("--distance-m", type=float, default=0.0)
    p.add_argument("--max-speed-mps", type=float, default=0.0)
    p.add_argument("--max-accel-mps2", type=float, default=0.0)
    p.add_argument("--valid-for-ms", type=int, default=500)
    p.add_argument("--stale-ms", type=float, default=300.0)
    p.add_argument("--origin-id", type=int, default=1)
    p.add_argument("--field-origin-lat-deg", type=float, default=0.0)
    p.add_argument("--field-origin-lon-deg", type=float, default=0.0)
    p.add_argument("--field-origin-alt-m", type=float, default=0.0)
    p.add_argument("--field-origin-rate-hz", type=float, default=0.2)
    p.add_argument("--allow-nonhold-command", action="store_true")
    p.add_argument("--send-corridor-plan", action="store_true")
    p.add_argument("--corridor-plan-rate-hz", type=float, default=1.0)
    p.add_argument("--corridor-plan-valid-for-ms", type=int, default=32000)
    p.add_argument("--rendezvous-x-m", type=float, default=-1.5526)
    p.add_argument("--rendezvous-y-m", type=float, default=-4.2237)
    p.add_argument("--tangent-dir-x", type=float, default=0.9386)
    p.add_argument("--tangent-dir-y", type=float, default=-0.3450)
    p.add_argument("--corridor-length-m", type=float, default=8.214)
    p.add_argument("--ahead-distance-m", type=float, default=0.35)
    p.add_argument("--mini-arrival-delay-ms", type=int, default=25724)
    p.add_argument("--trigger-phase-rad", type=float, default=4.360)
    p.add_argument("--mini-speed-mps", type=float, default=0.9)
    p.add_argument("--carrier-max-speed-mps", type=float, default=0.7)
    p.add_argument("--target-front-gap-m", type=float, default=0.35)
    p.add_argument("--terminal-completion-budget-ms", type=int, default=2000)
    p.add_argument("--completion-hold-ms", type=int, default=500)
    p.add_argument("--plan-timing-guard-ms", type=int, default=100)
    p.add_argument("--local-command-watchdog-ms", type=int, default=750)
    p.add_argument("--send-staged-mission-plan", action="store_true")
    p.add_argument("--staged-plan-rate-hz", type=float, default=1.0)
    p.add_argument("--staged-plan-valid-for-ms", type=int, default=120000)
    p.add_argument("--staged-lead-delay-ms", type=int, default=5000)
    p.add_argument("--staged-lead-distance-m", type=float, default=2.0)
    p.add_argument("--staged-straight-distance-m", type=float, default=5.0)
    p.add_argument("--staged-lateral-offset-m", type=float, default=6.0)
    p.add_argument("--staged-turn-radius-m", type=float, default=3.0)
    p.add_argument("--staged-mini-speed-mps", type=float, default=0.12)
    p.add_argument("--staged-carrier-speed-mps", type=float, default=0.06)
    p.set_defaults(
        func=carrier_role,
        transport="mavlink-serial",
        port=(
            "/dev/serial/by-id/"
            "usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"
        ),
        source_system=2,
        target_system=1,
        expected_source_system=1,
    )

    p = sub.add_parser("mini", help="Mini endpoint dry run.")
    add_common(p)
    p.add_argument("--state-rate-hz", type=float, default=50.0)
    p.add_argument(
        "--state-source",
        choices=["simulated", "mavros-local"],
        default="simulated",
    )
    p.add_argument("--mavros-namespace", default="/mavros")
    p.add_argument("--state-sample-timeout-sec", type=float, default=2.0)
    p.add_argument("--vehicle-id", type=int, default=1)
    p.add_argument("--simulate-orbit", action="store_true")
    p.add_argument("--radius-m", type=float, default=4.5)
    p.add_argument("--speed-mps", type=float, default=0.9)
    p.add_argument("--phase-rad", type=float, default=0.0)
    p.add_argument("--origin-id", type=int, default=1)
    p.add_argument("--local-max-speed-mps", type=float, default=1.0)
    p.add_argument("--local-max-yaw-rate-radps", type=float, default=0.6)
    p.add_argument("--local-max-accel-mps2", type=float, default=0.5)
    p.add_argument("--local-max-plan-ttl-ms", type=int, default=120000)
    p.add_argument("--command-watchdog-ms", type=int, default=750)
    p.set_defaults(
        func=mini_role,
        transport="mavros-router",
        port=None,
        source_system=1,
        target_system=2,
        expected_source_system=2,
    )

    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    if args.print_frame_sizes:
        print_frame_sizes()
    return int(args.func(args))


if __name__ == "__main__":
    raise SystemExit(main())
