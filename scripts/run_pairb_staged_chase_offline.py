#!/usr/bin/env python3

"""Replay the staged chase contract through real Pair B codecs, without I/O."""

from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import asdict
from pathlib import Path

REPO_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_DIR / "src"))

from lr24_command_guard import CommandGuardPolicy  # noqa: E402
from lr24_compact_protocol import (  # noqa: E402
    FrameReader,
    HealthFlag,
    MessageType,
    MiniState,
    Phase,
    Role,
    encode_frame,
)
from lr24_live_follower import MiniLiveFollower  # noqa: E402
from pairb_staged_chase import (  # noqa: E402
    REQUIRED_MINI_HEALTH,
    ChasePhase,
    StagedChaseConfig,
    StagedChaseCoordinator,
    build_staged_mission_plan,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", choices=("nominal", "stale", "all"), default="all")
    parser.add_argument("--output", type=Path)
    return parser.parse_args()


def decode_one(msg_type: MessageType, payload: bytes):
    reader = FrameReader()
    frames = reader.feed(encode_frame(msg_type, payload))
    if len(frames) != 1 or reader.crc_errors or reader.length_errors:
        raise RuntimeError("Pair B codec replay failed")
    return frames[0]


def mini_state(seq: int, now_ms: int, x_m: float) -> MiniState:
    return MiniState(
        vehicle_id=1,
        seq=seq,
        timestamp_ms=now_ms,
        x_m=x_m,
        y_m=0.05 * math.sin(x_m),
        vx_mps=0.60,
        vy_mps=0.0,
        yaw_rad=0.0,
        omega_radps=0.0,
        health=REQUIRED_MINI_HEALTH,
        origin_id=0,
    )


def nominal() -> dict[str, object]:
    config = StagedChaseConfig(
        lead_delay_ms=2000,
        lead_distance_m=1.0,
        mission_timeout_ms=30000,
        plan_validity_ms=30000,
    )
    coordinator = StagedChaseCoordinator(config)
    coordinator.set_carrier_ready(True)
    coordinator.authorize_start()
    follower = MiniLiveFollower(CommandGuardPolicy(target_role=Role.MINI))
    plan = build_staged_mission_plan(config, seq=1, sender_monotonic_ms=1000)
    plan_outcome = follower.ingest(
        decode_one(MessageType.STAGED_MISSION_PLAN, plan.encode()), 5000
    )
    events = []
    local_start_count = 0
    for index in range(31):
        now_ms = 1000 + index * 100
        state = mini_state(index, now_ms, index * 0.06)
        coordinator.accept_mini_state(state, now_ms)
        decision = coordinator.step(now_ms)
        if decision.start_local_carrier:
            local_start_count += 1
        command_outcome = follower.ingest(
            decode_one(MessageType.PLAN_COMMAND, decision.remote_command.encode()),
            5000 + index * 100,
        )
        events.append(
            {
                "t_ms": now_ms,
                "phase": decision.phase.value,
                "lead_m": round(decision.lead_distance_m, 3),
                "remote_gate": command_outcome.gate_result.decision.value,
                "remote_executor": command_outcome.executor_output.decision.value,
                "start_local": decision.start_local_carrier,
            }
        )
    return {
        "pass": bool(
            plan_outcome.gate_result.reason == "staged_mission_plan"
            and local_start_count == 1
            and events[-1]["phase"] == ChasePhase.BOTH_ACTIVE.value
            and follower.executor.counters.nonzero_output_count == 0
        ),
        "plan_gate": plan_outcome.gate_result.reason,
        "local_start_count": local_start_count,
        "executor": asdict(follower.executor.counters),
        "events": events,
    }


def stale() -> dict[str, object]:
    config = StagedChaseConfig(
        lead_delay_ms=1000,
        lead_distance_m=0.5,
        mission_timeout_ms=10000,
        plan_validity_ms=10000,
    )
    coordinator = StagedChaseCoordinator(config)
    coordinator.set_carrier_ready(True)
    coordinator.authorize_start()
    coordinator.accept_mini_state(mini_state(0, 1000, 0.0), 1000)
    coordinator.step(1000)
    decision = coordinator.step(1000 + config.state_timeout_ms + 1)
    return {
        "pass": bool(
            decision.phase == ChasePhase.ABORTED
            and decision.abort is not None
            and decision.remote_command.phase == Phase.STOP
            and decision.stop_local_carrier
        ),
        "phase": decision.phase.value,
        "reason": decision.reason,
        "abort": decision.abort.reason.name if decision.abort else None,
    }


def main() -> int:
    args = parse_args()
    summary: dict[str, object] = {"mode": "offline_no_io", "hardware_access": False}
    if args.scenario in {"nominal", "all"}:
        summary["nominal"] = nominal()
    if args.scenario in {"stale", "all"}:
        summary["stale"] = stale()
    summary["pass"] = all(
        item["pass"] for item in summary.values() if isinstance(item, dict) and "pass" in item
    )
    text = json.dumps(summary, indent=2, sort_keys=True)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text + "\n", encoding="utf-8")
    print(text)
    return 0 if summary["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
