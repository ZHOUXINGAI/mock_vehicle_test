#!/usr/bin/env python3
"""Offline analysis for paired LR24 Pair B no-motion benchmark artifacts."""

from __future__ import annotations

import csv
import json
import math
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable


SCHEMA_VERSION = "lr24-pairb-benchmark-report-v1"
UINT32_MASK = 0xFFFFFFFF
UINT32_HALF = 0x80000000

CARRIER_CSV_FIELDS = (
    "role", "event", "mono_ms", "seq", "phase", "stale_ms",
    "x_m", "y_m", "v_mps", "omega_radps",
)
MINI_CSV_FIELDS = (
    "role", "event", "mono_ms", "seq", "phase",
    "x_m", "y_m", "v_mps", "omega_radps",
)
ALLOWED_EVENTS = {
    "carrier": {"rx_state", "tx_corridor_plan", "tx_command"},
    "mini": {"tx_state", "rx_corridor_plan", "rx_command"},
}
SUMMARY_KEYS = {
    "carrier": {
        "states_rx", "state_seq_gaps", "commands_tx",
        "corridor_plans_tx", "field_origins_tx",
    },
    "mini": {
        "states_tx", "commands_rx", "command_seq_gaps",
        "corridor_plans_rx", "corridor_plan_seq_gaps",
        "rejected", "aborts_rx",
    },
}
ENDPOINT_RE = re.compile(r"(\d+)\.(\d+)->(\d+)\.(\d+)(?=\s|$)")


class ArtifactError(ValueError):
    """An input artifact or contract cannot be parsed safely."""


@dataclass(frozen=True)
class Endpoint:
    source_system: int
    source_component: int
    target_system: int
    target_component: int

    @classmethod
    def parse(cls, line: str) -> "Endpoint | None":
        match = ENDPOINT_RE.search(line)
        if not match:
            return None
        return cls(*(int(value) for value in match.groups()))

    def __str__(self) -> str:
        return (
            f"{self.source_system}.{self.source_component}->"
            f"{self.target_system}.{self.target_component}"
        )

    def as_dict(self) -> dict[str, int | str]:
        return {
            "text": str(self),
            "source_system": self.source_system,
            "source_component": self.source_component,
            "target_system": self.target_system,
            "target_component": self.target_component,
        }


@dataclass(frozen=True)
class Event:
    role: str
    event: str
    mono_ms: int
    seq: int
    phase: str
    stale_ms: float | None
    v_mps: float | None
    omega_radps: float | None


@dataclass(frozen=True)
class RunArtifacts:
    role: str
    log_path: Path
    csv_path: Path
    endpoint: Endpoint
    summary: dict[str, int]
    events: tuple[Event, ...]


@dataclass(frozen=True)
class Contract:
    path: Path
    carrier_endpoint: Endpoint
    mini_endpoint: Endpoint
    state_stale_ms: int
    command_watchdog_ms: int


def load_contract(path: str | Path) -> Contract:
    contract_path = Path(path)
    try:
        data = json.loads(contract_path.read_text(encoding="utf-8"))
        if data["contract"] != "lr24-pairb-v1":
            raise ArtifactError("unsupported Pair B contract")
        carrier = data["orin1_carrier"]
        mini = data["orin2_mini"]
        timeouts = data["timeouts_ms"]
        return Contract(
            path=contract_path,
            carrier_endpoint=Endpoint(
                int(carrier["mav_sys_id"]),
                int(carrier["mav_component_id"]),
                int(mini["mav_sys_id"]),
                int(mini["mav_component_id"]),
            ),
            mini_endpoint=Endpoint(
                int(mini["mav_sys_id"]),
                int(mini["mav_component_id"]),
                int(carrier["mav_sys_id"]),
                int(carrier["mav_component_id"]),
            ),
            state_stale_ms=int(timeouts["mini_state_stale"]),
            command_watchdog_ms=int(timeouts["mini_command_watchdog"]),
        )
    except ArtifactError:
        raise
    except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
        raise ArtifactError(f"cannot load contract {contract_path}: {exc}") from exc


def _uint32(value: str, field: str, path: Path, line: int) -> int:
    try:
        parsed = int(value)
    except ValueError as exc:
        raise ArtifactError(f"{path}:{line}: invalid {field}={value!r}") from exc
    if not 0 <= parsed <= UINT32_MASK:
        raise ArtifactError(f"{path}:{line}: {field} outside uint32 range")
    return parsed


def _optional_float(
    value: str | None, field: str, path: Path, line: int
) -> float | None:
    if value in (None, ""):
        return None
    try:
        parsed = float(value)
    except ValueError as exc:
        raise ArtifactError(f"{path}:{line}: invalid {field}={value!r}") from exc
    if not math.isfinite(parsed):
        raise ArtifactError(f"{path}:{line}: non-finite {field}")
    return parsed


def _read_csv(path: Path, role: str) -> tuple[Event, ...]:
    expected = CARRIER_CSV_FIELDS if role == "carrier" else MINI_CSV_FIELDS
    events: list[Event] = []
    try:
        with path.open(newline="", encoding="utf-8-sig") as handle:
            reader = csv.DictReader(handle)
            actual = tuple(reader.fieldnames or ())
            if actual != expected:
                raise ArtifactError(
                    f"{path}: CSV schema mismatch; expected {expected}, got {actual}"
                )
            for line, row in enumerate(reader, start=2):
                if None in row:
                    raise ArtifactError(f"{path}:{line}: too many CSV columns")
                if row["role"] != role:
                    raise ArtifactError(
                        f"{path}:{line}: expected role {role!r}, got {row['role']!r}"
                    )
                event_name = row["event"]
                if event_name not in ALLOWED_EVENTS[role]:
                    raise ArtifactError(
                        f"{path}:{line}: unsupported event {event_name!r}"
                    )
                for field in ("x_m", "y_m"):
                    _optional_float(row.get(field), field, path, line)
                phase = row["phase"].strip().upper()
                v_mps = _optional_float(row.get("v_mps"), "v_mps", path, line)
                omega = _optional_float(
                    row.get("omega_radps"), "omega_radps", path, line
                )
                if event_name in {"tx_command", "rx_command"} and (
                    not phase or v_mps is None or omega is None
                ):
                    raise ArtifactError(
                        f"{path}:{line}: command requires phase, v_mps and omega_radps"
                    )
                events.append(
                    Event(
                        role=role,
                        event=event_name,
                        mono_ms=_uint32(row["mono_ms"], "mono_ms", path, line),
                        seq=_uint32(row["seq"], "seq", path, line),
                        phase=phase,
                        stale_ms=_optional_float(
                            row.get("stale_ms"), "stale_ms", path, line
                        ),
                        v_mps=v_mps,
                        omega_radps=omega,
                    )
                )
    except ArtifactError:
        raise
    except OSError as exc:
        raise ArtifactError(f"cannot read CSV {path}: {exc}") from exc
    return tuple(events)


def _read_log(path: Path, role: str) -> tuple[Endpoint, dict[str, int]]:
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except (OSError, UnicodeError) as exc:
        raise ArtifactError(f"cannot read log {path}: {exc}") from exc
    endpoints = [value for line in lines if (value := Endpoint.parse(line))]
    if not endpoints:
        raise ArtifactError(f"{path}: dry-run transport endpoint not found")
    if len(set(endpoints)) != 1:
        raise ArtifactError(f"{path}: multiple different transport endpoints found")
    prefix = f"{role} summary "
    summaries = [line for line in lines if line.startswith(prefix)]
    if not summaries:
        raise ArtifactError(f"{path}: {role} summary not found")
    summary = {
        key: int(value)
        for key, value in re.findall(r"(\w+)=(-?\d+)", summaries[-1])
    }
    missing = SUMMARY_KEYS[role] - summary.keys()
    if missing:
        raise ArtifactError(f"{path}: summary missing {sorted(missing)}")
    if any(summary[key] < 0 for key in SUMMARY_KEYS[role]):
        raise ArtifactError(f"{path}: summary contains a negative count")
    return endpoints[-1], summary


def load_run(
    log_path: str | Path, csv_path: str | Path, role: str
) -> RunArtifacts:
    if role not in {"carrier", "mini"}:
        raise ArtifactError(f"unsupported role {role!r}")
    log = Path(log_path)
    csv_file = Path(csv_path)
    endpoint, summary = _read_log(log, role)
    return RunArtifacts(
        role, log, csv_file, endpoint, summary, _read_csv(csv_file, role)
    )


def _events(run: RunArtifacts, name: str) -> list[Event]:
    return [event for event in run.events if event.event == name]


def sequence_metrics(events: Iterable[Event]) -> dict[str, int | None]:
    ordered = list(events)
    missing = duplicates = out_of_order = reversals = 0
    max_local_gap = 0
    for previous, current in zip(ordered, ordered[1:]):
        seq_delta = (current.seq - previous.seq) & UINT32_MASK
        if seq_delta == 0:
            duplicates += 1
        elif seq_delta < UINT32_HALF:
            missing += seq_delta - 1
        else:
            out_of_order += 1
        time_delta = (current.mono_ms - previous.mono_ms) & UINT32_MASK
        if time_delta >= UINT32_HALF:
            reversals += 1
        else:
            max_local_gap = max(max_local_gap, time_delta)
    return {
        "count": len(ordered),
        "unique_count": len({event.seq for event in ordered}),
        "first_seq": ordered[0].seq if ordered else None,
        "last_seq": ordered[-1].seq if ordered else None,
        "missing": missing,
        "duplicates": duplicates,
        "out_of_order": out_of_order,
        "max_local_gap_ms": max_local_gap if len(ordered) >= 2 else None,
        "local_time_reversals": reversals,
    }


def paired_coverage(
    tx_events: Iterable[Event], rx_events: Iterable[Event]
) -> dict[str, int | float]:
    transmitted = list(tx_events)
    received = list(rx_events)
    if not transmitted or not received:
        return {
            "eligible_tx": 0,
            "unique_rx": len({event.seq for event in received}),
            "matched": 0,
            "unexpected_rx": len({event.seq for event in received}),
            "coverage": 0.0,
        }
    base = received[0].seq
    rx_offsets = {
        delta
        for event in received
        if (delta := (event.seq - base) & UINT32_MASK) < UINT32_HALF
    }
    span = max(rx_offsets)
    tx_offsets = {
        delta
        for event in transmitted
        if (delta := (event.seq - base) & UINT32_MASK) <= span
    }
    matched = len(tx_offsets & rx_offsets)
    return {
        "eligible_tx": len(tx_offsets),
        "unique_rx": len(rx_offsets),
        "matched": matched,
        "unexpected_rx": len(rx_offsets - tx_offsets),
        "coverage": matched / len(tx_offsets) if tx_offsets else 0.0,
    }


def _sequence_ok(metrics: dict[str, int | None]) -> bool:
    return all(
        metrics[key] == 0
        for key in (
            "missing", "duplicates", "out_of_order", "local_time_reversals"
        )
    )


def _broadcast_endpoint_allowed(actual: Endpoint, directed: Endpoint) -> bool:
    return (
        actual.source_system == directed.source_system
        and actual.source_component == directed.source_component
        and (
            (
                actual.target_system == directed.target_system
                and actual.target_component == directed.target_component
            )
            or (actual.target_system == 0 and actual.target_component == 0)
        )
    )


def analyze_pair(
    *,
    carrier_log: str | Path,
    carrier_csv: str | Path,
    mini_log: str | Path,
    mini_csv: str | Path,
    contract: Contract,
    mode: str = "targeted",
    min_state_coverage: float = 0.95,
    min_command_coverage: float = 0.95,
    min_plan_coverage: float = 0.95,
    state_stale_ms: int | None = None,
    command_watchdog_ms: int | None = None,
) -> dict[str, Any]:
    if mode not in {"targeted", "broadcast"}:
        raise ArtifactError(f"unsupported mode {mode!r}")
    for name, value in (
        ("min_state_coverage", min_state_coverage),
        ("min_command_coverage", min_command_coverage),
        ("min_plan_coverage", min_plan_coverage),
    ):
        if not 0.0 <= value <= 1.0:
            raise ArtifactError(f"{name} must be in [0, 1]")
    stale_limit = (
        contract.state_stale_ms if state_stale_ms is None else state_stale_ms
    )
    watchdog_limit = (
        contract.command_watchdog_ms
        if command_watchdog_ms is None
        else command_watchdog_ms
    )
    if stale_limit <= 0 or watchdog_limit <= 0:
        raise ArtifactError("timeout thresholds must be positive")

    carrier = load_run(carrier_log, carrier_csv, "carrier")
    mini = load_run(mini_log, mini_csv, "mini")
    carrier_states = _events(carrier, "rx_state")
    mini_states = _events(mini, "tx_state")
    carrier_commands = _events(carrier, "tx_command")
    mini_commands = _events(mini, "rx_command")
    carrier_plans = _events(carrier, "tx_corridor_plan")
    mini_plans = _events(mini, "rx_corridor_plan")

    state_tx = sequence_metrics(mini_states)
    state_rx = sequence_metrics(carrier_states)
    command_tx = sequence_metrics(carrier_commands)
    command_rx = sequence_metrics(mini_commands)
    plan_tx = sequence_metrics(carrier_plans)
    plan_rx = sequence_metrics(mini_plans)
    state_coverage = paired_coverage(mini_states, carrier_states)
    command_coverage = paired_coverage(carrier_commands, mini_commands)
    plan_coverage = paired_coverage(carrier_plans, mini_plans)

    stale_values = [
        event.stale_ms
        for event in carrier_commands
        if event.stale_ms is not None
    ]
    max_reported_stale = max(stale_values) if stale_values else None
    missing_stale_after_state = 0
    if carrier_states:
        first_state = carrier_states[0].mono_ms
        for event in carrier_commands:
            delta = (event.mono_ms - first_state) & UINT32_MASK
            if delta < UINT32_HALF and event.stale_ms is None:
                missing_stale_after_state += 1

    command_violations = []
    for side, events in (
        ("carrier_tx", carrier_commands),
        ("mini_rx", mini_commands),
    ):
        for event in events:
            if (
                event.phase != "HOLD"
                or abs(event.v_mps or 0.0) > 1.0e-6
                or abs(event.omega_radps or 0.0) > 1.0e-6
            ):
                command_violations.append(
                    {
                        "side": side,
                        "seq": event.seq,
                        "phase": event.phase,
                        "v_mps": event.v_mps,
                        "omega_radps": event.omega_radps,
                    }
                )

    checks: list[dict[str, Any]] = []

    def check(code: str, passed: bool, message: str, evidence: Any) -> None:
        checks.append(
            {
                "code": code,
                "pass": bool(passed),
                "message": message,
                "evidence": evidence,
            }
        )

    if mode == "targeted":
        check(
            "carrier_endpoint_targeted",
            carrier.endpoint == contract.carrier_endpoint,
            "Carrier endpoint must be directed to Mini.",
            {
                "actual": str(carrier.endpoint),
                "expected": str(contract.carrier_endpoint),
            },
        )
        check(
            "mini_endpoint_targeted",
            mini.endpoint == contract.mini_endpoint,
            "Mini endpoint must be directed to Carrier.",
            {
                "actual": str(mini.endpoint),
                "expected": str(contract.mini_endpoint),
            },
        )
    else:
        check(
            "carrier_endpoint_broadcast_allowed",
            _broadcast_endpoint_allowed(
                carrier.endpoint, contract.carrier_endpoint
            ),
            "Broadcast diagnostics allow the directed target or 0.0.",
            {"actual": str(carrier.endpoint)},
        )
        check(
            "mini_endpoint_broadcast_allowed",
            _broadcast_endpoint_allowed(mini.endpoint, contract.mini_endpoint),
            "Broadcast diagnostics allow the directed target or 0.0.",
            {"actual": str(mini.endpoint)},
        )
        check(
            "broadcast_diagnostic_only",
            True,
            "Broadcast mode never proves directed acceptance.",
            {"directed_acceptance": False},
        )

    check(
        "carrier_summary_consistent",
        carrier.summary["states_rx"] == len(carrier_states)
        and carrier.summary["commands_tx"] == len(carrier_commands)
        and carrier.summary["corridor_plans_tx"] == len(carrier_plans),
        "Carrier log summary must match Carrier CSV counts.",
        {"summary": carrier.summary},
    )
    check(
        "mini_summary_consistent",
        mini.summary["states_tx"] == len(mini_states)
        and mini.summary["commands_rx"] == len(mini_commands)
        and mini.summary["corridor_plans_rx"] == len(mini_plans),
        "Mini log summary must match Mini CSV counts.",
        {"summary": mini.summary},
    )
    check(
        "summary_sequence_gaps_consistent",
        carrier.summary["state_seq_gaps"] == state_rx["missing"]
        and mini.summary["command_seq_gaps"] == command_rx["missing"]
        and mini.summary["corridor_plan_seq_gaps"] == plan_rx["missing"],
        "Log sequence-gap summaries must match modular CSV analysis.",
        {
            "carrier_state": [
                carrier.summary["state_seq_gaps"], state_rx["missing"]
            ],
            "mini_command": [
                mini.summary["command_seq_gaps"], command_rx["missing"]
            ],
            "mini_plan": [
                mini.summary["corridor_plan_seq_gaps"], plan_rx["missing"]
            ],
        },
    )
    check(
        "mini_state_present",
        bool(mini_states) and bool(carrier_states),
        "MiniState must be transmitted by Mini and received by Carrier.",
        {"tx": len(mini_states), "rx": len(carrier_states)},
    )
    check(
        "mini_state_sequence",
        _sequence_ok(state_tx) and _sequence_ok(state_rx),
        "MiniState sequences must be contiguous and locally time ordered.",
        {"tx": state_tx, "rx": state_rx},
    )
    check(
        "mini_state_coverage",
        state_coverage["coverage"] >= min_state_coverage
        and state_coverage["unexpected_rx"] == 0,
        "Carrier must receive the required MiniState sequence coverage.",
        {"threshold": min_state_coverage, **state_coverage},
    )
    check(
        "state_stale_timeout",
        bool(carrier_states)
        and (
            state_rx["max_local_gap_ms"] is None
            or state_rx["max_local_gap_ms"] <= stale_limit
        )
        and (
            max_reported_stale is None
            or max_reported_stale <= stale_limit
        )
        and missing_stale_after_state == 0,
        "Carrier-local state gaps and reported stale age must stay in limit.",
        {
            "limit_ms": stale_limit,
            "max_rx_gap_ms": state_rx["max_local_gap_ms"],
            "max_reported_stale_ms": max_reported_stale,
            "missing_stale_after_first_state": missing_stale_after_state,
        },
    )
    check(
        "hold_zero_command_present",
        bool(carrier_commands) and bool(mini_commands),
        "Carrier must transmit and Mini must receive PlanCommand frames.",
        {"tx": len(carrier_commands), "rx": len(mini_commands)},
    )
    check(
        "hold_zero_command",
        not command_violations,
        "Every command must be zero-speed HOLD.",
        {
            "violations": command_violations[:20],
            "violation_count": len(command_violations),
        },
    )
    check(
        "plan_command_sequence",
        _sequence_ok(command_tx) and _sequence_ok(command_rx),
        "PlanCommand sequences must be contiguous and locally time ordered.",
        {"tx": command_tx, "rx": command_rx},
    )
    check(
        "plan_command_coverage",
        command_coverage["coverage"] >= min_command_coverage
        and command_coverage["unexpected_rx"] == 0,
        "Mini must receive the required PlanCommand sequence coverage.",
        {"threshold": min_command_coverage, **command_coverage},
    )
    check(
        "mini_watchdog_timeout",
        bool(mini_commands)
        and (
            command_rx["max_local_gap_ms"] is None
            or command_rx["max_local_gap_ms"] <= watchdog_limit
        ),
        "Mini-local command gaps must stay within the watchdog limit.",
        {
            "limit_ms": watchdog_limit,
            "max_rx_gap_ms": command_rx["max_local_gap_ms"],
        },
    )
    check(
        "corridor_plan_present",
        bool(carrier_plans) and bool(mini_plans),
        "Carrier must transmit and Mini must receive CorridorPlan frames.",
        {"tx": len(carrier_plans), "rx": len(mini_plans)},
    )
    check(
        "corridor_plan_sequence",
        _sequence_ok(plan_tx) and _sequence_ok(plan_rx),
        "CorridorPlan sequences must be contiguous and locally time ordered.",
        {"tx": plan_tx, "rx": plan_rx},
    )
    check(
        "corridor_plan_coverage",
        plan_coverage["coverage"] >= min_plan_coverage
        and plan_coverage["unexpected_rx"] == 0,
        "Mini must receive the required CorridorPlan sequence coverage.",
        {"threshold": min_plan_coverage, **plan_coverage},
    )
    check(
        "field_origin_sent",
        carrier.summary["field_origins_tx"] > 0,
        "Carrier must report at least one transmitted FieldOrigin.",
        {"field_origins_tx": carrier.summary["field_origins_tx"]},
    )
    check(
        "mini_gate_rejects",
        mini.summary["rejected"] == 0,
        "Mini command gate must report zero rejects.",
        {"rejected": mini.summary["rejected"]},
    )
    check(
        "mini_abort",
        mini.summary["aborts_rx"] == 0,
        "Mini must not receive Abort during the HOLD benchmark.",
        {"aborts_rx": mini.summary["aborts_rx"]},
    )

    failures = [item for item in checks if not item["pass"]]
    passed = not failures
    return {
        "schema_version": SCHEMA_VERSION,
        "inputs": {
            "config": str(contract.path.resolve()),
            "carrier_log": str(carrier.log_path.resolve()),
            "carrier_csv": str(carrier.csv_path.resolve()),
            "mini_log": str(mini.log_path.resolve()),
            "mini_csv": str(mini.csv_path.resolve()),
        },
        "mode": mode,
        "diagnostic_only": mode == "broadcast",
        "directed_acceptance": mode == "targeted" and passed,
        "endpoints": {
            "carrier": carrier.endpoint.as_dict(),
            "mini": mini.endpoint.as_dict(),
        },
        "metrics": {
            "carrier": {
                "summary": carrier.summary,
                "rx_state": state_rx,
                "tx_command": command_tx,
                "tx_corridor_plan": plan_tx,
                "max_reported_state_stale_ms": max_reported_stale,
            },
            "mini": {
                "summary": mini.summary,
                "tx_state": state_tx,
                "rx_command": command_rx,
                "rx_corridor_plan": plan_rx,
            },
            "paired": {
                "mini_state": state_coverage,
                "plan_command": command_coverage,
                "corridor_plan": plan_coverage,
            },
        },
        "checks": checks,
        "failures": failures,
        "pass": passed,
    }
