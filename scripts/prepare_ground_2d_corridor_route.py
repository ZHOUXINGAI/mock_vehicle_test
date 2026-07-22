#!/usr/bin/env python3

"""Offline review for the first ground 2D CorridorPlan rover route.

This script intentionally does not import ROS, connect to MAVROS, arm PX4, or
publish motor commands. It turns the docking-side route request into a field
fit and bench-prep summary for the rover side.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
from datetime import datetime
from pathlib import Path
from typing import Any


REPO_DIR = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = (
    REPO_DIR / "config/ground_2d/finite_field_slow_carrier_route_2026_06_25.json"
)
DEFAULT_RESULTS_ROOT = REPO_DIR / "results/ground_2d_corridor_preflight"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Review the finite-field slow-carrier route without touching hardware."
    )
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--field-width-m", type=float, default=30.0)
    parser.add_argument("--field-height-m", type=float, default=30.0)
    parser.add_argument("--wheel-track-m", type=float, default=0.0)
    parser.add_argument("--results-root", type=Path, default=DEFAULT_RESULTS_ROOT)
    parser.add_argument("--no-write-results", action="store_true")
    return parser.parse_args()


def load_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        data = json.load(handle)
    if not isinstance(data, dict):
        raise ValueError(f"Config root must be an object: {path}")
    return data


def fmt(value: float, digits: int = 3) -> str:
    return f"{value:.{digits}f}"


def wheel_speeds(v_mps: float, omega_radps: float, track_m: float) -> tuple[float, float] | None:
    if track_m <= 0.0:
        return None
    left = v_mps - omega_radps * track_m / 2.0
    right = v_mps + omega_radps * track_m / 2.0
    return left, right


def build_summary(cfg: dict[str, Any], args: argparse.Namespace) -> tuple[str, list[dict[str, str]]]:
    field = cfg["field"]
    mini = cfg["mini"]
    carrier = cfg["carrier"]
    evidence = cfg["simulation_evidence"]

    bbox_w, bbox_h = [float(x) for x in field["sim_bounding_box_until_first_pass_m"]]
    rec_w, rec_h = [float(x) for x in field["recommended_safety_area_m"]]
    hard_w, hard_h = [float(x) for x in field["hard_demo_limit_m"]]

    radius = float(mini["orbit_radius_m"])
    mini_speed = float(mini["speed_mps"])
    carrier_speed = float(carrier["max_speed_mps"])
    carrier_accel = float(carrier["max_accel_mps2"])
    terminal_path = float(carrier["terminal_path_until_first_pass_m"])
    carrier_arc_len = float(carrier["arc_length_m"])
    carrier_arc_duration = float(carrier["arc_duration_sec"])
    terminal_shape_ok = evidence.get("terminal_corridor_shape_ok")
    terminal_lateral_mode = evidence.get("terminal_lateral_mode", "unknown")
    max_terminal_lateral = evidence.get("max_terminal_corridor_lateral_abs_m")
    max_terminal_heading = evidence.get("max_terminal_heading_error_deg")

    orbit_circ = 2.0 * math.pi * radius
    mini_orbit_time = orbit_circ / mini_speed
    mini_omega = mini_speed / radius
    terminal_time_mini = terminal_path / mini_speed
    terminal_time_carrier_at_max = terminal_path / carrier_speed
    carrier_arc_avg_speed = carrier_arc_len / carrier_arc_duration
    carrier_ramp_time = carrier_speed / carrier_accel

    field_margin_x = (float(args.field_width_m) - bbox_w) / 2.0
    field_margin_y = (float(args.field_height_m) - bbox_h) / 2.0
    rec_margin_x = (rec_w - bbox_w) / 2.0
    rec_margin_y = (rec_h - bbox_h) / 2.0
    hard_margin_x = (hard_w - bbox_w) / 2.0
    hard_margin_y = (hard_h - bbox_h) / 2.0

    target_speed_exceeds_existing_caps = (
        mini_speed > 0.35 or carrier_speed > 0.35
    )

    simulation_evidence_lines = [
        f"Completed: {evidence['completed']}",
        f"First pass time: {fmt(float(evidence['first_pass_time_sec']), 2)}s",
        f"Front violations: {evidence['front_violations_until_first_pass']}",
        f"Minimum terminal distance: {fmt(float(evidence['minimum_terminal_distance_m']))}m",
        f"Terminal lateral mode: {terminal_lateral_mode}",
        f"Terminal corridor shape ok: {terminal_shape_ok}",
    ]
    if max_terminal_lateral is not None:
        simulation_evidence_lines.append(
            f"Max terminal corridor lateral error: {fmt(float(max_terminal_lateral), 4)}m"
        )
    if max_terminal_heading is not None:
        simulation_evidence_lines.append(
            f"Max terminal heading error: {fmt(float(max_terminal_heading), 2)}deg"
        )

    lines = [
        "# Ground 2D Corridor Route Preflight Review",
        "",
        f"Config: {args.config}",
        f"Route: {cfg['name']}",
        f"Source log: {cfg['source']['log']}",
        "",
        "## Field Fit",
        "",
        f"Sim bbox until first pass: {fmt(bbox_w, 2)}m x {fmt(bbox_h, 2)}m",
        f"Requested field: {fmt(args.field_width_m, 2)}m x {fmt(args.field_height_m, 2)}m",
        f"Margin in requested field: x={fmt(field_margin_x, 2)}m y={fmt(field_margin_y, 2)}m",
        f"Margin in recommended 20m x 20m area: x={fmt(rec_margin_x, 2)}m y={fmt(rec_margin_y, 2)}m",
        f"Margin in hard 30m x 30m demo limit: x={fmt(hard_margin_x, 2)}m y={fmt(hard_margin_y, 2)}m",
        f"Field >= recommended safety area: {args.field_width_m >= rec_w and args.field_height_m >= rec_h}",
        f"Route bbox fits requested field: {args.field_width_m >= bbox_w and args.field_height_m >= bbox_h}",
        "",
        "## Route Timing",
        "",
        f"Mini one-lap circle length: {fmt(orbit_circ)}m",
        f"Mini one-lap time at {fmt(mini_speed, 2)}m/s: {fmt(mini_orbit_time)}s",
        f"Mini ccw circle yaw rate target: +{fmt(mini_omega)}rad/s",
        f"Mini terminal time over {fmt(terminal_path)}m at {fmt(mini_speed, 2)}m/s: {fmt(terminal_time_mini)}s",
        f"Carrier arc average speed from sim: {fmt(carrier_arc_avg_speed)}m/s",
        f"Carrier accel ramp time to {fmt(carrier_speed, 2)}m/s at {fmt(carrier_accel, 2)}m/s^2: {fmt(carrier_ramp_time)}s",
        f"Carrier terminal time over {fmt(terminal_path)}m at max {fmt(carrier_speed, 2)}m/s: {fmt(terminal_time_carrier_at_max)}s",
        f"Mini speed > Carrier speed: {mini_speed > carrier_speed}",
        "",
        "## Simulation Evidence",
        "",
        *simulation_evidence_lines,
        "",
        "## Rover Execution Recommendation",
        "",
        "Preferred final interface: continuous route follower with bounded v/omega primitives.",
        "Immediate safe interface: short primitives for bench validation before any ground run.",
        "Do not use a raw full-route ground run before wheel/yaw sign and stop paths are rechecked.",
        "Pose/waypoint-only control is not the first choice because the current proven rover path is BODY_NED velocity primitives.",
        "",
        "## Current Blocking Notes",
        "",
        "- Current proven scripts use conservative speed caps around 0.20-0.35m/s.",
        f"- Requested route targets {fmt(mini_speed, 2)}m/s and {fmt(carrier_speed, 2)}m/s.",
        f"- Target speeds exceed current proven caps: {target_speed_exceeds_existing_caps}",
        "- Actual 0.9/0.7m/s must be bench-calibrated before field use.",
        "- Two rover local frames, start placement, and synchronized start procedure still need to be defined.",
        "- PX4/QGC parameter export is still missing for this active rover baseline.",
        "",
        "## First Bench Checks",
        "",
        "1. Verify RC/QGC/manual stop and physical power cutoff.",
        "2. Verify Arduino timeout brake by removing/invalidating PWM input while wheels are lifted.",
        "3. Verify Mini role wheel response at low speed, then approach 0.9m/s only if stable.",
        "4. Verify Carrier role wheel response at low speed, then approach 0.7m/s only if stable.",
        "5. Verify ccw/left yaw sign; current BODY_NED L-turn baseline used TURN_DIRECTION_SIGN=-1.0 for left.",
        "6. Verify no unexpected reverse, in-place spin, or mode/arm surprise.",
    ]

    primitives = [
        {
            "role": "mini",
            "stage": "bench_ccw_circle",
            "command_type": "v_omega",
            "target_v_mps": fmt(mini_speed),
            "target_omega_radps": fmt(mini_omega),
            "duration_sec": fmt(mini_orbit_time),
            "notes": "one full lifted-wheel equivalent; field run is R=4.5m ccw circle",
        },
        {
            "role": "mini",
            "stage": "terminal_tangent",
            "command_type": "v_omega",
            "target_v_mps": fmt(mini_speed),
            "target_omega_radps": fmt(0.0),
            "duration_sec": fmt(terminal_time_mini),
            "notes": "tangent direction (0.939,-0.345), after full orbit only",
        },
        {
            "role": "carrier",
            "stage": "arc_to_corridor",
            "command_type": "bounded_route_follower",
            "target_v_mps": f"<= {fmt(carrier_speed)}",
            "target_omega_radps": "TBD",
            "duration_sec": fmt(carrier_arc_duration),
            "notes": f"sim arc length {fmt(carrier_arc_len)}m; keep ahead before terminal corridor",
        },
        {
            "role": "carrier",
            "stage": "terminal_corridor",
            "command_type": "v_omega",
            "target_v_mps": f"<= {fmt(carrier_speed)}",
            "target_omega_radps": fmt(0.0),
            "duration_sec": fmt(terminal_time_carrier_at_max),
            "notes": "must remain ahead of Mini; first demo pass distance <0.5m",
        },
    ]

    if args.wheel_track_m > 0.0:
        lines.extend(["", "## Optional Wheel-Track Calculation", ""])
        speeds = wheel_speeds(mini_speed, mini_omega, args.wheel_track_m)
        if speeds is not None:
            left, right = speeds
            lines.append(
                f"Mini ccw circle with track {fmt(args.wheel_track_m)}m: "
                f"left={fmt(left)}m/s right={fmt(right)}m/s"
            )
            lines.append("For ccw/left, right side should spin faster than left side.")

    return "\n".join(lines) + "\n", primitives


def write_outputs(summary: str, primitives: list[dict[str, str]], root: Path) -> Path:
    run_id = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = root / run_id
    out_dir.mkdir(parents=True, exist_ok=False)

    (out_dir / "route_summary.md").write_text(summary, encoding="utf-8")
    with (out_dir / "primitive_plan.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=[
                "role",
                "stage",
                "command_type",
                "target_v_mps",
                "target_omega_radps",
                "duration_sec",
                "notes",
            ],
        )
        writer.writeheader()
        writer.writerows(primitives)

    latest = root / "latest"
    try:
        if latest.is_symlink() or latest.exists():
            latest.unlink()
        os.symlink(out_dir, latest)
    except OSError:
        pass
    return out_dir


def main() -> int:
    args = parse_args()
    cfg = load_json(args.config)
    summary, primitives = build_summary(cfg, args)
    print(summary)

    if args.no_write_results:
        return 0

    out_dir = write_outputs(summary, primitives, args.results_root)
    print(f"Wrote preflight outputs to: {out_dir}")
    print(f"Latest symlink: {args.results_root / 'latest'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
