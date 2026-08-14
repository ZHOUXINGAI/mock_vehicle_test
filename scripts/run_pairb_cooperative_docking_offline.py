#!/usr/bin/env python3

"""Run hardware-free first-stage cooperative docking acceptance scenarios."""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from dataclasses import asdict
from datetime import datetime
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from src.pairb_cooperative_docking import (  # noqa: E402
    CooperationObservation,
    CooperationPhase,
    CooperativePlannerConfig,
    CoordinationMode,
    TemporalCoordinator,
    TemporalCoordinatorConfig,
    build_cooperative_docking_plan,
    build_plan_commands,
    compact_corridor_plan,
    tangent_direction,
)


SCENARIOS = ("nominal", "mini_lag", "persistent_mini_lag", "carrier_lag", "link_loss")


def move_toward(current: float, target: float, maximum_delta: float) -> float:
    if current < target:
        return min(target, current + maximum_delta)
    return max(target, current - maximum_delta)


def trajectory_pose(trajectory, progress_m: float) -> tuple[float, float, float]:
    sample = trajectory.sample(progress_m)
    return sample.x_m, sample.y_m, sample.tangent_yaw_rad


def mini_orbit_pose(config: CooperativePlannerConfig, phase_rad: float) -> tuple[float, float, float]:
    direction = tangent_direction(phase_rad, config.turn_direction)
    return (
        config.orbit_center[0] + config.orbit_radius_m * math.cos(phase_rad),
        config.orbit_center[1] + config.orbit_radius_m * math.sin(phase_rad),
        math.atan2(direction[1], direction[0]),
    )


def scenario_factors(scenario: str, elapsed_s: float) -> tuple[float, float, float]:
    mini_factor = 1.0
    carrier_factor = 1.0
    mini_age_s = 0.05
    if scenario == "mini_lag" and 8.0 <= elapsed_s <= 14.0:
        mini_factor = 0.75
    elif scenario == "persistent_mini_lag" and elapsed_s >= 6.0:
        mini_factor = 0.08
    elif scenario == "carrier_lag" and elapsed_s >= 6.0:
        carrier_factor = 0.35
    elif scenario == "link_loss" and elapsed_s >= 8.0:
        mini_age_s = 2.0
    return mini_factor, carrier_factor, mini_age_s


def run_scenario(
    scenario: str,
    output_dir: Path,
    planner_config: CooperativePlannerConfig,
    dt_s: float = 0.05,
) -> dict:
    output_dir.mkdir(parents=True, exist_ok=True)
    lap_time_s = 2.0 * math.pi * planner_config.orbit_radius_m / planner_config.mini_speed_mps
    mini_phase = 0.0
    preplan_rows: list[dict] = []
    elapsed = 0.0
    sign = 1.0 if planner_config.turn_direction == "ccw" else -1.0
    while elapsed + 1.0e-9 < lap_time_s:
        x_m, y_m, yaw_rad = mini_orbit_pose(planner_config, mini_phase)
        preplan_rows.append(
            {
                "time_s": elapsed,
                "mission_phase": "ORBIT_QUALIFICATION",
                "coordination_mode": "HOLD",
                "carrier_x_m": 0.0,
                "carrier_y_m": 0.0,
                "mini_x_m": x_m,
                "mini_y_m": y_m,
                "carrier_speed_mps": 0.0,
                "mini_speed_mps": planner_config.mini_speed_mps,
                "carrier_speed_limit_mps": 0.0,
                "mini_speed_limit_mps": planner_config.mini_speed_mps,
                "carrier_speed_min_mps": planner_config.carrier_min_speed_mps,
                "carrier_speed_max_mps": planner_config.carrier_max_speed_mps,
                "mini_speed_min_mps": planner_config.mini_min_speed_mps,
                "mini_speed_max_mps": planner_config.mini_max_speed_mps,
                "rendezvous_speed_mps": math.nan,
                "relative_speed_mps": -planner_config.mini_speed_mps,
                "terminal_capture_duration_s": 0.0,
                "terminal_capture_qualified": 0,
                "front_gap_m": math.nan,
                "eta_error_s": math.nan,
                "reason": "mini_full_orbit_pending",
            }
        )
        mini_phase += sign * planner_config.mini_speed_mps / planner_config.orbit_radius_m * dt_s
        elapsed += dt_s

    plan = build_cooperative_docking_plan(
        plan_id=101,
        origin_id=101,
        generated_at_ms=int(round(elapsed * 1000.0)),
        carrier_position=(0.0, 0.0),
        carrier_yaw_rad=0.0,
        mini_phase_rad=mini_phase,
        qualified_orbit_laps=elapsed / lap_time_s,
        config=planner_config,
    )
    compact = compact_corridor_plan(plan, sequence=1, config=planner_config)
    compact.encode()
    coordinator = TemporalCoordinator(
        TemporalCoordinatorConfig(
            carrier_min_speed_mps=planner_config.carrier_min_speed_mps,
            carrier_nominal_speed_mps=planner_config.carrier_nominal_speed_mps,
            carrier_max_speed_mps=planner_config.carrier_max_speed_mps,
            carrier_terminal_min_speed_mps=planner_config.carrier_terminal_min_speed_mps,
            carrier_terminal_max_speed_mps=planner_config.carrier_terminal_max_speed_mps,
            mini_min_speed_mps=planner_config.mini_min_speed_mps,
            mini_nominal_speed_mps=planner_config.mini_speed_mps,
            mini_max_speed_mps=planner_config.mini_max_speed_mps,
            mini_terminal_min_speed_mps=planner_config.mini_terminal_min_speed_mps,
            mini_terminal_max_speed_mps=planner_config.mini_terminal_max_speed_mps,
            minimum_tracking_speed_mps=planner_config.carrier_min_tracking_speed_mps,
            target_front_gap_m=planner_config.target_front_gap_m,
            terminal_lead_time_s=planner_config.terminal_lead_time_s,
        )
    )

    rows = list(preplan_rows)
    runtime_s = 0.0
    carrier_progress = 0.0
    carrier_speed = 0.0
    mini_orbit_remaining = plan.mini_exit_delta_rad * plan.orbit_radius_m
    mini_terminal_progress = 0.0
    mini_speed = planner_config.mini_speed_mps
    approach_length = plan.carrier_approach_length_m
    outcome = "TIMEOUT"
    slowdown_samples = 0
    hold_samples = 0
    abort_reason = None
    carrier_ahead_violations = 0
    mini_cutout_time_s = None
    carrier_terminal_time_s = None
    command_sequence = 10
    max_runtime_s = max(90.0, plan.mini_exit_delay_s + 60.0)
    while runtime_s <= max_runtime_s:
        mini_factor, carrier_factor, mini_age_s = scenario_factors(scenario, runtime_s)
        carrier_terminal_progress = max(0.0, carrier_progress - approach_length)
        carrier_at_terminal = carrier_progress >= approach_length - 1.0e-6
        mini_at_terminal = mini_orbit_remaining <= 1.0e-6
        phase = (
            CooperationPhase.TERMINAL
            if carrier_at_terminal or mini_at_terminal
            else CooperationPhase.APPROACH
        )
        if phase == CooperationPhase.APPROACH:
            carrier_remaining = max(0.0, approach_length - carrier_progress)
            mini_remaining = max(0.0, mini_orbit_remaining)
            front_gap = None
        else:
            carrier_remaining = max(
                0.0, plan.terminal_length_m - carrier_terminal_progress
            )
            mini_remaining = max(
                0.0, plan.mini_terminal_path.length_m - mini_terminal_progress
            )
            mini_corridor_progress = (
                mini_terminal_progress if mini_at_terminal else -mini_orbit_remaining
            )
            front_gap = carrier_terminal_progress - mini_corridor_progress
            if front_gap <= 0.0:
                carrier_ahead_violations += 1
        decision = coordinator.step(
            CooperationObservation(
                now_s=runtime_s,
                phase=phase,
                carrier_remaining_m=carrier_remaining,
                mini_remaining_m=mini_remaining,
                carrier_speed_mps=carrier_speed,
                mini_speed_mps=mini_speed,
                carrier_state_age_s=0.05,
                mini_state_age_s=mini_age_s,
                front_gap_m=front_gap,
            )
        )
        carrier_command, mini_command = build_plan_commands(
            plan,
            decision,
            phase=phase,
            sequence=command_sequence,
            timestamp_ms=int(round((elapsed + runtime_s) * 1000.0)),
            config=planner_config,
        )
        carrier_command.encode()
        mini_command.encode()
        command_sequence += 2
        if decision.mode == CoordinationMode.SLOW_CARRIER:
            slowdown_samples += 1
        elif decision.mode == CoordinationMode.HOLD:
            hold_samples += 1
        elif decision.mode == CoordinationMode.ABORT:
            outcome = "ABORT"
            abort_reason = decision.reason

        if runtime_s < plan.carrier_start_delay_s:
            carrier_target = 0.0
        else:
            carrier_target = decision.carrier_speed_limit_mps * carrier_factor
        mini_target = decision.mini_speed_limit_mps * mini_factor
        carrier_speed = move_toward(
            carrier_speed,
            carrier_target,
            planner_config.carrier_max_accel_mps2 * dt_s,
        )
        mini_speed = move_toward(
            mini_speed,
            mini_target,
            planner_config.mini_max_accel_mps2 * dt_s,
        )
        carrier_progress = min(
            plan.carrier_path.length_m,
            carrier_progress + carrier_speed * dt_s,
        )
        if mini_orbit_remaining > 0.0:
            travelled = min(mini_orbit_remaining, mini_speed * dt_s)
            mini_orbit_remaining -= travelled
            mini_phase += sign * travelled / plan.orbit_radius_m
            if mini_orbit_remaining <= 1.0e-6 and mini_cutout_time_s is None:
                mini_cutout_time_s = runtime_s
        else:
            mini_terminal_progress = min(
                plan.mini_terminal_path.length_m,
                mini_terminal_progress + mini_speed * dt_s,
            )
        if carrier_at_terminal and carrier_terminal_time_s is None:
            carrier_terminal_time_s = runtime_s

        carrier_x, carrier_y, _ = trajectory_pose(plan.carrier_path, carrier_progress)
        if mini_orbit_remaining > 0.0:
            mini_x, mini_y, _ = mini_orbit_pose(planner_config, mini_phase)
        else:
            mini_x, mini_y, _ = trajectory_pose(
                plan.mini_terminal_path, mini_terminal_progress
            )
        rows.append(
            {
                "time_s": elapsed + runtime_s,
                "mission_phase": phase.value,
                "coordination_mode": decision.mode.value,
                "carrier_x_m": carrier_x,
                "carrier_y_m": carrier_y,
                "mini_x_m": mini_x,
                "mini_y_m": mini_y,
                "carrier_speed_mps": carrier_speed,
                "mini_speed_mps": mini_speed,
                "carrier_speed_limit_mps": decision.carrier_speed_limit_mps,
                "mini_speed_limit_mps": decision.mini_speed_limit_mps,
                "carrier_speed_min_mps": planner_config.carrier_min_speed_mps,
                "carrier_speed_max_mps": planner_config.carrier_max_speed_mps,
                "mini_speed_min_mps": planner_config.mini_min_speed_mps,
                "mini_speed_max_mps": planner_config.mini_max_speed_mps,
                "rendezvous_speed_mps": decision.rendezvous_speed_mps,
                "relative_speed_mps": decision.relative_speed_mps,
                "terminal_capture_duration_s": decision.terminal_capture_duration_s,
                "terminal_capture_qualified": int(decision.terminal_capture_qualified),
                "front_gap_m": front_gap if front_gap is not None else math.nan,
                "eta_error_s": decision.eta_error_s,
                "reason": decision.reason,
            }
        )
        if outcome == "ABORT":
            break
        if (
            carrier_progress >= plan.carrier_path.length_m - 0.02
            and mini_terminal_progress >= plan.mini_terminal_path.length_m - 0.02
            and decision.terminal_capture_qualified
        ):
            outcome = "COMPLETE"
            break
        runtime_s += dt_s

    timeline_path = output_dir / "timeline.csv"
    with timeline_path.open("w", newline="", encoding="ascii") as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)

    expected = (
        "ABORT"
        if scenario in {"persistent_mini_lag", "carrier_lag", "link_loss"}
        else "COMPLETE"
    )
    summary = {
        "scenario": scenario,
        "outcome": outcome,
        "expected_outcome": expected,
        "acceptance_pass": outcome == expected,
        "required_orbit_laps": planner_config.required_orbit_laps,
        "qualified_orbit_laps": elapsed / lap_time_s,
        "plan_generated_after_full_lap": elapsed / lap_time_s >= 1.0,
        "plan_id": plan.plan_id,
        "candidate_count": plan.candidate_count,
        "extra_orbits": plan.extra_orbits,
        "carrier_curve_length_m": plan.carrier_curve_metrics.length_m,
        "carrier_curve_max_abs_curvature_inv_m": plan.carrier_curve_metrics.max_abs_curvature_inv_m,
        "carrier_curve_forward_only": plan.carrier_curve_metrics.forward_only,
        "carrier_curve_self_intersects": plan.carrier_curve_metrics.self_intersects,
        "mini_speed_range_mps": list(plan.mini_speed_range_mps),
        "carrier_speed_range_mps": list(plan.carrier_speed_range_mps),
        "terminal_speed_range_mps": list(plan.terminal_speed_range_mps),
        "rendezvous_speed_mps": plan.rendezvous_speed_mps,
        "terminal_capture_required_s": coordinator.config.terminal_capture_hold_s,
        "terminal_capture_qualified": bool(
            rows[-1]["terminal_capture_qualified"]
        ),
        "mini_cutout_time_after_plan_s": mini_cutout_time_s,
        "carrier_terminal_time_after_plan_s": carrier_terminal_time_s,
        "carrier_ahead_violations": carrier_ahead_violations,
        "carrier_slowdown_samples": slowdown_samples,
        "hold_samples": hold_samples,
        "abort_reason": abort_reason,
        "pairb_runtime_types": [
            "MiniState",
            "CorridorPlan",
            "PlanCommand",
            "MissionStatus",
            "Abort",
        ],
        "timeline": str(timeline_path),
    }
    if scenario == "mini_lag":
        summary["acceptance_pass"] = bool(
            summary["acceptance_pass"] and slowdown_samples > 0
        )
    if outcome == "COMPLETE":
        summary["acceptance_pass"] = bool(
            summary["acceptance_pass"] and carrier_ahead_violations == 0
        )
    (output_dir / "summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="ascii"
    )
    plan_metadata = {
        key: value
        for key, value in asdict(plan).items()
        if key not in {"carrier_path", "mini_terminal_path"}
    }
    plan_metadata["terminal_capture_required_s"] = (
        coordinator.config.terminal_capture_hold_s
    )
    plan_metadata["carrier_path"] = [
        [point.x_m, point.y_m, point.requested_speed_mps, point.phase]
        for point in plan.carrier_path.points
    ]
    plan_metadata["mini_terminal_path"] = [
        [point.x_m, point.y_m, point.requested_speed_mps, point.phase]
        for point in plan.mini_terminal_path.points
    ]
    (output_dir / "plan.json").write_text(
        json.dumps(plan_metadata, indent=2, sort_keys=True) + "\n", encoding="ascii"
    )
    save_plots(output_dir, rows, plan, planner_config)
    return summary


def save_plots(output_dir: Path, rows: list[dict], plan, config) -> None:
    try:
        from PIL import Image, ImageDraw
    except ImportError:
        return
    carrier_actual_x = [row["carrier_x_m"] for row in rows]
    carrier_actual_y = [row["carrier_y_m"] for row in rows]
    mini_actual_x = [row["mini_x_m"] for row in rows]
    mini_actual_y = [row["mini_y_m"] for row in rows]
    carrier_plan_x = [point.x_m for point in plan.carrier_path.points]
    carrier_plan_y = [point.y_m for point in plan.carrier_path.points]
    mini_terminal_x = [point.x_m for point in plan.mini_terminal_path.points]
    mini_terminal_y = [point.y_m for point in plan.mini_terminal_path.points]
    theta = [2.0 * math.pi * index / 240 for index in range(241)]
    orbit_x = [config.orbit_center[0] + config.orbit_radius_m * math.cos(value) for value in theta]
    orbit_y = [config.orbit_center[1] + config.orbit_radius_m * math.sin(value) for value in theta]
    width, height, margin = 1100, 850, 70
    all_x = orbit_x + carrier_plan_x + mini_terminal_x + carrier_actual_x + mini_actual_x
    all_y = orbit_y + carrier_plan_y + mini_terminal_y + carrier_actual_y + mini_actual_y
    min_x, max_x = min(all_x), max(all_x)
    min_y, max_y = min(all_y), max(all_y)
    span = max(max_x - min_x, max_y - min_y, 1.0)
    center_x = 0.5 * (min_x + max_x)
    center_y = 0.5 * (min_y + max_y)
    scale = min(width - 2 * margin, height - 2 * margin) / span

    def pixel(point):
        return (
            width / 2 + (point[0] - center_x) * scale,
            height / 2 - (point[1] - center_y) * scale,
        )

    image = Image.new("RGB", (width, height), (28, 30, 34))
    draw = ImageDraw.Draw(image)
    grid_step = max(1.0, 10 ** math.floor(math.log10(span / 8.0)))
    grid_x = math.floor(min_x / grid_step) * grid_step
    while grid_x <= max_x:
        x_pixel = pixel((grid_x, 0.0))[0]
        draw.line((x_pixel, margin, x_pixel, height - margin), fill=(65, 68, 74), width=1)
        grid_x += grid_step
    grid_y = math.floor(min_y / grid_step) * grid_step
    while grid_y <= max_y:
        y_pixel = pixel((0.0, grid_y))[1]
        draw.line((margin, y_pixel, width - margin, y_pixel), fill=(65, 68, 74), width=1)
        grid_y += grid_step

    def line(xs, ys, color, line_width):
        draw.line([pixel(point) for point in zip(xs, ys)], fill=color, width=line_width)

    line(orbit_x, orbit_y, (225, 80, 240), 3)
    line(carrier_plan_x, carrier_plan_y, (0, 190, 255), 5)
    line(mini_terminal_x, mini_terminal_y, (90, 120, 255), 5)
    line(carrier_actual_x, carrier_actual_y, (255, 210, 0), 3)
    line(mini_actual_x, mini_actual_y, (40, 230, 100), 3)
    tangent_pixel = pixel(plan.tangent_point)
    draw.ellipse(
        (
            tangent_pixel[0] - 7,
            tangent_pixel[1] - 7,
            tangent_pixel[0] + 7,
            tangent_pixel[1] + 7,
        ),
        fill=(255, 70, 70),
    )
    draw.text((20, 16), "Pair B cooperative plan / actual", fill=(235, 235, 235))
    draw.text((20, 38), "cyan Carrier plan | yellow Carrier | magenta Mini orbit | green Mini", fill=(220, 220, 220))
    image.save(output_dir / "trajectory.png")

    times = [row["time_s"] for row in rows]
    chart = Image.new("RGB", (1100, 700), (250, 250, 250))
    chart_draw = ImageDraw.Draw(chart)

    def chart_line(values, color, top, bottom, value_min, value_max):
        if value_max <= value_min:
            value_max = value_min + 1.0
        coordinates = []
        for time_s, value in zip(times, values):
            if not math.isfinite(value):
                continue
            x_pixel = margin + (time_s - times[0]) / max(times[-1] - times[0], 1.0) * (1100 - 2 * margin)
            y_pixel = bottom - (value - value_min) / (value_max - value_min) * (bottom - top)
            coordinates.append((x_pixel, y_pixel))
        if len(coordinates) >= 2:
            chart_draw.line(coordinates, fill=color, width=3)

    chart_draw.rectangle((margin, 50, 1030, 310), outline=(100, 100, 100), width=2)
    chart_draw.rectangle((margin, 390, 1030, 650), outline=(100, 100, 100), width=2)
    chart_line([row["carrier_speed_mps"] for row in rows], (230, 170, 0), 50, 310, 0.0, 0.22)
    chart_line([row["carrier_speed_limit_mps"] for row in rows], (0, 130, 220), 50, 310, 0.0, 0.22)
    chart_line([row["mini_speed_mps"] for row in rows], (20, 170, 70), 50, 310, 0.0, 0.22)
    finite_gaps = [row["front_gap_m"] for row in rows if math.isfinite(row["front_gap_m"])]
    gap_max = max([config.target_front_gap_m * 2.0, *finite_gaps], default=1.0)
    chart_line([row["front_gap_m"] for row in rows], (130, 60, 190), 390, 650, 0.0, gap_max)
    target_y = 650 - config.target_front_gap_m / gap_max * (650 - 390)
    chart_draw.line((margin, target_y, 1030, target_y), fill=(70, 70, 70), width=2)
    chart_draw.text((margin, 18), "Speed: yellow Carrier actual | blue envelope | green Mini", fill=(30, 30, 30))
    chart_draw.text((margin, 355), "Carrier-ahead gap: purple | target: gray", fill=(30, 30, 30))
    chart.save(output_dir / "speed_and_gap.png")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Pure-software Pair B cooperative docking acceptance simulation."
    )
    parser.add_argument("--scenario", choices=(*SCENARIOS, "all"), default="all")
    parser.add_argument(
        "--output-dir",
        default="results/pairb_cooperative_docking_offline",
    )
    parser.add_argument("--no-timestamp", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    root = Path(args.output_dir)
    if not args.no_timestamp:
        root /= datetime.now().strftime("%Y%m%d_%H%M%S")
    scenarios = SCENARIOS if args.scenario == "all" else (args.scenario,)
    config = CooperativePlannerConfig()
    summaries = []
    for scenario in scenarios:
        summary = run_scenario(scenario, root / scenario, config)
        summaries.append(summary)
        print(
            f"{scenario}: outcome={summary['outcome']} "
            f"pass={summary['acceptance_pass']} "
            f"slowdown={summary['carrier_slowdown_samples']} "
            f"ahead_violations={summary['carrier_ahead_violations']}"
        )
    aggregate = {
        "all_pass": all(summary["acceptance_pass"] for summary in summaries),
        "scenarios": summaries,
    }
    root.mkdir(parents=True, exist_ok=True)
    (root / "acceptance.json").write_text(
        json.dumps(aggregate, indent=2, sort_keys=True) + "\n", encoding="ascii"
    )
    print(f"artifacts={root}")
    return 0 if aggregate["all_pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
