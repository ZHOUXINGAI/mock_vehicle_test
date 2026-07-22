#!/usr/bin/env python3

# Local execution copy from:
#   /home/jetson/easydocking/scripts/run_ground_2d_corridor_sim.py
# Keep this file close to upstream unless rover-field execution requires a
# clearly documented adaptation.

import argparse
import csv
import math
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import List, Tuple


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def wrap_pi(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def norm2(vector: Tuple[float, float]) -> float:
    return math.hypot(vector[0], vector[1])


def unit(vector: Tuple[float, float]) -> Tuple[float, float]:
    length = norm2(vector)
    if length < 1e-9:
        return (1.0, 0.0)
    return (vector[0] / length, vector[1] / length)


def dot(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return a[0] * b[0] + a[1] * b[1]


@dataclass
class CorridorPlan2D:
    orbit_center: Tuple[float, float]
    orbit_radius: float
    tangent_point: Tuple[float, float]
    tangent_dir: Tuple[float, float]
    arc_center: Tuple[float, float]
    arc_radius: float
    arc_phi_start: float
    arc_delta_phi: float
    arc_length: float
    mini_arrival_delay: float
    carrier_arc_duration: float
    carrier_arc_speed: float
    trigger_phase: float


def compute_corridor_plan(
    carrier_pos: Tuple[float, float],
    mini_phase: float,
    orbit_center: Tuple[float, float],
    orbit_radius: float,
    mini_speed: float,
    carrier_speed_max: float,
    carrier_speed_min: float,
    terminal_lead_time: float,
    turn_direction: str,
) -> CorridorPlan2D:
    cx, cy = carrier_pos
    ox, oy = orbit_center
    oc = (cx - ox, cy - oy)
    distance_oc = norm2(oc)
    if distance_oc <= orbit_radius * 1.001:
        raise ValueError("carrier must start outside the mini orbit circle")

    sign = 1.0 if turn_direction == "ccw" else -1.0

    def tangent_direction(theta: float) -> Tuple[float, float]:
        return (-sign * math.sin(theta), sign * math.cos(theta))

    alpha = math.atan2(oc[1], oc[0])
    beta = math.asin(orbit_radius / distance_oc)
    theta_1 = alpha + beta
    theta_2 = alpha - beta
    t1 = (ox + orbit_radius * math.cos(theta_1), oy + orbit_radius * math.sin(theta_1))
    t2 = (ox + orbit_radius * math.cos(theta_2), oy + orbit_radius * math.sin(theta_2))
    score_1 = dot(unit((t1[0] - cx, t1[1] - cy)), tangent_direction(theta_1))
    score_2 = dot(unit((t2[0] - cx, t2[1] - cy)), tangent_direction(theta_2))
    theta_t, tangent_point = (theta_1, t1) if score_1 >= score_2 else (theta_2, t2)
    tangent_dir = tangent_direction(theta_t)

    delta_theta = sign * (theta_t - mini_phase)
    while delta_theta <= 0.0:
        delta_theta += 2.0 * math.pi
    omega = mini_speed / orbit_radius
    mini_arrival_delay = max(delta_theta / omega, 0.1)

    # Same geometric construction as docking_controller.cpp:
    # arc center lies on O->T and makes |C-M_arc| == |T-M_arc|.
    tx, ty = tangent_point
    ot = (tx - ox, ty - oy)
    cos_alpha = dot(oc, ot) / (distance_oc * orbit_radius)
    numerator = distance_oc * distance_oc - orbit_radius * orbit_radius
    denominator = 2.0 * orbit_radius * (distance_oc * cos_alpha - orbit_radius)
    k = numerator / denominator if abs(denominator) > 0.01 else 2.0
    arc_center = (ox + k * ot[0], oy + k * ot[1])
    arc_radius = abs(k - 1.0) * orbit_radius
    arc_phi_start = math.atan2(cy - arc_center[1], cx - arc_center[0])
    arc_phi_end = math.atan2(ty - arc_center[1], tx - arc_center[0])
    arc_delta_phi = wrap_pi(arc_phi_end - arc_phi_start)
    arc_length = arc_radius * abs(arc_delta_phi)

    min_duration = arc_length / max(carrier_speed_max, 1e-3)
    max_duration = arc_length / max(carrier_speed_min, 1e-3)
    carrier_arc_duration = clamp(
        mini_arrival_delay - terminal_lead_time,
        min_duration,
        max_duration,
    )
    carrier_arc_speed = arc_length / max(carrier_arc_duration, 1e-3)
    trigger_phase = theta_t % (2.0 * math.pi)
    return CorridorPlan2D(
        orbit_center=orbit_center,
        orbit_radius=orbit_radius,
        tangent_point=tangent_point,
        tangent_dir=tangent_dir,
        arc_center=arc_center,
        arc_radius=arc_radius,
        arc_phi_start=arc_phi_start,
        arc_delta_phi=arc_delta_phi,
        arc_length=arc_length,
        mini_arrival_delay=mini_arrival_delay,
        carrier_arc_duration=carrier_arc_duration,
        carrier_arc_speed=carrier_arc_speed,
        trigger_phase=trigger_phase,
    )


def simulate(args: argparse.Namespace) -> Tuple[Path, dict]:
    output_dir = Path(args.output_dir)
    if args.timestamp:
        output_dir = output_dir / datetime.now().strftime("%Y%m%d_%H%M%S_ground_2d")
    output_dir.mkdir(parents=True, exist_ok=True)

    dt = args.dt
    sign = 1.0 if args.turn_direction == "ccw" else -1.0
    orbit_center = (args.orbit_center_x, args.orbit_center_y)
    mini_phase = math.radians(args.mini_start_phase_deg)
    carrier_pos = [args.carrier_x, args.carrier_y]
    mini_pos = [
        orbit_center[0] + args.orbit_radius * math.cos(mini_phase),
        orbit_center[1] + args.orbit_radius * math.sin(mini_phase),
    ]
    carrier_vel = [0.0, 0.0]
    mini_vel = [0.0, 0.0]
    rows: List[dict] = []
    plan = None
    start_time = args.required_laps * 2.0 * math.pi * args.orbit_radius / args.mini_speed
    arc_started_at = None
    terminal_started_at = None
    completed = False
    min_distance = math.inf
    min_terminal_distance = math.inf
    front_violations = 0
    front_violations_until_first_pass = 0
    front_samples = 0
    front_samples_until_first_pass = 0
    max_carrier_accel = 0.0
    path_after_terminal_start = 0.0
    path_until_first_pass = math.nan
    first_pass_time = math.nan
    previous_terminal_carrier_pos = None
    max_terminal_corridor_lateral_abs = 0.0
    max_terminal_heading_error_deg = 0.0
    min_x = min(carrier_pos[0], mini_pos[0])
    max_x = max(carrier_pos[0], mini_pos[0])
    min_y = min(carrier_pos[1], mini_pos[1])
    max_y = max(carrier_pos[1], mini_pos[1])
    min_x_until_first_pass = min_x
    max_x_until_first_pass = max_x
    min_y_until_first_pass = min_y
    max_y_until_first_pass = max_y

    total_duration = args.duration
    step_count = int(total_duration / dt)
    for step in range(step_count + 1):
        time_s = step * dt
        if plan is None and time_s >= start_time:
            plan = compute_corridor_plan(
                tuple(carrier_pos),
                mini_phase,
                orbit_center,
                args.orbit_radius,
                args.mini_speed,
                args.carrier_max_speed,
                args.carrier_min_speed,
                args.terminal_lead_time,
                args.turn_direction,
            )
            arc_started_at = time_s

        if plan is not None and arc_started_at is not None:
            plan_time = time_s - arc_started_at
        else:
            plan_time = 0.0

        mini_glide = plan is not None and plan_time >= plan.mini_arrival_delay
        if mini_glide:
            mini_vel = [
                plan.tangent_dir[0] * args.mini_speed,
                plan.tangent_dir[1] * args.mini_speed,
            ]
            mini_pos[0] += mini_vel[0] * dt
            mini_pos[1] += mini_vel[1] * dt
        else:
            mini_phase += sign * (args.mini_speed / args.orbit_radius) * dt
            mini_pos = [
                orbit_center[0] + args.orbit_radius * math.cos(mini_phase),
                orbit_center[1] + args.orbit_radius * math.sin(mini_phase),
            ]
            mini_vel = [
                -sign * math.sin(mini_phase) * args.mini_speed,
                sign * math.cos(mini_phase) * args.mini_speed,
            ]

        if plan is None:
            carrier_cmd = (0.0, 0.0)
        else:
            if plan_time <= plan.carrier_arc_duration:
                tau = clamp(plan_time / max(plan.carrier_arc_duration, 1e-3), 0.0, 1.0)
                phi = plan.arc_phi_start + plan.arc_delta_phi * tau
                target = (
                    plan.arc_center[0] + plan.arc_radius * math.cos(phi),
                    plan.arc_center[1] + plan.arc_radius * math.sin(phi),
                )
                to_target = (target[0] - carrier_pos[0], target[1] - carrier_pos[1])
                direction = unit(to_target)
                speed = clamp(1.20 * norm2(to_target) + 0.30, 0.0, args.carrier_max_speed)
                carrier_cmd = (direction[0] * speed, direction[1] * speed)
            else:
                if terminal_started_at is None:
                    terminal_started_at = time_s
                    previous_terminal_carrier_pos = tuple(carrier_pos)
                tangent = plan.tangent_dir
                lateral = (-tangent[1], tangent[0])
                rel = (carrier_pos[0] - mini_pos[0], carrier_pos[1] - mini_pos[1])
                front_gap = dot(rel, tangent)
                rel_vel = (carrier_vel[0] - mini_vel[0], carrier_vel[1] - mini_vel[1])
                front_rate = dot(rel_vel, tangent)
                target_front_gap = args.target_front_gap
                tangent_speed = args.mini_speed + clamp(
                    args.front_kp * (target_front_gap - front_gap) - args.front_kd * front_rate,
                    -args.carrier_speed_delta,
                    args.carrier_speed_delta,
                )
                if front_gap < args.front_guard_gap:
                    tangent_speed = max(tangent_speed, args.mini_speed + args.front_guard_boost)
                lateral_speed = 0.0
                if args.terminal_lateral_mode == "chase":
                    lateral_gap = dot(rel, lateral)
                    lateral_rate = dot(rel_vel, lateral)
                    lateral_speed = clamp(
                        -args.lateral_kp * lateral_gap - args.lateral_kd * lateral_rate,
                        -args.lateral_speed_limit,
                        args.lateral_speed_limit,
                    )
                carrier_cmd = (
                    tangent[0] * tangent_speed + lateral[0] * lateral_speed,
                    tangent[1] * tangent_speed + lateral[1] * lateral_speed,
                )

        cmd_speed = norm2(carrier_cmd)
        if cmd_speed > args.carrier_max_speed:
            scale = args.carrier_max_speed / cmd_speed
            carrier_cmd = (carrier_cmd[0] * scale, carrier_cmd[1] * scale)

        delta_velocity = (
            carrier_cmd[0] - carrier_vel[0],
            carrier_cmd[1] - carrier_vel[1],
        )
        delta_speed = norm2(delta_velocity)
        max_delta_speed = args.carrier_max_accel * dt
        if delta_speed > max_delta_speed and delta_speed > 1.0e-12:
            scale = max_delta_speed / delta_speed
            delta_velocity = (
                delta_velocity[0] * scale,
                delta_velocity[1] * scale,
            )
        carrier_vel[0] += delta_velocity[0]
        carrier_vel[1] += delta_velocity[1]
        carrier_pos[0] += carrier_vel[0] * dt
        carrier_pos[1] += carrier_vel[1] * dt

        carrier_speed = norm2(tuple(carrier_vel))
        carrier_accel = norm2(delta_velocity) / max(dt, 1e-9)
        max_carrier_accel = max(max_carrier_accel, carrier_accel)

        if terminal_started_at is not None and previous_terminal_carrier_pos is not None:
            path_after_terminal_start += norm2(
                (
                    carrier_pos[0] - previous_terminal_carrier_pos[0],
                    carrier_pos[1] - previous_terminal_carrier_pos[1],
                )
            )
            previous_terminal_carrier_pos = tuple(carrier_pos)

        rel = (carrier_pos[0] - mini_pos[0], carrier_pos[1] - mini_pos[1])
        distance = norm2(rel)
        min_x = min(min_x, carrier_pos[0], mini_pos[0])
        max_x = max(max_x, carrier_pos[0], mini_pos[0])
        min_y = min(min_y, carrier_pos[1], mini_pos[1])
        max_y = max(max_y, carrier_pos[1], mini_pos[1])
        if not completed:
            min_x_until_first_pass = min(min_x_until_first_pass, carrier_pos[0], mini_pos[0])
            max_x_until_first_pass = max(max_x_until_first_pass, carrier_pos[0], mini_pos[0])
            min_y_until_first_pass = min(min_y_until_first_pass, carrier_pos[1], mini_pos[1])
            max_y_until_first_pass = max(max_y_until_first_pass, carrier_pos[1], mini_pos[1])
        min_distance = min(min_distance, distance)
        if terminal_started_at is not None:
            min_terminal_distance = min(min_terminal_distance, distance)
        front_gap = math.nan
        lateral_gap = math.nan
        carrier_corridor_lateral_error = math.nan
        carrier_heading_error_deg = math.nan
        if plan is not None:
            tangent = plan.tangent_dir
            lateral = (-tangent[1], tangent[0])
            front_gap = dot(rel, tangent)
            lateral_gap = dot(rel, lateral)
            if terminal_started_at is not None:
                rel_to_tangent_line = (
                    carrier_pos[0] - plan.tangent_point[0],
                    carrier_pos[1] - plan.tangent_point[1],
                )
                carrier_corridor_lateral_error = dot(rel_to_tangent_line, lateral)
                max_terminal_corridor_lateral_abs = max(
                    max_terminal_corridor_lateral_abs,
                    abs(carrier_corridor_lateral_error),
                )
                if carrier_speed > 0.05:
                    heading_dot = clamp(dot(unit(tuple(carrier_vel)), tangent), -1.0, 1.0)
                    carrier_heading_error_deg = math.degrees(math.acos(heading_dot))
                    max_terminal_heading_error_deg = max(
                        max_terminal_heading_error_deg,
                        abs(carrier_heading_error_deg),
                    )
            if distance < args.front_check_distance:
                front_samples += 1
                if front_gap < 0.0:
                    front_violations += 1
                if not completed:
                    front_samples_until_first_pass += 1
                    if front_gap < 0.0:
                        front_violations_until_first_pass += 1

        if (
            terminal_started_at is not None
            and distance <= args.pass_distance
            and front_gap >= 0.0
            and abs(lateral_gap) <= args.pass_lateral_abs
        ):
            if not completed:
                path_until_first_pass = path_after_terminal_start
                first_pass_time = time_s
            completed = True

        rows.append(
            {
                "t": time_s,
                "phase": "IDLE" if plan is None else ("ORBIT_TO_T" if not mini_glide else "TERMINAL"),
                "carrier_x": carrier_pos[0],
                "carrier_y": carrier_pos[1],
                "carrier_vx": carrier_vel[0],
                "carrier_vy": carrier_vel[1],
                "mini_x": mini_pos[0],
                "mini_y": mini_pos[1],
                "mini_vx": mini_vel[0],
                "mini_vy": mini_vel[1],
                "distance": distance,
                "front_gap": front_gap,
                "lateral_gap": lateral_gap,
                "carrier_corridor_lateral_error": carrier_corridor_lateral_error,
                "carrier_heading_error_deg": carrier_heading_error_deg,
                "mini_phase_rad": mini_phase % (2.0 * math.pi),
            }
        )
        if completed and args.stop_on_pass:
            break

    csv_path = output_dir / "ground_2d_log.csv"
    with csv_path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    summary = {
        "completed": completed,
        "min_distance_m": min_distance,
        "min_terminal_distance_m": min_terminal_distance,
        "front_violations": front_violations,
        "front_samples": front_samples,
        "front_violations_until_first_pass": front_violations_until_first_pass,
        "front_samples_until_first_pass": front_samples_until_first_pass,
        "terminal_path_length_m": path_after_terminal_start,
        "terminal_path_until_first_pass_m": path_until_first_pass,
        "first_pass_time_sec": first_pass_time,
        "max_carrier_accel_mps2": max_carrier_accel,
        "bbox_min_x_m": min_x,
        "bbox_max_x_m": max_x,
        "bbox_min_y_m": min_y,
        "bbox_max_y_m": max_y,
        "bbox_width_m": max_x - min_x,
        "bbox_height_m": max_y - min_y,
        "bbox_min_x_until_first_pass_m": min_x_until_first_pass,
        "bbox_max_x_until_first_pass_m": max_x_until_first_pass,
        "bbox_min_y_until_first_pass_m": min_y_until_first_pass,
        "bbox_max_y_until_first_pass_m": max_y_until_first_pass,
        "bbox_width_until_first_pass_m": max_x_until_first_pass - min_x_until_first_pass,
        "bbox_height_until_first_pass_m": max_y_until_first_pass - min_y_until_first_pass,
        "max_terminal_corridor_lateral_abs_m": max_terminal_corridor_lateral_abs,
        "max_terminal_heading_error_deg": max_terminal_heading_error_deg,
        "terminal_corridor_shape_ok": (
            max_terminal_corridor_lateral_abs <= args.max_terminal_corridor_lateral_abs
            and max_terminal_heading_error_deg <= args.max_terminal_heading_error_deg
        ),
        "required_laps": args.required_laps,
        "mini_speed_mps": args.mini_speed,
        "carrier_max_speed_mps": args.carrier_max_speed,
        "terminal_lateral_mode": args.terminal_lateral_mode,
    }
    if plan is not None:
        summary.update(
            {
                "tangent_point_x": plan.tangent_point[0],
                "tangent_point_y": plan.tangent_point[1],
                "tangent_dir_x": plan.tangent_dir[0],
                "tangent_dir_y": plan.tangent_dir[1],
                "arc_length_m": plan.arc_length,
                "carrier_arc_duration_sec": plan.carrier_arc_duration,
                "mini_arrival_delay_sec": plan.mini_arrival_delay,
                "trigger_phase_deg": math.degrees(plan.trigger_phase),
            }
        )
    with (output_dir / "summary.txt").open("w") as stream:
        for key, value in summary.items():
            stream.write(f"{key}={value}\n")

    if not args.no_plots:
        save_plots(output_dir, rows, plan, orbit_center, args.orbit_radius)
    return output_dir, summary


def save_plots(
    output_dir: Path,
    rows: List[dict],
    plan: CorridorPlan2D,
    orbit_center: Tuple[float, float],
    orbit_radius: float,
) -> None:
    try:
        import matplotlib.pyplot as plt
    except Exception:
        return

    carrier_x = [row["carrier_x"] for row in rows]
    carrier_y = [row["carrier_y"] for row in rows]
    mini_x = [row["mini_x"] for row in rows]
    mini_y = [row["mini_y"] for row in rows]
    times = [row["t"] for row in rows]
    carrier_speed = [math.hypot(row["carrier_vx"], row["carrier_vy"]) for row in rows]
    mini_speed = [math.hypot(row["mini_vx"], row["mini_vy"]) for row in rows]
    distance = [row["distance"] for row in rows]
    front_gap = [row["front_gap"] for row in rows]
    corridor_lateral_error = [row["carrier_corridor_lateral_error"] for row in rows]

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.plot(carrier_x, carrier_y, "g-", label="Carrier rover")
    ax.plot(mini_x, mini_y, "r-", label="Mini rover")
    theta = [2.0 * math.pi * i / 240 for i in range(241)]
    ax.plot(
        [orbit_center[0] + orbit_radius * math.cos(value) for value in theta],
        [orbit_center[1] + orbit_radius * math.sin(value) for value in theta],
        "r--",
        alpha=0.35,
        label="Mini orbit",
    )
    if plan is not None:
        ax.scatter([plan.tangent_point[0]], [plan.tangent_point[1]], c="m", marker="x", s=80, label="T")
        ax.arrow(
            plan.tangent_point[0],
            plan.tangent_point[1],
            plan.tangent_dir[0] * 3.0,
            plan.tangent_dir[1] * 3.0,
            color="m",
            width=0.025,
            length_includes_head=True,
            alpha=0.8,
        )
        ax.plot(
            [
                plan.tangent_point[0],
                plan.tangent_point[0] + plan.tangent_dir[0] * 10.0,
            ],
            [
                plan.tangent_point[1],
                plan.tangent_point[1] + plan.tangent_dir[1] * 10.0,
            ],
            "m:",
            alpha=0.8,
            label="terminal tangent corridor",
        )
        phis = [
            plan.arc_phi_start + plan.arc_delta_phi * i / 120
            for i in range(121)
        ]
        ax.plot(
            [plan.arc_center[0] + plan.arc_radius * math.cos(value) for value in phis],
            [plan.arc_center[1] + plan.arc_radius * math.sin(value) for value in phis],
            "m--",
            alpha=0.7,
            label="CorridorPlan arc",
        )
    ax.axis("equal")
    ax.grid(True, alpha=0.3)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("Ground 2D CorridorPlan")
    ax.legend()
    fig.tight_layout()
    fig.savefig(output_dir / "trajectory_xy.png", dpi=160)
    plt.close(fig)

    fig, axes = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
    axes[0].plot(times, carrier_speed, "g-", label="Carrier")
    axes[0].plot(times, mini_speed, "r-", label="Mini")
    axes[0].set_ylabel("speed [m/s]")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend()
    axes[1].plot(times, distance, "k-")
    axes[1].set_ylabel("distance [m]")
    axes[1].grid(True, alpha=0.3)
    axes[2].plot(times, front_gap, "b-")
    axes[2].axhline(0.0, color="r", linestyle="--", linewidth=1.0)
    axes[2].set_ylabel("front gap [m]")
    axes[2].set_xlabel("time [s]")
    axes[2].grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(output_dir / "speed_distance_front.png", dpi=160)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(9, 4.5))
    ax.plot(times, carrier_speed, "g-", label="Carrier")
    ax.plot(times, mini_speed, "r-", label="Mini")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("speed [m/s]")
    ax.set_title("Speed Profile")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(output_dir / "speed_profile.png", dpi=160)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(9, 4.5))
    ax.plot(times, distance, "k-")
    ax.axhline(0.5, color="tab:orange", linestyle="--", linewidth=1.0, label="0.5m pass target")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("distance [m]")
    ax.set_title("Distance Convergence")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(output_dir / "distance_convergence.png", dpi=160)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(9, 4.5))
    ax.plot(times, front_gap, "b-")
    ax.axhline(0.0, color="r", linestyle="--", linewidth=1.0, label="front violation boundary")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("front gap [m]")
    ax.set_title("Carrier Front Gap")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(output_dir / "front_gap.png", dpi=160)
    plt.close(fig)

    lateral_gap = [row["lateral_gap"] for row in rows]
    fig, ax = plt.subplots(figsize=(9, 4.5))
    ax.plot(times, lateral_gap, "tab:purple")
    ax.axhline(0.0, color="k", linestyle="--", linewidth=1.0)
    ax.set_xlabel("time [s]")
    ax.set_ylabel("lateral gap [m]")
    ax.set_title("Corridor Lateral Gap")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(output_dir / "lateral_gap.png", dpi=160)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(9, 4.5))
    ax.plot(times, corridor_lateral_error, "tab:cyan")
    ax.axhline(0.0, color="k", linestyle="--", linewidth=1.0)
    ax.set_xlabel("time [s]")
    ax.set_ylabel("carrier lateral error [m]")
    ax.set_title("Carrier Error From Tangent Corridor")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(output_dir / "carrier_corridor_lateral_error.png", dpi=160)
    plt.close(fig)

    phase_to_code = {"IDLE": 0, "ORBIT_TO_T": 1, "TERMINAL": 2}
    phase_codes = [phase_to_code.get(str(row["phase"]), -1) for row in rows]
    fig, ax = plt.subplots(figsize=(9, 3.8))
    ax.step(times, phase_codes, where="post")
    ax.set_yticks([0, 1, 2])
    ax.set_yticklabels(["IDLE", "ORBIT_TO_T", "TERMINAL"])
    ax.set_xlabel("time [s]")
    ax.set_title("Phase Timeline")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(output_dir / "phase_timeline.png", dpi=160)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.plot(carrier_x, carrier_y, "g-", label="Carrier rover")
    ax.plot(mini_x, mini_y, "r-", label="Mini rover")
    ax.scatter([carrier_x[0]], [carrier_y[0]], c="g", marker="o", s=50, label="Carrier start")
    ax.scatter([mini_x[0]], [mini_y[0]], c="r", marker="o", s=50, label="Mini start")
    ax.scatter([carrier_x[-1]], [carrier_y[-1]], c="g", marker="s", s=45, label="Carrier final")
    ax.scatter([mini_x[-1]], [mini_y[-1]], c="r", marker="s", s=45, label="Mini final")
    ax.plot(
        [orbit_center[0] + orbit_radius * math.cos(value) for value in theta],
        [orbit_center[1] + orbit_radius * math.sin(value) for value in theta],
        "r--",
        alpha=0.35,
        label="Mini orbit",
    )
    if plan is not None:
        ax.scatter([plan.tangent_point[0]], [plan.tangent_point[1]], c="m", marker="x", s=80, label="T")
        ax.arrow(
            plan.tangent_point[0],
            plan.tangent_point[1],
            plan.tangent_dir[0] * 3.0,
            plan.tangent_dir[1] * 3.0,
            color="m",
            width=0.025,
            length_includes_head=True,
            alpha=0.8,
        )
        ax.plot(
            [
                plan.tangent_point[0],
                plan.tangent_point[0] + plan.tangent_dir[0] * 10.0,
            ],
            [
                plan.tangent_point[1],
                plan.tangent_point[1] + plan.tangent_dir[1] * 10.0,
            ],
            "m:",
            alpha=0.8,
            label="terminal tangent corridor",
        )
        ax.plot(
            [plan.arc_center[0] + plan.arc_radius * math.cos(value) for value in phis],
            [plan.arc_center[1] + plan.arc_radius * math.sin(value) for value in phis],
            "m--",
            alpha=0.7,
            label="CorridorPlan arc",
        )
    ax.axis("equal")
    ax.grid(True, alpha=0.3)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("Ground 2D CorridorPlan Full Trajectory")
    ax.legend(loc="best", fontsize=8)
    fig.tight_layout()
    fig.savefig(output_dir / "trajectory_xy_full.png", dpi=160)
    plt.close(fig)

    artifacts = [
        "ground_2d_log.csv",
        "summary.txt",
        "trajectory_xy.png",
        "trajectory_xy_full.png",
        "speed_profile.png",
        "distance_convergence.png",
        "front_gap.png",
        "lateral_gap.png",
        "carrier_corridor_lateral_error.png",
        "phase_timeline.png",
        "speed_distance_front.png",
    ]
    with (output_dir / "artifacts.txt").open("w") as stream:
        for name in artifacts:
            stream.write(f"{name}\n")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run a 2D ground CorridorPlan simulation.")
    parser.add_argument("--output-dir", default="results/ground_2d")
    parser.add_argument("--timestamp", action="store_true", default=True)
    parser.add_argument("--duration", type=float, default=90.0)
    parser.add_argument("--dt", type=float, default=0.02)
    parser.add_argument("--orbit-center-x", type=float, default=0.0)
    parser.add_argument("--orbit-center-y", type=float, default=0.0)
    parser.add_argument("--orbit-radius", type=float, default=4.5)
    parser.add_argument("--mini-speed", type=float, default=0.9)
    parser.add_argument("--mini-start-phase-deg", type=float, default=315.0)
    parser.add_argument("--required-laps", type=float, default=1.0)
    parser.add_argument("--turn-direction", choices=["ccw", "cw"], default="ccw")
    parser.add_argument("--carrier-x", type=float, default=-7.0)
    parser.add_argument("--carrier-y", type=float, default=-6.0)
    parser.add_argument("--carrier-max-speed", type=float, default=0.70)
    parser.add_argument("--carrier-min-speed", type=float, default=0.35)
    parser.add_argument("--carrier-max-accel", type=float, default=0.30)
    parser.add_argument("--terminal-lead-time", type=float, default=0.8)
    parser.add_argument("--target-front-gap", type=float, default=0.35)
    parser.add_argument("--front-guard-gap", type=float, default=0.12)
    parser.add_argument("--front-guard-boost", type=float, default=0.35)
    parser.add_argument("--front-kp", type=float, default=0.65)
    parser.add_argument("--front-kd", type=float, default=0.20)
    parser.add_argument("--carrier-speed-delta", type=float, default=0.45)
    parser.add_argument(
        "--terminal-lateral-mode",
        choices=["straight-corridor", "chase"],
        default="straight-corridor",
        help="Use the planned tangent corridor by default; chase mode reproduces the old lateral PD preview.",
    )
    parser.add_argument("--lateral-kp", type=float, default=0.85)
    parser.add_argument("--lateral-kd", type=float, default=0.25)
    parser.add_argument("--lateral-speed-limit", type=float, default=0.45)
    parser.add_argument("--max-terminal-corridor-lateral-abs", type=float, default=0.20)
    parser.add_argument("--max-terminal-heading-error-deg", type=float, default=10.0)
    parser.add_argument("--front-check-distance", type=float, default=3.0)
    parser.add_argument("--pass-distance", type=float, default=0.50)
    parser.add_argument("--pass-lateral-abs", type=float, default=0.25)
    parser.add_argument("--stop-on-pass", action="store_true", default=True)
    parser.add_argument("--continue-after-pass", action="store_false", dest="stop_on_pass")
    parser.add_argument("--no-plots", action="store_true", default=False)
    return parser.parse_args()


def main() -> None:
    output_dir, summary = simulate(parse_args())
    print(f"output_dir={output_dir}")
    for key, value in summary.items():
        print(f"{key}={value}")


if __name__ == "__main__":
    main()
