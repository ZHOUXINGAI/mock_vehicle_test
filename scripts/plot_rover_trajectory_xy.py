#!/usr/bin/env python3

"""Render planned and actual rover trajectories in a plan-aligned XY frame."""

from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from src.rover_rviz_trajectory import TracePoint, trace_to_plan_frame


def load_trace(path: Path) -> tuple[TracePoint, ...]:
    with path.open(newline="", encoding="ascii") as stream:
        rows = tuple(csv.DictReader(stream))
    points = tuple(
        TracePoint(float(row["x_m"]), float(row["y_m"]), float(row["yaw_rad"]))
        for row in rows
    )
    if not points:
        raise ValueError(f"trajectory is empty: {path}")
    return points


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("planned_csv", type=Path)
    parser.add_argument("actual_csv", type=Path)
    parser.add_argument("output_png", type=Path)
    args = parser.parse_args()

    planned = load_trace(args.planned_csv)
    actual = load_trace(args.actual_csv)
    if len(planned) < 2:
        raise SystemExit("planned trajectory needs at least two points")
    plan_yaw = math.atan2(
        planned[1].y_m - planned[0].y_m,
        planned[1].x_m - planned[0].x_m,
    )
    planned_xy = trace_to_plan_frame(planned, planned[0], plan_yaw)
    actual_xy = trace_to_plan_frame(actual, planned[0], plan_yaw)

    from PIL import Image, ImageDraw

    width, height = 1600, 900
    margin_left, margin_right, margin_top, margin_bottom = 120, 60, 80, 100
    all_points = planned_xy + actual_xy
    x_min = min(point.x_m for point in all_points)
    x_max = max(point.x_m for point in all_points)
    y_min = min(point.y_m for point in all_points)
    y_max = max(point.y_m for point in all_points)
    x_padding = max(0.5, 0.05 * max(1.0, x_max - x_min))
    y_padding = max(0.5, 0.10 * max(1.0, y_max - y_min))
    x_min -= x_padding
    x_max += x_padding
    y_min -= y_padding
    y_max += y_padding
    plot_width = width - margin_left - margin_right
    plot_height = height - margin_top - margin_bottom
    scale = min(plot_width / (x_max - x_min), plot_height / (y_max - y_min))

    def pixel(point: TracePoint) -> tuple[int, int]:
        return (
            round(margin_left + (point.x_m - x_min) * scale),
            round(height - margin_bottom - (point.y_m - y_min) * scale),
        )

    image = Image.new("RGB", (width, height), "#f7f8fa")
    draw = ImageDraw.Draw(image)
    grid_color = "#d7dce2"
    for x_tick in range(math.floor(x_min), math.ceil(x_max) + 1):
        px, _ = pixel(TracePoint(float(x_tick), 0.0, 0.0))
        draw.line((px, margin_top, px, height - margin_bottom), fill=grid_color, width=1)
        draw.text((px - 8, height - margin_bottom + 14), str(x_tick), fill="#4a5159")
    for y_tick in range(math.floor(y_min), math.ceil(y_max) + 1):
        _, py = pixel(TracePoint(0.0, float(y_tick), 0.0))
        draw.line((margin_left, py, width - margin_right, py), fill=grid_color, width=1)
        draw.text((margin_left - 48, py - 7), str(y_tick), fill="#4a5159")
    zero_left = pixel(TracePoint(x_min, 0.0, 0.0))
    zero_right = pixel(TracePoint(x_max, 0.0, 0.0))
    draw.line((*zero_left, *zero_right), fill="#69727d", width=2)
    draw.line([pixel(point) for point in planned_xy], fill="#00aee8", width=6)
    draw.line([pixel(point) for point in actual_xy], fill="#e5ad00", width=6)
    end_pixel = pixel(actual_xy[-1])
    draw.ellipse(
        (end_pixel[0] - 8, end_pixel[1] - 8, end_pixel[0] + 8, end_pixel[1] + 8),
        fill="#d43d3d",
    )
    end_dx_m = actual_xy[-1].x_m - planned_xy[-1].x_m
    end_dy_m = actual_xy[-1].y_m - planned_xy[-1].y_m
    draw.text((margin_left, 28), "Rover Offboard trajectory tracking", fill="#20252b")
    draw.text((width // 2 - 90, height - 42), "Plan X / along-track (m)", fill="#20252b")
    draw.text((18, 42), "Plan Y / cross-track (m, left positive)", fill="#20252b")
    draw.line((width - 440, 36, width - 380, 36), fill="#00aee8", width=6)
    draw.text((width - 368, 28), "Planned", fill="#20252b")
    draw.line((width - 260, 36, width - 200, 36), fill="#e5ad00", width=6)
    draw.text((width - 188, 28), "Actual", fill="#20252b")
    draw.text(
        (max(margin_left, end_pixel[0] - 210), max(margin_top, end_pixel[1] - 28)),
        f"end offset dx={end_dx_m:+.2f}, dy={end_dy_m:+.2f} m",
        fill="#a52626",
    )
    args.output_png.parent.mkdir(parents=True, exist_ok=True)
    image.save(args.output_png, format="PNG")
    print(
        f"TRAJECTORY_PLOT output={args.output_png} "
        f"end=({actual_xy[-1].x_m:.3f},{actual_xy[-1].y_m:+.3f}) "
        f"offset=({end_dx_m:+.3f},{end_dy_m:+.3f})",
        flush=True,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
