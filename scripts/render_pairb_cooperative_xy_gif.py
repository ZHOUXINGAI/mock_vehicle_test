#!/usr/bin/env python3

"""Render a time-correct XY GIF from one cooperative docking result log."""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from scripts.run_pairb_virtual_mini_hil_rviz import load_replay_bundle  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--replay-dir", required=True)
    parser.add_argument("--output", default="")
    parser.add_argument("--speedup", type=float, default=4.0)
    parser.add_argument("--fps", type=float, default=12.0)
    parser.add_argument("--max-frames", type=int, default=600)
    parser.add_argument("--width", type=int, default=900)
    parser.add_argument("--height", type=int, default=700)
    return parser.parse_args()


def _load_font(size: int):
    from PIL import ImageFont

    path = Path("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf")
    if path.is_file():
        return ImageFont.truetype(str(path), size)
    return ImageFont.load_default()


def _sample_indices(
    rows, speedup: float, fps: float, max_frames: int
) -> tuple[list[int], int]:
    duration_s = rows[-1].time_s - rows[0].time_s
    # GIF stores frame delays in 10 ms units. Select the representable delay
    # first, then derive frame count so requested log-time speed remains true.
    frame_duration_ms = max(20, round((1000.0 / fps) / 10.0) * 10)
    requested_intervals = max(
        1, round(duration_s / (speedup * frame_duration_ms / 1000.0))
    )
    frame_count = min(requested_intervals + 1, max_frames)
    if frame_count == max_frames and requested_intervals + 1 > max_frames:
        frame_duration_ms = max(
            20,
            round(
                duration_s / speedup / (frame_count - 1) * 1000.0 / 10.0
            )
            * 10,
        )
    indices = []
    cursor = 0
    for frame in range(frame_count):
        source_time = rows[0].time_s + frame * duration_s / (frame_count - 1)
        while cursor + 1 < len(rows) and rows[cursor + 1].time_s <= source_time:
            cursor += 1
        indices.append(cursor)
    return indices, frame_duration_ms


def render_gif(
    replay_dir: Path,
    output: Path,
    *,
    speedup: float,
    fps: float,
    max_frames: int,
    width: int,
    height: int,
) -> dict:
    from PIL import Image, ImageDraw

    if not math.isfinite(speedup) or not 0.25 <= speedup <= 20.0:
        raise ValueError("speedup must be within [0.25, 20]")
    if not math.isfinite(fps) or not 5.0 <= fps <= 30.0:
        raise ValueError("fps must be within [5, 30]")
    if not 2 <= max_frames <= 2000:
        raise ValueError("max_frames must be within [2, 2000]")
    if width < 640 or height < 480:
        raise ValueError("GIF dimensions are too small")

    bundle = load_replay_bundle(replay_dir)
    rows = bundle.rows
    indices, frame_duration_ms = _sample_indices(rows, speedup, fps, max_frames)
    plot_left, plot_top, plot_right, plot_bottom = 70, 80, width - 40, height - 145
    all_x = [point.x_m for point in bundle.carrier_plan]
    all_x.extend(point.x_m for point in bundle.mini_plan)
    all_x.extend(row.carrier_x_m for row in rows)
    all_x.extend(row.mini_x_m for row in rows)
    all_y = [point.y_m for point in bundle.carrier_plan]
    all_y.extend(point.y_m for point in bundle.mini_plan)
    all_y.extend(row.carrier_y_m for row in rows)
    all_y.extend(row.mini_y_m for row in rows)
    min_x, max_x = min(all_x), max(all_x)
    min_y, max_y = min(all_y), max(all_y)
    span_x = max(max_x - min_x, 1.0)
    span_y = max(max_y - min_y, 1.0)
    scale = min(
        (plot_right - plot_left) / (span_x * 1.15),
        (plot_bottom - plot_top) / (span_y * 1.15),
    )
    center_x = 0.5 * (min_x + max_x)
    center_y = 0.5 * (min_y + max_y)

    def pixel(x_m: float, y_m: float) -> tuple[int, int]:
        return (
            round(0.5 * (plot_left + plot_right) + (x_m - center_x) * scale),
            round(0.5 * (plot_top + plot_bottom) - (y_m - center_y) * scale),
        )

    background = Image.new("RGB", (width, height), (28, 30, 34))
    base = ImageDraw.Draw(background)
    title_font = _load_font(22)
    text_font = _load_font(16)
    small_font = _load_font(14)
    base.text((28, 20), "Pair B cooperative docking | XY replay", fill=(242, 242, 242), font=title_font)
    base.text(
        (28, 50),
        f"{speedup:.1f}x log time | cyan/magenta plan | yellow/green execution",
        fill=(200, 204, 210),
        font=small_font,
    )
    grid_step = max(0.5, 10 ** math.floor(math.log10(max(span_x, span_y) / 6.0)))
    x_value = math.floor(min_x / grid_step) * grid_step
    while x_value <= max_x + 1.0e-9:
        x_pixel = pixel(x_value, center_y)[0]
        base.line((x_pixel, plot_top, x_pixel, plot_bottom), fill=(64, 68, 74), width=1)
        x_value += grid_step
    y_value = math.floor(min_y / grid_step) * grid_step
    while y_value <= max_y + 1.0e-9:
        y_pixel = pixel(center_x, y_value)[1]
        base.line((plot_left, y_pixel, plot_right, y_pixel), fill=(64, 68, 74), width=1)
        y_value += grid_step
    base.rectangle((plot_left, plot_top, plot_right, plot_bottom), outline=(110, 114, 122), width=2)
    base.line(
        [pixel(point.x_m, point.y_m) for point in bundle.carrier_plan],
        fill=(0, 155, 215),
        width=3,
    )
    base.line(
        [pixel(point.x_m, point.y_m) for point in bundle.mini_plan],
        fill=(185, 55, 210),
        width=3,
    )

    frames = []
    carrier_pixels: list[tuple[int, int]] = []
    mini_pixels: list[tuple[int, int]] = []
    previous_index = -1
    for frame_number, row_index in enumerate(indices):
        for row in rows[previous_index + 1 : row_index + 1]:
            carrier_pixels.append(pixel(row.carrier_x_m, row.carrier_y_m))
            mini_pixels.append(pixel(row.mini_x_m, row.mini_y_m))
        previous_index = row_index
        row = rows[row_index]
        image = background.copy()
        draw = ImageDraw.Draw(image)
        if len(carrier_pixels) >= 2:
            draw.line(carrier_pixels, fill=(255, 205, 0), width=5)
        if len(mini_pixels) >= 2:
            draw.line(mini_pixels, fill=(35, 230, 105), width=5)
        carrier_now = carrier_pixels[-1]
        mini_now = mini_pixels[-1]
        draw.ellipse(
            (carrier_now[0] - 6, carrier_now[1] - 6, carrier_now[0] + 6, carrier_now[1] + 6),
            fill=(255, 225, 40),
        )
        draw.ellipse(
            (mini_now[0] - 6, mini_now[1] - 6, mini_now[0] + 6, mini_now[1] + 6),
            fill=(60, 255, 125),
        )
        elapsed = row.time_s - rows[0].time_s
        gap = "n/a" if not math.isfinite(row.front_gap_m) else f"{row.front_gap_m:.2f} m"
        capture = (
            "PASS"
            if row.terminal_capture_qualified
            else f"{row.terminal_capture_duration_s:.1f}/{bundle.terminal_capture_required_s:.1f} s"
        )
        draw.rectangle((20, height - 125, width - 20, height - 18), fill=(38, 41, 47), outline=(90, 94, 102), width=1)
        draw.text(
            (34, height - 114),
            f"t={elapsed:6.1f}s  phase={row.mission_phase}  coordination={row.coordination_mode}",
            fill=(242, 242, 242),
            font=text_font,
        )
        draw.text(
            (34, height - 83),
            f"Carrier {row.carrier_speed_mps:.3f} m/s   Mini {row.mini_speed_mps:.3f} m/s   dv={row.relative_speed_mps:+.3f} m/s",
            fill=(220, 224, 230),
            font=small_font,
        )
        draw.text(
            (34, height - 55),
            f"front gap={gap}   rendezvous={bundle.rendezvous_speed_mps:.3f} m/s   capture={capture}   {row.reason}",
            fill=(220, 224, 230),
            font=small_font,
        )
        frames.append(image.quantize(colors=96))
        if frame_number and frame_number % 100 == 0:
            print(f"rendered {frame_number}/{len(indices)} frames")

    output.parent.mkdir(parents=True, exist_ok=True)
    frames[0].save(
        output,
        save_all=True,
        append_images=frames[1:],
        duration=frame_duration_ms,
        loop=0,
        disposal=2,
        optimize=False,
    )
    return {
        "output": str(output),
        "frames": len(frames),
        "source_duration_s": rows[-1].time_s - rows[0].time_s,
        "gif_duration_s": (len(frames) - 1) * frame_duration_ms / 1000.0,
        "effective_speedup": (rows[-1].time_s - rows[0].time_s)
        / ((len(frames) - 1) * frame_duration_ms / 1000.0),
        "effective_fps": 1000.0 / frame_duration_ms,
    }


def main() -> int:
    args = parse_args()
    replay_dir = Path(args.replay_dir)
    output = Path(args.output) if args.output else replay_dir / f"trajectory_xy_{args.speedup:g}x.gif"
    summary = render_gif(
        replay_dir,
        output,
        speedup=args.speedup,
        fps=args.fps,
        max_frames=args.max_frames,
        width=args.width,
        height=args.height,
    )
    print(
        f"GIF_COMPLETE output={summary['output']} frames={summary['frames']} "
        f"source={summary['source_duration_s']:.2f}s gif={summary['gif_duration_s']:.2f}s "
        f"speedup={summary['effective_speedup']:.3f}x"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
