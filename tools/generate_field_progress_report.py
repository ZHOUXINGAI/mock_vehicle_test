#!/usr/bin/env python3

from __future__ import annotations

import csv
import html
import math
from pathlib import Path
import re

from PIL import Image, ImageDraw, ImageFont


REPO = Path(__file__).resolve().parents[1]
DOCS = REPO / "docs"
ASSETS = DOCS / "report_assets" / "field_progress_20260810"
HTML_OUT = DOCS / "offboard_trajectory_pairb_two_rover_summary_2026-08-10.html"

STRAIGHT_DIR = (
    REPO
    / "results/orin1_outdoor_forward_5m/"
    "orin1_outdoor_forward_20260808_192923"
)
UTURN_DIR = (
    REPO
    / "results/orin2_outdoor_forward_5m/"
    "orin2_outdoor_left_uturn_20260809_135807"
)
SROUTE_DIR = (
    REPO
    / "results/orin2_outdoor_forward_5m/"
    "orin2_outdoor_s_bend_return_20260809_142825"
)
DUAL_DIR = REPO / "results/pairb_dual_trajectory/plan8115"
DUAL_LOG = (
    REPO
    / "results/pairb_staged_live/20260809_plan8115_carrier/supervisor.log"
)

FONT_REGULAR = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
FONT_BOLD = "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"

BG = "#ffffff"
INK = "#20252b"
MUTED = "#68727d"
GRID = "#d9dee3"
PLAN = "#168bd2"
ACTUAL = "#e4a400"
CARRIER = "#e0a400"
MINI = "#19a95b"
RED = "#d1495b"
BLUE = "#1f6f9f"


def font(size: int, *, bold: bool = False) -> ImageFont.FreeTypeFont:
    path = FONT_BOLD if bold and Path(FONT_BOLD).is_file() else FONT_REGULAR
    return ImageFont.truetype(path, size=size)


def read_xy(path: Path) -> tuple[list[tuple[float, float]], float]:
    points: list[tuple[float, float]] = []
    yaw0 = 0.0
    with path.open(newline="", encoding="ascii") as stream:
        for index, row in enumerate(csv.DictReader(stream)):
            points.append((float(row["x_m"]), float(row["y_m"])))
            if index == 0:
                yaw0 = float(row["yaw_rad"])
    if not points:
        raise ValueError(f"empty trajectory: {path}")
    return points, yaw0


def body_aligned(
    points: list[tuple[float, float]],
    origin: tuple[float, float],
    yaw_rad: float,
) -> list[tuple[float, float]]:
    c = math.cos(yaw_rad)
    s = math.sin(yaw_rad)
    transformed: list[tuple[float, float]] = []
    for x_m, y_m in points:
        dx = x_m - origin[0]
        dy = y_m - origin[1]
        transformed.append((c * dx + s * dy, -s * dx + c * dy))
    return transformed


def trajectory_length(points: list[tuple[float, float]]) -> float:
    return sum(math.dist(a, b) for a, b in zip(points, points[1:]))


def point_segment_distance(
    point: tuple[float, float],
    start: tuple[float, float],
    end: tuple[float, float],
) -> float:
    px, py = point
    ax, ay = start
    bx, by = end
    dx = bx - ax
    dy = by - ay
    denominator = dx * dx + dy * dy
    if denominator <= 1.0e-12:
        return math.dist(point, start)
    ratio = ((px - ax) * dx + (py - ay) * dy) / denominator
    ratio = max(0.0, min(1.0, ratio))
    projection = (ax + ratio * dx, ay + ratio * dy)
    return math.dist(point, projection)


def trajectory_stats(plan: list[tuple[float, float]], actual: list[tuple[float, float]]) -> dict[str, float]:
    errors = [
        min(point_segment_distance(point, a, b) for a, b in zip(plan, plan[1:]))
        for point in actual
    ]
    sorted_errors = sorted(errors)
    p95_index = max(0, math.ceil(0.95 * len(sorted_errors)) - 1)
    return {
        "plan_length": trajectory_length(plan),
        "actual_length": trajectory_length(actual),
        "rms": math.sqrt(sum(error * error for error in errors) / len(errors)),
        "p95": sorted_errors[p95_index],
        "maximum": max(errors),
        "endpoint": errors[-1],
    }


def nice_step(span: float, target_lines: int = 7) -> float:
    rough = max(span / target_lines, 0.1)
    exponent = 10.0 ** math.floor(math.log10(rough))
    normalized = rough / exponent
    factor = 1.0 if normalized <= 1.0 else 2.0 if normalized <= 2.0 else 5.0 if normalized <= 5.0 else 10.0
    return factor * exponent


def draw_xy_plot(
    output: Path,
    title: str,
    series: list[tuple[str, list[tuple[float, float]], str]],
    note: str,
) -> None:
    width, height = 1600, 980
    left, top, right, bottom = 145, 120, 65, 125
    image = Image.new("RGB", (width, height), BG)
    draw = ImageDraw.Draw(image)
    draw.text((left, 32), title, font=font(38, bold=True), fill=INK)
    draw.text((left, 78), note, font=font(22), fill=MUTED)

    all_points = [point for _, points, _ in series for point in points]
    min_x = min(point[0] for point in all_points)
    max_x = max(point[0] for point in all_points)
    min_y = min(point[1] for point in all_points)
    max_y = max(point[1] for point in all_points)
    padding = max(0.45, 0.06 * max(max_x - min_x, max_y - min_y, 1.0))
    min_x -= padding
    max_x += padding
    min_y -= padding
    max_y += padding

    plot_width = width - left - right
    plot_height = height - top - bottom
    scale = min(plot_width / max(max_x - min_x, 1.0e-6), plot_height / max(max_y - min_y, 1.0e-6))
    used_width = (max_x - min_x) * scale
    used_height = (max_y - min_y) * scale
    x_offset = left + (plot_width - used_width) / 2.0
    y_offset = top + (plot_height - used_height) / 2.0

    def pixel(point: tuple[float, float]) -> tuple[int, int]:
        return (
            round(x_offset + (point[0] - min_x) * scale),
            round(y_offset + (max_y - point[1]) * scale),
        )

    step = nice_step(max(max_x - min_x, max_y - min_y))
    grid_font = font(18)
    x_grid = math.floor(min_x / step) * step
    while x_grid <= max_x + 1.0e-9:
        x_pixel = pixel((x_grid, min_y))[0]
        draw.line((x_pixel, top, x_pixel, height - bottom), fill=GRID, width=1)
        draw.text((x_pixel - 18, height - bottom + 12), f"{x_grid:g}", font=grid_font, fill=MUTED)
        x_grid += step
    y_grid = math.floor(min_y / step) * step
    while y_grid <= max_y + 1.0e-9:
        y_pixel = pixel((min_x, y_grid))[1]
        draw.line((left, y_pixel, width - right, y_pixel), fill=GRID, width=1)
        draw.text((left - 62, y_pixel - 11), f"{y_grid:g}", font=grid_font, fill=MUTED)
        y_grid += step

    draw.rectangle((left, top, width - right, height - bottom), outline="#8f99a3", width=2)
    draw.text((width // 2 - 95, height - 55), "沿初始航向 / m", font=font(22), fill=INK)
    draw.text((25, top + plot_height // 2 - 20), "横向 / m", font=font(22), fill=INK)

    legend_x = left + 18
    legend_y = top + 18
    for label, points, color in series:
        pixels = [pixel(point) for point in points]
        draw.line(pixels, fill=color, width=7, joint="curve")
        start = pixels[0]
        end = pixels[-1]
        draw.ellipse((start[0] - 8, start[1] - 8, start[0] + 8, start[1] + 8), fill=color, outline=INK, width=2)
        draw.rectangle((end[0] - 8, end[1] - 8, end[0] + 8, end[1] + 8), fill=color, outline=INK, width=2)
        draw.line((legend_x, legend_y + 12, legend_x + 48, legend_y + 12), fill=color, width=7)
        draw.text((legend_x + 62, legend_y), label, font=font(22), fill=INK)
        legend_y += 36

    output.parent.mkdir(parents=True, exist_ok=True)
    image.save(output, quality=95)


def draw_architecture(output: Path) -> None:
    width, height = 1600, 900
    image = Image.new("RGB", (width, height), "#f7f9fb")
    draw = ImageDraw.Draw(image)
    draw.text((80, 38), "两车实车验证的软件与控制链路", font=font(42, bold=True), fill=INK)
    draw.text((80, 92), "高层任务低频协同，本车轨迹跟踪高频闭环，安全动作始终本地优先", font=font(24), fill=MUTED)

    def box(x: int, y: int, w: int, h: int, title: str, lines: list[str], color: str) -> None:
        draw.rounded_rectangle((x, y, x + w, y + h), radius=18, fill="white", outline=color, width=4)
        draw.rectangle((x, y, x + w, y + 56), fill=color)
        draw.text((x + 20, y + 10), title, font=font(27, bold=True), fill="white")
        for index, line in enumerate(lines):
            draw.text((x + 22, y + 77 + index * 34), line, font=font(21), fill=INK)

    box(70, 180, 420, 235, "Orin2 / Carrier（总控）", ["生成阶段任务与共享原点", "接收 MiniState", "发布 PlanCommand / Abort"], BLUE)
    box(1110, 180, 420, 235, "Orin1 / Mini（执行端）", ["验证序号、时效与健康状态", "执行本地轨迹", "回传 MiniState / MissionStatus"], "#198754")
    box(70, 540, 420, 225, "Carrier 本地闭环", ["路径投影 + 前视点", "曲率前馈 + 横向反馈", "BODY_NED → PX4 → Arduino"], "#bd7b00")
    box(1110, 540, 420, 225, "Mini 本地闭环", ["相同通用轨迹跟踪器", "独立超时、归零与恢复", "BODY_NED → PX4 → Arduino"], "#bd7b00")
    box(585, 250, 430, 210, "Pair B / LR24", ["MAVLink TUNNEL，57600", "低频紧凑消息，不传 ROS2 DDS", "失联/过期 → STOP / ABORT"], "#6f42c1")
    box(585, 570, 430, 180, "现场安全层", ["RC Kill / QGC Disarm", "状态新鲜度、模式与进度门", "零输出 → Disarm → MANUAL"], RED)

    arrow_color = "#56616c"
    for start, end in [
        ((490, 295), (585, 295)),
        ((1015, 295), (1110, 295)),
    ]:
        draw.line((*start, *end), fill=arrow_color, width=6)
        ex, ey = end
        draw.polygon([(ex, ey), (ex - 14, ey - 9), (ex - 14, ey + 9)], fill=arrow_color)
    for start, end in [((280, 415), (280, 540)), ((1320, 415), (1320, 540))]:
        draw.line((*start, *end), fill=arrow_color, width=6)
        ex, ey = end
        draw.polygon([(ex, ey), (ex - 9, ey - 14), (ex + 9, ey - 14)], fill=arrow_color)
    draw.line((800, 460, 800, 570), fill=arrow_color, width=6)
    draw.polygon([(800, 570), (790, 553), (810, 553)], fill=arrow_color)
    image.save(output, quality=95)


def terminal_gap_samples(log_path: Path) -> tuple[list[float], list[float]]:
    longitudinal: list[float] = []
    lateral: list[float] = []
    pattern = re.compile(
        r"CARRIER_TERMINAL_GAP(?:_MATCH sample=\d+/\d+)?"
        r"(?: distance=[^ ]+)? longitudinal=([+-]?\d+\.\d+)m"
        r" lateral=([+-]?\d+\.\d+)m"
    )
    for line in log_path.read_text(encoding="utf-8", errors="replace").splitlines():
        match = pattern.search(line)
        if match:
            sample = (float(match.group(1)), float(match.group(2)))
            if not longitudinal or sample != (longitudinal[-1], lateral[-1]):
                longitudinal.append(sample[0])
                lateral.append(sample[1])
    if not longitudinal:
        raise ValueError("terminal gap samples are missing")
    return longitudinal, lateral


def draw_gap_plot(output: Path, longitudinal: list[float], lateral: list[float]) -> None:
    width, height = 1600, 880
    left, top, right, bottom = 145, 120, 70, 120
    image = Image.new("RGB", (width, height), BG)
    draw = ImageDraw.Draw(image)
    draw.text((left, 32), "计划 8115：Mini 停车后的终端间距收敛", font=font(38, bold=True), fill=INK)
    draw.text((left, 78), "日志约 0.5 s/样本；目标纵向间距 2.0 m，判定窗口 ±0.75 m", font=font(22), fill=MUTED)
    plot_w = width - left - right
    plot_h = height - top - bottom
    max_y = max(10.5, max(longitudinal) + 0.5)
    min_y = -1.5

    def px(index: int, value: float) -> tuple[int, int]:
        x = left + index * plot_w / max(1, len(longitudinal) - 1)
        y = top + (max_y - value) * plot_h / (max_y - min_y)
        return round(x), round(y)

    band_top = px(0, 2.75)[1]
    band_bottom = px(0, 1.25)[1]
    draw.rectangle((left, band_top, width - right, band_bottom), fill="#e2f3e8")
    for value in range(-1, 11):
        y = px(0, float(value))[1]
        draw.line((left, y, width - right, y), fill=GRID, width=1)
        draw.text((left - 62, y - 11), str(value), font=font(18), fill=MUTED)
    for index in range(0, len(longitudinal), 4):
        x = px(index, min_y)[0]
        draw.line((x, top, x, height - bottom), fill=GRID, width=1)
        draw.text((x - 12, height - bottom + 12), f"{index * 0.5:g}", font=font(18), fill=MUTED)
    draw.rectangle((left, top, width - right, height - bottom), outline="#8f99a3", width=2)
    draw.text((width // 2 - 75, height - 54), "相对时间 / s", font=font(22), fill=INK)
    draw.text((26, top + plot_h // 2), "间距 / m", font=font(22), fill=INK)

    long_pixels = [px(index, value) for index, value in enumerate(longitudinal)]
    lat_pixels = [px(index, value) for index, value in enumerate(lateral)]
    draw.line(long_pixels, fill=BLUE, width=7, joint="curve")
    draw.line(lat_pixels, fill=RED, width=6, joint="curve")
    end_x, end_y = long_pixels[-1]
    draw.ellipse((end_x - 9, end_y - 9, end_x + 9, end_y + 9), fill=BLUE)
    draw.text((end_x - 245, end_y - 48), f"估计纵向 {longitudinal[-1]:.2f} m", font=font(22, bold=True), fill=BLUE)
    draw.text((end_x - 245, end_y + 8), "现场卷尺约 3.2 m", font=font(22, bold=True), fill=INK)
    draw.line((left + 20, top + 24, left + 74, top + 24), fill=BLUE, width=7)
    draw.text((left + 88, top + 10), "纵向间距", font=font(21), fill=INK)
    draw.line((left + 250, top + 24, left + 304, top + 24), fill=RED, width=7)
    draw.text((left + 318, top + 10), "横向偏差", font=font(21), fill=INK)
    image.save(output, quality=95)


def metric_row(name: str, stats: dict[str, float], result: str) -> str:
    return (
        "<tr>"
        f"<td>{html.escape(name)}</td>"
        f"<td>{stats['plan_length']:.2f}</td>"
        f"<td>{stats['rms']:.2f}</td>"
        f"<td>{stats['p95']:.2f}</td>"
        f"<td>{stats['endpoint']:.2f}</td>"
        f"<td>{html.escape(result)}</td>"
        "</tr>"
    )


def build_report() -> None:
    ASSETS.mkdir(parents=True, exist_ok=True)

    straight_plan, straight_yaw = read_xy(STRAIGHT_DIR / "planned_trajectory.csv")
    straight_actual, _ = read_xy(STRAIGHT_DIR / "actual_trajectory.csv")
    uturn_plan, uturn_yaw = read_xy(UTURN_DIR / "planned_trajectory.csv")
    uturn_actual, _ = read_xy(UTURN_DIR / "actual_trajectory.csv")
    sroute_plan, sroute_yaw = read_xy(SROUTE_DIR / "planned_trajectory.csv")
    sroute_actual, _ = read_xy(SROUTE_DIR / "actual_trajectory.csv")
    carrier_actual, carrier_yaw = read_xy(DUAL_DIR / "carrier/actual_trajectory.csv")
    mini_actual, _ = read_xy(DUAL_DIR / "mini/actual_trajectory.csv")
    carrier_plan, _ = read_xy(DUAL_DIR / "carrier/planned_trajectory.csv")
    mini_plan, _ = read_xy(DUAL_DIR / "mini/planned_trajectory.csv")

    straight_stats = trajectory_stats(straight_plan, straight_actual)
    uturn_stats = trajectory_stats(uturn_plan, uturn_actual)
    sroute_stats = trajectory_stats(sroute_plan, sroute_actual)
    carrier_stats = trajectory_stats(carrier_plan, carrier_actual)
    mini_stats = trajectory_stats(mini_plan, mini_actual)

    straight_origin = straight_plan[0]
    uturn_origin = uturn_plan[0]
    sroute_origin = sroute_plan[0]
    dual_origin = carrier_actual[0]
    draw_architecture(ASSETS / "system_architecture.png")
    draw_xy_plot(
        ASSETS / "straight_5m.png",
        "基础 Offboard：5 m 直线",
        [
            ("规划轨迹", body_aligned(straight_plan, straight_origin, straight_yaw), PLAN),
            ("实际轨迹", body_aligned(straight_actual, straight_origin, straight_yaw), ACTUAL),
        ],
        "Orin1，2026-08-08 19:29:23；日志完成 5.000 m，安全恢复正常",
    )
    draw_xy_plot(
        ASSETS / "uturn.png",
        "通用轨迹跟踪：直线 + 180° 左转 + 返回直线",
        [
            ("规划轨迹", body_aligned(uturn_plan, uturn_origin, uturn_yaw), PLAN),
            ("实际轨迹", body_aligned(uturn_actual, uturn_origin, uturn_yaw), ACTUAL),
        ],
        "Orin2，2026-08-09 13:58:07；全程完成，终点最近路径误差约 0.05 m",
    )
    draw_xy_plot(
        ASSETS / "sroute.png",
        "连续曲线验证：直线 + S 弯 + 掉头返回",
        [
            ("规划轨迹", body_aligned(sroute_plan, sroute_origin, sroute_yaw), PLAN),
            ("实际轨迹", body_aligned(sroute_actual, sroute_origin, sroute_yaw), ACTUAL),
        ],
        "Orin2，2026-08-09 14:28:25；几何跟踪良好，末段因意外 Disarm 安全退出",
    )
    draw_xy_plot(
        ASSETS / "dual_actual.png",
        "Pair B 双车实测：两条实际轨迹（计划 8115）",
        [
            ("Carrier / Orin2 实际", body_aligned(carrier_actual, dual_origin, carrier_yaw), CARRIER),
            ("Mini / Orin1 实际", body_aligned(mini_actual, dual_origin, carrier_yaw), MINI),
        ],
        "共享 Carrier-GPS ENU 经初始队形平移注册；图中不显示规划线",
    )
    longitudinal, lateral = terminal_gap_samples(DUAL_LOG)
    draw_gap_plot(ASSETS / "terminal_gap.png", longitudinal, lateral)

    rows = "".join(
        [
            metric_row("5 m 直线", straight_stats, "完成；MANUAL/disarmed"),
            metric_row("U 形轨迹", uturn_stats, "完成；MANUAL/disarmed"),
            metric_row("连续 S 轨迹", sroute_stats, "末段意外 Disarm；安全恢复"),
            metric_row("8115 Carrier", carrier_stats, "按终端间距提前停止"),
            metric_row("8115 Mini", mini_stats, "完成全路线"),
        ]
    )

    architecture = (ASSETS / "system_architecture.png").relative_to(DOCS).as_posix()
    straight_image = (ASSETS / "straight_5m.png").relative_to(DOCS).as_posix()
    uturn_image = (ASSETS / "uturn.png").relative_to(DOCS).as_posix()
    sroute_image = (ASSETS / "sroute.png").relative_to(DOCS).as_posix()
    dual_image = (ASSETS / "dual_actual.png").relative_to(DOCS).as_posix()
    gap_image = (ASSETS / "terminal_gap.png").relative_to(DOCS).as_posix()

    document = f"""<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<title>Offboard、轨迹跟踪与双车协同阶段总结</title>
<style>
@page {{ size: A4; margin: 18mm 18mm 18mm 20mm; }}
body {{ font-family: 'Noto Sans CJK SC', 'Microsoft YaHei', sans-serif; color: #20252b; font-size: 11pt; line-height: 1.55; }}
h1 {{ color: #174a6e; font-size: 24pt; margin-top: 18pt; border-bottom: 2px solid #1f6f9f; padding-bottom: 6pt; }}
h2 {{ color: #1f6f9f; font-size: 17pt; margin-top: 16pt; }}
h3 {{ color: #355f78; font-size: 13pt; margin-top: 12pt; }}
p {{ margin: 5pt 0; }}
ul, ol {{ margin: 5pt 0 8pt 20pt; }}
li {{ margin: 2pt 0; }}
table {{ border-collapse: collapse; width: 100%; margin: 9pt 0 13pt 0; font-size: 9.5pt; }}
th {{ background: #1f6f9f; color: white; border: 1px solid #9da8b1; padding: 6pt; }}
td {{ border: 1px solid #b7c0c8; padding: 5pt; vertical-align: top; }}
tr:nth-child(even) td {{ background: #f3f6f8; }}
.cover {{ page-break-after: always; text-align: center; padding-top: 55mm; }}
.cover-title {{ font-size: 28pt; font-weight: bold; color: #174a6e; line-height: 1.25; }}
.cover-subtitle {{ font-size: 16pt; color: #56616c; margin-top: 18pt; }}
.cover-meta {{ font-size: 11pt; color: #68727d; margin-top: 48pt; }}
.summary {{ color: #294a5e; font-weight: bold; margin: 10pt 0; }}
.warning {{ color: #7a4d00; margin: 9pt 0; }}
.success {{ color: #176b45; margin: 9pt 0; }}
.figure {{ text-align: center; margin: 12pt 0 14pt 0; page-break-inside: avoid; }}
.figure img {{ width: 16.2cm; max-height: 18.2cm; object-fit: contain; }}
.caption {{ color: #56616c; font-size: 9pt; margin-top: 4pt; }}
.page-break {{ page-break-before: always; }}
.code {{ font-family: 'DejaVu Sans Mono', monospace; background: #f1f3f5; border: 1px solid #ccd2d7; padding: 7pt; white-space: pre-wrap; font-size: 8.5pt; }}
.small {{ font-size: 9pt; color: #68727d; }}
</style>
</head>
<body>
<div class="cover">
  <div class="cover-title">Offboard、轨迹跟踪与双车协同<br>阶段工作总结</div>
  <div class="cover-subtitle">阶段技术报告（教师汇报版）<br>差速小车对二维空中对接算法的实车验证</div>
  <div class="cover-meta">测试平台：Orin2 / Carrier（MAV_SYS_ID=2）与 Orin1 / Mini（MAV_SYS_ID=1）<br>整理日期：2026-08-10<br>数据来源：PX4 / MAVROS / Pair B 实车日志与轨迹 CSV</div>
</div>

<h1 class="page-break">1. 概要</h1>
<div class="summary">
本阶段完成了从“单车能否稳定进入 Offboard 并前进”到“任意连续二维轨迹跟踪”，再到“由 Carrier 通过 Pair B 编排两车分阶段运动”的完整验证链。当前结果证明了控制链路、通用轨迹跟踪器和低频双车协调框架可用；普通单点 GPS 仍限制末端相对距离精度，因此结果不能等同于厘米级对接。
</div>
<ul>
  <li><b>基础链路：</b>形成了 MAVROS → PX4 Differential Rover → MAIN1/2 → Arduino → D24A → 四轮的稳定链路，并保留 RC Kill、模式、超时和最终恢复保护。</li>
  <li><b>单车控制：</b>解决进入 Offboard 后重复等待、起步先修航向、BODY_NED 转向语义、PWM 死区和状态时序等问题。</li>
  <li><b>轨迹跟踪：</b>从直线扩展到 U 形、S 形和连续变曲率轨迹，控制器按路径几何工作，而非为某一条路线写死动作。</li>
  <li><b>双车协同：</b>Orin2 作为 Carrier/leader，Orin1 作为 Mini/executor；运动命令和状态只经 Pair B 传输，两车各自在本机闭环跟踪。</li>
  <li><b>最终实测：</b>计划 8115 中 Mini 先行，Carrier 延迟启动；Mini 停车后 Carrier 继续接近，系统估计停车间距 2.60 m，现场测量约 3.20 m，两车均安全回到 MANUAL/disarmed。</li>
</ul>

<div class="figure"><img src="{architecture}" width="520"><div class="caption">图 1  当前两车任务、通信、本地闭环与安全链路</div></div>

<h2>1.1 系统边界</h2>
<p>当前小车是空中对接算法的二维验证工具。高层关注的是任务阶段、共享走廊和相对运动；低层由每辆车独立完成路径跟踪。Pair B 只传输 MiniState、FieldOrigin、PlanCommand、MissionStatus 和 Abort 等低频紧凑消息，不承载 ROS2 DDS、图像或高频转向闭环。</p>

<h1 class="page-break">2. 第一阶段：基础 Offboard 验证</h1>
<h2>2.1 控制原理</h2>
<p>程序在本车坐标系 <b>BODY_NED</b> 中发布速度指令。对当前 PX4 rover 路径，前进量主要由 <code>linear.x</code> 表示，差速转向由 <code>linear.y</code> 映射；不能假设 <code>angular.z</code> 一定进入差速控制器。PX4 再把左右执行器输出转换为 MAIN1/MAIN2 PWM，Arduino 负责死区、方向映射和 D24A 电机驱动。</p>
<h3>受控进入与退出</h3>
<ol>
  <li>确认 MANUAL、disarmed、状态新鲜、GPS/Local Position 有效、RC Kill 可用。</li>
  <li>20 Hz 发布全零 setpoint 预流。</li>
  <li>请求并验证真实 OFFBOARD；随后只请求一次 Arm，并以状态回读为准。</li>
  <li>立即开始受限动作，不再在 armed+OFFBOARD 后额外等待 2 s。</li>
  <li>结束或异常时先归零，再验证 Disarm 和 MANUAL；任一步无法确认都按失败处理。</li>
</ol>

<h2>2.2 关键问题与修复</h2>
<table>
<tr><th>问题</th><th>现象</th><th>处理</th></tr>
<tr><td>post-OFFBOARD 冗余等待</td><td>进入 OFFBOARD 后仍保持 2 s 零输出，可能与自动解锁/状态保持窗口冲突</td><td>保留模式前零预流，观察到 armed+OFFBOARD 后立即进入第一段动作</td></tr>
<tr><td>起步先修 yaw</td><td>车辆先左右摆头，再沿预设方向前进</td><td>以进入任务时的当前航向/短距离地面航迹为参考，不做预对准动作</td></tr>
<tr><td>方向与死区</td><td>前进变后退、低速不动或左右轮响应不一致</td><td>逐层核对 PX4 REV、MAIN1/2、Arduino 四轮方向和 PWM 死区；控制层只使用已验证映射</td></tr>
<tr><td>状态竞态</td><td>首帧 mode 出现 CMODE 或 State 约 1 Hz，程序误判缺失/陈旧</td><td>有限等待完整 MANUAL 安全前态；State 新鲜阈值按实测周期设置为 2 s</td></tr>
<tr><td>异常退出不可辨识</td><td>早期日志把 Disarm、Offboard exit、断连和陈旧状态混为一类</td><td>细分失败原因并同步记录 State、ExtendedState 与 STATUSTEXT</td></tr>
</table>

<div class="figure"><img src="{straight_image}" width="520"><div class="caption">图 2  5 m 基础 Offboard 直线。蓝色为规划，黄色为实际；坐标已旋转到初始车体航向。</div></div>
<div class="success"><b>代表性结果：</b>2026-08-08 19:29:23，车辆完成日志进度 5.000 m，终点最近路径误差约 {straight_stats['endpoint']:.2f} m，最终状态 MANUAL/disarmed，未执行起步航向对准。</div>

<h1 class="page-break">3. 第二阶段：通用二维轨迹跟踪</h1>
<h2>3.1 控制器思路</h2>
<p>轨迹先按弧长离散为二维点列。每个控制周期使用当前位置在路径上的局部投影，沿路径向前选择速度相关的前视点，再结合参考曲率、横向误差和航向误差生成有界转向量。最后把几何命令转换为本车 BODY_NED 的前进/横向速度。该方法接近带曲率前馈和反馈修正的几何前视控制，不是 MPC。</p>
<p>通用性来自“路径几何 → 本地闭环”的统一接口：直线、圆弧、U 形、S 形和后续走廊只更换路径点，不更换底层状态机和安全门。曲率、曲率变化率、最大车体转向量和转向变化率在执行前及运行时都受限。</p>

<h2>3.2 从 U 形到连续 S 曲线</h2>
<div class="figure"><img src="{uturn_image}" width="520"><div class="caption">图 3  直线 + 180° 左转 + 返回直线。该组完整到达终点并安全恢复。</div></div>
<p>U 形轨迹验证了直线与定半径转弯的连续切换。代表性成功组的路径长度为 {uturn_stats['plan_length']:.2f} m，最近路径 RMS 误差约 {uturn_stats['rms']:.2f} m，终点误差约 {uturn_stats['endpoint']:.2f} m。</p>

<div class="figure"><img src="{sroute_image}" width="520"><div class="caption">图 4  连续 S 曲线与掉头返回。轨迹几何跟踪良好；该组末段由意外 Disarm 触发安全退出。</div></div>
<p>S 曲线进一步暴露并修复了固定前视、曲率反馈不足、转向突变和出弯误差等问题。图示组在退出前已运行约 {sroute_stats['actual_length']:.2f} m，最近路径 RMS 误差约 {sroute_stats['rms']:.2f} m。该组没有被记为完整成功，因为日志明确记录 <code>unexpected_disarm</code>；它的价值是证明几何跟踪质量和异常恢复，而不是掩盖中断。</p>

<h2>3.3 日志指标汇总</h2>
<table>
<tr><th>记录</th><th>规划长度 / m</th><th>RMS 最近路径误差 / m</th><th>P95 / m</th><th>终点最近路径误差 / m</th><th>结果</th></tr>
{rows}
</table>
<p class="small">注：误差由实际采样点到最近规划线段的欧氏距离重新计算。该指标适合比较路径形状，不代表 RTK 级绝对定位精度；普通 GPS 漂移会同时影响规划参考、实际轨迹和双车相对距离。</p>

<h1 class="page-break">4. 第三阶段：Pair B 双车运动</h1>
<h2>4.1 角色与状态机</h2>
<table>
<tr><th>节点</th><th>当前角色</th><th>职责</th></tr>
<tr><td>Orin2 / MAV_SYS_ID=2</td><td>Carrier / leader</td><td>锁定共享 FieldOrigin，接收 MiniState，决定阶段并发布 PlanCommand/Abort，同时执行本车轨迹</td></tr>
<tr><td>Orin1 / MAV_SYS_ID=1</td><td>Mini / executor</td><td>验证 plan_id、seq、TTL 与安全状态，执行本地轨迹并回传 MiniState/MissionStatus</td></tr>
<tr><td>Ground / Boss</td><td>现场安全与监控</td><td>RC Kill、QGC、场地确认、日志观察；不作为运行时高频控制器</td></tr>
</table>
<p>最终测试采用分阶段追随逻辑：两车就绪后 Mini 先出发；Carrier 同时满足 5 s 延迟和 Mini 领先 2 m 后启动。两车都使用各自本地轨迹跟踪器，Pair B 不发送每个控制周期的电机量。Mini 完成后先发送终态，Carrier 继续根据共享坐标中的相对位置接近，达到目标间距后停止。</p>

<h2>4.2 共享坐标</h2>
<p>Carrier 发布 FieldOrigin，双方将 GPS/Local Position 转成同一 ENU。由于两套 PX4 local frame 起点不同，测试用现场已知队形“Mini 在 Carrier 前方 0.50 m”进行平移注册；yaw 仅用于方向注册。计划 8115 的原始 GPS 相对量为 (+0.73, +0.02) m，注册后 Mini 起点约为 (-0.01, +0.47) m。</p>
<div class="warning"><b>边界：</b>这种注册可验证轨迹形状与阶段逻辑，但不能消除两台普通 GPS 的独立漂移。它不适合证明 0.1–0.3 m 级对接精度。</div>

<div class="figure"><img src="{dual_image}" width="520"><div class="caption">图 5  计划 8115 的两条实际轨迹。黄色为 Carrier/Orin2，绿色为 Mini/Orin1；按要求不显示规划线。</div></div>

<h2>4.3 计划 8115 结果</h2>
<ul>
  <li>Mini 通过 Pair B 接受任务并先行；Carrier 在记录到领先 2.07 m 后释放本地启动门。</li>
  <li>Mini 完成 46.791 m 路线并发送 COMPLETE；已修复“Disarm 状态先到、COMPLETE 后到”导致 Carrier 误停的竞态。</li>
  <li>Mini 停车时系统估计 Carrier 落后约 9.83 m；Carrier 随后继续运行约 10 s。</li>
  <li>连续 3 个终端样本满足条件后停止：纵向 2.60 m、横向 -0.43 m、航向误差 -7.8°。</li>
  <li>现场卷尺测量实际纵向间距约 3.20 m，与系统估计相差约 0.60 m，符合两套普通 GPS 的叠加误差量级。</li>
  <li>两端 MISSION_RC=0、FINAL_RECOVERY_SAFE=True，最终均为 MANUAL/disarmed；清理后无 MAVROS/Offboard 残留进程。</li>
</ul>

<div class="figure"><img src="{gap_image}" width="520"><div class="caption">图 6  Mini 停车后 Carrier 的终端间距收敛。绿色带为 2.0 ± 0.75 m 的当前判定窗口。</div></div>

<h1 class="page-break">5. 安全机制与工程经验</h1>
<h2>5.1 始终保留的保护</h2>
<ul>
  <li>实车运动必须有本轮现场确认，RC Kill 与 QGC Disarm 可用。</li>
  <li>任何 State/pose/GPS 陈旧、断连、OFFBOARD 退出、意外 Disarm、路径误差越界或进度停滞均 fail-closed。</li>
  <li>程序自身不重试 Arm；Arm 只在真实 OFFBOARD 后请求一次。</li>
  <li>成功、失败和中断都执行零输出、Disarm、MANUAL 的有限恢复，并以状态回读而非 service success 为准。</li>
  <li>Pair B 命令带 plan_id、序号、时间戳与有效期；过期、重复、跨会话或校验失败的命令不进入执行器。</li>
  <li>Abort 优先级最高；两车高频转向控制均留在本机，不依赖无线链路实时闭环。</li>
</ul>

<h2>5.2 本阶段主要工程结论</h2>
<ol>
  <li>先保证方向、死区和安全恢复，再讨论轨迹控制；底层符号错误不能用 PID 掩盖。</li>
  <li>路径跟踪应使用通用路径几何和本车闭环，不应为每条测试路线编写固定动作时间表。</li>
  <li>起点变化本身不是问题：规划在每次启动后锁定当前位置和参考航向，轨迹在相对坐标中生成。</li>
  <li>可视化必须与控制解耦。RViz/PNG 用于观察和复盘，发布失败不能改变 setpoint 或安全门。</li>
  <li>双车同步不是“同时发两条 SSH 命令”，而是共享任务阶段、原点、状态和终态确认；实际运动协调只走 Pair B。</li>
</ol>

<h1>6. 局限与下一步</h1>
<table>
<tr><th>当前限制</th><th>影响</th><th>建议</th></tr>
<tr><td>普通单点 GPS</td><td>双车相对间距出现约 0.6 m 的单次测量差，厘米级结论不可信</td><td>更换 RTK；两车确认 FIX、基线与时间同步后再收紧终端容差</td></tr>
<tr><td>轮速没有编码器闭环</td><td>同一速度请求在不同地面、电压和底盘上响应不同</td><td>小车阶段继续依靠位置反馈和有界几何跟踪；上飞机后由飞控速度/姿态闭环承担</td></tr>
<tr><td>当前不是完整 EasyDocking</td><td>已验证的是分阶段 S 路线追随，不是轨道切出与末端切线对接</td><td>下一步接入 Mini 稳定绕圈一周、Carrier 生成平滑进场、双方进入同一 terminal tangent corridor</td></tr>
<tr><td>当前控制器不是 MPC</td><td>无法直接表达复杂时域约束和最优性</td><td>先用已验证的解析轨迹 + 几何闭环完成系统联调；飞机阶段再评估 MPC/非线性优化的必要性</td></tr>
</table>
<div class="success"><b>阶段结论：</b>小车平台已经完成“可控 Offboard → 通用二维轨迹跟踪 → Pair B 双车分阶段运动”的主链路验证。下一阶段的重点不应继续追求普通 GPS 下的厘米级小车间距，而应升级 RTK，并把当前通用执行接口接入 EasyDocking 的绕圈、切线切出和共享末端走廊。</div>

<h1>附录：代表性日志索引</h1>
<div class="code">基础直线：results/orin1_outdoor_forward_5m/orin1_outdoor_forward_20260808_192923/
U 形轨迹：results/orin2_outdoor_forward_5m/orin2_outdoor_left_uturn_20260809_135807/
连续 S 轨迹：results/orin2_outdoor_forward_5m/orin2_outdoor_s_bend_return_20260809_142825/
双车轨迹：results/pairb_dual_trajectory/plan8115/
双车监督日志：results/pairb_staged_live/20260809_plan8115_carrier/supervisor.log</div>
<p class="small">本文档的轨迹误差、路线长度和终端间距图均由上述 CSV/日志自动生成；未读取或启动任何车辆进程。</p>
</body>
</html>
"""
    HTML_OUT.write_text(document, encoding="utf-8")
    print(f"HTML={HTML_OUT}")
    for name, stats in (
        ("straight", straight_stats),
        ("uturn", uturn_stats),
        ("sroute", sroute_stats),
        ("carrier8115", carrier_stats),
        ("mini8115", mini_stats),
    ):
        print(name, " ".join(f"{key}={value:.4f}" for key, value in stats.items()))


if __name__ == "__main__":
    build_report()
