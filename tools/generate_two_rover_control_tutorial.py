#!/usr/bin/env python3

from __future__ import annotations

import html
import math
from pathlib import Path

from PIL import Image, ImageDraw

from generate_field_progress_report import (
    ACTUAL,
    ASSETS,
    BG,
    BLUE,
    CARRIER,
    DOCS,
    GRID,
    INK,
    MINI,
    MUTED,
    PLAN,
    RED,
    font,
)


HTML_OUT = DOCS / "two_rover_trajectory_pairb_ros2_control_tutorial_2026-08-10.html"


def rounded_box(
    draw: ImageDraw.ImageDraw,
    bounds: tuple[int, int, int, int],
    title: str,
    lines: list[str],
    color: str,
    *,
    title_size: int = 25,
    body_size: int = 20,
) -> None:
    x1, y1, x2, y2 = bounds
    draw.rounded_rectangle(bounds, radius=16, fill="white", outline=color, width=4)
    draw.rectangle((x1, y1, x2, y1 + 52), fill=color)
    draw.text((x1 + 16, y1 + 9), title, font=font(title_size, bold=True), fill="white")
    for index, line in enumerate(lines):
        draw.text((x1 + 17, y1 + 70 + index * 30), line, font=font(body_size), fill=INK)


def horizontal_arrow(
    draw: ImageDraw.ImageDraw,
    start: tuple[int, int],
    end: tuple[int, int],
    color: str = "#56616c",
    width: int = 5,
) -> None:
    draw.line((*start, *end), fill=color, width=width)
    direction = 1 if end[0] >= start[0] else -1
    draw.polygon(
        [end, (end[0] - direction * 15, end[1] - 9), (end[0] - direction * 15, end[1] + 9)],
        fill=color,
    )


def vertical_arrow(
    draw: ImageDraw.ImageDraw,
    start: tuple[int, int],
    end: tuple[int, int],
    color: str = "#56616c",
    width: int = 5,
) -> None:
    draw.line((*start, *end), fill=color, width=width)
    direction = 1 if end[1] >= start[1] else -1
    draw.polygon(
        [end, (end[0] - 9, end[1] - direction * 15), (end[0] + 9, end[1] - direction * 15)],
        fill=color,
    )


def draw_local_control_pipeline(output: Path) -> None:
    image = Image.new("RGB", (1800, 1040), "#f7f9fb")
    draw = ImageDraw.Draw(image)
    draw.text((70, 35), "一辆小车如何把轨迹变成四个轮子的动作", font=font(42, bold=True), fill=INK)
    draw.text((70, 90), "高层给“路线”，本地控制器每个周期只回答：现在向前多少、向哪边修多少", font=font(24), fill=MUTED)

    boxes = [
        ((55, 185, 315, 385), "轨迹点列", ["x, y, yaw", "按弧长排列", "直线/圆弧/S弯"], BLUE),
        ((355, 185, 615, 385), "路径投影", ["找最近路段", "得到进度 s", "得到横向误差"], "#397a9c"),
        ((655, 185, 915, 385), "自适应前视", ["沿路径向前看", "速度越快看越远", "避免只盯脚下"], "#5b6fb2"),
        ((955, 185, 1215, 385), "转向合成", ["参考曲率前馈", "横向/航向反馈", "积分消除慢偏差"], "#6f42c1"),
        ((1255, 185, 1515, 385), "BODY_NED", ["linear.x 前进", "linear.y 差速转向", "限幅+变化率限制"], "#bd7b00"),
    ]
    for bounds, title, lines, color in boxes:
        rounded_box(draw, bounds, title, lines, color)
    for index in range(len(boxes) - 1):
        horizontal_arrow(draw, (boxes[index][0][2] + 6, 285), (boxes[index + 1][0][0] - 6, 285))

    rounded_box(draw, (1255, 510, 1515, 735), "MAVROS / PX4", ["20 Hz setpoint", "OFFBOARD 状态门", "差分执行器输出"], "#198754")
    rounded_box(draw, (955, 510, 1215, 735), "MAIN1 / MAIN2", ["左右侧 PWM", "1500 μs 为中点", "REV 参数已校验"], "#198754")
    rounded_box(draw, (655, 510, 915, 735), "Arduino", ["读左右 PWM", "死区与安全超时", "映射四个轮子"], "#198754")
    rounded_box(draw, (355, 510, 615, 735), "D24A 驱动", ["四路方向与 PWM", "A/B/C/D 电机", "物理功率执行"], "#198754")
    rounded_box(draw, (55, 510, 315, 735), "四个车轮", ["产生真实位移", "GPS/IMU 再观测", "形成闭环"], "#198754")
    vertical_arrow(draw, (1385, 385), (1385, 510))
    for start_x, end_x in [(1255, 1215), (955, 915), (655, 615), (355, 315)]:
        horizontal_arrow(draw, (start_x - 6, 622), (end_x + 6, 622), color="#198754")

    draw.rounded_rectangle((190, 825, 1610, 960), radius=18, fill="#fff4e8", outline=RED, width=4)
    draw.text((225, 844), "安全旁路（任何时候都能覆盖上面的正常链路）", font=font(28, bold=True), fill=RED)
    draw.text((225, 892), "RC Kill / 状态陈旧 / 断连 / OFFBOARD 退出 / 意外 Disarm / 误差越界 → 立即零输出 → Disarm → MANUAL", font=font(23), fill=INK)
    image.save(output, quality=95)


def draw_coordinate_frames(output: Path) -> None:
    image = Image.new("RGB", (1600, 900), BG)
    draw = ImageDraw.Draw(image)
    draw.text((70, 35), "两个坐标系：地图上的路，必须先翻译成车身动作", font=font(42, bold=True), fill=INK)
    draw.text((70, 90), "Field ENU 用于两车共享位置；BODY_NED 用于本车速度命令", font=font(24), fill=MUTED)

    draw.rounded_rectangle((80, 170, 730, 760), radius=18, fill="#f3f8fb", outline=BLUE, width=4)
    draw.text((115, 195), "共享场地坐标 Field ENU", font=font(30, bold=True), fill=BLUE)
    origin = (365, 570)
    draw.ellipse((origin[0] - 9, origin[1] - 9, origin[0] + 9, origin[1] + 9), fill=INK)
    horizontal_arrow(draw, origin, (640, 570), color=RED, width=7)
    vertical_arrow(draw, origin, (365, 285), color="#198754", width=7)
    draw.text((590, 595), "+x East", font=font(24, bold=True), fill=RED)
    draw.text((390, 280), "+y North", font=font(24, bold=True), fill="#198754")
    draw.arc((285, 490, 445, 650), start=205, end=318, fill="#6f42c1", width=7)
    draw.text((155, 650), "yaw：从 East 起，逆时针为正", font=font(22), fill="#6f42c1")
    draw.text((130, 705), "用途：FieldOrigin、MiniState、终端间距", font=font(21), fill=INK)

    draw.rounded_rectangle((870, 170, 1520, 760), radius=18, fill="#fff8ef", outline="#bd7b00", width=4)
    draw.text((905, 195), "本车坐标 BODY_NED", font=font(30, bold=True), fill="#bd7b00")
    center = (1195, 500)
    draw.rounded_rectangle((1120, 440, 1270, 560), radius=24, fill="#d8e5ec", outline=INK, width=3)
    draw.ellipse((1110, 448, 1130, 485), fill="#444")
    draw.ellipse((1110, 515, 1130, 552), fill="#444")
    draw.ellipse((1260, 448, 1280, 485), fill="#444")
    draw.ellipse((1260, 515, 1280, 552), fill="#444")
    vertical_arrow(draw, center, (1195, 285), color=BLUE, width=7)
    horizontal_arrow(draw, center, (1435, 500), color=RED, width=7)
    draw.text((1218, 285), "+x 车头前方", font=font(24, bold=True), fill=BLUE)
    draw.text((1330, 525), "+y 车体右方", font=font(24, bold=True), fill=RED)
    draw.text((925, 650), "本项目实测语义：", font=font(22, bold=True), fill=INK)
    draw.text((925, 690), "linear.x → 前进；linear.y → 差速转向", font=font(21), fill=INK)
    draw.text((925, 725), "angular.z 在当前 PX4 rover 路径中可能被忽略", font=font(19), fill=MUTED)

    horizontal_arrow(draw, (735, 465), (865, 465), color="#56616c", width=6)
    draw.text((748, 395), "用当前 yaw", font=font(21, bold=True), fill=INK)
    draw.text((744, 425), "旋转变换", font=font(21, bold=True), fill=INK)
    image.save(output, quality=95)


def draw_pairb_wire(output: Path) -> None:
    image = Image.new("RGB", (1900, 1120), "#f7f9fb")
    draw = ImageDraw.Draw(image)
    draw.text((70, 35), "Pair B：两台车的“对讲机”，不是远程方向盘", font=font(42, bold=True), fill=INK)
    draw.text((70, 90), "软件角色与电台 GROUND/VEHICLE 模式是两件事；当前 Orin2 是 Carrier，但电台仍接 VEHICLE 端", font=font(23), fill=MUTED)

    rounded_box(draw, (55, 190, 350, 430), "Orin2 / Carrier", ["领导状态机", "FieldOrigin / Command", "ROS2 host-local"], BLUE)
    rounded_box(draw, (415, 190, 710, 430), "MAVROS Router", ["/pairb_tunnel/source", "/pairb_tunnel/sink", "系统 2，组件 242"], "#397a9c")
    rounded_box(draw, (775, 190, 1070, 430), "Mini Pixhawk", ["USB ↔ MAVLink Router", "TELEM2 转发", "不解析 L2 内容"], "#6f42c1")
    rounded_box(draw, (1135, 190, 1430, 430), "LR24 VEHICLE", ["TELEM2，57600", "ADDR=1102", "FHSS 全双工"], "#6f42c1")
    rounded_box(draw, (1495, 190, 1845, 430), "无线 Pair B", ["低速 2.4 KB/s", "500 mW", "一帧一条 TUNNEL"], "#bd7b00")
    for x1, x2 in [(350, 415), (710, 775), (1070, 1135), (1430, 1495)]:
        horizontal_arrow(draw, (x1 + 6, 270), (x2 - 6, 270), color=BLUE)
        horizontal_arrow(draw, (x2 - 6, 350), (x1 + 6, 350), color=MINI)

    rounded_box(draw, (1495, 650, 1845, 890), "LR24 GROUND", ["CP2102 USB", "57600", "ADDR=1102"], "#6f42c1")
    rounded_box(draw, (1115, 650, 1430, 890), "Orin1 / Mini", ["直接打开 CP2102", "验证命令与 TTL", "执行本地轨迹"], "#198754")
    rounded_box(draw, (705, 650, 1050, 890), "L2 紧凑帧", ["magic/version/type", "payload + CRC16", "seq / time / plan_id"], "#bd7b00")
    rounded_box(draw, (250, 650, 640, 890), "传输的消息", ["MiniState / FieldOrigin", "PlanCommand / MissionStatus", "CorridorPlan / Abort"], BLUE)
    vertical_arrow(draw, (1670, 430), (1670, 650), color="#56616c", width=7)
    horizontal_arrow(draw, (1495, 730), (1430, 730), color=BLUE)
    horizontal_arrow(draw, (1115, 810), (1050, 810), color=MINI)
    horizontal_arrow(draw, (705, 730), (640, 730), color=BLUE)

    draw.rounded_rectangle((250, 960, 1845, 1055), radius=15, fill="#fff4e8", outline=RED, width=3)
    draw.text((285, 982), "绝不通过 Pair B 传：ROS2 DDS、图像、Git、日志文件、SSH 命令或电机高频闭环。", font=font(24, bold=True), fill=RED)
    image.save(output, quality=95)


def draw_offboard_state_machine(output: Path) -> None:
    image = Image.new("RGB", (1800, 1110), BG)
    draw = ImageDraw.Draw(image)
    draw.text((70, 35), "单车 Offboard 状态机：每一步都要“请求 + 回读确认”", font=font(42, bold=True), fill=INK)
    draw.text((70, 90), "service 返回 success 只说明请求被接收，不能代替 PX4 State 的真实变化", font=font(24), fill=MUTED)

    states = [
        ("PRECHECK", ["connected / fresh", "MANUAL + disarmed", "GPS/pose + RC Kill"]),
        ("ZERO PRESTREAM", ["20 Hz 全零", "持续 2 s", "建立 Offboard 信号"]),
        ("REQUEST OFFBOARD", ["最多 3 次", "每次等 state.mode", "失败即恢复"]),
        ("ARM ONCE", ["只在 OFFBOARD 后", "只请求一次", "2 s 内看 armed"]),
        ("TRACK", ["发布有界 BODY_NED", "监控进度与误差", "状态异常即退出"]),
        ("ZERO + RECOVERY", ["零输出 burst", "Disarm 并确认", "MANUAL 并确认"]),
    ]
    positions = [(55, 190), (350, 190), (645, 190), (940, 190), (1235, 190), (1235, 590)]
    colors = [BLUE, "#397a9c", "#6f42c1", "#bd7b00", "#198754", RED]
    for (title, lines), (x, y), color in zip(states, positions, colors):
        rounded_box(draw, (x, y, x + 245, y + 205), title, lines, color, title_size=21, body_size=18)
    for index in range(4):
        horizontal_arrow(draw, (positions[index][0] + 250, 292), (positions[index + 1][0] - 5, 292))
    vertical_arrow(draw, (1357, 395), (1357, 590))

    failure_x = 90
    failure_y = 610
    draw.rounded_rectangle((failure_x, failure_y, 1060, 1015), radius=18, fill="#fff5f5", outline=RED, width=4)
    draw.text((failure_x + 25, failure_y + 20), "任何阶段都可能触发 RECOVERY", font=font(30, bold=True), fill=RED)
    failures = [
        "state_missing / state_stale / disconnected",
        "precheck_not_manual / unexpected_disarm",
        "offboard_exit / Arm rejected / mode timeout",
        "cross_track_limit / heading_error_limit / stall",
        "Pair B Abort / command expired / operator Kill",
    ]
    for index, line in enumerate(failures):
        draw.text((failure_x + 45, failure_y + 80 + index * 50), "• " + line, font=font(22), fill=INK)
    horizontal_arrow(draw, (1060, 810), (1230, 810), color=RED, width=7)
    draw.text((1080, 748), "fail-closed", font=font(22, bold=True), fill=RED)
    draw.text((1165, 1030), "只有 MANUAL + disarmed 被回读确认，才报告安全结束", font=font(22, bold=True), fill=INK)
    image.save(output, quality=95)


def draw_dual_sequence(output: Path) -> None:
    image = Image.new("RGB", (1800, 1500), "#f8fafb")
    draw = ImageDraw.Draw(image)
    draw.text((70, 35), "双车计划 8115 的完整时序", font=font(42, bold=True), fill=INK)
    draw.text((70, 90), "竖直方向代表时间向下；蓝色是 Carrier→Mini，绿色是 Mini→Carrier", font=font(23), fill=MUTED)
    lanes = [(160, "Boss / Ground"), (600, "Orin2 Carrier"), (1050, "Pair B"), (1500, "Orin1 Mini")]
    for x, label in lanes:
        draw.text((x - 100, 150), label, font=font(25, bold=True), fill=INK)
        draw.line((x, 205, x, 1415), fill="#aeb7bf", width=3)

    events = [
        (250, 160, 600, "本轮现场确认", RED),
        (320, 600, 1050, "FieldOrigin + HOLD", BLUE),
        (380, 1500, 1050, "MiniState：ready", MINI),
        (440, 1050, 600, "MiniState + health", MINI),
        (510, 600, 1050, "PlanCommand：TRAJECTORY", BLUE),
        (570, 1050, 1500, "转发启动命令", BLUE),
        (650, 1500, 1500, "本地 OFFBOARD → Arm → 跟踪", MINI),
        (730, 1500, 1050, "持续 MiniState 10 Hz", MINI),
        (810, 600, 600, "5 s 且 Mini 领先 2 m", BLUE),
        (875, 600, 600, "Carrier 本地 OFFBOARD → Arm", CARRIER),
        (950, 600, 1050, "HOLD/状态/Abort 继续交换", BLUE),
        (1030, 1500, 1050, "MissionStatus：COMPLETE（先发）", MINI),
        (1095, 1500, 1050, "停车后的 MiniState", MINI),
        (1165, 1050, 600, "COMPLETE + 终点位置", MINI),
        (1230, 600, 600, "Carrier 继续接近", CARRIER),
        (1300, 600, 600, "纵向 2.60 m，连续 3 样本", CARRIER),
        (1370, 600, 1050, "STOP / terminal ack", BLUE),
    ]
    for y, x1, x2, label, color in events:
        if x1 == x2:
            draw.rounded_rectangle((x1 - 150, y - 22, x1 + 150, y + 28), radius=10, fill="white", outline=color, width=3)
            draw.text((x1 - 135, y - 12), label, font=font(18, bold=True), fill=color)
        else:
            horizontal_arrow(draw, (x1, y), (x2, y), color=color, width=5)
            text_x = min(x1, x2) + abs(x2 - x1) // 2 - min(180, len(label) * 9)
            draw.rectangle((text_x - 5, y - 28, text_x + len(label) * 18 + 10, y - 2), fill="#f8fafb")
            draw.text((text_x, y - 28), label, font=font(18, bold=True), fill=color)
    image.save(output, quality=95)


def link(path: str, line: int | None = None) -> str:
    label = f"{path}:{line}" if line else path
    return f"<code>{html.escape(label)}</code>"


def build_tutorial() -> None:
    ASSETS.mkdir(parents=True, exist_ok=True)
    draw_local_control_pipeline(ASSETS / "tutorial_local_control.png")
    draw_coordinate_frames(ASSETS / "tutorial_frames.png")
    draw_pairb_wire(ASSETS / "tutorial_pairb_wire.png")
    draw_offboard_state_machine(ASSETS / "tutorial_offboard_state_machine.png")
    draw_dual_sequence(ASSETS / "tutorial_dual_sequence.png")

    def asset(name: str) -> str:
        return (ASSETS / name).relative_to(DOCS).as_posix()

    code_map = [
        ("单车现场启动器", "scripts/run_orin2_outdoor_forward_5m.sh", "检查参数/by-id，启动 MAVROS、日志和任务进程；Orin1 包装器复用它"),
        ("单车任务与安全状态机", "src/orin2_outdoor_forward_5m.py", "构建相对轨迹、OFFBOARD/Arm/恢复、发布 BODY_NED、记录 CSV"),
        ("通用轨迹跟踪器", "src/orin2_trajectory_tracker.py", "路径投影、前视点、曲率前馈/反馈、限幅、直线/U形/S形轨迹生成"),
        ("ROS2 轨迹可视化", "src/rover_rviz_trajectory.py", "发布 planned_path、actual_path、vehicle_pose 与 lookahead_target"),
        ("Pair B 二进制协议", "src/lr24_compact_protocol.py", "MiniState、PlanCommand、MissionStatus、FieldOrigin、Abort 编解码与 CRC"),
        ("MAVLink TUNNEL 适配", "src/lr24_mavlink_tunnel.py", "一条 L2 帧装入一条 MAVLink 2 TUNNEL；Orin2 接 MAVROS Router"),
        ("命令安全门", "src/lr24_command_guard.py", "检查角色、plan_id、seq、TTL、速度/加速度和 Abort latch"),
        ("本地 Mini 状态源", "src/mavros_mini_state_source.py", "汇总 MAVROS state/pose/velocity/IMU/GPS 为 MiniState"),
        ("双车协调核心", "src/pairb_staged_chase.py", "Mini 先行、Carrier 延迟启动、终端间距与 Abort 决策"),
        ("双车实车总控", "scripts/run_pairb_staged_chase_live.py", "启动两车 worker，交换 Pair B 消息，管理共享原点和任务终态"),
        ("Pair B 物理与报文契约", "docs/lr24_pairb_wire_contract_v1.md", "接线、串口、帧格式、频率预算、共享 ENU 和安全边界"),
    ]
    code_rows = "".join(
        f"<tr><td>{html.escape(role)}</td><td><code>{html.escape(path)}</code></td><td>{html.escape(description)}</td></tr>"
        for role, path, description in code_map
    )

    document = f"""<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<title>从零理解 ROS2、轨迹跟踪与 Pair B 双车控制</title>
<style>
@page {{ size: A4; margin: 17mm 17mm 18mm 19mm; }}
body {{ font-family: 'Noto Sans CJK SC', 'Microsoft YaHei', sans-serif; color: #20252b; font-size: 10.8pt; line-height: 1.62; }}
h1 {{ color: #174a6e; font-size: 23pt; border-bottom: 2px solid #1f6f9f; padding-bottom: 5pt; margin-top: 18pt; }}
h2 {{ color: #1f6f9f; font-size: 16.5pt; margin-top: 15pt; }}
h3 {{ color: #355f78; font-size: 13pt; margin-top: 11pt; }}
p {{ margin: 5pt 0; }}
ul, ol {{ margin: 5pt 0 8pt 20pt; }}
li {{ margin: 2pt 0; }}
code {{ font-family: 'DejaVu Sans Mono', monospace; background: #eef1f3; font-size: 8.8pt; }}
table {{ border-collapse: collapse; width: 100%; margin: 9pt 0 13pt 0; font-size: 9.2pt; }}
th {{ background: #1f6f9f; color: white; border: 1px solid #9da8b1; padding: 6pt; }}
td {{ border: 1px solid #b7c0c8; padding: 5pt; vertical-align: top; }}
tr:nth-child(even) td {{ background: #f3f6f8; }}
.cover {{ page-break-after: always; text-align: center; padding-top: 48mm; }}
.cover-title {{ font-size: 27pt; font-weight: bold; color: #174a6e; line-height: 1.3; }}
.cover-subtitle {{ font-size: 15pt; color: #56616c; margin-top: 17pt; }}
.cover-meta {{ font-size: 10.5pt; color: #68727d; margin-top: 45pt; }}
.page-break {{ page-break-before: always; }}
.figure {{ text-align: center; margin: 11pt 0 14pt 0; page-break-inside: avoid; }}
.figure img {{ width: 16.4cm; max-height: 20cm; object-fit: contain; }}
.caption {{ color: #56616c; font-size: 9pt; margin-top: 4pt; }}
.analogy {{ color: #294a5e; font-weight: bold; margin: 9pt 0; }}
.warning {{ color: #7a4d00; margin: 9pt 0; }}
.safety {{ color: #8a2937; margin: 9pt 0; }}
.success {{ color: #176b45; margin: 9pt 0; }}
.codeblock {{ font-family: 'DejaVu Sans Mono', monospace; background: #f1f3f5; border: 1px solid #ccd2d7; padding: 7pt; white-space: pre-wrap; font-size: 8.4pt; }}
.small {{ font-size: 9pt; color: #68727d; }}
</style>
</head>
<body>
<div class="cover">
  <div class="cover-title">从零理解小车轨迹跟踪<br>ROS2、PX4 与 Pair B 双车控制</div>
  <div class="cover-subtitle">结合本仓库代码和计划 8115 实车日志的入门教程</div>
  <div class="cover-meta">适合读者：第一次接触 ROS2 / MAVROS / Offboard / 多车协调<br>当前角色：Orin2 = Carrier/leader；Orin1 = Mini/executor<br>整理日期：2026-08-10</div>
</div>

<h1 class="page-break">1. 先建立一张整体地图</h1>
<div class="analogy"><b>可以把整套系统想成一支车队：</b>Carrier 是“领队和任务指挥员”，每辆车上的轨迹跟踪器是“本地司机”，PX4 是“稳定、守规矩的底盘控制器”，Arduino + D24A 是“把左右轮指令真正变成电流的执行层”，Pair B 是两车之间的“对讲机”。对讲机只说任务和状态，不替司机每 50 ms 打一次方向盘。</div>
<table>
<tr><th>组件</th><th>最简单的理解</th><th>实际职责</th></tr>
<tr><td>Boss / RC / QGC</td><td>现场安全员</td><td>确认场地、RC Kill、观察状态、必要时人工终止</td></tr>
<tr><td>Orin2 Carrier</td><td>领队</td><td>确定共享原点和阶段，接收 MiniState，发送命令，也执行自己的轨迹</td></tr>
<tr><td>Orin1 Mini</td><td>队员</td><td>验证无线命令是否新鲜合法，再启动本地轨迹</td></tr>
<tr><td>ROS2 + MAVROS</td><td>软件总线和翻译官</td><td>把 PX4 的状态变成 ROS topic，把 ROS setpoint/service 变成 MAVLink</td></tr>
<tr><td>PX4 Differential Rover</td><td>底盘飞控</td><td>管理模式、Arm、安全检查和左右执行器输出</td></tr>
<tr><td>Arduino + D24A</td><td>电机执行器</td><td>读取 MAIN1/MAIN2 PWM，处理死区/方向/超时，驱动四个轮子</td></tr>
</table>
<div class="figure"><img src="{asset('system_architecture.png')}" width="520"><div class="caption">图 1  系统分层：任务协调与本地高频闭环相互分离</div></div>

<h1 class="page-break">2. 一条轨迹是怎样让小车动起来的</h1>
<h2>2.1 轨迹不是“一串定时动作”</h2>
<p>最初可以写“前进 5 s、右转 3 s”，但地面摩擦、电池电压、轮子差异一变，车就走不到同一个位置。现在的轨迹是按米表示的一串二维点：每个点包含 <code>x_m</code>、<code>y_m</code> 和参考 <code>yaw_rad</code>，相邻点之间还有弧长进度 <code>s</code>。控制器关心“车现在离路径哪里最近、应该向前看多远”，而不是“计时器走了几秒”。</p>
<p>轨迹结构在 {link('src/orin2_trajectory_tracker.py', 27)}；直线、U 形和 S 形生成器分别在 {link('src/orin2_trajectory_tracker.py', 996)}、{link('src/orin2_trajectory_tracker.py', 1017)}、{link('src/orin2_trajectory_tracker.py', 1081)}。</p>

<h2>2.2 每个控制周期的六步</h2>
<ol>
  <li><b>读状态：</b>从 MAVROS 获取位置、速度、姿态、模式和 GPS 健康。</li>
  <li><b>找路径投影：</b>在当前位置附近寻找最近路径线段，得到进度 <code>s</code> 和横向误差。代码：{link('src/orin2_trajectory_tracker.py', 269)}。</li>
  <li><b>选前视点：</b>沿路径向前看一段距离。速度高时看远一点，速度低时看近一点。</li>
  <li><b>合成转向：</b>参考曲率告诉车“这条路本来要弯多少”；横向/航向反馈告诉车“现在还要额外修多少”。核心步骤：{link('src/orin2_trajectory_tracker.py', 585)}。</li>
  <li><b>限幅：</b>限制最大车体转向角、转向变化率、曲率修正和速度，避免突然 sharp turn。</li>
  <li><b>发布：</b>转换成 BODY_NED <code>linear.x</code>/<code>linear.y</code>，20 Hz 发给 MAVROS/PX4。</li>
</ol>
<div class="figure"><img src="{asset('tutorial_local_control.png')}" width="520"><div class="caption">图 2  从路径点到四轮电机的完整本地控制链</div></div>

<h2>2.3 为什么要有“前馈 + 反馈”</h2>
<div class="analogy"><b>开车过弯的直观例子：</b>看到前方道路本来就是左弯，提前向左打方向，这是“曲率前馈”；如果车已经偏到道路右边，再多向左修一点，这是“横向反馈”。只用前馈会无法抵消轮胎和地面误差，只用反馈则会在进入弯道后才发现偏了，容易滞后和摆动。</div>
<p>本项目把参考曲率、横向误差、航向误差和受限积分组合，再用近似关系把曲率变成 BODY_NED 的转向 bearing。相关函数包括：</p>
<ul>
  <li>{link('src/orin2_trajectory_tracker.py', 399)}：读取前方参考曲率；</li>
  <li>{link('src/orin2_trajectory_tracker.py', 478)}：计算有界反馈曲率；</li>
  <li>{link('src/orin2_trajectory_tracker.py', 526)}：更新横向误差积分并处理饱和；</li>
  <li>{link('src/orin2_trajectory_tracker.py', 369)}：限制转向 bearing 的变化率。</li>
</ul>

<h2>2.4 当前控制器到底是什么</h2>
<p>当前控制器更准确的名称是：<b>基于 Pure Pursuit 的几何轨迹跟踪器，加上路径曲率前馈、横向误差反馈/积分补偿、车辆适配和速度/转向限幅</b>。它每约 50 ms 读取一次车辆状态并重新计算命令，所以它是闭环自动驾驶；“不是 MPC”不等于“没有反馈”。完整单步入口在 {link('src/orin2_trajectory_tracker.py', 585)}。</p>
<p>Pure Pursuit 把车辆朝向前视点的夹角记为 <code>alpha</code>，前视距离记为 <code>L</code>，用下面的几何关系得到反馈曲率：</p>
<div class="codeblock">feedback_curvature = 2 * sin(alpha) / L</div>
<p>控制器再读取规划路径本身的参考曲率 <code>reference_curvature</code>。可以把两部分理解为：</p>
<ul>
  <li><b>参考曲率前馈：</b>道路本来要弯多少，车辆提前知道；</li>
  <li><b>Pure Pursuit 反馈：</b>车辆当前位置和姿态已经偏了多少；</li>
  <li><b>有限积分：</b>长期向同一侧偏时，逐渐抵消左右轮、摩擦和死区造成的固定偏差；</li>
  <li><b>车辆适配：</b>把几何曲率变成当前 PX4 Differential Rover 能执行的 BODY_NED bearing。</li>
</ul>
<p>核心合成过程在 {link('src/orin2_trajectory_tracker.py', 673)} 到 {link('src/orin2_trajectory_tracker.py', 745)}。当前实车入口默认使用约 1.10 m 基础前视、0.90～1.80 m 动态范围、1.28 倍反馈适配、0.04 的横向积分增益和 45 deg/s 转向变化率，参数集中在 {link('scripts/run_orin2_outdoor_forward_5m.sh', 141)}。</p>
<div class="warning"><b>它也不是普通 PID：</b>主转向量来自车辆、前视点和路径之间的非线性几何关系；代码中的积分项只是消除持续横向偏差。没有直接使用一组 <code>Kp/Ki/Kd</code> 对单个误差做完整 PID。</div>

<h2>2.5 它和 MPC 有什么区别</h2>
<p>MPC 是 Model Predictive Control，即模型预测控制。它不只问“现在该往哪边修”，而是用车辆模型模拟未来一段时间，比较许多候选控制序列，再选择综合代价最小的一组。只执行第一步，下一周期根据新状态重新求解。</p>
<div class="codeblock">典型 MPC：
minimize  sum(Qy * lateral_error^2
            + Qyaw * heading_error^2
            + Ru * control^2
            + Rdu * control_change^2)
subject to:
    state[k+1] = vehicle_model(state[k], control[k])
    speed / steering / acceleration / safety-distance constraints</div>
<h3 class="page-break">2.5.1 一张表看懂差别</h3>
<table>
<tr><th>比较项</th><th>当前几何控制器</th><th>MPC</th></tr>
<tr><td>基本思路</td><td>投影到路径，寻找前视点，立即计算曲率</td><td>预测未来 N 步，求解最优控制序列</td></tr>
<tr><td>未来信息</td><td>通过前视点和参考曲率做局部预判</td><td>明确计算未来位置、姿态、速度和约束</td></tr>
<tr><td>车辆模型</td><td>只需少量几何/车辆适配参数</td><td>需要可信的运动学或动力学模型</td></tr>
<tr><td>约束处理</td><td>命令算出后再限速、限转向、限变化率</td><td>把转向、速度、加速度和安全距离直接放入优化</td></tr>
<tr><td>计算量</td><td>小，20 Hz 运行简单可靠</td><td>较大，每周期都要运行数值优化器</td></tr>
<tr><td>双车关系</td><td>Pair B 做高层阶段协调，两车各自闭环</td><td>可以预测两车未来轨迹，把相对距离纳入代价和约束</td></tr>
<tr><td>主要风险</td><td>急弯和执行器迟滞需要调前视/增益</td><td>模型不准、求解超时或权重不当会导致效果变差</td></tr>
</table>
<p>形象地说，当前控制器像司机“看着前方一段路持续修方向”；MPC 像司机在脑中同时试演未来几秒的多种走法，然后选择既贴路、又平顺、还不违反限制的一种。</p>
<h3>2.5.2 为什么现阶段没有直接换成 MPC</h3>
<p>小车阶段的首要目标是验证 Offboard、共享坐标、通用轨迹、Pair B 消息和双车状态机。当前控制器计算便宜、故障容易解释，而且已经在直线、U 形和连续变曲率轨迹上形成实车证据。此时直接引入 MPC，会同时增加车辆建模、求解器实时性、权重选择和约束调试等变量。</p>
<p>MPC 也不会自动修复 GPS 漂移、磁场干扰、坐标系符号错误、PWM 死区、轮胎打滑或 Pair B 过期消息。这些基础状态和执行链必须先可靠，否则 MPC 只是基于错误输入做更复杂的计算。</p>
<h3>2.5.3 迁移到飞机时怎样分层</h3>
<p>合理演进不是推翻现有系统，而是保持分层：EasyDocking/Carrier 继续生成共享走廊和任务阶段；预测控制层根据两架飞机模型、未来走廊和相对约束生成速度/姿态参考；PX4 仍负责底层姿态、速度和执行器稳定。小车上的几何控制器仍可作为简单基线和故障回退，用来判断新 MPC 是否真的带来收益。</p>

<h1 class="page-break">3. 坐标系：为什么图上向左，车身命令可能是负 y</h1>
<div class="figure"><img src="{asset('tutorial_frames.png')}" width="520"><div class="caption">图 3  Field ENU 与 BODY_NED 的用途和方向</div></div>
<p><b>Field ENU</b> 是两车共享的地图坐标：+x 向东，+y 向北，yaw 从东向逆时针为正。它适合表达“Carrier 在 Mini 后方几米”“共享走廊朝哪个方向”。</p>
<p><b>BODY_NED</b> 是每辆车自己的命令坐标：+x 指向车头，+y 指向车体右侧。地图上的目标向量必须减去当前位置，再按当前 yaw 旋转到车体坐标，才能变成“向前多少、向哪边修多少”。</p>
<div class="warning"><b>本项目的关键实测：</b>当前 PX4 Differential Rover 的 MAVROS velocity 路径中，差速转向要核验 <code>linear.y</code>；不能直接照搬常见移动机器人用 <code>angular.z</code> 的习惯。发布和 BODY_NED 回读在 {link('src/orin2_outdoor_forward_5m.py', 1316)} 与 {link('src/orin2_outdoor_forward_5m.py', 1586)}。</div>

<h1 class="page-break">4. ROS2、MAVROS、PX4 和 Arduino 如何接力</h1>
<h2>4.1 ROS2 / MAVROS 侧</h2>
<table>
<tr><th>接口</th><th>用途</th><th>程序怎么用</th></tr>
<tr><td><code>/mavros/state</code></td><td>connected、armed、mode、manual_input</td><td>判断能否进入、是否意外退出</td></tr>
<tr><td><code>/mavros/local_position/pose</code></td><td>本地位置和姿态</td><td>路径投影、yaw、实际轨迹</td></tr>
<tr><td><code>/mavros/local_position/velocity_local</code></td><td>实际速度</td><td>前视距离和停滞检测</td></tr>
<tr><td><code>/mavros/global_position/global</code></td><td>WGS84 GPS</td><td>共享 FieldOrigin 和 GPS 健康</td></tr>
<tr><td><code>/mavros/statustext/recv</code></td><td>PX4 文字事件</td><td>解释 Arm 拒绝、failsafe、磁干扰等</td></tr>
<tr><td><code>/mavros/setpoint_velocity/cmd_vel</code></td><td>BODY_NED 速度 setpoint</td><td>正常时 20 Hz 发布；异常时归零</td></tr>
<tr><td><code>/mavros/set_mode</code></td><td>请求 OFFBOARD / MANUAL</td><td>请求后继续看 <code>state.mode</code></td></tr>
<tr><td><code>/mavros/cmd/arming</code></td><td>Arm / Disarm</td><td>Arm 只请求一次；Disarm 后看 <code>state.armed</code></td></tr>
</table>
<p>这些 publisher/subscriber/client 集中在 {link('src/orin2_outdoor_forward_5m.py', 1284)} 之后。两车运行时都设置 <code>ROS_LOCALHOST_ONLY=1</code>，防止同名 <code>/mavros/*</code> topic 在局域网 DDS 中互相串车。</p>

<h2>4.2 PX4 到四个轮子</h2>
<p>PX4 rover controller 把前进和转向需求混合成左右两个执行器，输出 MAIN1/MAIN2。Arduino 只看到两侧 PWM，不知道全局轨迹。它负责：</p>
<ul>
  <li>判断 PWM 是否在合法范围、是否超时；</li>
  <li>以 1500 μs 为中点处理正反向和死区；</li>
  <li>左侧命令同时驱动左前/左后，右侧命令同时驱动右前/右后；</li>
  <li>输入无效或命令为零时关闭电机输出。</li>
</ul>
<p>参数和固定 by-id 检查在 {link('scripts/run_orin2_outdoor_forward_5m.sh', 297)}；Arduino 当前映射见 <code>arduino/d24a_pixhawk_differential_pwm_bridge/d24a_pixhawk_differential_pwm_bridge.ino</code> 和 <code>docs/d24a_current_motor_mapping.md</code>。</p>

<h1 class="page-break">5. Offboard 为什么必须用状态机</h1>
<div class="figure"><img src="{asset('tutorial_offboard_state_machine.png')}" width="520"><div class="caption">图 4  单车进入、运动与退出的 fail-closed 状态机</div></div>
<p>OFFBOARD 不是“调一个 service 就开始跑”。PX4 要先看到持续 setpoint 流，才允许保持 OFFBOARD。正确顺序是：</p>
<div class="codeblock">MANUAL + disarmed + fresh state
  → 20 Hz 全零预流 2 s
  → request OFFBOARD，等待 state.mode == OFFBOARD
  → request Arm once，等待 state.armed == true
  → 运行轨迹
  → 全零 burst
  → request Disarm，等待 armed == false
  → request MANUAL，等待 mode == MANUAL</div>
<p>早期脚本在已经观察到 armed+OFFBOARD 后又等待 2 s，消耗了 PX4 的状态/自动解锁窗口。现在只保留进入模式之前的零预流，真实进入后立即开始第一段受限动作。外部 Pair B 启动门在 {link('src/orin2_outdoor_forward_5m.py', 2080)}，核心 live 状态机从 {link('src/orin2_outdoor_forward_5m.py', 1284)} 开始。</p>
<div class="safety"><b>最重要的原则：</b>service success 不是车辆状态。代码必须看到 PX4 State 真正变成 OFFBOARD/armed，退出时也必须看到 MANUAL/disarmed。任何中间状态超时都进入零输出恢复，不自动重 Arm。</div>

<h1 class="page-break">6. Pair B 到底怎么传命令</h1>
<div class="figure"><img src="{asset('tutorial_pairb_wire.png')}" width="520"><div class="caption">图 5  Pair B 从 Orin2 ROS2 进程，经 Pixhawk TELEM2 和 LR24 到 Orin1 的物理/软件链路</div></div>
<h2>6.1 为什么 Orin2 没有 Pair B USB 串口</h2>
<p>Orin2 一侧的 Pair B VEHICLE 电台接在 Mini Pixhawk 的 TELEM2。Orin2 程序先把紧凑帧装入 MAVLink TUNNEL，再通过本机 Pixhawk USB 和 MAVROS Router 送到 TELEM2。Orin1 一侧的 GROUND 电台直接通过 CP2102 USB 接 Linux，因此 Orin1 程序直接打开稳定的 <code>/dev/serial/by-id/...</code>。</p>
<p>TUNNEL 适配代码：{link('src/lr24_mavlink_tunnel.py', 45)}；Orin2 ROS2 Router publisher/subscriber：{link('src/lr24_mavlink_tunnel.py', 275)}；实车总控根据角色选择 transport：{link('scripts/run_pairb_staged_chase_live.py', 703)}。</p>

<h2>6.2 一条消息有哪几层</h2>
<div class="codeblock">LR24 无线字节流
└─ MAVLink 2 TUNNEL（msgid 385，component 242）
   └─ L2 紧凑帧
      ├─ magic = "L2"
      ├─ version / message_type / payload_length
      ├─ payload（MiniState、PlanCommand…）
      └─ CRC16/CCITT</div>
<p>L2 帧头和 CRC 实现在 {link('src/lr24_compact_protocol.py', 24)} 与 {link('src/lr24_compact_protocol.py', 176)}。一条 L2 帧只放进一条 TUNNEL，不拆分、不拼接，因此接收端容易 fail-closed 校验。</p>

<h2>6.3 最重要的消息</h2>
<table>
<tr><th>消息</th><th>方向</th><th>关键字段</th><th>用途</th></tr>
<tr><td>FieldOrigin</td><td>Carrier → Mini</td><td>origin_id、经纬高、seq、timestamp</td><td>建立共同 ENU；origin_id 不匹配就拒绝任务</td></tr>
<tr><td>MiniState</td><td>Mini → Carrier</td><td>x/y、vx/vy、yaw、omega、health、origin_id</td><td>让 Carrier 知道 Mini 在哪里、是否健康</td></tr>
<tr><td>PlanCommand</td><td>Carrier → Mini</td><td>plan_id、phase、seq、timestamp、valid_until、v/omega</td><td>告诉 Mini 当前阶段；HOLD/STOP 必须全零</td></tr>
<tr><td>MissionStatus</td><td>双向</td><td>WAITING/RUNNING/COMPLETE/FAILED/STOPPED</td><td>独立于 MAVROS health 表示本地 worker 生命周期</td></tr>
<tr><td>Abort</td><td>双向</td><td>source、reason、plan_id、seq、timestamp</td><td>最高优先级；接收后锁存停止</td></tr>
<tr><td>CorridorPlan</td><td>Carrier → Mini</td><td>切点、切线、走廊长度、到达时序、前后间距</td><td>后续 EasyDocking 轨道切出与共享末端走廊</td></tr>
</table>
<p>结构定义位置：MiniState {link('src/lr24_compact_protocol.py', 254)}，PlanCommand {link('src/lr24_compact_protocol.py', 313)}，MissionStatus {link('src/lr24_compact_protocol.py', 467)}，Abort {link('src/lr24_compact_protocol.py', 693)}，FieldOrigin {link('src/lr24_compact_protocol.py', 727)}。</p>

<h2>6.4 没有同步时钟，TTL 怎么办</h2>
<p>两台 Orin 的 monotonic clock 数值不需要相等。发送端只把 <code>timestamp_ms</code> 和 <code>valid_until_ms</code> 的差值当作 TTL；接收端在本地收到消息时重新启动这个 TTL 计时。同时还有本地 watchdog。序号采用 uint32 回绕规则，重复和旧命令被拒绝。安全门在 {link('src/lr24_command_guard.py', 51)}。</p>

<h1 class="page-break">7. 双车运动具体如何实现</h1>
<div class="figure"><img src="{asset('tutorial_dual_sequence.png')}" width="520"><div class="caption">图 6  计划 8115 的消息与动作时序</div></div>
<h2>7.1 启动前</h2>
<ol>
  <li>两台车分别启动本地 MAVROS 和轨迹 worker，但 worker 被管道 gate 卡在 MANUAL/disarmed，不发布非零 setpoint。</li>
  <li>Carrier 从本车 GPS 锁定 FieldOrigin 并广播。</li>
  <li>Mini 把自身 GPS/Local Position 转到共享 ENU，持续发送 MiniState。</li>
  <li>Carrier 检查本车 ready、Mini health、origin_id、Pair B 会话和现场授权，条件不全就只发 HOLD。</li>
</ol>
<p>worker 进程包装和 gate：{link('scripts/run_pairb_staged_chase_live.py', 391)}；等待 MAVROS/FieldOrigin：{link('scripts/run_pairb_staged_chase_live.py', 644)}、{link('scripts/run_pairb_staged_chase_live.py', 670)}。</p>

<h2>7.2 Mini 先走，Carrier 后走</h2>
<p>Carrier 的协调核心是 {link('src/pairb_staged_chase.py', 242)}。它先把阶段从 HOLD 切到 MINI_ACTIVE，向 Mini 发送 TRAJECTORY。Mini 接受后释放本地 worker gate，自行完成 OFFBOARD→Arm→轨迹跟踪。Carrier 同时要求：</p>
<ul>
  <li>Mini 已经运动至少 5 s；</li>
  <li>Mini 相对起点前进至少 2 m；</li>
  <li>两车状态与 Pair B 消息仍新鲜。</li>
</ul>
<p>全部成立后才进入 BOTH_ACTIVE，释放 Carrier 本地 worker。这里的“同时运动”不是同一 CPU 时钟同一微秒起步，而是共享阶段约束下的有界先后顺序。</p>

<h2>7.3 Mini 停后 Carrier 为什么还能继续</h2>
<p>Mini 完成路线后会 Disarm，因此后续 MiniState 不再带完整 executor-ready health。早期版本可能先收到这个状态，再收到 COMPLETE，于是 Carrier 误判 Mini 故障并跟着停。修复后：</p>
<ol>
  <li>Mini 在第一条停车后 MiniState 之前，立即发送一次 MissionStatus=COMPLETE；</li>
  <li>Carrier 对“位置仍有效但 executor-ready 已撤销”的状态提供 500 ms 有界终态等待；</li>
  <li>COMPLETE 一旦通过 plan_id/seq/时效验证就锁存，不要求反复重发；</li>
  <li>Carrier 继续使用新鲜 MiniState 位置计算终端纵向/横向间距。</li>
</ol>
<p>终态优先发送：{link('scripts/run_pairb_staged_chase_live.py', 595)}；终端间距计算与停止判定：{link('src/pairb_staged_chase.py', 109)}、{link('src/pairb_staged_chase.py', 143)}。</p>

<h2>7.4 计划 8115 的实车数据</h2>
<ul>
  <li>初始队形注册：Mini 在 Carrier 前方 0.50 m；原始 GPS 相对量 (+0.73,+0.02) m。</li>
  <li>Carrier 在 Mini 领先 2.07 m 后启动。</li>
  <li>Mini 完成后，Carrier 估计初始落后 9.83 m，随后继续约 10 s。</li>
  <li>终端连续 3 个样本满足窗口：纵向 2.60 m、横向 -0.43 m。</li>
  <li>现场卷尺约 3.20 m，普通双 GPS 叠加误差约 0.60 m。</li>
  <li>两端最终 <code>MISSION_RC=0</code>、<code>FINAL_RECOVERY_SAFE=True</code>。</li>
</ul>
<div class="figure"><img src="{asset('dual_actual.png')}" width="520"><div class="caption">图 7  计划 8115 两车实际轨迹（Carrier 黄色、Mini 绿色）</div></div>
<div class="figure"><img src="{asset('terminal_gap.png')}" width="520"><div class="caption">图 8  Mini 停止后 Carrier 继续靠近，直到终端间距窗口</div></div>

<h1 class="page-break">8. RViz 和日志怎么看</h1>
<h2>8.1 RViz 只负责“看”，不负责“开”</h2>
<p>单车 controller 会发布 planned_path、actual_path、vehicle_pose 和 lookahead_target。双车总控把两车轨迹转换到共享 frame 后发布到 <code>/pairb/carrier/*</code> 与 <code>/pairb/mini/*</code>。当前 RViz 配置按现场要求只显示两条实际轨迹。</p>
<p>可视化代码：<code>src/rover_rviz_trajectory.py</code>、<code>src/pairb_dual_trajectory.py</code>；配置：<code>config/rviz/pairb_dual_trajectory.rviz</code>。即使 RViz 崩溃，控制器和安全状态机仍独立运行。</p>

<h2>8.2 单车 mission.log 的阅读顺序</h2>
<div class="codeblock">BODY_NED_VERIFIED
EXTERNAL_START_GATE_ACCEPTED
ZERO_PRESTREAM
OFFBOARD_REQUEST_ACCEPTED
ARM_ONCE_RESPONSE
TRAJECTORY_REFERENCE_LOCKED
TRAJECTORY_PROGRESS ... cross= ... curvature= ... body=(x,y)
TRAJECTORY_TARGET_REACHED 或 MISSION_ABORT
FINAL_RECOVERY_SAFE=True</div>
<p>先看任务是否真的进入 OFFBOARD/armed，再看 reference 是怎样锁定的，然后看 progress/cross/curvature。最后必须看 recovery；只看到“车停了”不等于程序证明了 MANUAL/disarmed。</p>

<h2>8.3 双车 supervisor.log 的阅读顺序</h2>
<div class="codeblock">LIVE_READY
FIELD_ORIGIN_LOCKED
FORMATION_REGISTERED
PAIRB_DECISION phase=mini_active
MINI_START_RELEASED
PAIRB_DECISION phase=both_active
CARRIER_START_RELEASED
MINI_TERMINAL_STATUS_SENT state=COMPLETE
CARRIER_TERMINAL_GAP ...
CARRIER_TERMINAL_STOP_REQUESTED
MISSION_RC=0</div>
<p>代表性文件：<code>results/pairb_staged_live/20260809_plan8115_carrier/supervisor.log</code>；双车 CSV：<code>results/pairb_dual_trajectory/plan8115/</code>。</p>

<h1 class="page-break">9. 仓库代码地图</h1>
<table>
<tr><th>模块</th><th>仓库位置</th><th>主要内容</th></tr>
{code_rows}
</table>
<h2>9.1 推荐阅读顺序</h2>
<ol>
  <li>先读 <code>docs/orin2_outdoor_forward_5m_runbook.md</code>，理解安全流程。</li>
  <li>看 <code>src/orin2_trajectory_tracker.py:585</code>，配合图 2 理解单步跟踪。</li>
  <li>看 <code>src/orin2_outdoor_forward_5m.py:1284</code>，理解 ROS2 与 Offboard 状态机。</li>
  <li>读 <code>docs/lr24_pairb_wire_contract_v1.md</code>，再看协议结构体。</li>
  <li>最后看 <code>src/pairb_staged_chase.py</code> 和 <code>scripts/run_pairb_staged_chase_live.py</code>，把两车状态机串起来。</li>
</ol>

<h1>10. 当前做到哪里，下一步是什么</h1>
<div class="success"><b>已经做到：</b>单车 Offboard 安全进入/退出、通用二维轨迹跟踪、Pair B 双向任务/状态、Mini 先行与 Carrier 延迟追随、Mini 停后 Carrier 继续到目标间距、双车 RViz 和日志闭环。</div>
<div class="warning"><b>还没有做到：</b>当前双车 S 路线不是完整空中 EasyDocking。真正下一阶段要让 Mini 先稳定绕圈一周，Carrier 根据状态生成平滑进场，Mini 在指定相位沿切线切出，双方进入同一 terminal tangent corridor，并始终满足 Carrier 在前。</div>
<p>上 RTK 后，先确认两车均为 FIX、共享原点一致、相对静止位置稳定，再把当前约 ±0.75 m 的终端窗口逐步收紧。不要用一组普通 GPS 误差去硬调固定补偿。</p>

<h1>附录：一句话记忆</h1>
<ul>
  <li><b>轨迹跟踪：</b>先找自己在路上的位置，再看前方一点，结合道路弯度和偏差打方向。</li>
  <li><b>Offboard：</b>先用零流证明“我会持续发命令”，再切模式、Arm、运动；结束必须回读安全状态。</li>
  <li><b>Pair B：</b>只传任务、状态和急停，不传高频方向盘。</li>
  <li><b>双车：</b>Carrier 决定阶段，两车各自闭环；无线失效时，本车必须能自己停。</li>
  <li><b>定位：</b>普通 GPS 能看大轨迹形状，RTK 才适合收紧末端相对距离。</li>
</ul>
</body>
</html>
"""
    HTML_OUT.write_text(document, encoding="utf-8")
    print(f"HTML={HTML_OUT}")


if __name__ == "__main__":
    build_tutorial()
