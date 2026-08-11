#!/usr/bin/env python3

from __future__ import annotations

import html
import math
import re
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


REPO = Path(__file__).resolve().parents[1]
DOCS = REPO / "docs"
FONT_REGULAR = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
FONT_BOLD = "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"
INK = "#20252b"
MUTED = "#68727d"
BLUE = "#1f6f9f"
CARRIER = "#e0a400"
MINI = "#19a95b"
RED = "#d1495b"


def font(size: int, *, bold: bool = False) -> ImageFont.FreeTypeFont:
    path = FONT_BOLD if bold and Path(FONT_BOLD).is_file() else FONT_REGULAR
    return ImageFont.truetype(path, size=size)


SOURCE = DOCS / "three_stage_mock_to_aerial_docking_roadmap_2026_08_11.md"
ASSETS = DOCS / "report_assets" / "three_stage_roadmap_20260811"
HTML_OUT = DOCS / "three_stage_mock_to_aerial_docking_roadmap_2026-08-11.html"


def arrow(
    draw: ImageDraw.ImageDraw,
    start: tuple[int, int],
    end: tuple[int, int],
    color: str = "#59636e",
    width: int = 6,
) -> None:
    draw.line((*start, *end), fill=color, width=width)
    angle = math.atan2(end[1] - start[1], end[0] - start[0])
    length = 19
    spread = 0.48
    draw.polygon(
        [
            end,
            (
                round(end[0] - length * math.cos(angle - spread)),
                round(end[1] - length * math.sin(angle - spread)),
            ),
            (
                round(end[0] - length * math.cos(angle + spread)),
                round(end[1] - length * math.sin(angle + spread)),
            ),
        ],
        fill=color,
    )


def box(
    draw: ImageDraw.ImageDraw,
    bounds: tuple[int, int, int, int],
    title: str,
    lines: list[str],
    color: str,
    title_size: int = 29,
    body_size: int = 22,
) -> None:
    x1, y1, x2, y2 = bounds
    draw.rounded_rectangle(bounds, radius=18, fill="white", outline=color, width=5)
    draw.rounded_rectangle((x1, y1, x2, y1 + 66), radius=18, fill=color)
    draw.rectangle((x1, y1 + 35, x2, y1 + 66), fill=color)
    draw.text((x1 + 20, y1 + 13), title, font=font(title_size, bold=True), fill="white")
    for index, line in enumerate(lines):
        draw.text(
            (x1 + 22, y1 + 88 + index * 38),
            line,
            font=font(body_size),
            fill=INK,
        )


def draw_roadmap(output: Path) -> None:
    image = Image.new("RGB", (1900, 1080), "#f7f9fb")
    draw = ImageDraw.Draw(image)
    draw.text((75, 40), "从双车验证到真实双机空中对接", font=font(46, bold=True), fill=INK)
    draw.text((75, 100), "逐级替换执行平台，保留规划、通信、状态机和安全边界", font=font(25), fill=MUTED)
    stages = [
        (
            (60, 205, 580, 690),
            "第一阶段：双车",
            [
                "Carrier 统一发布任务",
                "Mini 绕圈后切线切出",
                "动态时空走廊与速度协同",
                "验证 Pair B、状态机和日志",
                "出口：真正协同，不是简单跟随",
            ],
            BLUE,
        ),
        (
            (690, 205, 1210, 690),
            "第二阶段：双四轴",
            [
                "快四轴受约束模拟固定翼 Mini",
                "慢四轴作为 Carrier",
                "二维走廊扩展到三维",
                "先大间距，再无接触接近",
                "出口：固定翼等效约束下协同",
            ],
            "#6f42c1",
        ),
        (
            (1320, 205, 1840, 690),
            "第三阶段：真实固定翼",
            [
                "真实固定翼替换快四轴",
                "加入空速、风、TECS 和飞行包线",
                "RTK/视觉承担终端相对导航",
                "验证复飞和接触后状态",
                "出口：受控真实空中对接",
            ],
            "#198754",
        ),
    ]
    for bounds, title, lines, color in stages:
        box(draw, bounds, title, lines, color)
    arrow(draw, (590, 445), (680, 445), width=8)
    arrow(draw, (1220, 445), (1310, 445), width=8)

    draw.rounded_rectangle(
        (120, 785, 1780, 990),
        radius=22,
        fill="#fff8ec",
        outline="#bd7b00",
        width=4,
    )
    draw.text((160, 810), "跨阶段保留的核心资产", font=font(31, bold=True), fill="#9a6400")
    retained = [
        "Carrier leader",
        "CorridorPlan / Pair B",
        "本地闭环适配器",
        "共享坐标与时效",
        "Abort / RC / 日志",
    ]
    for index, item in enumerate(retained):
        x = 160 + index * 315
        draw.rounded_rectangle(
            (x, 875, x + 275, 955),
            radius=14,
            fill="white",
            outline="#bd7b00",
            width=3,
        )
        draw.text((x + 14, 898), item, font=font(19, bold=True), fill=INK)
    output.parent.mkdir(parents=True, exist_ok=True)
    image.save(output, quality=95)


def draw_stage1_geometry(output: Path) -> None:
    image = Image.new("RGB", (1900, 1160), "#fbfcfd")
    draw = ImageDraw.Draw(image)
    draw.text((70, 38), "第一阶段：圆周、切线、共享终端走廊", font=font(44, bold=True), fill=INK)
    draw.text((70, 96), "两车执行不同路径，但由 Carrier 规划到同一个未来时空目标", font=font(24), fill=MUTED)

    center = (510, 535)
    radius = 245
    circle = (
        center[0] - radius,
        center[1] - radius,
        center[0] + radius,
        center[1] + radius,
    )
    draw.ellipse(circle, outline=MINI, width=10)
    draw.arc(circle, 25, 320, fill=MINI, width=13)
    draw.text((285, 820), "Mini：先完整稳定绕行一圈", font=font(25, bold=True), fill=MINI)

    tangent = (510, 290)
    corridor_end = (1690, 290)
    arrow(draw, tangent, corridor_end, color=MINI, width=11)
    draw.ellipse(
        (tangent[0] - 13, tangent[1] - 13, tangent[0] + 13, tangent[1] + 13),
        fill=RED,
    )
    draw.text((470, 230), "切点 T", font=font(23, bold=True), fill=RED)
    draw.line((tangent[0], 238, 1770, 238), fill="#bda6d8", width=3)
    draw.line((tangent[0], 342, 1770, 342), fill="#bda6d8", width=3)
    draw.text((1010, 195), "共同 terminal tangent corridor", font=font(25, bold=True), fill="#6f42c1")
    draw.text((1190, 375), "Carrier 在前，Mini 从后方收敛", font=font(23, bold=True), fill=BLUE)

    carrier_start = (170, 1010)
    control1 = (50, 600)
    control2 = (220, 290)
    points: list[tuple[int, int]] = []
    for index in range(101):
        t = index / 100.0
        omt = 1.0 - t
        x = (
            omt**3 * carrier_start[0]
            + 3 * omt**2 * t * control1[0]
            + 3 * omt * t**2 * control2[0]
            + t**3 * tangent[0]
        )
        y = (
            omt**3 * carrier_start[1]
            + 3 * omt**2 * t * control1[1]
            + 3 * omt * t**2 * control2[1]
            + t**3 * tangent[1]
        )
        points.append((round(x), round(y)))
    draw.line(points, fill=CARRIER, width=12, joint="curve")
    arrow(draw, points[-15], tangent, color=CARRIER, width=10)
    draw.ellipse(
        (
            carrier_start[0] - 14,
            carrier_start[1] - 14,
            carrier_start[0] + 14,
            carrier_start[1] + 14,
        ),
        fill=CARRIER,
    )
    draw.text((70, 1040), "Carrier 起点和初始航向", font=font(23, bold=True), fill=CARRIER)
    draw.text((370, 900), "航向可行的平滑接近", font=font(25, bold=True), fill=CARRIER)

    box(
        draw,
        (930, 650, 1800, 1055),
        "Carrier 的协同判断",
        [
            "比较两车沿程进度 s 和预计到达时间 ETA",
            "Mini 落后：Carrier 有界降速",
            "Mini 过近：Carrier 加速或命令 Mini 降速",
            "持续不同步、状态陈旧或断链：HOLD / ABORT",
            "只调整速度包络，不追逐瞬时 GPS 点",
        ],
        BLUE,
        title_size=30,
        body_size=22,
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    image.save(output, quality=95)


def draw_control_layers(output: Path) -> None:
    image = Image.new("RGB", (1900, 1080), "#f7f9fb")
    draw = ImageDraw.Draw(image)
    draw.text((70, 38), "规划低频，本车轨迹控制高频", font=font(44, bold=True), fill=INK)
    draw.text((70, 96), "Pair B 延迟不能直接变成车轮或飞行器的高频控制抖动", font=font(24), fill=MUTED)

    box(
        draw,
        (80, 210, 480, 515),
        "Mini → Carrier",
        ["MiniState 5–10 Hz", "位置 / 速度 / yaw", "健康 / 阶段 / 报文时间"],
        MINI,
    )
    box(
        draw,
        (750, 175, 1150, 560),
        "Carrier Planner",
        [
            "选择短切线和走廊",
            "生成两条未来轨迹",
            "计算 gap / ETA 误差",
            "更新阶段与速度包络",
            "不可行时输出 Abort",
        ],
        BLUE,
    )
    box(
        draw,
        (1420, 210, 1820, 515),
        "Carrier → Mini",
        ["CorridorPlan 低频", "PlanCommand / phase", "速度边界 / TTL / Abort"],
        "#6f42c1",
    )
    arrow(draw, (490, 350), (740, 350), color=MINI, width=8)
    arrow(draw, (1160, 350), (1410, 350), color=BLUE, width=8)

    box(
        draw,
        (230, 700, 750, 990),
        "Orin2 Carrier 本地控制",
        ["20 Hz 轨迹投影和动态前视", "曲率前馈 + 反馈/积分", "BODY_NED → PX4 → 四轮"],
        CARRIER,
    )
    box(
        draw,
        (1150, 700, 1670, 990),
        "Orin1 Mini 本地控制",
        ["校验 plan_id / seq / TTL", "20 Hz 跟踪自己的轨迹", "超时、RC Stop、故障本地停车"],
        MINI,
    )
    arrow(draw, (850, 570), (610, 690), color=CARRIER, width=7)
    arrow(draw, (1050, 570), (1290, 690), color=MINI, width=7)
    draw.rounded_rectangle(
        (520, 585, 1380, 665),
        radius=14,
        fill="#fff4e8",
        outline=RED,
        width=3,
    )
    draw.text((555, 607), "Pair B 不发送每 50 ms 的方向盘/电机命令", font=font(25, bold=True), fill=RED)
    output.parent.mkdir(parents=True, exist_ok=True)
    image.save(output, quality=95)


def inline_markup(value: str) -> str:
    escaped = html.escape(value)
    escaped = re.sub(r"\*\*(.+?)\*\*", r"<b>\1</b>", escaped)
    escaped = re.sub(r"\x60([^\x60]+)\x60", r"<code>\1</code>", escaped)
    return escaped


def markdown_to_html(source: str) -> str:
    lines = source.splitlines()
    output: list[str] = []
    paragraph: list[str] = []
    list_type: str | None = None
    code_lines: list[str] = []
    in_code = False

    def flush_paragraph() -> None:
        if paragraph:
            output.append("<p>" + inline_markup(" ".join(paragraph)) + "</p>")
            paragraph.clear()

    def close_list() -> None:
        nonlocal list_type
        if list_type is not None:
            output.append(f"</{list_type}>")
            list_type = None

    for raw_line in lines:
        line = raw_line.rstrip()
        if line.startswith("\x60\x60\x60"):
            flush_paragraph()
            close_list()
            if in_code:
                output.append("<pre>" + html.escape("\n".join(code_lines)) + "</pre>")
                code_lines.clear()
                in_code = False
            else:
                in_code = True
            continue
        if in_code:
            code_lines.append(line)
            continue
        if not line:
            flush_paragraph()
            close_list()
            continue

        heading = re.match(r"^(#{1,4})\s+(.+)$", line)
        if heading:
            flush_paragraph()
            close_list()
            level = len(heading.group(1))
            title = inline_markup(heading.group(2))
            css_class = ' class="page-break"' if level == 2 else ""
            output.append(f"<h{level}{css_class}>{title}</h{level}>")
            if heading.group(2) == "总体阶段":
                output.append(
                    '<div class="figure"><img src="ROADMAP_IMAGE" width="520">'
                    '<div class="caption">图 1　三阶段路线及跨阶段保留的软件资产</div></div>'
                )
            if heading.group(2).startswith("第一阶段：双车协同"):
                output.append(
                    '<div class="figure"><img src="GEOMETRY_IMAGE" width="520">'
                    '<div class="caption">图 2　圆周切出、Carrier 平滑接近和共享终端切线走廊</div></div>'
                )
            continue

        bullet = re.match(r"^-\s+(.+)$", line)
        ordered = re.match(r"^\d+\.\s+(.+)$", line)
        if bullet or ordered:
            flush_paragraph()
            expected = "ul" if bullet else "ol"
            if list_type != expected:
                close_list()
                list_type = expected
                output.append(f"<{list_type}>")
            item = bullet.group(1) if bullet else ordered.group(1)
            output.append("<li>" + inline_markup(item) + "</li>")
            continue
        paragraph.append(line.strip())

    flush_paragraph()
    close_list()
    if in_code:
        output.append("<pre>" + html.escape("\n".join(code_lines)) + "</pre>")
    return "\n".join(output)


def build_report() -> None:
    ASSETS.mkdir(parents=True, exist_ok=True)
    roadmap = ASSETS / "three_stage_roadmap.png"
    geometry = ASSETS / "stage1_geometry.png"
    controls = ASSETS / "stage1_control_layers.png"
    draw_roadmap(roadmap)
    draw_stage1_geometry(geometry)
    draw_control_layers(controls)

    body = markdown_to_html(SOURCE.read_text(encoding="utf-8"))
    roadmap_url = roadmap.relative_to(DOCS).as_posix()
    geometry_url = geometry.relative_to(DOCS).as_posix()
    controls_url = controls.relative_to(DOCS).as_posix()
    body = body.replace("ROADMAP_IMAGE", roadmap_url)
    body = body.replace("GEOMETRY_IMAGE", geometry_url)

    review = f"""
<h1 class="page-break">审查重点：第一阶段怎样证明“真正协同”</h1>
<div class="decision"><b>判定原则：</b>两车分别完成预录轨迹不算协同。必须故意制造 Mini 或 Carrier 的进度滞后，并在另一台车的速度包络、实际速度和终端到达时间中看到正确响应。</div>
<div class="figure"><img src="{controls_url}" width="520"><div class="caption">图 3　Carrier 低频规划协调，两车本地高频闭环</div></div>
<div class="equation">gap = s_carrier - s_mini
gap_error = gap - desired_gap

Mini 落后：Carrier 在安全包线内降速
Mini 过近：Carrier 加速或要求 Mini 降速
长期无法同步：HOLD / ABORT</div>
<table>
<tr><th>离线案例</th><th>注入条件</th><th>必须观察到的结果</th></tr>
<tr><td>Nominal</td><td>速度和链路正常</td><td>一圈后切出，进入同一走廊，间距收敛</td></tr>
<tr><td>Mini 落后</td><td>降低 Mini 可用速度</td><td>Carrier 有界降速；持续不可达则 Abort</td></tr>
<tr><td>Carrier 落后</td><td>降低 Carrier 可用速度</td><td>延后 Mini 切出或调整 Mini 速度，禁止超车</td></tr>
<tr><td>Pair B 陈旧</td><td>停止或延迟报文</td><td>本地 watchdog 触发 HOLD/STOP，不沿用旧命令</td></tr>
</table>
<div class="success"><b>建议的下一项开发：</b>先完成纯软件“Mini 落后 → Carrier 自动降速”案例。它最直接地证明系统已从简单跟随升级为协同控制。</div>
<h1>审查时需要确认的四项决策</h1>
<table>
<tr><th>待确认项</th><th>建议基线</th></tr>
<tr><td>Stage 1 终端目标间距</td><td>普通 GPS 阶段保持大间距，RTK 稳定后再收紧</td></tr>
<tr><td>速度协同优先级</td><td>优先调 Carrier；Mini 始终满足固定翼等效最小前进速度</td></tr>
<tr><td>走廊更新方式</td><td>几何计划尽量不变，低频更新阶段和速度包络</td></tr>
<tr><td>Carrier 平滑接近路线</td><td>比较 biarc 与 clothoid，再按曲率、长度和实现复杂度选择</td></tr>
</table>
"""

    css = """
@page { size: A4; margin: 17mm 17mm 18mm 19mm; }
body { font-family: "Noto Sans CJK SC", "Microsoft YaHei", sans-serif; color: #20252b; font-size: 10.8pt; line-height: 1.62; }
h1 { color: #173f5f; font-size: 21pt; border-bottom: 2px solid #1f6f9f; padding-bottom: 5pt; margin-top: 16pt; }
h2 { color: #1f6f9f; font-size: 15pt; margin-top: 14pt; }
h3 { color: #425466; font-size: 12.5pt; margin-top: 11pt; }
p { margin: 5pt 0; }
ul, ol { margin-top: 4pt; margin-bottom: 7pt; }
li { margin-bottom: 3pt; }
table { border-collapse: collapse; width: 100%; margin: 9pt 0 13pt 0; font-size: 9.5pt; page-break-inside: avoid; }
th { background: #1f6f9f; color: white; padding: 6pt; border: 1px solid #c4ccd3; }
td { padding: 6pt; border: 1px solid #c4ccd3; vertical-align: top; }
tr:nth-child(even) td { background: #f4f7f9; }
code { font-family: "Liberation Mono", monospace; background: #eef1f3; padding: 1pt 2pt; }
pre, .equation { font-family: "Liberation Mono", monospace; background: #f4f6f8; border-left: 4px solid #718096; padding: 9pt; white-space: pre-wrap; }
.cover { page-break-after: always; text-align: center; padding-top: 47mm; }
.cover-title { font-size: 29pt; color: #173f5f; font-weight: bold; line-height: 1.32; }
.cover-subtitle { margin-top: 20pt; font-size: 15pt; color: #59636e; }
.cover-meta { margin-top: 75pt; font-size: 10.5pt; color: #68727d; line-height: 1.8; }
.page-break { page-break-before: always; }
.figure { text-align: center; margin: 10pt 0 13pt 0; page-break-inside: avoid; }
.figure img { max-width: 100%; }
.caption { color: #68727d; font-size: 9pt; margin-top: 4pt; }
.decision { background: #fff8ec; border-left: 5px solid #bd7b00; padding: 10pt 12pt; margin: 9pt 0 13pt 0; }
.success { background: #eef8f1; border-left: 5px solid #198754; padding: 10pt 12pt; margin: 9pt 0 13pt 0; }
"""

    cover = """
<div class="cover">
  <div class="cover-title">从双车到双机空中对接<br>三阶段实施方案</div>
  <div class="cover-subtitle">双车协同模拟 → 双四轴验证 → 真实固定翼 Mini</div>
  <div class="cover-meta">项目角色：Orin2 = Carrier / leader；Orin1 = Mini / executor<br>方案日期：2026-08-11<br>用途：项目路线与阶段验收审查</div>
</div>
"""
    document = (
        '<!DOCTYPE html><html lang="zh-CN"><head><meta charset="utf-8">'
        "<title>从双车到双机空中对接的三阶段实施方案</title><style>"
        + css
        + "</style></head><body>"
        + cover
        + body
        + review
        + '<p class="small">权威正文来源：docs/three_stage_mock_to_aerial_docking_roadmap_2026_08_11.md</p>'
        + "</body></html>"
    )
    HTML_OUT.write_text(document, encoding="utf-8")
    print(f"HTML={HTML_OUT.resolve()}")
    for image_path in (roadmap, geometry, controls):
        print(f"ASSET={image_path.resolve()}")


if __name__ == "__main__":
    build_report()
