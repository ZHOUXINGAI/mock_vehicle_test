"""Human-readable activity formatting for Ground and Orin consoles."""

from __future__ import annotations

import json
import textwrap
from pathlib import Path
from typing import Any


AGENT_LABELS = {
    "boss": "Ground/Boss",
    "orin1-carrier": "Orin1/Carrier",
    "orin2-mini": "Orin2/Mini",
}

EVENT_MARKERS = {
    "online": "🟢",
    "dispatched": "🚀",
    "accepted": "📥",
    "progress": "⏳",
    "activity": "🔹",
    "peer_dispatched": "📤",
    "peer_request_rejected": "⚠️",
    "completed": "✅",
    "blocked": "⛔",
    "failed": "❌",
    "rejected": "🚫",
    "duplicate": "↩️",
}


def _single_line(value: Any, limit: int = 800) -> str:
    text = " ".join(str(value or "").split())
    if len(text) > limit:
        return text[: limit - 1] + "…"
    return text


def _command_text(item: dict[str, Any]) -> str:
    command = item.get("command", "")
    if isinstance(command, list):
        command = " ".join(str(part) for part in command)
    return _single_line(command)


def format_codex_activity(line: str, stream: str = "stdout") -> dict[str, str] | None:
    """Turn one codex-exec JSONL line into a safe, readable activity update.

    This intentionally exposes observable work (commands, tools, file changes and
    agent messages), not private model reasoning.
    """
    text = line.strip()
    if not text:
        return None
    try:
        event = json.loads(text)
    except json.JSONDecodeError:
        if stream == "stderr":
            return {"kind": "error", "summary": f"Codex 错误：{_single_line(text)}"}
        return None
    if not isinstance(event, dict):
        return None

    event_type = str(event.get("type", ""))
    if event_type == "thread.started":
        thread_id = _single_line(event.get("thread_id") or event.get("threadId"))
        suffix = f"（会话 {thread_id[:8]}）" if thread_id else ""
        return {"kind": "session", "summary": f"Codex 会话已启动{suffix}"}
    if event_type == "turn.started":
        return {"kind": "turn", "summary": "Codex 开始处理任务"}
    if event_type == "turn.completed":
        return {"kind": "turn", "summary": "Codex 本轮处理完成"}
    if event_type in {"turn.failed", "error"}:
        message = event.get("message") or event.get("error") or "未知错误"
        return {"kind": "error", "summary": f"Codex 执行失败：{_single_line(message)}"}

    item = event.get("item")
    if not isinstance(item, dict):
        return None
    item_type = str(item.get("type", ""))
    completed = event_type == "item.completed"
    started = event_type == "item.started"

    if item_type in {"agent_message", "message"} and completed:
        message = item.get("text") or item.get("message") or item.get("content")
        if message:
            return {"kind": "message", "summary": f"Codex：{_single_line(message, 2000)}"}
    if item_type == "command_execution":
        command = _command_text(item)
        if started:
            return {"kind": "command", "summary": f"运行命令：{command}"}
        if completed:
            exit_code = item.get("exit_code")
            status = "完成" if exit_code in (None, 0) else f"失败（exit={exit_code}）"
            return {"kind": "command", "summary": f"命令{status}：{command}"}
    if item_type in {"file_change", "file_changes"}:
        changes = item.get("changes")
        paths: list[str] = []
        if isinstance(changes, list):
            for change in changes:
                if isinstance(change, dict):
                    path = change.get("path") or change.get("file")
                    if path:
                        paths.append(str(path))
        path = item.get("path") or item.get("file")
        if path:
            paths.append(str(path))
        names = ", ".join(Path(value).name for value in paths[:8]) or "仓库文件"
        verb = "已修改" if completed else "正在修改"
        return {"kind": "file", "summary": f"{verb}：{names}"}
    if item_type in {"mcp_tool_call", "tool_call"}:
        name = item.get("tool") or item.get("name") or "工具"
        verb = "工具调用完成" if completed else "调用工具"
        return {"kind": "tool", "summary": f"{verb}：{_single_line(name)}"}
    if item_type == "web_search":
        query = item.get("query") or item.get("text") or ""
        return {"kind": "search", "summary": f"搜索资料：{_single_line(query)}"}
    if item_type in {"todo_list", "plan"} and completed:
        return {"kind": "plan", "summary": "工作计划已更新"}
    if item_type == "reasoning" and started:
        return {"kind": "analysis", "summary": "正在分析下一步（不显示模型隐藏思维）"}
    return None


def format_event_for_console(event: dict[str, Any]) -> str:
    """Format one coordination event for the operator-facing live console."""
    event_type = str(event.get("event_type", "event"))
    agent_id = str(event.get("agent_id", "unknown"))
    agent = AGENT_LABELS.get(agent_id, agent_id)
    marker = EVENT_MARKERS.get(event_type, "•")
    created_at = str(event.get("created_at", ""))
    clock = created_at[11:19] if len(created_at) >= 19 else "--:--:--"
    task_id = str(event.get("task_id") or "")
    task_suffix = f"  task={task_id[:8]}" if task_id else ""
    summary = _single_line(event.get("summary"))

    if event_type == "accepted":
        objective = _single_line(event.get("objective"))
        summary = f"已接收任务：{objective or summary}"
    elif event_type == "dispatched":
        target = AGENT_LABELS.get(str(event.get("to_agent", "")), str(event.get("to_agent", "")))
        objective = _single_line(event.get("objective"))
        summary = f"向 {target} 下达任务：{objective or summary}"
    elif event_type == "online":
        summary = f"工作端在线：{summary}"
    elif event_type == "peer_dispatched":
        peer = AGENT_LABELS.get(str(event.get("peer_agent", "")), str(event.get("peer_agent", "")))
        summary = f"已向 {peer} 交接任务"
    elif event_type == "activity":
        summary = summary or "正在工作"
    elif event_type == "completed":
        summary = f"任务完成：{summary}"
    elif event_type == "blocked":
        summary = f"任务受阻：{summary}"
    elif event_type == "failed":
        summary = f"任务失败：{summary}"
    elif event_type == "rejected":
        summary = f"任务被拒绝：{summary}"

    return f"[{clock}] {marker} {agent}  {summary}{task_suffix}"


def _chat_box(title: str, body: str, footer: str = "", width: int = 96) -> str:
    inner = max(40, width - 4)
    title_text = f" {title} "
    top = "╭─" + title_text + "─" * max(1, inner - len(title_text))
    lines = [top]
    paragraphs = str(body or "").splitlines() or [""]
    for paragraph in paragraphs:
        wrapped = textwrap.wrap(
            paragraph,
            width=inner,
            replace_whitespace=False,
            drop_whitespace=True,
        ) or [""]
        lines.extend(f"│ {line}" for line in wrapped)
    if footer:
        lines.append("├" + "─" * (inner + 1))
        lines.append(f"│ {footer}")
    lines.append("╰" + "─" * (inner + 1))
    return "\n".join(lines)


def _structured_codex_message(summary: str) -> tuple[str, str]:
    text = summary.strip()
    for prefix in ("Codex 回复：", "Codex："):
        if text.startswith(prefix):
            text = text[len(prefix) :].strip()
            break
    try:
        payload = json.loads(text)
    except (json.JSONDecodeError, TypeError):
        return text, ""
    if not isinstance(payload, dict):
        return text, ""
    heading = str(payload.get("summary") or payload.get("status") or "Codex reply")
    details = str(payload.get("details") or "")
    return heading, details


def format_event_as_chat(event: dict[str, Any]) -> str:
    """Render one coordination event as a read-only terminal chat transcript."""
    event_type = str(event.get("event_type", "event"))
    agent_id = str(event.get("agent_id", "unknown"))
    agent = AGENT_LABELS.get(agent_id, agent_id)
    created_at = str(event.get("created_at", ""))
    clock = created_at[11:19] if len(created_at) >= 19 else "--:--:--"
    task_id = str(event.get("task_id") or "")
    footer = f"{clock}  task={task_id[:8]}" if task_id else clock
    summary = str(event.get("summary") or "")

    if event_type in {"dispatched", "accepted"}:
        target_id = str(event.get("to_agent") or agent_id)
        target = AGENT_LABELS.get(target_id, target_id)
        objective = str(event.get("objective") or summary)
        source = AGENT_LABELS.get(str(event.get("from_agent") or "boss"), "Ground/Boss")
        return _chat_box(f"👤 {source} → {target}", objective, footer)

    if event_type == "activity":
        kind = str(event.get("activity_kind") or "")
        if kind == "message":
            heading, details = _structured_codex_message(summary)
            body = heading if not details else f"{heading}\n\n{details}"
            return _chat_box(f"🤖 {agent}", body, footer)
        marker = {
            "command": "🔧",
            "tool": "🧰",
            "file": "📝",
            "search": "🔎",
            "analysis": "💭",
            "plan": "📋",
            "session": "🔗",
            "turn": "•",
            "error": "❌",
        }.get(kind, "•")
        return f"[{clock}] {marker} {agent}  {summary}  task={task_id[:8]}"

    if event_type in {"completed", "blocked", "failed", "rejected"}:
        marker = {
            "completed": "✅",
            "blocked": "⛔",
            "failed": "❌",
            "rejected": "🚫",
        }[event_type]
        details = str(event.get("details") or "")
        body = summary if not details else f"{summary}\n\n{details}"
        return _chat_box(f"{marker} {agent} {event_type}", body, footer)

    if event_type == "peer_dispatched":
        peer_id = str(event.get("peer_agent") or "")
        peer = AGENT_LABELS.get(peer_id, peer_id)
        objective = str(event.get("peer_objective") or summary)
        return _chat_box(f"📤 {agent} → {peer}", objective, footer)

    if event_type == "online":
        return f"[{clock}] 🟢 {agent} 在线"

    return format_event_for_console(event)
