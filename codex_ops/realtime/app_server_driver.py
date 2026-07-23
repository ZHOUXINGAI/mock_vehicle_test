"""Drive a durable Codex thread through the official local app-server protocol."""

from __future__ import annotations

import json
import logging
import os
import queue
import re
import signal
import subprocess
import threading
import time
from pathlib import Path
from typing import Any, Callable, TextIO

from .codex_driver import CodexDriverConfig, CodexRunResult
from .protocol import TaskEnvelope
from .safety import WorkerPolicy


LOG = logging.getLogger("codex-agentd.app-server")
_THREAD_ID_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._:-]{5,255}$")


def format_app_server_activity(message: dict[str, Any]) -> dict[str, str] | None:
    """Format stable app-server lifecycle events without exposing hidden reasoning."""
    method = str(message.get("method", ""))
    params = message.get("params")
    if not isinstance(params, dict):
        return None
    if method == "thread/started":
        thread = params.get("thread")
        thread_id = str(thread.get("id", "")) if isinstance(thread, dict) else ""
        suffix = f"（thread {thread_id[:8]}）" if thread_id else ""
        return {"kind": "session", "summary": f"Codex 会话已连接{suffix}"}
    if method == "turn/started":
        return {"kind": "turn", "summary": "Codex 开始处理任务"}
    if method == "turn/completed":
        turn = params.get("turn")
        status = str(turn.get("status", "")) if isinstance(turn, dict) else ""
        return {"kind": "turn", "summary": f"Codex 本轮结束：{status or 'unknown'}"}
    if method == "error":
        error = params.get("error")
        if isinstance(error, dict):
            error = error.get("message")
        return {"kind": "error", "summary": f"Codex 错误：{' '.join(str(error).split())}"}
    if method not in {"item/started", "item/completed"}:
        return None
    item = params.get("item")
    if not isinstance(item, dict):
        return None
    item_type = str(item.get("type", ""))
    completed = method == "item/completed"
    if item_type == "agentMessage" and completed:
        text = " ".join(str(item.get("text", "")).split())
        if text:
            return {"kind": "message", "summary": f"Codex：{text[:2000]}"}
    if item_type == "reasoning" and not completed:
        return {"kind": "analysis", "summary": "正在分析下一步（不显示模型隐藏思维）"}
    if item_type == "commandExecution":
        command = " ".join(str(item.get("command", "")).split())
        if completed:
            status = str(item.get("status", "completed"))
            exit_code = item.get("exitCode")
            suffix = f"，exit={exit_code}" if exit_code is not None else ""
            return {"kind": "command", "summary": f"命令{status}{suffix}：{command}"}
        return {"kind": "command", "summary": f"运行命令：{command}"}
    if item_type == "fileChange":
        paths = []
        for change in item.get("changes", []):
            if isinstance(change, dict) and change.get("path"):
                paths.append(Path(str(change["path"])).name)
        names = ", ".join(paths[:8]) or "仓库文件"
        verb = "文件修改完成" if completed else "准备修改文件"
        return {"kind": "file", "summary": f"{verb}：{names}"}
    if item_type == "mcpToolCall":
        name = f"{item.get('server', '')}/{item.get('tool', '')}".strip("/")
        verb = "工具调用完成" if completed else "调用工具"
        return {"kind": "tool", "summary": f"{verb}：{name}"}
    if item_type == "webSearch":
        return {"kind": "search", "summary": f"搜索资料：{item.get('query', '')}"}
    if item_type == "plan" and completed:
        return {"kind": "plan", "summary": "工作计划已更新"}
    return None


class AppServerDriver:
    """One-task-at-a-time adapter around `codex app-server` over local stdio."""

    def __init__(self, config: CodexDriverConfig, policy: WorkerPolicy) -> None:
        self.config = config
        self.policy = policy
        config.session_file.parent.mkdir(parents=True, exist_ok=True)
        config.result_dir.mkdir(parents=True, exist_ok=True)

    def load_thread_id(self) -> str:
        try:
            value = json.loads(self.config.session_file.read_text(encoding="utf-8"))
            thread_id = str(value.get("thread_id") or value.get("session_id") or "")
            return thread_id if _THREAD_ID_RE.fullmatch(thread_id) else ""
        except (OSError, ValueError, TypeError):
            return ""

    def save_thread_id(self, thread_id: str) -> None:
        if not _THREAD_ID_RE.fullmatch(thread_id):
            raise RuntimeError(f"app-server returned invalid thread id: {thread_id!r}")
        temporary = self.config.session_file.with_suffix(".tmp")
        temporary.write_text(
            json.dumps({"agent_id": self.config.agent_id, "thread_id": thread_id}, indent=2)
            + "\n",
            encoding="utf-8",
        )
        temporary.replace(self.config.session_file)

    @staticmethod
    def _subprocess_env(codex_home: Path) -> dict[str, str]:
        environment = dict(os.environ)
        environment["CODEX_HOME"] = str(codex_home)
        for key in tuple(environment):
            upper = key.upper()
            if upper.startswith(("NATS_", "CODEX_COORD_")) and upper != "CODEX_HOME":
                environment.pop(key, None)
        return environment

    def task_prompt(self, task: TaskEnvelope) -> str:
        context = "\n".join(f"- {item}" for item in task.context_files) or "- none"
        acceptance = "\n".join(f"- {item}" for item in task.acceptance) or "- none"
        return f"""You are the dedicated visible coordination Codex for {self.config.agent_id}.
Role: {self.config.role}

{self.policy.prompt_guard()}

TASK ENVELOPE
- task_id: {task.task_id}
- root_task_id: {task.root_task_id}
- parent_task_id: {task.parent_task_id or 'none'}
- from: {task.from_agent}
- to: {task.to_agent}
- type: {task.task_type}
- repo: {task.repo}
- base_commit: {task.base_commit or 'not specified'}
- hop_count: {task.hop_count}/{task.max_hops}

OBJECTIVE
{task.objective}

CONTEXT FILES
{context}

ACCEPTANCE
{acceptance}

Read repository guidance and current shared meeting state before acting. Return the
configured structured result. Put work for the peer in peer_requests; the local bridge
will deliver it automatically. Do not ask the boss or user to relay normal peer messages."""

    @staticmethod
    def _reader(
        stream: TextIO,
        messages: queue.Queue[dict[str, Any]],
        event_log: TextIO,
    ) -> None:
        for line in iter(stream.readline, ""):
            event_log.write(line)
            event_log.flush()
            try:
                value = json.loads(line)
            except json.JSONDecodeError:
                continue
            if isinstance(value, dict):
                messages.put(value)

    @staticmethod
    def _stderr_reader(
        stream: TextIO,
        stderr_log: TextIO,
        activity_callback: Callable[[dict[str, str]], None] | None,
    ) -> None:
        for line in iter(stream.readline, ""):
            stderr_log.write(line)
            stderr_log.flush()
            text = " ".join(line.split())
            if text:
                activity = {"kind": "error", "summary": f"app-server：{text[:2000]}"}
                LOG.info("%s", activity["summary"])
                if activity_callback:
                    activity_callback(activity)

    @staticmethod
    def _write(process: subprocess.Popen[str], message: dict[str, Any]) -> None:
        if process.stdin is None:
            raise RuntimeError("app-server stdin is unavailable")
        process.stdin.write(json.dumps(message, ensure_ascii=False, separators=(",", ":")) + "\n")
        process.stdin.flush()

    def _handle_message(
        self,
        process: subprocess.Popen[str],
        message: dict[str, Any],
        activity_callback: Callable[[dict[str, str]], None] | None,
        final_messages: list[str],
    ) -> None:
        if "method" in message and "id" in message:
            method = str(message.get("method", ""))
            if method in {
                "item/commandExecution/requestApproval",
                "item/fileChange/requestApproval",
            }:
                self._write(process, {"id": message["id"], "result": {"decision": "decline"}})
            else:
                self._write(
                    process,
                    {
                        "id": message["id"],
                        "error": {"code": -32601, "message": f"unsupported request: {method}"},
                    },
                )
            return
        activity = format_app_server_activity(message)
        if activity:
            LOG.info("%s", activity["summary"])
            if activity_callback:
                activity_callback(activity)
        if message.get("method") == "item/completed":
            params = message.get("params")
            item = params.get("item") if isinstance(params, dict) else None
            if isinstance(item, dict) and item.get("type") == "agentMessage":
                text = item.get("text")
                if isinstance(text, str) and text.strip():
                    final_messages.append(text)

    def _wait_response(
        self,
        *,
        process: subprocess.Popen[str],
        messages: queue.Queue[dict[str, Any]],
        request_id: int,
        deadline: float,
        activity_callback: Callable[[dict[str, str]], None] | None,
        final_messages: list[str],
    ) -> dict[str, Any]:
        while time.monotonic() < deadline:
            try:
                message = messages.get(timeout=min(1.0, max(0.01, deadline - time.monotonic())))
            except queue.Empty:
                if process.poll() is not None:
                    raise RuntimeError(f"app-server exited before response: {process.returncode}")
                continue
            if message.get("id") == request_id and ("result" in message or "error" in message):
                if "error" in message:
                    raise RuntimeError(f"app-server request failed: {message['error']}")
                result = message.get("result")
                return result if isinstance(result, dict) else {}
            self._handle_message(process, message, activity_callback, final_messages)
        raise TimeoutError(f"app-server request {request_id} timed out")

    def run(
        self,
        task: TaskEnvelope,
        repo: Path,
        activity_callback: Callable[[dict[str, str]], None] | None = None,
    ) -> CodexRunResult:
        if not self.config.enabled:
            output = {
                "status": "completed",
                "summary": "app-server bridge transport test completed with Codex disabled",
                "details": "No Codex process or hardware process was started.",
                "peer_requests": [],
                "artifacts": [],
                "requires_boss": False,
            }
            return CodexRunResult("completed", output["summary"], output, "", 0, "", "")

        run_dir = self.config.result_dir / task.task_id
        run_dir.mkdir(parents=True, exist_ok=True)
        event_path = run_dir / "app_server_events.jsonl"
        stderr_path = run_dir / "app_server_stderr.log"
        deadline = time.monotonic() + self.config.timeout_sec
        message_queue: queue.Queue[dict[str, Any]] = queue.Queue()
        final_messages: list[str] = []
        thread_id = self.load_thread_id()
        turn_id = ""
        process: subprocess.Popen[str] | None = None

        with event_path.open("w", encoding="utf-8") as event_log, stderr_path.open(
            "w", encoding="utf-8"
        ) as stderr_log:
            process = subprocess.Popen(
                [self.config.binary, "app-server", "--listen", "stdio://"],
                cwd=repo,
                env=self._subprocess_env(self.config.codex_home),
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1,
                start_new_session=True,
            )
            assert process.stdout is not None
            assert process.stderr is not None
            stdout_thread = threading.Thread(
                target=self._reader,
                args=(process.stdout, message_queue, event_log),
                daemon=True,
            )
            stderr_thread = threading.Thread(
                target=self._stderr_reader,
                args=(process.stderr, stderr_log, activity_callback),
                daemon=True,
            )
            stdout_thread.start()
            stderr_thread.start()
            request_id = 1
            try:
                self._write(
                    process,
                    {
                        "method": "initialize",
                        "id": request_id,
                        "params": {
                            "clientInfo": {
                                "name": "mock_vehicle_codex_bridge",
                                "title": "Mock Vehicle Codex Bridge",
                                "version": "1.0.0",
                            }
                        },
                    },
                )
                self._wait_response(
                    process=process,
                    messages=message_queue,
                    request_id=request_id,
                    deadline=deadline,
                    activity_callback=activity_callback,
                    final_messages=final_messages,
                )
                self._write(process, {"method": "initialized", "params": {}})

                request_id += 1
                if thread_id:
                    method = "thread/resume"
                    params: dict[str, Any] = {
                        "threadId": thread_id,
                        "cwd": str(repo),
                        "approvalPolicy": "never",
                        "sandbox": "read-only",
                    }
                else:
                    method = "thread/start"
                    params = {
                        "cwd": str(repo),
                        "approvalPolicy": "never",
                        "sandbox": "read-only",
                        "serviceName": "mock_vehicle_codex_bridge",
                    }
                    if self.config.model:
                        params["model"] = self.config.model
                thread_result = self._wait_after_send(
                    process=process,
                    messages=message_queue,
                    request_id=request_id,
                    method=method,
                    params=params,
                    deadline=deadline,
                    activity_callback=activity_callback,
                    final_messages=final_messages,
                )
                thread = thread_result.get("thread")
                if not isinstance(thread, dict) or not thread.get("id"):
                    raise RuntimeError("app-server did not return a thread id")
                thread_id = str(thread["id"])
                self.save_thread_id(thread_id)

                output_schema = json.loads(
                    self.config.output_schema.read_text(encoding="utf-8")
                )
                request_id += 1
                turn_result = self._wait_after_send(
                    process=process,
                    messages=message_queue,
                    request_id=request_id,
                    method="turn/start",
                    params={
                        "threadId": thread_id,
                        "clientUserMessageId": task.task_id,
                        "input": [{"type": "text", "text": self.task_prompt(task)}],
                        "cwd": str(repo),
                        "approvalPolicy": "never",
                        "sandboxPolicy": {"type": "read-only"},
                        "outputSchema": output_schema,
                    },
                    deadline=deadline,
                    activity_callback=activity_callback,
                    final_messages=final_messages,
                )
                turn = turn_result.get("turn")
                if not isinstance(turn, dict) or not turn.get("id"):
                    raise RuntimeError("app-server did not return a turn id")
                turn_id = str(turn["id"])

                completed_turn: dict[str, Any] | None = None
                while time.monotonic() < deadline:
                    try:
                        message = message_queue.get(timeout=1.0)
                    except queue.Empty:
                        if process.poll() is not None:
                            raise RuntimeError(f"app-server exited during turn: {process.returncode}")
                        continue
                    self._handle_message(process, message, activity_callback, final_messages)
                    if message.get("method") == "turn/completed":
                        event_params = message.get("params")
                        candidate = (
                            event_params.get("turn") if isinstance(event_params, dict) else None
                        )
                        if isinstance(candidate, dict) and str(candidate.get("id")) == turn_id:
                            completed_turn = candidate
                            break
                if completed_turn is None:
                    raise TimeoutError(f"Codex turn exceeded {self.config.timeout_sec}s timeout")
                if str(completed_turn.get("status")) != "completed":
                    error = completed_turn.get("error")
                    raise RuntimeError(f"Codex turn ended {completed_turn.get('status')}: {error}")
            except Exception as exc:
                if process.poll() is None and thread_id and turn_id:
                    try:
                        request_id += 1
                        self._write(
                            process,
                            {
                                "method": "turn/interrupt",
                                "id": request_id,
                                "params": {"threadId": thread_id, "turnId": turn_id},
                            },
                        )
                    except Exception:
                        pass
                return CodexRunResult(
                    "failed",
                    str(exc),
                    {},
                    thread_id,
                    1,
                    str(event_path),
                    str(stderr_path),
                )
            finally:
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGTERM)
                    try:
                        process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait(timeout=5)
                stdout_thread.join(timeout=2)
                stderr_thread.join(timeout=2)
                for stream in (process.stdin, process.stdout, process.stderr):
                    if stream is not None:
                        stream.close()

        text = final_messages[-1] if final_messages else ""
        try:
            output = json.loads(text)
        except json.JSONDecodeError as exc:
            return CodexRunResult(
                "failed",
                f"Codex did not return valid structured output: {exc}",
                {},
                thread_id,
                1,
                str(event_path),
                str(stderr_path),
            )
        status = str(output.get("status", "failed"))
        if status not in {"completed", "blocked", "failed"}:
            status = "failed"
        summary = str(output.get("summary", ""))[:20_000]
        return CodexRunResult(
            status,
            summary,
            output,
            thread_id,
            0 if status == "completed" else 1,
            str(event_path),
            str(stderr_path),
        )

    def _wait_after_send(
        self,
        *,
        process: subprocess.Popen[str],
        messages: queue.Queue[dict[str, Any]],
        request_id: int,
        method: str,
        params: dict[str, Any],
        deadline: float,
        activity_callback: Callable[[dict[str, str]], None] | None,
        final_messages: list[str],
    ) -> dict[str, Any]:
        self._write(process, {"method": method, "id": request_id, "params": params})
        return self._wait_response(
            process=process,
            messages=messages,
            request_id=request_id,
            deadline=deadline,
            activity_callback=activity_callback,
            final_messages=final_messages,
        )
