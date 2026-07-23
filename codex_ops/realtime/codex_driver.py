"""Run or resume one dedicated Codex automation thread safely."""

from __future__ import annotations

import json
import logging
import os
import re
import signal
import subprocess
import threading
from dataclasses import dataclass
from pathlib import Path
from typing import Any, BinaryIO, Callable

from .console import format_codex_activity
from .protocol import TaskEnvelope
from .safety import WorkerPolicy


_UUID_RE = re.compile(
    r"^[0-9a-fA-F]{8}-[0-9a-fA-F]{4}-[0-9a-fA-F]{4}-[0-9a-fA-F]{4}-[0-9a-fA-F]{12}$"
)
LOG = logging.getLogger("codex-agentd.codex")


def mirror_stream(
    source: BinaryIO,
    destination: BinaryIO,
    label: str,
    activity_callback: Callable[[dict[str, str]], None] | None = None,
) -> None:
    """Persist raw Codex output and mirror meaningful activity to the operator."""
    for line in iter(source.readline, b""):
        destination.write(line)
        destination.flush()
        text = line.decode("utf-8", errors="replace").rstrip()
        activity = format_codex_activity(text, label)
        if activity:
            LOG.info("%s", activity["summary"])
            if activity_callback:
                activity_callback(activity)


@dataclass(frozen=True)
class CodexDriverConfig:
    agent_id: str
    role: str
    codex_home: Path
    session_file: Path
    output_schema: Path
    result_dir: Path
    binary: str = "codex"
    backend: str = "exec"
    timeout_sec: int = 1800
    profile: str = ""
    model: str = ""
    enabled: bool = True


@dataclass(frozen=True)
class CodexRunResult:
    status: str
    summary: str
    output: dict[str, Any]
    session_id: str
    exit_code: int
    event_log: str
    stderr_log: str


class CodexDriver:
    def __init__(self, config: CodexDriverConfig, policy: WorkerPolicy) -> None:
        self.config = config
        self.policy = policy
        config.session_file.parent.mkdir(parents=True, exist_ok=True)
        config.result_dir.mkdir(parents=True, exist_ok=True)

    def load_session_id(self) -> str:
        try:
            value = json.loads(self.config.session_file.read_text(encoding="utf-8"))
            session_id = str(value.get("session_id", ""))
            return session_id if _UUID_RE.fullmatch(session_id) else ""
        except (OSError, ValueError, TypeError):
            return ""

    def save_session_id(self, session_id: str) -> None:
        if not _UUID_RE.fullmatch(session_id):
            return
        temporary = self.config.session_file.with_suffix(".tmp")
        temporary.write_text(
            json.dumps({"agent_id": self.config.agent_id, "session_id": session_id}, indent=2)
            + "\n",
            encoding="utf-8",
        )
        temporary.replace(self.config.session_file)

    def task_prompt(self, task: TaskEnvelope) -> str:
        context = "\n".join(f"- {item}" for item in task.context_files) or "- none"
        acceptance = "\n".join(f"- {item}" for item in task.acceptance) or "- none"
        return f"""You are the dedicated automation Codex for {self.config.agent_id}.
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

Read repository guidance and current shared meeting state before acting. Return only the
configured structured result. Put work for the peer in peer_requests; the local agentd
will deliver it automatically. Do not tell the boss to relay a message manually."""

    def build_command(
        self,
        *,
        repo: Path,
        prompt: str,
        result_file: Path,
        session_id: str = "",
    ) -> list[str]:
        command = [
            self.config.binary,
            "--cd",
            str(repo),
            "--sandbox",
            self.policy.sandbox_mode,
            "--ask-for-approval",
            "never",
        ]
        if self.config.model:
            command.extend(["--model", self.config.model])
        if self.config.profile:
            command.extend(["--profile", self.config.profile])
        command.append("exec")
        if session_id:
            command.extend(["resume", "--json"])
        else:
            command.append("--json")
        command.extend(
            [
                "--output-schema",
                str(self.config.output_schema),
                "--output-last-message",
                str(result_file),
            ]
        )
        if session_id:
            command.append(session_id)
        command.append(prompt)
        return command

    @staticmethod
    def parse_session_id(event_log: Path) -> str:
        try:
            lines = event_log.read_text(encoding="utf-8").splitlines()
        except OSError:
            return ""
        for line in lines:
            try:
                value = json.loads(line)
            except json.JSONDecodeError:
                continue
            candidates = [
                value.get("thread_id"),
                value.get("threadId"),
                value.get("session_id"),
            ]
            thread = value.get("thread")
            if isinstance(thread, dict):
                candidates.extend([thread.get("id"), thread.get("thread_id")])
            for candidate in candidates:
                if isinstance(candidate, str) and _UUID_RE.fullmatch(candidate):
                    return candidate
        return ""

    @staticmethod
    def _subprocess_env(codex_home: Path) -> dict[str, str]:
        environment = dict(os.environ)
        environment["CODEX_HOME"] = str(codex_home)
        for key in tuple(environment):
            upper = key.upper()
            if upper.startswith(("NATS_", "CODEX_COORD_")) and upper != "CODEX_HOME":
                environment.pop(key, None)
        return environment

    def run(
        self,
        task: TaskEnvelope,
        repo: Path,
        activity_callback: Callable[[dict[str, str]], None] | None = None,
    ) -> CodexRunResult:
        if not self.config.enabled:
            output = {
                "status": "completed",
                "summary": "agentd transport smoke test completed with Codex execution disabled",
                "details": "No Codex process or hardware process was started.",
                "peer_requests": [],
                "artifacts": [],
                "requires_boss": False,
            }
            return CodexRunResult("completed", output["summary"], output, "", 0, "", "")

        run_dir = self.config.result_dir / task.task_id
        run_dir.mkdir(parents=True, exist_ok=True)
        result_file = run_dir / "result.json"
        event_log = run_dir / "codex_events.jsonl"
        stderr_log = run_dir / "codex_stderr.log"
        session_id = self.load_session_id()
        command = self.build_command(
            repo=repo,
            prompt=self.task_prompt(task),
            result_file=result_file,
            session_id=session_id,
        )

        with event_log.open("wb") as stdout_log, stderr_log.open("wb") as stderr_log_file:
            process = subprocess.Popen(
                command,
                cwd=repo,
                env=self._subprocess_env(self.config.codex_home),
                stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                start_new_session=True,
                bufsize=0,
            )
            assert process.stdout is not None
            assert process.stderr is not None
            stdout_thread = threading.Thread(
                target=mirror_stream,
                args=(process.stdout, stdout_log, "stdout", activity_callback),
                daemon=True,
            )
            stderr_thread = threading.Thread(
                target=mirror_stream,
                args=(process.stderr, stderr_log_file, "stderr", activity_callback),
                daemon=True,
            )
            stdout_thread.start()
            stderr_thread.start()
            try:
                exit_code = process.wait(timeout=self.config.timeout_sec)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=5)
                return CodexRunResult(
                    "failed",
                    f"Codex exceeded {self.config.timeout_sec}s timeout",
                    {},
                    session_id,
                    124,
                    str(event_log),
                    str(stderr_log),
                )
            finally:
                stdout_thread.join(timeout=5)
                stderr_thread.join(timeout=5)

        discovered = self.parse_session_id(event_log)
        if discovered:
            session_id = discovered
            self.save_session_id(session_id)
        try:
            output = json.loads(result_file.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            return CodexRunResult(
                "failed",
                f"Codex did not produce valid structured output: {exc}",
                {},
                session_id,
                exit_code,
                str(event_log),
                str(stderr_log),
            )
        status = str(output.get("status", "failed"))
        if status not in {"completed", "blocked", "failed"}:
            status = "failed"
        summary = str(output.get("summary", ""))[:20_000]
        if exit_code != 0 and status == "completed":
            status = "failed"
            summary = f"Codex exited {exit_code}: {summary}"
        return CodexRunResult(
            status,
            summary,
            output,
            session_id,
            exit_code,
            str(event_log),
            str(stderr_log),
        )
