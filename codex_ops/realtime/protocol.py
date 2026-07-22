"""Versioned messages for the cloud-backed Codex coordination channel."""

from __future__ import annotations

import json
import re
import uuid
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from typing import Any


PROTOCOL_VERSION = 1
AGENT_IDS = {"orin1-carrier", "orin2-mini", "boss"}
TASK_TYPES = {"analysis", "code", "review", "diagnostic", "peer_request"}
TERMINAL_EVENT_TYPES = {"completed", "blocked", "rejected", "failed"}
_AGENT_RE = re.compile(r"^[a-z0-9][a-z0-9-]{1,62}$")


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def new_id() -> str:
    return str(uuid.uuid4())


@dataclass(frozen=True)
class TaskSafety:
    """Capabilities requested by a task; all hardware capabilities default off."""

    motion_allowed: bool = False
    arming_allowed: bool = False
    offboard_allowed: bool = False
    actuator_access: bool = False
    hardware_write_allowed: bool = False

    def requests_hardware(self) -> bool:
        return any(asdict(self).values())

    @classmethod
    def from_dict(cls, value: dict[str, Any] | None) -> "TaskSafety":
        raw = value or {}
        known = {name: bool(raw.get(name, False)) for name in cls.__dataclass_fields__}
        return cls(**known)


@dataclass(frozen=True)
class TaskEnvelope:
    version: int
    task_id: str
    root_task_id: str
    parent_task_id: str | None
    created_at: str
    from_agent: str
    to_agent: str
    task_type: str
    objective: str
    repo: str
    base_commit: str = ""
    context_files: tuple[str, ...] = field(default_factory=tuple)
    acceptance: tuple[str, ...] = field(default_factory=tuple)
    safety: TaskSafety = field(default_factory=TaskSafety)
    hop_count: int = 0
    max_hops: int = 4

    @classmethod
    def create(
        cls,
        *,
        from_agent: str,
        to_agent: str,
        task_type: str,
        objective: str,
        repo: str,
        base_commit: str = "",
        context_files: list[str] | tuple[str, ...] = (),
        acceptance: list[str] | tuple[str, ...] = (),
        safety: TaskSafety | None = None,
        parent: "TaskEnvelope | None" = None,
    ) -> "TaskEnvelope":
        task_id = new_id()
        task = cls(
            version=PROTOCOL_VERSION,
            task_id=task_id,
            root_task_id=parent.root_task_id if parent else task_id,
            parent_task_id=parent.task_id if parent else None,
            created_at=utc_now(),
            from_agent=from_agent,
            to_agent=to_agent,
            task_type=task_type,
            objective=objective.strip(),
            repo=repo,
            base_commit=base_commit,
            context_files=tuple(context_files),
            acceptance=tuple(acceptance),
            safety=safety or TaskSafety(),
            hop_count=(parent.hop_count + 1) if parent else 0,
            max_hops=parent.max_hops if parent else 4,
        )
        task.validate()
        return task

    def validate(self) -> None:
        if self.version != PROTOCOL_VERSION:
            raise ValueError(f"unsupported task version: {self.version}")
        for label, value in (("from_agent", self.from_agent), ("to_agent", self.to_agent)):
            if not _AGENT_RE.fullmatch(value):
                raise ValueError(f"invalid {label}: {value!r}")
        if self.task_type not in TASK_TYPES:
            raise ValueError(f"invalid task_type: {self.task_type!r}")
        if not self.objective or len(self.objective) > 20_000:
            raise ValueError("objective must contain 1..20000 characters")
        if not self.repo:
            raise ValueError("repo is required")
        for label, value in (
            ("task_id", self.task_id),
            ("root_task_id", self.root_task_id),
        ):
            try:
                uuid.UUID(value)
            except ValueError as exc:
                raise ValueError(f"invalid {label}: {value!r}") from exc
        if self.parent_task_id:
            try:
                uuid.UUID(self.parent_task_id)
            except ValueError as exc:
                raise ValueError(f"invalid parent_task_id: {self.parent_task_id!r}") from exc
        if self.hop_count < 0 or self.max_hops < 0 or self.hop_count > self.max_hops:
            raise ValueError("invalid peer task hop count")
        if len(self.context_files) > 64 or len(self.acceptance) > 64:
            raise ValueError("too many context or acceptance entries")

    def to_dict(self) -> dict[str, Any]:
        value = asdict(self)
        value["context_files"] = list(self.context_files)
        value["acceptance"] = list(self.acceptance)
        return value

    def to_json(self) -> bytes:
        return json.dumps(self.to_dict(), sort_keys=True, separators=(",", ":")).encode("utf-8")

    @classmethod
    def from_dict(cls, value: dict[str, Any]) -> "TaskEnvelope":
        task = cls(
            version=int(value["version"]),
            task_id=str(value["task_id"]),
            root_task_id=str(value["root_task_id"]),
            parent_task_id=value.get("parent_task_id"),
            created_at=str(value["created_at"]),
            from_agent=str(value["from_agent"]),
            to_agent=str(value["to_agent"]),
            task_type=str(value["task_type"]),
            objective=str(value["objective"]),
            repo=str(value["repo"]),
            base_commit=str(value.get("base_commit", "")),
            context_files=tuple(str(item) for item in value.get("context_files", [])),
            acceptance=tuple(str(item) for item in value.get("acceptance", [])),
            safety=TaskSafety.from_dict(value.get("safety")),
            hop_count=int(value.get("hop_count", 0)),
            max_hops=int(value.get("max_hops", 4)),
        )
        task.validate()
        return task

    @classmethod
    def from_json(cls, payload: bytes | str) -> "TaskEnvelope":
        return cls.from_dict(json.loads(payload))


def event_payload(
    *,
    agent_id: str,
    event_type: str,
    task: TaskEnvelope | None = None,
    summary: str = "",
    **details: Any,
) -> bytes:
    value: dict[str, Any] = {
        "version": PROTOCOL_VERSION,
        "event_id": new_id(),
        "created_at": utc_now(),
        "agent_id": agent_id,
        "event_type": event_type,
        "summary": summary,
    }
    if task:
        value.update(
            {
                "task_id": task.task_id,
                "root_task_id": task.root_task_id,
                "parent_task_id": task.parent_task_id,
            }
        )
    value.update({key: item for key, item in details.items() if item is not None})
    return json.dumps(value, sort_keys=True, separators=(",", ":")).encode("utf-8")
