"""Configuration loading shared by agentd and coordctl."""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from .codex_driver import CodexDriverConfig
from .nats_bus import NatsSettings
from .safety import WorkerPolicy


@dataclass(frozen=True)
class AgentConfig:
    agent_id: str
    role: str
    nats: NatsSettings
    policy: WorkerPolicy
    driver: CodexDriverConfig
    state_db: Path
    heartbeat_sec: float
    fetch_timeout_sec: float


def load_json(path: str | Path) -> dict[str, Any]:
    return json.loads(Path(path).expanduser().read_text(encoding="utf-8"))


def load_agent_config(path: str | Path) -> AgentConfig:
    source = Path(path).expanduser().resolve()
    raw = load_json(source)
    agent_id = str(raw["agent_id"])
    role = str(raw["role"])
    policy_raw = raw["policy"]
    policy = WorkerPolicy.create(
        agent_id=agent_id,
        mode=str(policy_raw.get("mode", "observe")),
        allowed_roots=[str(item) for item in policy_raw["allowed_roots"]],
        repo_map={str(key): str(value) for key, value in policy_raw.get("repo_map", {}).items()},
        allow_broadcast=bool(policy_raw.get("allow_broadcast", False)),
        max_hops=int(policy_raw.get("max_hops", 4)),
    )
    codex_raw = raw["codex"]
    driver = CodexDriverConfig(
        agent_id=agent_id,
        role=role,
        codex_home=Path(codex_raw["home"]).expanduser(),
        session_file=Path(codex_raw["session_file"]).expanduser(),
        output_schema=Path(codex_raw["output_schema"]).expanduser(),
        result_dir=Path(codex_raw["result_dir"]).expanduser(),
        binary=str(codex_raw.get("binary", "codex")),
        timeout_sec=int(codex_raw.get("timeout_sec", 1800)),
        profile=str(codex_raw.get("profile", "")),
        model=str(codex_raw.get("model", "")),
        enabled=bool(codex_raw.get("enabled", True)),
    )
    return AgentConfig(
        agent_id=agent_id,
        role=role,
        nats=NatsSettings.from_dict(raw["nats"]),
        policy=policy,
        driver=driver,
        state_db=Path(raw["state_db"]).expanduser(),
        heartbeat_sec=float(raw.get("heartbeat_sec", 10.0)),
        fetch_timeout_sec=float(raw.get("fetch_timeout_sec", 1.0)),
    )
