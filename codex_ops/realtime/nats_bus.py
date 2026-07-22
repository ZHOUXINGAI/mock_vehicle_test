"""Thin NATS JetStream adapter used by the worker and boss CLI."""

from __future__ import annotations

import os
import ssl
import asyncio
import contextlib
from dataclasses import dataclass
from pathlib import Path
from typing import Any


TASK_STREAM = "CODEX_TASKS"
EVENT_STREAM = "CODEX_EVENTS"
HEARTBEAT_STREAM = "CODEX_HEARTBEATS"


@dataclass(frozen=True)
class NatsSettings:
    servers: tuple[str, ...]
    ca_file: str
    cert_file: str
    key_file: str
    user: str = ""
    password_env: str = ""
    connect_timeout_sec: float = 10.0

    @classmethod
    def from_dict(cls, value: dict[str, Any]) -> "NatsSettings":
        return cls(
            servers=tuple(str(item) for item in value.get("servers", [])),
            ca_file=str(value.get("ca_file", "")),
            cert_file=str(value.get("cert_file", "")),
            key_file=str(value.get("key_file", "")),
            user=str(value.get("user", "")),
            password_env=str(value.get("password_env", "")),
            connect_timeout_sec=float(value.get("connect_timeout_sec", 10.0)),
        )

    def password(self) -> str:
        if not self.password_env:
            return ""
        value = os.environ.get(self.password_env, "")
        if not value:
            raise RuntimeError(f"required environment variable is empty: {self.password_env}")
        return value

    def validate(self) -> None:
        if not self.servers:
            raise RuntimeError("at least one NATS server is required")
        if not all(server.startswith("tls://") for server in self.servers):
            raise RuntimeError("only tls:// NATS endpoints are allowed")
        if not self.ca_file or not Path(self.ca_file).expanduser().is_file():
            raise RuntimeError(f"NATS CA file is missing: {self.ca_file}")
        if not self.cert_file or not Path(self.cert_file).expanduser().is_file():
            raise RuntimeError(f"NATS client certificate is missing: {self.cert_file}")
        if not self.key_file or not Path(self.key_file).expanduser().is_file():
            raise RuntimeError(f"NATS client key is missing: {self.key_file}")
        if bool(self.user) != bool(self.password_env):
            raise RuntimeError("NATS user and password_env must be configured together")


class NatsBus:
    def __init__(self, settings: NatsSettings, client_name: str) -> None:
        self.settings = settings
        self.client_name = client_name
        self.nc: Any = None
        self.js: Any = None

    async def connect(self) -> None:
        self.settings.validate()
        try:
            import nats
        except ImportError as exc:
            raise RuntimeError(
                "nats-py is not installed; run codex_ops/deploy/install_agentd.sh"
            ) from exc

        context = ssl.create_default_context(cafile=str(Path(self.settings.ca_file).expanduser()))
        context.load_cert_chain(
            str(Path(self.settings.cert_file).expanduser()),
            str(Path(self.settings.key_file).expanduser()),
        )
        connect_args: dict[str, Any] = dict(
            servers=list(self.settings.servers),
            tls=context,
            name=self.client_name,
            connect_timeout=self.settings.connect_timeout_sec,
            reconnect_time_wait=2,
            max_reconnect_attempts=-1,
        )
        if self.settings.user:
            connect_args["user"] = self.settings.user
            connect_args["password"] = self.settings.password()
        self.nc = await nats.connect(**connect_args)
        self.js = self.nc.jetstream()

    async def close(self) -> None:
        if self.nc and not self.nc.is_closed:
            try:
                await asyncio.wait_for(self.nc.drain(), timeout=5)
            except asyncio.CancelledError:
                raise
            except Exception:
                with contextlib.suppress(Exception):
                    await self.nc.close()

    async def bootstrap(self) -> None:
        if not self.js:
            raise RuntimeError("NATS is not connected")
        await self._ensure_stream(
            TASK_STREAM,
            ["codex.task.*"],
            max_age=7 * 24 * 3600,
            max_bytes=256 * 1024 * 1024,
        )
        await self._ensure_stream(
            EVENT_STREAM,
            ["codex.event.>"],
            max_age=30 * 24 * 3600,
            max_bytes=1024 * 1024 * 1024,
        )
        await self._ensure_stream(
            HEARTBEAT_STREAM,
            ["codex.heartbeat.*"],
            max_age=24 * 3600,
            max_bytes=16 * 1024 * 1024,
            max_msgs_per_subject=1,
        )

    async def _ensure_stream(
        self,
        name: str,
        subjects: list[str],
        *,
        max_age: float,
        max_bytes: int,
        max_msgs_per_subject: int = -1,
    ) -> None:
        from nats.js.api import DiscardPolicy, RetentionPolicy, StorageType, StreamConfig
        from nats.js.errors import NotFoundError

        config = StreamConfig(
            name=name,
            subjects=subjects,
            retention=RetentionPolicy.LIMITS,
            storage=StorageType.FILE,
            discard=DiscardPolicy.OLD,
            max_age=max_age,
            max_bytes=max_bytes,
            max_msgs_per_subject=max_msgs_per_subject,
            duplicate_window=120,
        )
        try:
            await self.js.stream_info(name)
        except NotFoundError:
            await self.js.add_stream(config=config)
        else:
            await self.js.update_stream(config=config)

    async def publish_task(self, subject: str, payload: bytes, task_id: str) -> None:
        await self.js.publish(subject, payload, headers={"Nats-Msg-Id": task_id})

    async def publish_event(self, agent_id: str, payload: bytes) -> None:
        await self.js.publish(f"codex.event.{agent_id}", payload)

    async def publish_heartbeat(self, agent_id: str, payload: bytes) -> None:
        await self.js.publish(f"codex.heartbeat.{agent_id}", payload)

    async def pull_subscribe(self, agent_id: str) -> Any:
        durable = "worker_" + agent_id.replace("-", "_")
        return await self.js.pull_subscribe(f"codex.task.{agent_id}", durable=durable)
