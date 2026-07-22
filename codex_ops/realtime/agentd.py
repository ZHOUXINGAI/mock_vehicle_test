#!/usr/bin/env python3
"""Persistent Orin worker that turns durable cloud tasks into Codex turns."""

from __future__ import annotations

import argparse
import asyncio
import contextlib
import json
import logging
import signal
from pathlib import Path
from typing import Any

from .codex_driver import CodexDriver, CodexRunResult
from .config import AgentConfig, load_agent_config
from .nats_bus import NatsBus
from .protocol import AGENT_IDS, TaskEnvelope, TaskSafety, event_payload, utc_now
from .safety import PolicyRejected
from .store import TaskStore


LOG = logging.getLogger("codex-agentd")


class AgentDaemon:
    def __init__(self, config: AgentConfig) -> None:
        self.config = config
        self.bus = NatsBus(config.nats, f"codex-agentd-{config.agent_id}")
        self.driver = CodexDriver(config.driver, config.policy)
        self.store = TaskStore(config.state_db)
        self.stop_event = asyncio.Event()

    async def emit(
        self,
        event_type: str,
        *,
        task: TaskEnvelope | None = None,
        summary: str = "",
        **details: Any,
    ) -> None:
        payload = event_payload(
            agent_id=self.config.agent_id,
            event_type=event_type,
            task=task,
            summary=summary,
            **details,
        )
        await self.bus.publish_event(self.config.agent_id, payload)
        LOG.info("event=%s task=%s summary=%s", event_type, task.task_id if task else "-", summary)

    async def heartbeat_loop(self) -> None:
        while not self.stop_event.is_set():
            payload = json.dumps(
                {
                    "version": 1,
                    "agent_id": self.config.agent_id,
                    "created_at": utc_now(),
                    "status": "online",
                    "mode": self.config.policy.mode,
                    "codex_enabled": self.config.driver.enabled,
                },
                sort_keys=True,
            ).encode("utf-8")
            try:
                await self.bus.publish_heartbeat(self.config.agent_id, payload)
            except Exception:
                LOG.exception("heartbeat publish failed")
            try:
                await asyncio.wait_for(self.stop_event.wait(), timeout=self.config.heartbeat_sec)
            except asyncio.TimeoutError:
                pass

    async def _keep_task_alive(self, message: Any, task: TaskEnvelope) -> None:
        while True:
            await asyncio.sleep(20)
            await message.in_progress()
            await self.emit("progress", task=task, summary="Codex turn still running")

    async def _dispatch_peer_requests(
        self, parent: TaskEnvelope, result: CodexRunResult
    ) -> list[str]:
        dispatched: list[str] = []
        requests = result.output.get("peer_requests", [])
        if not isinstance(requests, list):
            return dispatched
        if parent.hop_count >= min(parent.max_hops, self.config.policy.max_hops):
            if requests:
                await self.emit(
                    "peer_request_rejected",
                    task=parent,
                    summary="peer request hop limit reached",
                )
            return dispatched

        for request in requests[:8]:
            if not isinstance(request, dict):
                continue
            target = str(request.get("to", ""))
            if target not in AGENT_IDS - {"boss", self.config.agent_id}:
                await self.emit(
                    "peer_request_rejected",
                    task=parent,
                    summary=f"invalid peer target: {target!r}",
                )
                continue
            try:
                peer_task = TaskEnvelope.create(
                    from_agent=self.config.agent_id,
                    to_agent=target,
                    task_type=str(request.get("task_type", "peer_request")),
                    objective=str(request.get("objective", "")),
                    repo=parent.repo,
                    base_commit=parent.base_commit,
                    context_files=[str(item) for item in request.get("context_files", [])],
                    acceptance=[str(item) for item in request.get("acceptance", [])],
                    safety=TaskSafety(),
                    parent=parent,
                )
            except (TypeError, ValueError) as exc:
                await self.emit(
                    "peer_request_rejected",
                    task=parent,
                    summary=f"invalid peer request: {exc}",
                )
                continue
            await self.bus.publish_task(
                f"codex.task.{target}", peer_task.to_json(), peer_task.task_id
            )
            dispatched.append(peer_task.task_id)
            await self.emit(
                "peer_dispatched",
                task=parent,
                summary=f"dispatched peer task to {target}",
                peer_task_id=peer_task.task_id,
                peer_agent=target,
            )
        return dispatched

    async def handle_message(self, message: Any) -> None:
        task: TaskEnvelope | None = None
        try:
            task = TaskEnvelope.from_json(message.data)
            repo = self.config.policy.validate(task)
        except (KeyError, TypeError, ValueError, PolicyRejected) as exc:
            LOG.warning("rejected task payload: %s", exc)
            if task:
                self.store.claim(task)
                self.store.finish(task.task_id, "rejected", str(exc))
                await self.emit("rejected", task=task, summary=str(exc))
            await message.term()
            return

        claimed, attempts, prior_status = self.store.claim(task)
        if not claimed:
            await self.emit(
                "duplicate",
                task=task,
                summary=f"task already {prior_status}",
                attempts=attempts,
            )
            await message.ack()
            return

        await self.emit("accepted", task=task, summary="task accepted", attempts=attempts)
        keepalive = asyncio.create_task(self._keep_task_alive(message, task))
        try:
            result = await asyncio.to_thread(self.driver.run, task, repo)
        except Exception as exc:
            LOG.exception("Codex driver failed")
            self.store.finish(task.task_id, "failed", str(exc))
            await self.emit("failed", task=task, summary=f"Codex driver exception: {exc}")
            await message.nak(delay=10)
            return
        finally:
            keepalive.cancel()
            try:
                await keepalive
            except asyncio.CancelledError:
                pass

        peer_task_ids = await self._dispatch_peer_requests(task, result)
        self.store.finish(task.task_id, result.status, result.summary)
        await self.emit(
            result.status,
            task=task,
            summary=result.summary,
            exit_code=result.exit_code,
            session_id=result.session_id,
            event_log=result.event_log,
            stderr_log=result.stderr_log,
            details=result.output.get("details", ""),
            artifacts=result.output.get("artifacts", []),
            requires_boss=bool(result.output.get("requires_boss", False)),
            peer_task_ids=peer_task_ids,
        )
        await message.ack()

    async def fetch_or_stop(self, subscription: Any, timeout: float) -> list[Any] | None:
        fetch = asyncio.create_task(subscription.fetch(batch=1, timeout=timeout))
        stopped = asyncio.create_task(self.stop_event.wait())
        done, pending = await asyncio.wait({fetch, stopped}, return_when=asyncio.FIRST_COMPLETED)
        for task in pending:
            task.cancel()
        for task in pending:
            with contextlib.suppress(asyncio.CancelledError):
                await task
        if stopped in done and stopped.result():
            if fetch.done():
                with contextlib.suppress(asyncio.CancelledError, Exception):
                    fetch.result()
            else:
                fetch.cancel()
            return None
        return fetch.result()

    async def run(self, *, once: bool = False, once_timeout: float = 30.0) -> int:
        await self.bus.connect()
        subscription = await self.bus.pull_subscribe(self.config.agent_id)
        await self.emit(
            "online",
            summary=f"worker online in {self.config.policy.mode} mode",
            codex_enabled=self.config.driver.enabled,
        )
        heartbeat = asyncio.create_task(self.heartbeat_loop())
        handled = 0
        try:
            while not self.stop_event.is_set():
                try:
                    messages = await self.fetch_or_stop(
                        subscription, once_timeout if once else self.config.fetch_timeout_sec
                    )
                    if messages is None:
                        break
                except asyncio.TimeoutError:
                    if once:
                        return 2
                    continue
                except Exception as exc:
                    if exc.__class__.__name__ == "TimeoutError":
                        if once:
                            return 2
                        continue
                    raise
                for message in messages:
                    await self.handle_message(message)
                    handled += 1
                    if once and handled:
                        return 0
        finally:
            self.stop_event.set()
            heartbeat.cancel()
            try:
                await heartbeat
            except asyncio.CancelledError:
                pass
            self.store.close()
            await self.bus.close()
        return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--once", action="store_true", help="Process one task and exit.")
    parser.add_argument("--once-timeout", type=float, default=30.0)
    parser.add_argument("--log-level", default="INFO")
    return parser


async def async_main(args: argparse.Namespace) -> int:
    config = load_agent_config(args.config)
    daemon = AgentDaemon(config)
    loop = asyncio.get_running_loop()
    for signum in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(signum, daemon.stop_event.set)
    return await daemon.run(once=args.once, once_timeout=args.once_timeout)


def main() -> int:
    args = build_parser().parse_args()
    logging.basicConfig(
        level=getattr(logging, args.log_level.upper(), logging.INFO),
        format="%(asctime)s %(levelname)s %(name)s %(message)s",
    )
    return asyncio.run(async_main(args))


if __name__ == "__main__":
    raise SystemExit(main())
