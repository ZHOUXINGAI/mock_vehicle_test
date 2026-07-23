#!/usr/bin/env python3
"""Boss/operator CLI for the durable Codex coordination bus."""

from __future__ import annotations

import argparse
import asyncio
import json
from pathlib import Path
from typing import Any

from .config import load_json
from .console import format_event_as_chat, format_event_for_console
from .nats_bus import NatsBus, NatsSettings
from .protocol import TaskEnvelope, TaskSafety, event_payload


def load_bus(path: str) -> NatsBus:
    raw = load_json(path)
    return NatsBus(NatsSettings.from_dict(raw["nats"]), "codex-coordinator-boss")


def make_task(args: argparse.Namespace) -> TaskEnvelope:
    objective = args.objective
    if args.objective_file:
        objective = Path(args.objective_file).read_text(encoding="utf-8")
    return TaskEnvelope.create(
        from_agent=args.from_agent,
        to_agent=args.to,
        task_type=args.task_type,
        objective=objective,
        repo=args.repo,
        base_commit=args.base_commit,
        context_files=args.context_file or [],
        acceptance=args.acceptance or [],
        safety=TaskSafety(),
    )


async def cmd_bootstrap(args: argparse.Namespace) -> int:
    bus = load_bus(args.config)
    await bus.connect()
    try:
        await bus.bootstrap()
    finally:
        await bus.close()
    print("JetStream streams are ready: CODEX_TASKS CODEX_EVENTS CODEX_HEARTBEATS")
    return 0


async def cmd_send(args: argparse.Namespace) -> int:
    task = make_task(args)
    if args.dry_run:
        print(task.to_json().decode("utf-8"))
        return 0
    bus = load_bus(args.config)
    await bus.connect()
    subscription = None
    if args.wait > 0:
        subscription = await bus.nc.subscribe("codex.event.>")
    try:
        await bus.publish_event(
            "boss",
            event_payload(
                agent_id="boss",
                event_type="dispatched",
                task=task,
                summary="task dispatched",
                objective=task.objective,
                to_agent=task.to_agent,
                repo=task.repo,
                task_type=task.task_type,
            ),
        )
        await bus.publish_task(f"codex.task.{task.to_agent}", task.to_json(), task.task_id)
        print(json.dumps({"published": True, "task_id": task.task_id, "to": task.to_agent}))
        if not subscription:
            return 0
        deadline = asyncio.get_running_loop().time() + args.wait
        while asyncio.get_running_loop().time() < deadline:
            remaining = deadline - asyncio.get_running_loop().time()
            try:
                message = await subscription.next_msg(timeout=remaining)
            except Exception as exc:
                if exc.__class__.__name__ == "TimeoutError":
                    print(json.dumps({"task_id": task.task_id, "wait": "timeout"}))
                    return 2
                raise
            event = json.loads(message.data)
            if event.get("task_id") != task.task_id:
                continue
            print(json.dumps(event, ensure_ascii=False, sort_keys=True))
            if event.get("event_type") in {"completed", "blocked", "rejected", "failed"}:
                return 0 if event.get("event_type") == "completed" else 3
        return 2
    finally:
        await bus.close()


async def next_message_forever(subscription: Any) -> Any:
    """Wait across nats-py idle timeouts until a message arrives."""
    while True:
        try:
            return await subscription.next_msg(timeout=1.0)
        except Exception as exc:
            if exc.__class__.__name__ == "TimeoutError":
                continue
            raise


async def cmd_watch(args: argparse.Namespace) -> int:
    bus = load_bus(args.config)
    await bus.connect()
    subscription = await bus.nc.subscribe(args.subject)
    try:
        while True:
            message = await next_message_forever(subscription)
            text = message.data.decode("utf-8")
            if args.chat:
                try:
                    text = format_event_as_chat(json.loads(text))
                except (json.JSONDecodeError, TypeError, ValueError):
                    pass
            elif args.pretty:
                try:
                    text = format_event_for_console(json.loads(text))
                except (json.JSONDecodeError, TypeError, ValueError):
                    pass
            print(text, flush=True)
    finally:
        await bus.close()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True, help="Boss JSON config containing NATS TLS settings.")
    sub = parser.add_subparsers(dest="command", required=True)

    bootstrap = sub.add_parser("bootstrap", help="Create/update required JetStream streams.")
    bootstrap.set_defaults(func=cmd_bootstrap)

    send = sub.add_parser("send", help="Publish one no-hardware Codex task.")
    send.add_argument("--from-agent", default="boss")
    send.add_argument("--to", required=True, choices=["orin1-carrier", "orin2-mini"])
    send.add_argument(
        "--task-type",
        default="analysis",
        choices=["analysis", "code", "review", "diagnostic", "peer_request"],
    )
    source = send.add_mutually_exclusive_group(required=True)
    source.add_argument("--objective")
    source.add_argument("--objective-file")
    send.add_argument("--repo", default="mock_vehicle_test")
    send.add_argument("--base-commit", default="")
    send.add_argument("--context-file", action="append")
    send.add_argument("--acceptance", action="append")
    send.add_argument("--wait", type=float, default=0.0)
    send.add_argument("--dry-run", action="store_true")
    send.set_defaults(func=cmd_send)

    watch = sub.add_parser("watch", help="Watch live agent events or heartbeats.")
    watch.add_argument("--subject", default="codex.event.>")
    watch.add_argument("--pretty", action="store_true", help="Show readable operator activity.")
    watch.add_argument(
        "--chat",
        action="store_true",
        help="Show a read-only terminal chat transcript.",
    )
    watch.set_defaults(func=cmd_watch)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    return asyncio.run(args.func(args))


if __name__ == "__main__":
    raise SystemExit(main())
