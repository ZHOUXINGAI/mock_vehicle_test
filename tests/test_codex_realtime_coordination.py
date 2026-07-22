#!/usr/bin/env python3

from __future__ import annotations

import asyncio
import json
import os
import tempfile
import unittest
from pathlib import Path

from codex_ops.realtime.agentd import AgentDaemon
from codex_ops.realtime.codex_driver import (
    CodexDriver,
    CodexDriverConfig,
    CodexRunResult,
)
from codex_ops.realtime.config import AgentConfig
from codex_ops.realtime.nats_bus import NatsSettings
from codex_ops.realtime.protocol import TaskEnvelope, TaskSafety
from codex_ops.realtime.safety import PolicyRejected, WorkerPolicy
from codex_ops.realtime.store import TaskStore


class ProtocolTests(unittest.TestCase):
    def test_task_round_trip_and_peer_hop(self) -> None:
        parent = TaskEnvelope.create(
            from_agent="boss",
            to_agent="orin1-carrier",
            task_type="analysis",
            objective="Inspect the coordination protocol.",
            repo="mock_vehicle_test",
        )
        restored = TaskEnvelope.from_json(parent.to_json())
        self.assertEqual(restored, parent)

        child = TaskEnvelope.create(
            from_agent="orin1-carrier",
            to_agent="orin2-mini",
            task_type="peer_request",
            objective="Review the Mini contract.",
            repo=parent.repo,
            parent=parent,
        )
        self.assertEqual(child.root_task_id, parent.task_id)
        self.assertEqual(child.parent_task_id, parent.task_id)
        self.assertEqual(child.hop_count, 1)
        self.assertFalse(child.safety.requests_hardware())

    def test_invalid_hop_is_rejected(self) -> None:
        raw = TaskEnvelope.create(
            from_agent="boss",
            to_agent="orin1-carrier",
            task_type="analysis",
            objective="test",
            repo="mock_vehicle_test",
        ).to_dict()
        raw["hop_count"] = 5
        raw["max_hops"] = 4
        with self.assertRaisesRegex(ValueError, "hop count"):
            TaskEnvelope.from_dict(raw)


class PolicyTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name).resolve()
        self.policy = WorkerPolicy.create(
            agent_id="orin1-carrier",
            mode="observe",
            allowed_roots=[str(self.root)],
            repo_map={"mock_vehicle_test": str(self.root)},
        )

    def tearDown(self) -> None:
        self.temp.cleanup()

    def task(self, **changes: object) -> TaskEnvelope:
        values = {
            "from_agent": "boss",
            "to_agent": "orin1-carrier",
            "task_type": "analysis",
            "objective": "Inspect only.",
            "repo": "mock_vehicle_test",
            "safety": TaskSafety(),
        }
        values.update(changes)
        return TaskEnvelope.create(**values)  # type: ignore[arg-type]

    def test_alias_resolves_inside_allowed_root(self) -> None:
        self.assertEqual(self.policy.validate(self.task()), self.root)

    def test_hardware_capability_is_always_rejected(self) -> None:
        with self.assertRaisesRegex(PolicyRejected, "hardware capabilities"):
            self.policy.validate(self.task(safety=TaskSafety(actuator_access=True)))

    def test_code_task_is_rejected_in_observe_mode(self) -> None:
        with self.assertRaisesRegex(PolicyRejected, "observe mode"):
            self.policy.validate(self.task(task_type="code"))

    def test_outside_repo_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as outside:
            with self.assertRaisesRegex(PolicyRejected, "outside configured roots"):
                self.policy.validate(self.task(repo=outside))


class StoreTests(unittest.TestCase):
    def test_completed_task_is_idempotent(self) -> None:
        task = TaskEnvelope.create(
            from_agent="boss",
            to_agent="orin1-carrier",
            task_type="analysis",
            objective="test",
            repo="mock_vehicle_test",
        )
        with tempfile.TemporaryDirectory() as directory:
            store = TaskStore(Path(directory) / "state.sqlite3")
            self.assertEqual(store.claim(task), (True, 1, "running"))
            store.finish(task.task_id, "completed", "done")
            self.assertEqual(store.claim(task), (False, 1, "completed"))
            store.close()


class DriverTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temp = tempfile.TemporaryDirectory()
        root = Path(self.temp.name)
        self.repo = root / "repo"
        self.repo.mkdir()
        self.policy = WorkerPolicy.create(
            agent_id="orin1-carrier",
            mode="observe",
            allowed_roots=[str(self.repo)],
        )
        config = CodexDriverConfig(
            agent_id="orin1-carrier",
            role="test",
            codex_home=root / ".codex",
            session_file=root / "session.json",
            output_schema=root / "schema.json",
            result_dir=root / "runs",
            binary="/opt/codex/bin/codex",
            model="gpt-5.6-sol",
        )
        self.driver = CodexDriver(config, self.policy)

    def tearDown(self) -> None:
        self.temp.cleanup()

    def test_command_enforces_read_only_and_never_approval(self) -> None:
        command = self.driver.build_command(
            repo=self.repo,
            prompt="test",
            result_file=self.repo / "result.json",
        )
        self.assertIn("read-only", command)
        self.assertIn("never", command)
        self.assertIn("gpt-5.6-sol", command)
        self.assertEqual(command[0], "/opt/codex/bin/codex")
        self.assertNotIn("--dangerously-bypass-approvals-and-sandbox", command)

    def test_session_id_is_parsed_from_jsonl(self) -> None:
        session_id = "019e3b9d-8417-76d2-bf88-4ec59aba48c4"
        event_log = self.repo / "events.jsonl"
        event_log.write_text(
            json.dumps({"type": "thread.started", "thread_id": session_id}) + "\n",
            encoding="utf-8",
        )
        self.assertEqual(self.driver.parse_session_id(event_log), session_id)

    def test_broker_secret_is_not_inherited_by_codex(self) -> None:
        os.environ["CODEX_COORD_NATS_PASSWORD"] = "secret"
        os.environ["NATS_PASSWORD"] = "secret2"
        environment = self.driver._subprocess_env(Path("/tmp/codex-home"))
        self.assertNotIn("CODEX_COORD_NATS_PASSWORD", environment)
        self.assertNotIn("NATS_PASSWORD", environment)
        self.assertEqual(environment["CODEX_HOME"], "/tmp/codex-home")


class FakeBus:
    def __init__(self) -> None:
        self.tasks: list[tuple[str, bytes, str]] = []
        self.events: list[bytes] = []

    async def publish_task(self, subject: str, payload: bytes, task_id: str) -> None:
        self.tasks.append((subject, payload, task_id))

    async def publish_event(self, agent_id: str, payload: bytes) -> None:
        self.events.append(payload)


class BlockingSubscription:
    def __init__(self) -> None:
        self.started = asyncio.Event()
        self.cancelled = False

    async def fetch(self, **_: object) -> list[object]:
        self.started.set()
        try:
            await asyncio.sleep(3600)
        except asyncio.CancelledError:
            self.cancelled = True
            raise


class PeerDispatchTests(unittest.TestCase):
    def test_structured_peer_request_is_dispatched_without_hardware_rights(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            policy = WorkerPolicy.create(
                agent_id="orin1-carrier",
                mode="observe",
                allowed_roots=[str(root)],
                repo_map={"mock_vehicle_test": str(root)},
            )
            driver_config = CodexDriverConfig(
                agent_id="orin1-carrier",
                role="test",
                codex_home=root / ".codex",
                session_file=root / "session.json",
                output_schema=root / "schema.json",
                result_dir=root / "runs",
                enabled=False,
            )
            config = AgentConfig(
                agent_id="orin1-carrier",
                role="test",
                nats=NatsSettings(
                    ("tls://example:4222",),
                    str(root / "ca"),
                    str(root / "client.crt"),
                    str(root / "client.key"),
                ),
                policy=policy,
                driver=driver_config,
                state_db=root / "agent.sqlite3",
                heartbeat_sec=10,
                fetch_timeout_sec=1,
            )
            daemon = AgentDaemon(config)
            fake = FakeBus()
            daemon.bus = fake  # type: ignore[assignment]
            parent = TaskEnvelope.create(
                from_agent="boss",
                to_agent="orin1-carrier",
                task_type="analysis",
                objective="review",
                repo="mock_vehicle_test",
            )
            result = CodexRunResult(
                "completed",
                "done",
                {
                    "peer_requests": [
                        {
                            "to": "orin2-mini",
                            "task_type": "review",
                            "objective": "Review Mini behavior.",
                            "context_files": ["codex_ops/state/meeting_state.md"],
                            "acceptance": ["Return findings."],
                        }
                    ]
                },
                "",
                0,
                "",
                "",
            )
            dispatched = asyncio.run(daemon._dispatch_peer_requests(parent, result))
            self.assertEqual(len(dispatched), 1)
            self.assertEqual(fake.tasks[0][0], "codex.task.orin2-mini")
            peer = TaskEnvelope.from_json(fake.tasks[0][1])
            self.assertFalse(peer.safety.requests_hardware())
            self.assertEqual(peer.root_task_id, parent.task_id)
            daemon.store.close()


class ShutdownTests(unittest.TestCase):
    def test_fetch_stops_without_waiting_for_pull_timeout(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            policy = WorkerPolicy.create(
                agent_id="orin1-carrier",
                mode="observe",
                allowed_roots=[str(root)],
            )
            config = AgentConfig(
                agent_id="orin1-carrier",
                role="test",
                nats=NatsSettings(
                    ("tls://example:4222",),
                    str(root / "ca"),
                    str(root / "client.crt"),
                    str(root / "client.key"),
                ),
                policy=policy,
                driver=CodexDriverConfig(
                    agent_id="orin1-carrier",
                    role="test",
                    codex_home=root / ".codex",
                    session_file=root / "session.json",
                    output_schema=root / "schema.json",
                    result_dir=root / "runs",
                    enabled=False,
                ),
                state_db=root / "agent.sqlite3",
                heartbeat_sec=10,
                fetch_timeout_sec=300,
            )
            daemon = AgentDaemon(config)
            subscription = BlockingSubscription()

            async def exercise() -> None:
                pending = asyncio.create_task(daemon.fetch_or_stop(subscription, 300))
                await subscription.started.wait()
                daemon.stop_event.set()
                self.assertIsNone(await asyncio.wait_for(pending, timeout=1))
                self.assertTrue(subscription.cancelled)

            asyncio.run(exercise())
            daemon.store.close()


if __name__ == "__main__":
    unittest.main()
