#!/usr/bin/env python3

from __future__ import annotations

import asyncio
import io
import json
import os
import tempfile
import unittest
from pathlib import Path

from codex_ops.realtime.agentd import AgentDaemon
from codex_ops.realtime.app_server_driver import (
    AppServerDriver,
    format_app_server_activity,
)
from codex_ops.realtime.codex_driver import (
    CodexDriver,
    CodexDriverConfig,
    CodexRunResult,
    mirror_stream,
)
from codex_ops.realtime.config import AgentConfig
from codex_ops.realtime.console import (
    format_codex_activity,
    format_event_as_chat,
    format_event_for_console,
)
from codex_ops.realtime.coordctl import next_message_forever
from codex_ops.realtime.nats_bus import NatsSettings
from codex_ops.realtime.protocol import TaskEnvelope, TaskSafety
from codex_ops.realtime.safety import PolicyRejected, WorkerPolicy
from codex_ops.realtime.store import TaskStore
from codex_ops.scripts.set_agent_codex_enabled import update_config


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

    def test_codex_output_is_persisted_and_mirrored_to_journal(self) -> None:
        source = io.BytesIO(
            b'{"type":"item.completed","item":{"type":"agent_message",'
            b'"text":"Inspected the repository status."}}\n'
        )
        destination = io.BytesIO()
        activity: list[dict[str, str]] = []

        with self.assertLogs("codex-agentd.codex", level="INFO") as captured:
            mirror_stream(source, destination, "stdout", activity.append)

        self.assertEqual(destination.getvalue(), source.getvalue())
        self.assertIn("Codex：Inspected the repository status.", captured.output[0])
        self.assertEqual(activity[0]["kind"], "message")


class AppServerDriverTests(unittest.TestCase):
    def test_model_refresh_timeout_is_one_nonfatal_notice(self) -> None:
        line = (
            "ERROR codex_models_manager::manager: failed to refresh available "
            "models: timeout waiting for child process to exit\n"
        )
        source = io.StringIO(line + line + "ERROR real failure\n")
        destination = io.StringIO()
        activity: list[dict[str, str]] = []

        AppServerDriver._stderr_reader(source, destination, activity.append)

        self.assertEqual(destination.getvalue(), line + line + "ERROR real failure\n")
        self.assertEqual(
            activity,
            [
                {
                    "kind": "notice",
                    "summary": "模型列表后台刷新超时；继续使用当前模型，本次任务不受影响",
                },
                {
                    "kind": "error",
                    "summary": "app-server：ERROR real failure",
                },
            ],
        )

    def test_official_stdio_lifecycle_streams_activity_and_structured_result(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            repo = root / "repo"
            repo.mkdir()
            schema = root / "schema.json"
            schema.write_text(
                json.dumps({"type": "object", "additionalProperties": True}),
                encoding="utf-8",
            )
            fake = root / "fake-codex"
            fake.write_text(
                """#!/usr/bin/env python3
import json
import sys

result = {
    "status": "completed",
    "summary": "bridge pass",
    "details": "read-only",
    "peer_requests": [],
    "artifacts": [],
    "requires_boss": False,
}
for line in sys.stdin:
    message = json.loads(line)
    method = message.get("method")
    request_id = message.get("id")
    if method == "initialize":
        print(json.dumps({"id": request_id, "result": {"codexHome": "/tmp/codex"}}), flush=True)
    elif method == "initialized":
        continue
    elif method == "thread/resume":
        print(json.dumps({
            "id": request_id,
            "error": {
                "code": -32600,
                "message": "no rollout found for thread id stale_thread_123",
            },
        }), flush=True)
    elif method == "thread/start":
        assert message["params"]["sandbox"] == "read-only", message
        thread = {"id": "thread_12345", "turns": []}
        print(json.dumps({"id": request_id, "result": {"thread": thread}}), flush=True)
        print(json.dumps({"method": "thread/started", "params": {"thread": thread}}), flush=True)
    elif method == "turn/start":
        assert message["params"]["sandboxPolicy"]["type"] == "readOnly", message
        turn = {"id": "turn_12345", "status": "inProgress", "items": [], "error": None}
        print(json.dumps({"id": request_id, "result": {"turn": turn}}), flush=True)
        print(json.dumps({"method": "turn/started", "params": {"turn": turn}}), flush=True)
        item = {
            "id": "cmd_1",
            "type": "commandExecution",
            "command": "git status --short",
            "status": "inProgress",
        }
        print(json.dumps({"method": "item/started", "params": {"item": item}}), flush=True)
        item["status"] = "completed"
        item["exitCode"] = 0
        print(json.dumps({"method": "item/completed", "params": {"item": item}}), flush=True)
        answer = {"id": "msg_1", "type": "agentMessage", "text": json.dumps(result)}
        print(json.dumps({"method": "item/completed", "params": {"item": answer}}), flush=True)
        turn["status"] = "completed"
        print(json.dumps({"method": "turn/completed", "params": {"turn": turn}}), flush=True)
""",
                encoding="utf-8",
            )
            fake.chmod(0o755)
            policy = WorkerPolicy.create(
                agent_id="orin1-carrier",
                mode="observe",
                allowed_roots=[str(repo)],
                repo_map={"mock_vehicle_test": str(repo)},
            )
            session_file = root / "session.json"
            session_file.write_text(
                json.dumps(
                    {
                        "agent_id": "orin1-carrier",
                        "thread_id": "stale_thread_123",
                    }
                ),
                encoding="utf-8",
            )
            config = CodexDriverConfig(
                agent_id="orin1-carrier",
                role="test",
                codex_home=root / ".codex",
                session_file=session_file,
                output_schema=schema,
                result_dir=root / "runs",
                binary=str(fake),
                backend="app-server",
                timeout_sec=5,
            )
            driver = AppServerDriver(config, policy)
            task = TaskEnvelope.create(
                from_agent="boss",
                to_agent="orin1-carrier",
                task_type="analysis",
                objective="Read-only bridge test.",
                repo="mock_vehicle_test",
            )
            activity: list[dict[str, str]] = []

            result = driver.run(task, repo, activity.append)

            self.assertEqual(result.status, "completed")
            self.assertEqual(result.summary, "bridge pass")
            self.assertEqual(result.session_id, "thread_12345")
            self.assertEqual(driver.load_thread_id(), "thread_12345")
            self.assertTrue(
                any("stale" in item["summary"].lower() for item in activity)
            )
            self.assertTrue(any(item["kind"] == "command" for item in activity))
            self.assertTrue(any(item["kind"] == "message" for item in activity))
            self.assertTrue(Path(result.event_log).is_file())

    def test_app_server_reasoning_does_not_expose_private_text(self) -> None:
        activity = format_app_server_activity(
            {
                "method": "item/started",
                "params": {
                    "item": {
                        "id": "reason_1",
                        "type": "reasoning",
                        "content": ["private chain of thought"],
                    }
                },
            }
        )

        self.assertIsNotNone(activity)
        self.assertNotIn("private chain of thought", activity["summary"])  # type: ignore[index]

    def test_app_server_command_activity_is_human_readable(self) -> None:
        activity = format_app_server_activity(
            {
                "method": "item/started",
                "params": {
                    "item": {
                        "id": "command_1",
                        "type": "commandExecution",
                        "command": "git status --short",
                    }
                },
            }
        )

        self.assertEqual(
            activity,
            {
                "kind": "command",
                "summary": "运行命令：git status --short",
            },
        )


class ConsoleFormattingTests(unittest.TestCase):
    def test_chat_console_renders_ground_and_structured_codex_messages(self) -> None:
        dispatched = format_event_as_chat(
            {
                "event_type": "dispatched",
                "agent_id": "boss",
                "to_agent": "orin1-carrier",
                "from_agent": "boss",
                "task_id": "12345678-abcd",
                "created_at": "2026-07-23T13:30:00+08:00",
                "objective": "Inspect the repository without modifying it.",
            }
        )
        reply = format_event_as_chat(
            {
                "event_type": "activity",
                "activity_kind": "message",
                "agent_id": "orin1-carrier",
                "task_id": "12345678-abcd",
                "created_at": "2026-07-23T13:30:01+08:00",
                "summary": (
                    'Codex：{"status":"completed","summary":"Inspection passed",'
                    '"details":"No files were changed."}'
                ),
            }
        )
        accepted = format_event_as_chat(
            {
                "event_type": "accepted",
                "agent_id": "orin1-carrier",
                "task_id": "12345678-abcd",
                "created_at": "2026-07-23T13:30:01+08:00",
            }
        )
        completed = format_event_as_chat(
            {
                "event_type": "completed",
                "agent_id": "orin1-carrier",
                "task_id": "12345678-abcd",
                "created_at": "2026-07-23T13:30:02+08:00",
                "summary": "Inspection passed",
                "details": "No files were changed.",
            }
        )

        self.assertIn("Ground/Boss → Orin1/Carrier", dispatched)
        self.assertIn("Inspect the repository", dispatched)
        self.assertIn("🤖 Orin1/Carrier", reply)
        self.assertIn("Inspection passed", reply)
        self.assertIn("No files were changed.", reply)
        self.assertEqual(
            accepted,
            "[13:30:01] 📥 Orin1/Carrier 已接收任务  task=12345678",
        )
        self.assertEqual(
            completed,
            "[13:30:02] ✅ Orin1/Carrier 已完成：Inspection passed  task=12345678",
        )

    def test_command_events_are_readable(self) -> None:
        activity = format_codex_activity(
            json.dumps(
                {
                    "type": "item.started",
                    "item": {
                        "type": "command_execution",
                        "command": "/usr/bin/git status --short",
                    },
                }
            )
        )

        self.assertIsNotNone(activity)
        self.assertEqual(activity["kind"], "command")  # type: ignore[index]
        self.assertIn("git status --short", activity["summary"])  # type: ignore[index]

    def test_private_reasoning_text_is_not_exposed(self) -> None:
        activity = format_codex_activity(
            json.dumps(
                {
                    "type": "item.started",
                    "item": {"type": "reasoning", "text": "private model reasoning"},
                }
            )
        )

        self.assertIsNotNone(activity)
        self.assertNotIn("private model reasoning", activity["summary"])  # type: ignore[index]

    def test_accepted_event_includes_concrete_objective(self) -> None:
        line = format_event_for_console(
            {
                "created_at": "2026-07-23T12:34:56+00:00",
                "agent_id": "orin1-carrier",
                "event_type": "accepted",
                "task_id": "12345678-abcd",
                "summary": "task accepted",
                "objective": "Inspect Git status and report changed files.",
            }
        )

        self.assertIn("Orin1/Carrier", line)
        self.assertIn("Inspect Git status and report changed files.", line)
        self.assertIn("task=12345678", line)

    def test_ground_dispatch_names_target_and_objective(self) -> None:
        line = format_event_for_console(
            {
                "created_at": "2026-07-23T12:34:56+00:00",
                "agent_id": "boss",
                "event_type": "dispatched",
                "task_id": "87654321-abcd",
                "to_agent": "orin1-carrier",
                "objective": "Report transport status without starting Codex.",
            }
        )

        self.assertIn("Ground/Boss", line)
        self.assertIn("Orin1/Carrier", line)
        self.assertIn("Report transport status without starting Codex.", line)


class ConfigToggleTests(unittest.TestCase):
    def test_enable_requires_expected_agent_and_observe_mode(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "orin1-carrier.json"
            path.write_text(
                json.dumps(
                    {
                        "agent_id": "orin1-carrier",
                        "policy": {"mode": "observe"},
                        "codex": {"enabled": False},
                    }
                ),
                encoding="utf-8",
            )
            path.chmod(0o600)

            backup = update_config(
                path,
                enabled=True,
                require_agent="orin1-carrier",
                require_mode="observe",
            )

            self.assertIsNotNone(backup)
            self.assertTrue(backup.is_file())  # type: ignore[union-attr]
            self.assertTrue(json.loads(path.read_text())["codex"]["enabled"])
            self.assertEqual(path.stat().st_mode & 0o777, 0o600)

    def test_enable_refuses_wrong_agent(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "agent.json"
            path.write_text(
                json.dumps(
                    {
                        "agent_id": "orin2-mini",
                        "policy": {"mode": "observe"},
                        "codex": {"enabled": False},
                    }
                ),
                encoding="utf-8",
            )

            with self.assertRaisesRegex(RuntimeError, "agent mismatch"):
                update_config(
                    path,
                    enabled=True,
                    require_agent="orin1-carrier",
                    require_mode="observe",
                )


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


class TimeoutThenMessageSubscription:
    def __init__(self, message: object) -> None:
        self.message = message
        self.calls = 0

    async def next_msg(self, *, timeout: float) -> object:
        self.calls += 1
        if self.calls == 1:
            raise asyncio.TimeoutError
        return self.message


class WatchTests(unittest.TestCase):
    def test_idle_timeout_does_not_stop_watcher(self) -> None:
        expected = object()
        subscription = TimeoutThenMessageSubscription(expected)

        received = asyncio.run(next_message_forever(subscription))

        self.assertIs(received, expected)
        self.assertEqual(subscription.calls, 2)


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
