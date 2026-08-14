from __future__ import annotations

import dataclasses
import os
import signal
import sys
import unittest
from pathlib import Path
from unittest import mock

REPO_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_DIR / "scripts"))
sys.path.insert(0, str(REPO_DIR / "src"))

from lr24_command_guard import Decision, MiniCommandGate
from lr24_compact_protocol import (
    Frame,
    MessageType,
    MiniState,
    MissionExecutionState,
    MissionStatus,
    Role,
)
from pairb_live_mission import WorkerPhase
from pairb_live_mission import MiniMissionEndpointCore
from pairb_staged_chase import (
    ChasePhase,
    REQUIRED_MINI_HEALTH,
    StagedChaseCoordinator,
    build_staged_mission_plan,
)
from run_pairb_staged_chase_live import (
    GatedMissionWorker,
    MissionStatusSessionGate,
    abort_exit_ready,
    advance_periodic_deadline,
    build_parser,
    build_worker_environment,
    config_from_args,
    configure_host_local_ros_environment,
    drain_pairb_transport,
    missing_field_confirmations,
    mini_command_guard_policy,
    open_pairb_transport,
    consume_worker_result_bytes,
    runtime_shutdown_line,
    spin_pending_callbacks,
    supervisor_timeout_reason,
    terminal_worker_status_due,
    worker_runtime_hold_timeout_sec,
    worker_result_line,
    worker_status,
)
from orin2_outdoor_forward_5m import external_runtime_stop_token


class PairBStagedLiveConfigTest(unittest.TestCase):
    def test_periodic_deadline_preserves_rate_without_catch_up_burst(self) -> None:
        self.assertAlmostEqual(advance_periodic_deadline(10.0, 10.005, 0.02), 10.02)
        self.assertAlmostEqual(advance_periodic_deadline(10.0, 10.03, 0.02), 10.05)

    def test_terminal_worker_status_is_sent_once_with_priority(self) -> None:
        self.assertFalse(terminal_worker_status_due(WorkerPhase.RUNNING, False))
        self.assertTrue(terminal_worker_status_due(WorkerPhase.COMPLETE, False))
        self.assertTrue(terminal_worker_status_due(WorkerPhase.FAILED, False))
        self.assertTrue(terminal_worker_status_due(WorkerPhase.STOPPED, False))
        self.assertFalse(terminal_worker_status_due(WorkerPhase.COMPLETE, True))

    def test_ros_callback_batch_is_bounded_and_drains_multiple_callbacks(self) -> None:
        source = mock.Mock()
        spin_pending_callbacks(source, 7)
        self.assertEqual(source.spin_once.call_count, 7)
        source.spin_once.assert_called_with(0.0)
        for invalid in (0, -1, 1.5):
            with self.assertRaises(ValueError):
                spin_pending_callbacks(source, invalid)

    def test_dry_defaults_preserve_mini_first_safety_gap(self) -> None:
        args = build_parser().parse_args(["carrier"])
        config = config_from_args(args)
        self.assertEqual(config.lead_delay_ms, 5000)
        self.assertEqual(config.lead_distance_m, 2.0)
        self.assertGreater(config.mini_speed_mps, config.carrier_speed_mps)
        self.assertEqual(args.ready_wait_timeout_sec, 540.0)
        self.assertEqual(args.command_rate_hz, 10.0)
        self.assertEqual(args.state_rate_hz, 50.0)
        self.assertEqual(args.status_rate_hz, 5.0)
        self.assertEqual(args.baud, 115200)
        self.assertEqual(args.initial_mini_ahead_m, 0.5)

    def test_state_rate_accepts_50_hz_and_rejects_above_limit(self) -> None:
        parser = build_parser()
        config_from_args(parser.parse_args(["mini", "--state-rate-hz", "50"]))
        with self.assertRaisesRegex(ValueError, "state rate"):
            config_from_args(parser.parse_args(["mini", "--state-rate-hz", "50.1"]))

    def test_pairb_transport_matches_current_physical_topology(self) -> None:
        parser = build_parser()
        with mock.patch("run_pairb_staged_chase_live.make_transport") as make:
            open_pairb_transport(parser.parse_args(["mini"]))
            mini_call = make.call_args
        self.assertEqual(mini_call.args[0], "mavros-router")
        self.assertIsNone(mini_call.kwargs["port"])
        self.assertEqual(mini_call.kwargs["source_system"], 1)
        self.assertEqual(mini_call.kwargs["target_system"], 2)
        self.assertEqual(mini_call.kwargs["expected_source_system"], 2)

        with mock.patch("run_pairb_staged_chase_live.make_transport") as make:
            open_pairb_transport(parser.parse_args(["carrier"]))
            carrier_call = make.call_args
        self.assertEqual(carrier_call.args[0], "mavlink-serial")
        self.assertIn("CP2102", carrier_call.kwargs["port"])
        self.assertEqual(carrier_call.kwargs["source_system"], 2)
        self.assertEqual(carrier_call.kwargs["target_system"], 1)
        self.assertEqual(carrier_call.kwargs["expected_source_system"], 1)

    def test_initial_formation_lead_is_bounded(self) -> None:
        parser = build_parser()
        for value in ("0.09", "5.01", "nan"):
            with self.assertRaises(ValueError):
                config_from_args(
                    parser.parse_args(["carrier", "--initial-mini-ahead-m", value])
                )

    def test_worker_environment_is_host_local_and_pipe_gated(self) -> None:
        config = config_from_args(build_parser().parse_args(["mini"]))
        with mock.patch.dict(os.environ, {}, clear=True):
            env = build_worker_environment("mini", config, 17)
        self.assertEqual(env["ROS_LOCALHOST_ONLY"], "1")
        self.assertEqual(env["ROS_DOMAIN_ID"], "99")
        self.assertEqual(env["PAIRB_START_GATE_FD"], "17")
        self.assertEqual(env["PAIRB_START_PLAN_ID"], "1")
        self.assertEqual(env["MISSION_PROFILE"], "s_bend_return")
        self.assertEqual(env["PAIRB_START_GATE_TIMEOUT_SEC"], "600")

    def test_supervisor_forces_same_host_local_ros_scope_as_worker(self) -> None:
        environment = {"ROS_LOCALHOST_ONLY": "0"}
        configure_host_local_ros_environment(environment)
        self.assertEqual(environment["ROS_LOCALHOST_ONLY"], "1")
        self.assertEqual(environment["ROS_DOMAIN_ID"], "99")

    def test_completed_worker_hold_covers_slower_peer_mission_window(self) -> None:
        config = config_from_args(build_parser().parse_args(["mini"]))
        with mock.patch.dict(os.environ, {}, clear=True):
            env = build_worker_environment("mini", config, 17, 18, 19)

        self.assertEqual(worker_runtime_hold_timeout_sec(config), 190)
        self.assertEqual(env["PAIRB_RUNTIME_HOLD_TIMEOUT_SEC"], "190")

    def test_worker_environment_passes_separate_runtime_completion_gate(self) -> None:
        config = config_from_args(build_parser().parse_args(["carrier"]))
        with mock.patch.dict(os.environ, {}, clear=True):
            env = build_worker_environment("carrier", config, 17, 18, 19, 20)
        self.assertEqual(env["PAIRB_RUNTIME_STOP_FD"], "20")

    def test_ready_wait_and_motion_deadlines_are_separate(self) -> None:
        self.assertIsNone(
            supervisor_timeout_reason(
                539.9,
                ready_deadline=540.0,
                mission_deadline=None,
            )
        )
        self.assertEqual(
            supervisor_timeout_reason(
                540.0,
                ready_deadline=540.0,
                mission_deadline=None,
            ),
            "ready_wait_timeout",
        )
        self.assertIsNone(
            supervisor_timeout_reason(
                700.0,
                ready_deadline=540.0,
                mission_deadline=800.0,
            )
        )
        self.assertEqual(
            supervisor_timeout_reason(
                800.0,
                ready_deadline=540.0,
                mission_deadline=800.0,
            ),
            "mission_timeout",
        )

    def test_ready_wait_timeout_is_bounded(self) -> None:
        parser = build_parser()
        config_from_args(parser.parse_args(["mini", "--ready-wait-timeout-sec", "30"]))
        for value in ("29.9", "540.1"):
            with self.assertRaises(ValueError):
                config_from_args(
                    parser.parse_args(["mini", "--ready-wait-timeout-sec", value])
                )

    def test_mini_guard_accepts_configured_180_second_staged_plan(self) -> None:
        config = config_from_args(build_parser().parse_args(["mini"]))
        policy = mini_command_guard_policy(config)
        self.assertEqual(config.plan_validity_ms, 180000)
        self.assertEqual(config.command_ttl_ms, 2000)
        self.assertEqual(policy.max_plan_ttl_ms, config.plan_validity_ms)
        self.assertEqual(policy.command_watchdog_ms, 1500)
        plan = build_staged_mission_plan(
            config,
            seq=1,
            sender_monotonic_ms=1000,
        )
        accepted = MiniCommandGate(policy).ingest(
            Frame(MessageType.STAGED_MISSION_PLAN, plan.encode()),
            5000,
        )
        self.assertEqual(accepted.decision, Decision.ACCEPT)

        too_long = dataclasses.replace(plan, valid_until_ms=181001)
        rejected = MiniCommandGate(policy).ingest(
            Frame(MessageType.STAGED_MISSION_PLAN, too_long.encode()),
            5000,
        )
        self.assertEqual(rejected.decision, Decision.REJECT)
        self.assertEqual(rejected.reason, "invalid_staged_plan_ttl")

    def test_live_command_timing_tolerates_jitter_then_fails_closed(self) -> None:
        config = config_from_args(build_parser().parse_args(["mini"]))
        endpoint = MiniMissionEndpointCore(mini_command_guard_policy(config))
        endpoint.set_local_prestate_ready(True)
        plan = build_staged_mission_plan(
            config,
            seq=0,
            sender_monotonic_ms=1000,
        )
        endpoint.ingest(
            Frame(MessageType.STAGED_MISSION_PLAN, plan.encode()),
            5000,
        )
        coordinator = StagedChaseCoordinator(config)
        coordinator.set_carrier_ready(True)
        coordinator.authorize_start()
        coordinator.accept_mini_state(
            MiniState(
                vehicle_id=1,
                seq=0,
                timestamp_ms=1000,
                x_m=0.0,
                y_m=0.0,
                vx_mps=0.0,
                vy_mps=0.0,
                yaw_rad=0.0,
                omega_radps=0.0,
                health=REQUIRED_MINI_HEALTH,
                origin_id=config.plan_id,
            ),
            1000,
        )
        command = coordinator.step(1000).remote_command
        released = endpoint.ingest(
            Frame(MessageType.PLAN_COMMAND, command.encode()),
            5000,
        )

        self.assertTrue(released.release_start_gate)
        before_watchdog = endpoint.poll(6499)
        self.assertFalse(before_watchdog.stop_worker)
        self.assertEqual(before_watchdog.reason, "command_active")
        at_watchdog = endpoint.poll(6500)
        self.assertTrue(at_watchdog.stop_worker)
        self.assertEqual(at_watchdog.reason, "command_watchdog")

    def test_default_live_contract_releases_mini_then_carrier_once(self) -> None:
        config = config_from_args(build_parser().parse_args(["carrier"]))
        endpoint = MiniMissionEndpointCore(mini_command_guard_policy(config))
        endpoint.set_local_prestate_ready(True)
        coordinator = StagedChaseCoordinator(config)
        coordinator.set_carrier_ready(True)
        coordinator.authorize_start()

        plan = build_staged_mission_plan(
            config,
            seq=0,
            sender_monotonic_ms=1000,
        )
        plan_decision = endpoint.ingest(
            Frame(MessageType.STAGED_MISSION_PLAN, plan.encode()),
            5000,
        )
        self.assertEqual(plan_decision.gate_result.decision, Decision.ACCEPT)

        mini_release_count = 0
        carrier_release_count = 0
        for index in range(51):
            now_ms = 1000 + index * 100
            state = MiniState(
                vehicle_id=1,
                seq=index,
                timestamp_ms=now_ms,
                x_m=index * 0.04,
                y_m=0.0,
                vx_mps=0.4,
                vy_mps=0.0,
                yaw_rad=0.0,
                omega_radps=0.0,
                health=REQUIRED_MINI_HEALTH,
                origin_id=config.plan_id,
            )
            coordinator.accept_mini_state(state, now_ms)
            chase = coordinator.step(now_ms)
            mini = endpoint.ingest(
                Frame(MessageType.PLAN_COMMAND, chase.remote_command.encode()),
                5000 + index * 100,
            )
            mini_release_count += int(mini.release_start_gate)
            carrier_release_count += int(chase.start_local_carrier)
            if index < 50:
                self.assertNotEqual(chase.phase, ChasePhase.BOTH_ACTIVE)

        self.assertEqual(chase.phase, ChasePhase.BOTH_ACTIVE)
        self.assertEqual(mini_release_count, 1)
        self.assertEqual(carrier_release_count, 1)

    def test_worker_result_and_shutdown_tokens_are_exact_and_fragment_safe(self) -> None:
        line = worker_result_line(7, 0)
        buffered, result, error = consume_worker_result_bytes(b"", line[:9], 7)
        self.assertIsNone(result)
        self.assertIsNone(error)
        buffered, result, error = consume_worker_result_bytes(
            buffered, line[9:], 7
        )
        self.assertEqual(buffered, b"")
        self.assertEqual(result, 0)
        self.assertIsNone(error)
        self.assertEqual(runtime_shutdown_line(7), b"PAIRB_RUNTIME_SHUTDOWN plan_id=7\n")
        _, result, error = consume_worker_result_bytes(
            b"", worker_result_line(8, 0), 7
        )
        self.assertIsNone(result)
        self.assertEqual(error, "worker_result_token_mismatch")

    def test_all_field_confirmations_are_explicit(self) -> None:
        self.assertGreater(len(missing_field_confirmations({})), 5)
        complete = {
            name: "true" for name in missing_field_confirmations({})
        }
        self.assertEqual(missing_field_confirmations(complete), [])

    def test_worker_status_is_sticky_and_contains_no_motion_command(self) -> None:
        config = config_from_args(build_parser().parse_args(["mini"]))
        worker = GatedMissionWorker("mini", Path("/not/started"), config)
        waiting = worker_status(worker, role=Role.MINI, plan_id=1, seq=0)
        self.assertEqual(waiting.state, MissionExecutionState.WAITING)
        worker.phase = WorkerPhase.COMPLETE
        complete = worker_status(worker, role=Role.MINI, plan_id=1, seq=1)
        self.assertEqual(complete.state, MissionExecutionState.COMPLETE)
        self.assertFalse(hasattr(complete, "v_mps"))

    def test_status_session_rejects_old_terminal_until_current_waiting(self) -> None:
        gate = MissionStatusSessionGate(plan_id=8094, role=Role.MINI)

        old_terminal = MissionStatus(
            plan_id=8094,
            role=Role.MINI,
            state=MissionExecutionState.STOPPED,
            seq=99,
            timestamp_ms=1000,
            exit_code=0,
        )
        accepted, reason = gate.accept(old_terminal)
        self.assertFalse(accepted)
        self.assertEqual(reason, "terminal_or_running_before_current_waiting")

        waiting = dataclasses.replace(
            old_terminal,
            state=MissionExecutionState.WAITING,
            seq=0,
            timestamp_ms=2000,
        )
        self.assertTrue(gate.accept(waiting)[0])
        delayed_old_terminal = dataclasses.replace(old_terminal, seq=100)
        accepted, reason = gate.accept(delayed_old_terminal)
        self.assertFalse(accepted)
        self.assertEqual(reason, "stale_status_timestamp")

        running = dataclasses.replace(
            waiting,
            state=MissionExecutionState.RUNNING,
            seq=1,
            timestamp_ms=2100,
        )
        stopped = dataclasses.replace(
            running,
            state=MissionExecutionState.STOPPED,
            seq=2,
            timestamp_ms=2200,
        )
        self.assertTrue(gate.accept(running)[0])
        self.assertTrue(gate.accept(stopped)[0])

    def test_transport_drain_discards_pre_session_chunks(self) -> None:
        class FakeTransport:
            def __init__(self) -> None:
                self.calls = 0

            def receive(self, _timeout: float) -> list[bytes]:
                self.calls += 1
                return [b"old"] if self.calls <= 2 else []

        transport = FakeTransport()
        with mock.patch(
            "run_pairb_staged_chase_live.time.monotonic",
            side_effect=[0.0, 0.0, 0.0, 0.01, 0.01, 0.02],
        ):
            discarded = drain_pairb_transport(transport, duration_sec=0.02)

        self.assertEqual(discarded, 2)

    def test_abort_retransmits_until_ack_or_bounded_timeout(self) -> None:
        self.assertEqual(
            abort_exit_ready(
                now=10.4,
                abort_started=10.0,
                remote_terminal_ack=True,
            ),
            (False, "retransmitting_abort"),
        )
        self.assertEqual(
            abort_exit_ready(
                now=10.5,
                abort_started=10.0,
                remote_terminal_ack=True,
            ),
            (True, "remote_terminal_ack"),
        )
        self.assertEqual(
            abort_exit_ready(
                now=13.0,
                abort_started=10.0,
                remote_terminal_ack=False,
            ),
            (True, "abort_ack_timeout"),
        )

    def test_wrapper_keeps_timeout_child_in_supervisor_process_group(self) -> None:
        wrapper = (REPO_DIR / "scripts" / "run_orin2_outdoor_forward_5m.sh").read_text()
        self.assertIn(
            'setsid timeout --foreground --signal=INT',
            wrapper,
        )
        self.assertIn('kill -INT "$pid"', wrapper)

    def test_wrapper_cleans_mavros_by_process_group_not_only_leader(self) -> None:
        wrapper = (REPO_DIR / "scripts" / "run_orin2_outdoor_forward_5m.sh").read_text()
        self.assertIn('kill -0 -- "-$pgid"', wrapper)
        self.assertIn('if process_group_alive "$pid"; then', wrapper)
        self.assertIn('kill -INT -- "-$pid"', wrapper)
        self.assertIn('kill -TERM -- "-$pid"', wrapper)

    def test_runtime_shutdown_is_idempotent_after_worker_pipe_closes(self) -> None:
        config = config_from_args(build_parser().parse_args(["mini"]))
        worker = GatedMissionWorker("mini", Path("/not/started"), config)
        read_fd, write_fd = os.pipe()
        os.close(read_fd)
        worker.shutdown_write_fd = write_fd

        worker.shutdown_runtime()
        worker.shutdown_runtime()

        self.assertIsNone(worker.shutdown_write_fd)

    def test_terminal_stop_uses_exact_completion_token_without_signal(self) -> None:
        config = config_from_args(build_parser().parse_args(["carrier"]))
        worker = GatedMissionWorker("carrier", Path("/not/started"), config)
        read_fd, write_fd = os.pipe()
        worker.runtime_stop_write_fd = write_fd
        worker.phase = WorkerPhase.RUNNING

        worker.request_terminal_stop()

        self.assertEqual(os.read(read_fd, 256), external_runtime_stop_token(config.plan_id))
        self.assertIsNone(worker.runtime_stop_write_fd)
        os.close(read_fd)

    def test_stopping_completed_worker_releases_runtime_hold(self) -> None:
        config = config_from_args(build_parser().parse_args(["carrier"]))
        worker = GatedMissionWorker("carrier", Path("/not/started"), config)
        read_fd, write_fd = os.pipe()
        worker.shutdown_write_fd = write_fd
        worker.phase = WorkerPhase.COMPLETE

        worker.stop()

        self.assertEqual(os.read(read_fd, 256), runtime_shutdown_line(config.plan_id))
        self.assertEqual(worker.phase, WorkerPhase.STOPPED)
        os.close(read_fd)

    def test_waiting_worker_stop_closes_gate_without_interrupting_wrapper(self) -> None:
        config = config_from_args(build_parser().parse_args(["mini"]))
        worker = GatedMissionWorker("mini", Path("/not/started"), config)
        read_fd, write_fd = os.pipe()
        worker.gate_write_fd = write_fd
        worker.process = mock.Mock(pid=1234)
        worker.process.poll.return_value = None

        with mock.patch("os.killpg") as killpg:
            worker.stop()

        killpg.assert_not_called()
        self.assertEqual(worker.phase, WorkerPhase.STOPPED)
        self.assertIsNone(worker.gate_write_fd)
        os.close(read_fd)

    def test_running_worker_stop_interrupts_wrapper_group(self) -> None:
        config = config_from_args(build_parser().parse_args(["mini"]))
        worker = GatedMissionWorker("mini", Path("/not/started"), config)
        worker.phase = WorkerPhase.RUNNING
        worker.process = mock.Mock(pid=1234)
        worker.process.poll.return_value = None

        with mock.patch("os.killpg") as killpg:
            worker.stop()

        killpg.assert_called_once_with(1234, signal.SIGINT)


if __name__ == "__main__":
    unittest.main()
