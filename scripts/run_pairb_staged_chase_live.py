#!/usr/bin/env python3

"""Pair B-only staged two-rover S-route supervisor.

Dry-run is the default and performs no ROS, serial, MAVROS, PX4, or process I/O.
The live path starts each rover's existing guarded mission wrapper behind a
one-shot local pipe.  Only a validated Pair B TRAJECTORY phase releases it.
"""

from __future__ import annotations

import argparse
import dataclasses
import math
import os
import signal
import struct
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

REPO_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_DIR / "src"))

from lr24_command_guard import CommandGuardPolicy, Decision  # noqa: E402
from lr24_compact_protocol import (  # noqa: E402
    Abort,
    AbortReason,
    FieldOrigin,
    Frame,
    FrameReader,
    HealthFlag,
    MessageType,
    MiniState,
    MissionExecutionState,
    MissionStatus,
    Phase,
    Role,
    encode_frame,
    sequence_is_newer,
)
from lr24_mavlink_tunnel import TUNNEL_COMPONENT_ID, make_transport  # noqa: E402
from mavros_mini_state_source import MavrosMiniStateSource  # noqa: E402
from orin2_outdoor_forward_5m import (  # noqa: E402
    S_BEND_RETURN_EXECUTE_PHRASE,
    external_runtime_stop_token,
    external_start_gate_token,
)
from pairb_live_mission import MiniMissionEndpointCore, WorkerPhase  # noqa: E402
from pairb_dual_trajectory import (  # noqa: E402
    FormationRegistration,
    PairBDualTrajectoryRecorder,
    state_has_pose,
    states_share_expected_origin,
)
from pairb_staged_chase import (  # noqa: E402
    ChasePhase,
    StagedChaseConfig,
    StagedChaseCoordinator,
    build_staged_mission_plan,
    terminal_gap_is_reached,
    terminal_gap_is_unsafe,
    terminal_gap_metrics,
    validate_staged_chase_config,
)


LIVE_CONFIRMATION = "PAIRB_STAGED_S_BEND_BOTH_ROVERS_AREA_CLEAR_RC_KILL_READY"
DEFAULT_ROS_DOMAIN_ID = "99"
DEFAULT_READY_WAIT_TIMEOUT_SEC = 540.0
DEFAULT_COMMAND_RATE_HZ = 10.0
DEFAULT_STATE_RATE_HZ = 50.0
MAX_STATE_RATE_HZ = 50.0
PAIRB_POLL_TIMEOUT_SEC = 0.002
SUPERVISOR_IDLE_SLEEP_SEC = 0.002
LIVE_COMMAND_TTL_MS = 2000
LIVE_COMMAND_WATCHDOG_MS = 1500
WORKER_GATE_TIMEOUT_SEC = 600.0
WORKER_RUNTIME_HOLD_MARGIN_SEC = 10
DEFAULT_PAIRB_PORT = (
    "/dev/serial/by-id/"
    "usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"
)
REQUIRED_FIELD_CONFIRMATIONS = (
    "CONFIRM_GROUND_AREA_CLEAR",
    "CONFIRM_LOW_SPEED_GROUND_TEST",
    "CONFIRM_VEHICLE_DISARMED",
    "CONFIRM_RC_KILL_READY",
    "CONFIRM_QGC_DISARM_READY",
    "CONFIRM_PHYSICAL_POWER_CUTOFF_READY",
    "CONFIRM_REAL_GPS_3D_FIX",
    "CONFIRM_REAL_LOCAL_POSITION",
    "CONFIRM_CURRENT_DIFF_MAPPING",
    "CONFIRM_WHEELS_INSTALLED",
    "CONFIRM_CABLES_SECURED",
    "CONFIRM_FRESH_USER_START",
)
WORKER_PIPE_MAX_BYTES = 256
ABORT_RETRANSMIT_SEC = 3.0
ABORT_ACK_MIN_SEC = 0.5
ROS_CALLBACK_BATCH_SIZE = 32
TERMINAL_WORKER_PHASES = frozenset(
    {WorkerPhase.COMPLETE, WorkerPhase.FAILED, WorkerPhase.STOPPED}
)


def monotonic_ms() -> int:
    return int(time.monotonic() * 1000.0) & 0xFFFFFFFF


def spin_pending_callbacks(source: object, max_callbacks: int = ROS_CALLBACK_BATCH_SIZE) -> None:
    """Bound callback work while preventing high-rate pose/IMU starvation."""
    if not isinstance(max_callbacks, int) or max_callbacks < 1:
        raise ValueError("max_callbacks must be a positive integer")
    for _ in range(max_callbacks):
        source.spin_once(0.0)


class MissionStatusSessionGate:
    """Reject terminal status left in Pair B from an earlier process run."""

    def __init__(self, *, plan_id: int, role: Role) -> None:
        self.plan_id = plan_id
        self.role = role
        self.waiting_seen = False
        self.running_seen = False
        self.last_seq: int | None = None
        self.last_timestamp_ms: int | None = None

    def accept(self, status: MissionStatus) -> tuple[bool, str]:
        if status.plan_id != self.plan_id or status.role != self.role:
            return False, "wrong_session_identity"
        if not self.waiting_seen:
            if status.state != MissionExecutionState.WAITING:
                return False, "terminal_or_running_before_current_waiting"
            self._record(status)
            self.waiting_seen = True
            return True, "current_waiting_baseline"

        assert self.last_seq is not None
        assert self.last_timestamp_ms is not None
        timestamp_newer = sequence_is_newer(
            status.timestamp_ms,
            self.last_timestamp_ms,
        )
        if status.state == MissionExecutionState.WAITING and not self.running_seen:
            # A newly launched peer resets status seq to zero. Its monotonic
            # timestamp remains newer than any buffered status from that host.
            if not timestamp_newer:
                return False, "stale_waiting_timestamp"
            self._record(status)
            return True, "waiting_baseline_refreshed"
        if not sequence_is_newer(status.seq, self.last_seq):
            return False, "stale_status_sequence"
        if not timestamp_newer:
            return False, "stale_status_timestamp"
        self._record(status)
        if status.state == MissionExecutionState.RUNNING:
            self.running_seen = True
        return True, "current_status"

    def _record(self, status: MissionStatus) -> None:
        self.last_seq = status.seq
        self.last_timestamp_ms = status.timestamp_ms


def abort_exit_ready(
    *,
    now: float,
    abort_started: float,
    remote_terminal_ack: bool,
) -> tuple[bool, str]:
    elapsed = max(0.0, now - abort_started)
    if remote_terminal_ack and elapsed >= ABORT_ACK_MIN_SEC:
        return True, "remote_terminal_ack"
    if elapsed >= ABORT_RETRANSMIT_SEC:
        return True, "abort_ack_timeout"
    return False, "retransmitting_abort"


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("role", choices=("carrier", "mini"))
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--confirm", default="")
    parser.add_argument("--plan-id", type=int, default=1)
    parser.add_argument("--straight-distance-m", type=float, default=5.0)
    parser.add_argument("--turn-radius-m", type=float, default=3.0)
    parser.add_argument("--lateral-offset-m", type=float, default=6.0)
    parser.add_argument("--mini-speed-mps", type=float, default=0.12)
    parser.add_argument("--carrier-speed-mps", type=float, default=0.06)
    parser.add_argument("--lead-delay-sec", type=float, default=5.0)
    parser.add_argument("--lead-distance-m", type=float, default=2.0)
    parser.add_argument("--terminal-gap-m", type=float, default=2.0)
    parser.add_argument(
        "--initial-mini-ahead-m",
        type=float,
        default=0.5,
        help="measured initial Mini position ahead of Carrier for formation registration",
    )
    parser.add_argument(
        "--command-rate-hz",
        type=float,
        default=DEFAULT_COMMAND_RATE_HZ,
    )
    parser.add_argument(
        "--state-rate-hz",
        type=float,
        default=DEFAULT_STATE_RATE_HZ,
    )
    parser.add_argument("--status-rate-hz", type=float, default=5.0)
    parser.add_argument("--supervisor-timeout-sec", type=float, default=180.0)
    parser.add_argument(
        "--ready-wait-timeout-sec",
        type=float,
        default=DEFAULT_READY_WAIT_TIMEOUT_SEC,
        help="bounded MANUAL/disarmed wait before Pair B starts the mission",
    )
    parser.add_argument("--mavros-startup-timeout-sec", type=float, default=60.0)
    parser.add_argument("--pairb-port", default=DEFAULT_PAIRB_PORT)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--mavros-namespace", default="/mavros")
    parser.add_argument("--mavros-topic-prefix", default="/pairb_tunnel")
    parser.add_argument("--mavros-router-service", default="auto")
    parser.add_argument("--worker-wrapper")
    parser.add_argument("--dual-trajectory-artifact-dir", type=Path)
    return parser


def config_from_args(args: argparse.Namespace) -> StagedChaseConfig:
    if not 1.0 <= args.command_rate_hz <= 20.0:
        raise ValueError("command rate must be within [1, 20] Hz")
    if not 1.0 <= args.state_rate_hz <= MAX_STATE_RATE_HZ:
        raise ValueError(f"state rate must be within [1, {MAX_STATE_RATE_HZ:g}] Hz")
    if not 1.0 <= args.status_rate_hz <= 20.0:
        raise ValueError("status rate must be within [1, 20] Hz")
    if not 30.0 <= args.supervisor_timeout_sec <= 180.0:
        raise ValueError("supervisor timeout must be within [30, 180] seconds")
    if not 30.0 <= args.ready_wait_timeout_sec <= DEFAULT_READY_WAIT_TIMEOUT_SEC:
        raise ValueError("ready wait timeout must be within [30, 540] seconds")
    if not math.isfinite(args.initial_mini_ahead_m) or not (
        0.1 <= args.initial_mini_ahead_m <= 5.0
    ):
        raise ValueError("initial Mini lead must be within [0.1, 5.0] m")
    config = StagedChaseConfig(
        plan_id=args.plan_id,
        straight_distance_m=args.straight_distance_m,
        turn_radius_m=args.turn_radius_m,
        lateral_offset_m=args.lateral_offset_m,
        mini_speed_mps=args.mini_speed_mps,
        carrier_speed_mps=args.carrier_speed_mps,
        lead_delay_ms=round(args.lead_delay_sec * 1000.0),
        lead_distance_m=args.lead_distance_m,
        terminal_gap_m=args.terminal_gap_m,
        # The 2026-08-09 live run stopped after a sub-second command-stream
        # interruption. Keep transport validity wider than the local watchdog.
        command_ttl_ms=LIVE_COMMAND_TTL_MS,
        command_duration_ms=500,
        state_timeout_ms=750,
        mission_timeout_ms=round(args.supervisor_timeout_sec * 1000.0),
        plan_validity_ms=round(args.supervisor_timeout_sec * 1000.0),
    )
    validate_staged_chase_config(config)
    return config


def role_wrapper(role: str, override: str | None) -> Path:
    if override:
        return Path(override).expanduser().resolve()
    name = (
        "run_orin1_outdoor_forward_5m.sh"
        if role == "mini"
        else "run_orin2_outdoor_forward_5m.sh"
    )
    return REPO_DIR / "scripts" / name


def build_worker_environment(
    role: str,
    config: StagedChaseConfig,
    gate_fd: int,
    status_fd: int | None = None,
    shutdown_fd: int | None = None,
    runtime_stop_fd: int | None = None,
) -> dict[str, str]:
    if (status_fd is None) != (shutdown_fd is None):
        raise ValueError("worker status and shutdown FDs must be provided together")
    env = dict(os.environ)
    env.update(
        {
            "ROS_LOCALHOST_ONLY": "1",
            "ROS_DOMAIN_ID": env.get("ROS_DOMAIN_ID", DEFAULT_ROS_DOMAIN_ID),
            "MISSION_PROFILE": "s_bend_return",
            "FORWARD_DISTANCE_M": f"{config.straight_distance_m:.3f}",
            "TURN_RADIUS_M": f"{config.turn_radius_m:.3f}",
            "PAIRB_START_GATE_FD": str(gate_fd),
            "PAIRB_START_PLAN_ID": str(config.plan_id),
            "PAIRB_START_GATE_TIMEOUT_SEC": f"{WORKER_GATE_TIMEOUT_SEC:.0f}",
            "MAX_MOTION_SEC": "180",
        }
    )
    if status_fd is not None and shutdown_fd is not None:
        env["PAIRB_WORKER_STATUS_FD"] = str(status_fd)
        env["PAIRB_RUNTIME_SHUTDOWN_FD"] = str(shutdown_fd)
        env["PAIRB_RUNTIME_HOLD_TIMEOUT_SEC"] = str(
            worker_runtime_hold_timeout_sec(config)
        )
    if runtime_stop_fd is not None:
        env["PAIRB_RUNTIME_STOP_FD"] = str(runtime_stop_fd)
    if role == "mini":
        env["LINEAR_SPEED_MPS"] = f"{config.mini_speed_mps:.3f}"
        env["TURN_FORWARD_SPEED_MPS"] = f"{config.mini_speed_mps:.3f}"
    else:
        env["LINEAR_SPEED_MPS"] = f"{config.carrier_speed_mps:.3f}"
        env["TURN_FORWARD_SPEED_MPS"] = f"{config.carrier_speed_mps:.3f}"
    return env


def worker_runtime_hold_timeout_sec(config: StagedChaseConfig) -> int:
    """Keep a completed rover alive until the slower peer finishes or times out."""

    mission_timeout_sec = (config.mission_timeout_ms + 999) // 1000
    return max(30, mission_timeout_sec + WORKER_RUNTIME_HOLD_MARGIN_SEC)


def missing_field_confirmations(environment: dict[str, str]) -> list[str]:
    return [name for name in REQUIRED_FIELD_CONFIRMATIONS if environment.get(name) != "true"]


def configure_host_local_ros_environment(environment: dict[str, str]) -> None:
    environment.setdefault("ROS_DOMAIN_ID", DEFAULT_ROS_DOMAIN_ID)
    environment["ROS_LOCALHOST_ONLY"] = "1"


def mini_command_guard_policy(config: StagedChaseConfig) -> CommandGuardPolicy:
    return CommandGuardPolicy(
        target_role=Role.MINI,
        max_linear_speed_mps=0.25,
        max_yaw_rate_radps=0.6,
        max_accel_mps2=0.5,
        max_plan_ttl_ms=config.plan_validity_ms,
        command_watchdog_ms=LIVE_COMMAND_WATCHDOG_MS,
    )


def supervisor_timeout_reason(
    now: float,
    *,
    ready_deadline: float,
    mission_deadline: float | None,
) -> str | None:
    """Keep operator/setup waiting separate from bounded mission execution."""

    if mission_deadline is None:
        return "ready_wait_timeout" if now >= ready_deadline else None
    return "mission_timeout" if now >= mission_deadline else None


def worker_result_line(plan_id: int, return_code: int) -> bytes:
    external_start_gate_token(plan_id)
    if not -32768 <= return_code <= 32767:
        raise ValueError("worker return code is outside int16")
    return f"PAIRB_WORKER_RESULT plan_id={plan_id} rc={return_code}\n".encode("ascii")


def runtime_shutdown_line(plan_id: int) -> bytes:
    external_start_gate_token(plan_id)
    return f"PAIRB_RUNTIME_SHUTDOWN plan_id={plan_id}\n".encode("ascii")


def consume_worker_result_bytes(
    buffered: bytes,
    incoming: bytes,
    plan_id: int,
) -> tuple[bytes, int | None, str | None]:
    combined = buffered + incoming
    if len(combined) > WORKER_PIPE_MAX_BYTES:
        return b"", None, "worker_result_buffer_overflow"
    if b"\n" not in combined:
        return combined, None, None
    line, trailing = combined.split(b"\n", 1)
    if trailing:
        return b"", None, "worker_result_trailing_data"
    try:
        text = line.decode("ascii")
        prefix = f"PAIRB_WORKER_RESULT plan_id={plan_id} rc="
        if not text.startswith(prefix):
            return b"", None, "worker_result_token_mismatch"
        return_code = int(text[len(prefix) :])
        if worker_result_line(plan_id, return_code) != line + b"\n":
            return b"", None, "worker_result_noncanonical"
    except (UnicodeDecodeError, ValueError):
        return b"", None, "worker_result_malformed"
    return b"", return_code, None


class GatedMissionWorker:
    def __init__(self, role: str, wrapper: Path, config: StagedChaseConfig) -> None:
        self.role = role
        self.wrapper = wrapper
        self.config = config
        self.process: subprocess.Popen[bytes] | None = None
        self.gate_write_fd: int | None = None
        self.status_read_fd: int | None = None
        self.shutdown_write_fd: int | None = None
        self.runtime_stop_write_fd: int | None = None
        self.status_buffer = b""
        self.phase = WorkerPhase.WAITING
        self.return_code: int | None = None
        self.stop_requested = False

    def start(self) -> None:
        gate_read_fd, gate_write_fd = os.pipe()
        status_read_fd, status_write_fd = os.pipe()
        shutdown_read_fd, shutdown_write_fd = os.pipe()
        runtime_stop_read_fd, runtime_stop_write_fd = os.pipe()
        env = build_worker_environment(
            self.role,
            self.config,
            gate_read_fd,
            status_write_fd,
            shutdown_read_fd,
            runtime_stop_read_fd,
        )
        command = [
            str(self.wrapper),
            "--execute",
            "--confirm",
            S_BEND_RETURN_EXECUTE_PHRASE,
        ]
        try:
            self.process = subprocess.Popen(
                command,
                cwd=REPO_DIR,
                env=env,
                pass_fds=(
                    gate_read_fd,
                    status_write_fd,
                    shutdown_read_fd,
                    runtime_stop_read_fd,
                ),
                start_new_session=True,
            )
        except Exception:
            for fd in (
                gate_read_fd,
                gate_write_fd,
                status_read_fd,
                status_write_fd,
                shutdown_read_fd,
                shutdown_write_fd,
                runtime_stop_read_fd,
                runtime_stop_write_fd,
            ):
                os.close(fd)
            raise
        else:
            os.close(gate_read_fd)
            os.close(status_write_fd)
            os.close(shutdown_read_fd)
            os.close(runtime_stop_read_fd)
        self.gate_write_fd = gate_write_fd
        self.status_read_fd = status_read_fd
        self.shutdown_write_fd = shutdown_write_fd
        self.runtime_stop_write_fd = runtime_stop_write_fd
        os.set_blocking(status_read_fd, False)

    def release(self) -> None:
        if self.phase != WorkerPhase.WAITING or self.gate_write_fd is None:
            raise RuntimeError("worker start gate is not waiting")
        os.write(self.gate_write_fd, external_start_gate_token(self.config.plan_id))
        os.close(self.gate_write_fd)
        self.gate_write_fd = None
        self.phase = WorkerPhase.RUNNING

    def poll(self) -> int | None:
        if self.process is None:
            return None
        if self.status_read_fd is not None and self.return_code is None:
            try:
                incoming = os.read(self.status_read_fd, WORKER_PIPE_MAX_BYTES)
            except BlockingIOError:
                incoming = b""
            if incoming:
                self.status_buffer, reported, error = consume_worker_result_bytes(
                    self.status_buffer,
                    incoming,
                    self.config.plan_id,
                )
                if error is not None:
                    self.return_code = 10
                    self.phase = WorkerPhase.FAILED
                elif reported is not None:
                    self.return_code = reported
                    self.phase = (
                        WorkerPhase.COMPLETE
                        if reported == 0
                        else WorkerPhase.FAILED
                    )
        return_code = self.process.poll()
        if return_code is None:
            return self.return_code
        if self.return_code is None:
            self.return_code = return_code
            if self.stop_requested:
                self.phase = WorkerPhase.STOPPED
            else:
                self.phase = WorkerPhase.COMPLETE if return_code == 0 else WorkerPhase.FAILED
        return return_code

    def shutdown_runtime(self) -> None:
        if self.shutdown_write_fd is None:
            return
        shutdown_fd = self.shutdown_write_fd
        self.shutdown_write_fd = None
        try:
            os.write(
                shutdown_fd,
                runtime_shutdown_line(self.config.plan_id),
            )
        except OSError:
            # The worker may already have exited and closed its read end.
            pass
        finally:
            os.close(shutdown_fd)

    def request_terminal_stop(self) -> None:
        if self.phase != WorkerPhase.RUNNING or self.runtime_stop_write_fd is None:
            raise RuntimeError("worker runtime stop gate is not active")
        stop_fd = self.runtime_stop_write_fd
        self.runtime_stop_write_fd = None
        try:
            os.write(stop_fd, external_runtime_stop_token(self.config.plan_id))
        finally:
            os.close(stop_fd)

    def stop(self) -> None:
        was_running = self.phase == WorkerPhase.RUNNING
        was_complete = self.phase == WorkerPhase.COMPLETE
        if self.gate_write_fd is not None:
            os.close(self.gate_write_fd)
            self.gate_write_fd = None
        if was_complete:
            self.shutdown_runtime()
        elif self.shutdown_write_fd is not None:
            os.close(self.shutdown_write_fd)
            self.shutdown_write_fd = None
        if self.runtime_stop_write_fd is not None:
            os.close(self.runtime_stop_write_fd)
            self.runtime_stop_write_fd = None
        self.stop_requested = True
        if was_running and self.process is not None and self.process.poll() is None:
            os.killpg(self.process.pid, signal.SIGINT)
        self.phase = WorkerPhase.STOPPED

    def close(self) -> None:
        if self.process is None:
            return
        if self.process.poll() is None:
            if self.phase == WorkerPhase.COMPLETE:
                self.shutdown_runtime()
            else:
                self.stop()
        try:
            self.process.wait(timeout=30.0)
        except subprocess.TimeoutExpired:
            os.killpg(self.process.pid, signal.SIGTERM)
            self.process.wait(timeout=10.0)
        if self.status_read_fd is not None:
            os.close(self.status_read_fd)
            self.status_read_fd = None
        if self.runtime_stop_write_fd is not None:
            os.close(self.runtime_stop_write_fd)
            self.runtime_stop_write_fd = None


def worker_status(
    worker: GatedMissionWorker,
    *,
    role: Role,
    plan_id: int,
    seq: int,
) -> MissionStatus:
    state = {
        WorkerPhase.WAITING: MissionExecutionState.WAITING,
        WorkerPhase.RUNNING: MissionExecutionState.RUNNING,
        WorkerPhase.COMPLETE: MissionExecutionState.COMPLETE,
        WorkerPhase.FAILED: MissionExecutionState.FAILED,
        WorkerPhase.STOPPED: MissionExecutionState.STOPPED,
    }[worker.phase]
    return MissionStatus(
        plan_id=plan_id,
        role=role,
        state=state,
        seq=seq,
        timestamp_ms=monotonic_ms(),
        exit_code=worker.return_code or 0,
    )


def terminal_worker_status_due(phase: WorkerPhase, already_sent: bool) -> bool:
    """Prioritize one terminal status ahead of the first post-run MiniState."""
    return not already_sent and phase in TERMINAL_WORKER_PHASES


def send_frame(transport: object, msg_type: MessageType, payload: bytes) -> None:
    transport.send(encode_frame(msg_type, payload))


def send_abort_best_effort(
    transport: object | None,
    *,
    role: Role,
    plan_id: int,
    reason: AbortReason,
) -> None:
    if transport is None:
        return
    abort = Abort(
        source_role=role,
        reason=reason,
        plan_id=plan_id,
        seq=0,
        timestamp_ms=monotonic_ms(),
    )
    try:
        send_frame(transport, MessageType.ABORT, abort.encode())
    except Exception:
        pass


def read_frames(transport: object, reader: FrameReader) -> list[Frame]:
    frames: list[Frame] = []
    for data in transport.receive(PAIRB_POLL_TIMEOUT_SEC):
        frames.extend(reader.feed(data))
    return frames


def advance_periodic_deadline(deadline: float, now: float, period: float) -> float:
    """Advance without accumulating loop-time drift or sending catch-up bursts."""

    candidate = deadline + period
    return candidate if candidate > now else now + period


def drain_pairb_transport(transport: object, duration_sec: float = 0.5) -> int:
    """Discard frames queued before this supervisor established its session."""
    deadline = time.monotonic() + duration_sec
    discarded_chunks = 0
    while time.monotonic() < deadline:
        timeout = min(0.02, max(0.0, deadline - time.monotonic()))
        for _data in transport.receive(timeout):
            discarded_chunks += 1
    return discarded_chunks


def wait_for_mavros(
    source: MavrosMiniStateSource,
    worker: GatedMissionWorker,
    timeout_sec: float,
) -> None:
    deadline = time.monotonic() + timeout_sec
    next_log = 0.0
    while time.monotonic() < deadline:
        source.spin_once(0.1)
        if worker.poll() is not None:
            raise RuntimeError(f"mission worker exited before MAVROS: {worker.return_code}")
        if source.accumulator.state.connected:
            return
        now = time.monotonic()
        if now >= next_log:
            print(
                "WAIT_MAVROS "
                f"domain={os.environ.get('ROS_DOMAIN_ID', 'unset')} "
                f"localhost={os.environ.get('ROS_LOCALHOST_ONLY', 'unset')} "
                f"{source.status_text()}",
                flush=True,
            )
            next_log = now + 2.0
    raise RuntimeError("MAVROS did not become connected before startup timeout")


def wait_for_carrier_field_origin(
    source: MavrosMiniStateSource,
    worker: GatedMissionWorker,
    origin_id: int,
    timeout_sec: float = 15.0,
) -> FieldOrigin:
    deadline = time.monotonic() + timeout_sec
    next_log = 0.0
    while time.monotonic() < deadline:
        source.spin_once(0.1)
        if worker.poll() is not None:
            raise RuntimeError("mission worker exited before Carrier FieldOrigin")
        origin = source.field_origin_candidate(
            origin_id=origin_id,
            seq=0,
            timestamp_ms=monotonic_ms(),
        )
        if origin is not None:
            source.set_field_origin(origin)
            print(
                f"FIELD_ORIGIN_LOCKED id={origin.origin_id} "
                f"lat={origin.latitude_deg:.7f} lon={origin.longitude_deg:.7f} "
                "source=Carrier_GPS",
                flush=True,
            )
            return origin
        now = time.monotonic()
        if now >= next_log:
            print(f"WAIT_FIELD_ORIGIN {source.status_text()}", flush=True)
            next_log = now + 1.0
    raise RuntimeError("Carrier GPS did not provide a fresh FieldOrigin")


def open_pairb_transport(args: argparse.Namespace) -> object:
    if args.role == "mini":
        return make_transport(
            "mavros-router",
            port=None,
            baud=args.baud,
            source_system=1,
            target_system=2,
            source_component=TUNNEL_COMPONENT_ID,
            target_component=TUNNEL_COMPONENT_ID,
            expected_source_system=2,
            topic_prefix=args.mavros_topic_prefix,
            router_add_service=args.mavros_router_service,
        )
    return make_transport(
        "mavlink-serial",
        port=args.pairb_port,
        baud=args.baud,
        source_system=2,
        target_system=1,
        source_component=TUNNEL_COMPONENT_ID,
        target_component=TUNNEL_COMPONENT_ID,
        expected_source_system=1,
    )


def mini_loop(
    args: argparse.Namespace,
    config: StagedChaseConfig,
    worker: GatedMissionWorker,
    source: MavrosMiniStateSource,
    transport: object,
) -> int:
    reader = FrameReader()
    endpoint = MiniMissionEndpointCore(mini_command_guard_policy(config))
    carrier_status_session = MissionStatusSessionGate(
        plan_id=config.plan_id,
        role=Role.CARRIER,
    )
    state_seq = status_seq = 0
    loop_started = time.monotonic()
    next_state = next_status = next_poll = loop_started
    ready_deadline = time.monotonic() + args.ready_wait_timeout_sec
    mission_deadline: float | None = None
    carrier_complete = False
    terminal_since: float | None = None
    worker_result_logged = False
    terminal_status_sent = False
    last_log = 0.0
    last_gate_event: tuple[str, str] | None = None
    while True:
        spin_pending_callbacks(source)
        now = time.monotonic()
        timeout_reason = supervisor_timeout_reason(
            now,
            ready_deadline=ready_deadline,
            mission_deadline=mission_deadline,
        )
        if timeout_reason is not None:
            print(f"MINI_SUPERVISOR_TIMEOUT reason={timeout_reason}", flush=True)
            worker.stop()
            return 8
        return_code = worker.poll()
        if return_code is not None and endpoint.worker_phase in {
            WorkerPhase.WAITING,
            WorkerPhase.RUNNING,
        }:
            if not worker_result_logged:
                print(
                    f"MINI_WORKER_RESULT rc={return_code} phase={worker.phase.value}",
                    flush=True,
                )
                worker_result_logged = True
            endpoint.set_worker_result(return_code)
        endpoint.set_local_prestate_ready(
            worker.phase == WorkerPhase.WAITING
            and worker.poll() is None
            and source.safe_execution_prestate()
            and source.shared_field_ready(config.plan_id)
        )
        if worker.phase == WorkerPhase.RUNNING and not (
            source.execution_session_ready()
            and source.shared_field_ready(config.plan_id)
        ):
            print(
                f"MINI_EXECUTION_SESSION_LOST {source.status_text()}",
                flush=True,
            )
            worker.stop()
            endpoint.mark_worker_stopped()
            terminal_since = terminal_since or now

        if terminal_worker_status_due(worker.phase, terminal_status_sent):
            status = worker_status(
                worker,
                role=Role.MINI,
                plan_id=config.plan_id,
                seq=status_seq,
            )
            send_frame(transport, MessageType.MISSION_STATUS, status.encode())
            status_seq += 1
            next_status = advance_periodic_deadline(
                next_status, now, 1.0 / args.status_rate_hz
            )
            terminal_status_sent = True
            print(
                f"MINI_TERMINAL_STATUS_SENT state={status.state.name} "
                f"seq={status.seq} before_post_run_state=True",
                flush=True,
            )

        if now >= next_state:
            state = source.build(1, state_seq, monotonic_ms())
            session_ready = bool(
                worker.poll() is None
                and (
                    (
                        worker.phase == WorkerPhase.WAITING
                        and source.safe_execution_prestate()
                        and source.shared_field_ready(config.plan_id)
                    )
                    or (
                        worker.phase == WorkerPhase.RUNNING
                        and source.execution_session_ready()
                        and source.shared_field_ready(config.plan_id)
                    )
                )
            )
            if session_ready:
                state = dataclasses.replace(
                    state,
                    health=state.health
                    | int(HealthFlag.RC_STOP_READY | HealthFlag.EXECUTOR_READY),
                )
            send_frame(transport, MessageType.MINI_STATE, state.encode())
            state_seq += 1
            next_state = advance_periodic_deadline(
                next_state, now, 1.0 / args.state_rate_hz
            )

        if now >= next_status:
            status = worker_status(
                worker,
                role=Role.MINI,
                plan_id=config.plan_id,
                seq=status_seq,
            )
            send_frame(transport, MessageType.MISSION_STATUS, status.encode())
            status_seq += 1
            next_status = advance_periodic_deadline(
                next_status, now, 1.0 / args.status_rate_hz
            )

        for frame in read_frames(transport, reader):
            if frame.msg_type == MessageType.FIELD_ORIGIN:
                try:
                    origin = FieldOrigin.decode(frame.payload)
                    decision = endpoint.ingest(frame, monotonic_ms())
                except (ValueError, struct.error):
                    worker.stop()
                    return 7
                if (
                    decision.gate_result.decision != Decision.ACCEPT
                    or origin.origin_id != config.plan_id
                ):
                    print(
                        f"MINI_FIELD_ORIGIN_REJECT id={origin.origin_id} "
                        f"reason={decision.gate_result.reason}",
                        flush=True,
                    )
                    if worker.phase == WorkerPhase.RUNNING:
                        worker.stop()
                        return 7
                    continue
                source.set_field_origin(origin)
                continue
            if frame.msg_type == MessageType.MISSION_STATUS:
                try:
                    status = MissionStatus.decode(frame.payload)
                except (ValueError, struct.error):
                    worker.stop()
                    return 7
                accepted, reason = carrier_status_session.accept(status)
                if not accepted:
                    if status.state in {
                        MissionExecutionState.COMPLETE,
                        MissionExecutionState.FAILED,
                        MissionExecutionState.STOPPED,
                    }:
                        print(
                            f"MINI_IGNORED_STALE_CARRIER_STATUS "
                            f"state={status.state.name} reason={reason}",
                            flush=True,
                        )
                    continue
                if status.state == MissionExecutionState.COMPLETE:
                    carrier_complete = True
                elif status.state in {
                    MissionExecutionState.FAILED,
                    MissionExecutionState.STOPPED,
                }:
                    worker.stop()
                    return 7
                continue
            if frame.msg_type not in {
                MessageType.STAGED_MISSION_PLAN,
                MessageType.PLAN_COMMAND,
                MessageType.ABORT,
            }:
                continue
            try:
                decision = endpoint.ingest(frame, monotonic_ms())
            except (ValueError, struct.error):
                worker.stop()
                return 7
            gate_event = (decision.gate_result.decision.value, decision.reason)
            if gate_event != last_gate_event and (
                decision.release_start_gate
                or decision.stop_worker
                or decision.gate_result.decision == Decision.REJECT
            ):
                print(
                    f"MINI_GATE decision={gate_event[0]} reason={gate_event[1]} "
                    f"worker={worker.phase.value}",
                    flush=True,
                )
                last_gate_event = gate_event
            if decision.release_start_gate:
                worker.release()
                mission_deadline = now + args.supervisor_timeout_sec
                print("MINI_START_RELEASED source=PairB", flush=True)
            if decision.stop_worker:
                worker.stop()

        if now >= next_poll:
            decision = endpoint.poll(monotonic_ms())
            if decision.stop_worker:
                print(
                    f"MINI_COMMAND_WATCHDOG_STOP "
                    f"decision={decision.gate_result.decision.value} "
                    f"reason={decision.reason} watchdog_ms={LIVE_COMMAND_WATCHDOG_MS}",
                    flush=True,
                )
                worker.stop()
            next_poll = now + 0.05

        if worker.phase in {WorkerPhase.COMPLETE, WorkerPhase.FAILED, WorkerPhase.STOPPED}:
            if terminal_since is None:
                terminal_since = now
            if worker.phase == WorkerPhase.COMPLETE and carrier_complete:
                return 0
            if worker.phase != WorkerPhase.COMPLETE and now - terminal_since >= 2.0:
                return 7
        if now - last_log >= 1.0:
            print(
                f"mini worker={worker.phase.value} prestate={source.safe_execution_prestate()} "
                f"session={source.execution_session_ready()} "
                f"shared_origin={source.shared_field_ready(config.plan_id)} "
                f"carrier_complete={carrier_complete}",
                flush=True,
            )
            last_log = now
        time.sleep(SUPERVISOR_IDLE_SLEEP_SEC)


def carrier_loop(
    args: argparse.Namespace,
    config: StagedChaseConfig,
    worker: GatedMissionWorker,
    source: MavrosMiniStateSource,
    transport: object,
    trajectory_recorder: PairBDualTrajectoryRecorder | None = None,
    field_origin: FieldOrigin | None = None,
) -> int:
    if field_origin is None:
        raise ValueError("Carrier live loop requires a shared FieldOrigin")
    reader = FrameReader()
    coordinator = StagedChaseCoordinator(config)
    mini_status_session = MissionStatusSessionGate(
        plan_id=config.plan_id,
        role=Role.MINI,
    )
    coordinator.authorize_start()
    plan_seq = status_seq = field_origin_seq = 0
    next_plan = next_command = next_status = next_field_origin = 0.0
    ready_deadline = time.monotonic() + args.ready_wait_timeout_sec
    mission_deadline: float | None = None
    last_mini_status_rx: float | None = None
    completion_since: float | None = None
    abort_sent = False
    abort_started: float | None = None
    mini_terminal_ack = False
    latest_mini_state: MiniState | None = None
    formation_registration: FormationRegistration | None = None
    terminal_match_samples = 0
    terminal_last_mini_seq: int | None = None
    terminal_stop_requested = False
    last_terminal_metrics_log = 0.0
    last_log = 0.0
    last_decision_event: tuple[str, str] | None = None
    last_readiness_event: tuple[object, ...] | None = None
    while True:
        spin_pending_callbacks(source)
        now = time.monotonic()
        timeout_reason = supervisor_timeout_reason(
            now,
            ready_deadline=ready_deadline,
            mission_deadline=mission_deadline,
        )
        if timeout_reason is not None:
            print(f"CARRIER_SUPERVISOR_TIMEOUT reason={timeout_reason}", flush=True)
            coordinator.request_abort(AbortReason.LOCAL_SAFETY)
        return_code = worker.poll()
        if return_code is not None:
            if return_code == 0:
                if terminal_stop_requested:
                    coordinator.mark_carrier_complete()
                else:
                    print("CARRIER_COMPLETED_BEFORE_TERMINAL_GAP", flush=True)
                    coordinator.request_abort(AbortReason.LOCAL_SAFETY)
            else:
                coordinator.request_abort(AbortReason.LOCAL_SAFETY)
        worker_alive = return_code is None
        prestate_ready = source.safe_execution_prestate()
        execution_ready = source.execution_session_ready()
        shared_field_ready = source.shared_field_ready(config.plan_id)
        carrier_ready = bool(
            worker_alive
            and (
                (
                    worker.phase == WorkerPhase.WAITING
                    and prestate_ready
                    and shared_field_ready
                )
                or (
                    worker.phase == WorkerPhase.RUNNING
                    and execution_ready
                    and shared_field_ready
                )
            )
        )
        readiness_event = (
            worker.phase,
            worker_alive,
            prestate_ready,
            execution_ready,
            shared_field_ready,
            carrier_ready,
        )
        if readiness_event != last_readiness_event:
            print(
                "CARRIER_READINESS_CHANGE "
                f"worker={worker.phase.value} worker_alive={worker_alive} "
                f"prestate={prestate_ready} session={execution_ready} "
                f"shared_origin={shared_field_ready} ready={carrier_ready} "
                f"{source.status_text()}",
                flush=True,
            )
            last_readiness_event = readiness_event
        if worker.phase == WorkerPhase.RUNNING and not (
            execution_ready and shared_field_ready
        ):
            coordinator.request_abort(AbortReason.LOCAL_SAFETY)
        coordinator.set_carrier_ready(carrier_ready or worker.phase == WorkerPhase.COMPLETE)

        for frame in read_frames(transport, reader):
            if frame.msg_type == MessageType.MINI_STATE:
                try:
                    state = MiniState.decode(frame.payload)
                except (ValueError, struct.error):
                    coordinator.request_abort(AbortReason.STATE_INVALID)
                    continue
                carrier_state = source.build(2, 0, monotonic_ms())
                if formation_registration is None and (
                    states_share_expected_origin(
                        carrier_state,
                        state,
                        config.plan_id,
                    )
                    and state_has_pose(carrier_state)
                    and state_has_pose(state)
                ):
                    formation_registration = FormationRegistration.from_states(
                        carrier_state,
                        state,
                        mini_ahead_m=args.initial_mini_ahead_m,
                    )
                    registered = formation_registration.mini_registered_start
                    print(
                        "FORMATION_REGISTERED "
                        f"mini_ahead={args.initial_mini_ahead_m:.2f}m "
                        f"raw_relative=({state.x_m - carrier_state.x_m:+.2f},"
                        f"{state.y_m - carrier_state.y_m:+.2f})m "
                        f"raw_yaw_delta_deg={math.degrees(state.yaw_rad - carrier_state.yaw_rad):+.1f} "
                        f"registered_mini=({registered.x_m:.2f},{registered.y_m:.2f})",
                        flush=True,
                    )
                if formation_registration is None:
                    continue
                state = formation_registration.transform_state(state)
                if mini_status_session.waiting_seen:
                    if coordinator.accept_mini_state(state, monotonic_ms()):
                        latest_mini_state = state
                if trajectory_recorder is not None:
                    trajectory_recorder.observe(carrier_state, state)
                continue
            if frame.msg_type == MessageType.ABORT:
                try:
                    abort = Abort.decode(frame.payload)
                except (ValueError, struct.error):
                    coordinator.request_abort(AbortReason.STATE_INVALID)
                    continue
                print(
                    f"PAIRB_RX_ABORT source={abort.source_role.name} "
                    f"reason={abort.reason.name}",
                    flush=True,
                )
                coordinator.request_abort(abort.reason)
                continue
            if frame.msg_type == MessageType.MISSION_STATUS:
                try:
                    status = MissionStatus.decode(frame.payload)
                except (ValueError, struct.error):
                    coordinator.request_abort(AbortReason.STATE_INVALID)
                    continue
                accepted, reason = mini_status_session.accept(status)
                if not accepted:
                    if status.state in {
                        MissionExecutionState.COMPLETE,
                        MissionExecutionState.FAILED,
                        MissionExecutionState.STOPPED,
                    }:
                        print(
                            f"PAIRB_IGNORED_STALE_MINI_STATUS "
                            f"state={status.state.name} reason={reason}",
                            flush=True,
                        )
                    continue
                last_mini_status_rx = now
                if status.state == MissionExecutionState.COMPLETE:
                    mini_terminal_ack = True
                    coordinator.mark_mini_complete()
                elif status.state in {
                    MissionExecutionState.FAILED,
                    MissionExecutionState.STOPPED,
                }:
                    mini_terminal_ack = True
                    print(
                        f"PAIRB_MINI_STATUS state={status.state.name} "
                        f"exit_code={status.exit_code}",
                        flush=True,
                    )
                    coordinator.request_abort(AbortReason.LOCAL_SAFETY)

        if (
            coordinator.mini_complete
            and worker.phase == WorkerPhase.RUNNING
            and not terminal_stop_requested
            and latest_mini_state is not None
            and latest_mini_state.seq != terminal_last_mini_seq
            and coordinator.phase != ChasePhase.ABORTED
        ):
            terminal_last_mini_seq = latest_mini_state.seq
            carrier_state = source.build(2, 0, monotonic_ms())
            try:
                metrics = terminal_gap_metrics(
                    carrier_state,
                    latest_mini_state,
                    expected_origin_id=config.plan_id,
                )
            except ValueError as exc:
                print(f"CARRIER_TERMINAL_GAP_INVALID reason={exc}", flush=True)
                coordinator.request_abort(AbortReason.STATE_INVALID)
            else:
                if now - last_terminal_metrics_log >= 0.5:
                    print(
                        "CARRIER_TERMINAL_GAP "
                        f"distance={metrics.distance_m:.2f}m "
                        f"longitudinal={metrics.longitudinal_gap_m:.2f}m "
                        f"lateral={metrics.lateral_offset_m:+.2f}m "
                        f"heading_error_deg={math.degrees(metrics.heading_error_rad):+.1f}",
                        flush=True,
                    )
                    last_terminal_metrics_log = now
                if terminal_gap_is_unsafe(metrics, config):
                    print(
                        "CARRIER_TERMINAL_COLLISION_GUARD "
                        f"distance={metrics.distance_m:.2f}m",
                        flush=True,
                    )
                    coordinator.request_abort(AbortReason.LOCAL_SAFETY)
                elif terminal_gap_is_reached(metrics, config):
                    terminal_match_samples += 1
                    print(
                        "CARRIER_TERMINAL_GAP_MATCH "
                        f"sample={terminal_match_samples}/{config.terminal_confirm_samples} "
                        f"longitudinal={metrics.longitudinal_gap_m:.2f}m "
                        f"lateral={metrics.lateral_offset_m:+.2f}m "
                        f"heading_error_deg={math.degrees(metrics.heading_error_rad):+.1f}",
                        flush=True,
                    )
                    if terminal_match_samples >= config.terminal_confirm_samples:
                        worker.request_terminal_stop()
                        terminal_stop_requested = True
                        print("CARRIER_TERMINAL_STOP_REQUESTED source=shared_field_enu", flush=True)
                else:
                    terminal_match_samples = 0

        if now >= next_field_origin:
            current_origin = dataclasses.replace(
                field_origin,
                seq=field_origin_seq,
                timestamp_ms=monotonic_ms(),
            )
            send_frame(
                transport,
                MessageType.FIELD_ORIGIN,
                current_origin.encode(),
            )
            field_origin_seq += 1
            next_field_origin = now + 1.0

        if now >= next_plan:
            plan = build_staged_mission_plan(
                config,
                seq=plan_seq,
                sender_monotonic_ms=monotonic_ms(),
            )
            send_frame(transport, MessageType.STAGED_MISSION_PLAN, plan.encode())
            plan_seq += 1
            next_plan = now + 1.0

        if now >= next_status:
            status = worker_status(
                worker,
                role=Role.CARRIER,
                plan_id=config.plan_id,
                seq=status_seq,
            )
            send_frame(transport, MessageType.MISSION_STATUS, status.encode())
            status_seq += 1
            next_status = advance_periodic_deadline(
                next_status, now, 1.0 / args.status_rate_hz
            )

        if now >= next_command:
            decision = coordinator.step(monotonic_ms())
            decision_event = (decision.phase.value, decision.reason)
            if decision_event != last_decision_event:
                print(
                    f"PAIRB_DECISION phase={decision.phase.value} "
                    f"reason={decision.reason} lead={decision.lead_distance_m:.2f}m",
                    flush=True,
                )
                last_decision_event = decision_event
            if decision.start_local_carrier:
                worker.release()
                print(
                    f"CARRIER_START_RELEASED source=PairB lead={decision.lead_distance_m:.2f}m",
                    flush=True,
                )
            if decision.phase != ChasePhase.HOLD and mission_deadline is None:
                mission_deadline = now + args.supervisor_timeout_sec
            if decision.stop_local_carrier and worker.phase == WorkerPhase.RUNNING:
                worker.stop()
            send_frame(
                transport,
                MessageType.PLAN_COMMAND,
                decision.remote_command.encode(),
            )
            if decision.abort is not None:
                send_frame(transport, MessageType.ABORT, decision.abort.encode())
                abort_sent = True
            if decision.phase == ChasePhase.COMPLETE:
                if completion_since is None:
                    completion_since = now
                if now - completion_since >= 2.0:
                    return 0
            elif decision.phase == ChasePhase.ABORTED:
                worker.stop()
                if abort_started is None:
                    abort_started = now
                ready_to_exit, exit_reason = abort_exit_ready(
                    now=now,
                    abort_started=abort_started,
                    remote_terminal_ack=mini_terminal_ack,
                )
                if abort_sent and ready_to_exit:
                    print(
                        f"CARRIER_ABORT_EXIT reason={exit_reason} "
                        f"remote_terminal_ack={mini_terminal_ack}",
                        flush=True,
                    )
                    return 7
            next_command = advance_periodic_deadline(
                next_command, now, 1.0 / args.command_rate_hz
            )

        if coordinator.phase == ChasePhase.BOTH_ACTIVE and coordinator.mini_complete:
            # A validated COMPLETE status is terminal and latched. From here,
            # the timestamped MiniState stream is the only live input needed
            # for relative-gap control; requiring repeated COMPLETE packets
            # creates a redundant abort path during harmless status loss.
            mini_state_rx_ms = coordinator.mini_state_rx_ms
            mini_state_age_ms = (
                None
                if mini_state_rx_ms is None
                else monotonic_ms() - mini_state_rx_ms
            )
            if (
                mini_state_age_ms is None
                or mini_state_age_ms > config.state_timeout_ms
            ):
                print(
                    f"CARRIER_TERMINAL_MINI_STATE_STALE age_ms={mini_state_age_ms}",
                    flush=True,
                )
                coordinator.request_abort(AbortReason.LINK_STALE)
        if now - last_log >= 1.0:
            print(
                f"carrier phase={coordinator.phase.value} worker={worker.phase.value} "
                f"ready={carrier_ready}",
                flush=True,
            )
            last_log = now
        time.sleep(SUPERVISOR_IDLE_SLEEP_SEC)


def print_plan(args: argparse.Namespace, config: StagedChaseConfig) -> None:
    print("PAIR B STAGED TWO-ROVER S-ROUTE")
    print("roles=Orin2/system2 Carrier leader; Orin1/system1 Mini follower")
    print(
        f"role={args.role} plan={config.plan_id} straight={config.straight_distance_m:.1f}m "
        f"radius={config.turn_radius_m:.1f}m lateral={config.lateral_offset_m:.1f}m"
    )
    print(
        f"Mini starts first at request {config.mini_speed_mps:.2f}; Carrier request "
        f"{config.carrier_speed_mps:.2f} starts after both {config.lead_delay_ms / 1000:.1f}s "
        f"and {config.lead_distance_m:.1f}m lead"
    )
    print(
        f"terminal=Carrier continues after Mini COMPLETE and stops {config.terminal_gap_m:.1f}m "
        "behind in shared Carrier-GPS ENU"
    )
    print("runtime coordination=Pair B only; ROS host-local; SSH is not a motion channel")
    print(
        f"Pair B rates=MiniState {args.state_rate_hz:g}Hz; "
        f"PlanCommand {args.command_rate_hz:g}Hz; MissionStatus {args.status_rate_hz:g}Hz"
    )
    print("failure=command TTL/watchdog, stale MiniState, Abort, worker failure -> stop both")


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        config = config_from_args(args)
    except ValueError as exc:
        print(f"REFUSED: {exc}")
        return 2
    print_plan(args, config)
    if not args.execute:
        print("DRY RUN ONLY: no process, ROS, serial, MAVROS, PX4, Arm or setpoint I/O.")
        return 0
    if args.confirm != LIVE_CONFIRMATION:
        print(f"REFUSED: required --confirm {LIVE_CONFIRMATION}")
        return 2
    missing = missing_field_confirmations(dict(os.environ))
    if missing:
        print("REFUSED: missing current-run field confirmations:")
        for name in missing:
            print(f"  {name}=true")
        return 2
    configure_host_local_ros_environment(os.environ)
    wrapper = role_wrapper(args.role, args.worker_wrapper)
    if not wrapper.is_file():
        print(f"REFUSED: worker wrapper missing: {wrapper}")
        return 2

    worker = GatedMissionWorker(args.role, wrapper, config)
    source = transport = trajectory_recorder = field_origin = None
    try:
        worker.start()
        source = MavrosMiniStateSource(args.mavros_namespace, 2.0)
        wait_for_mavros(source, worker, args.mavros_startup_timeout_sec)
        if args.role == "carrier":
            field_origin = wait_for_carrier_field_origin(
                source,
                worker,
                config.plan_id,
            )
        transport = open_pairb_transport(args)
        discarded_chunks = drain_pairb_transport(transport)
        print(
            f"PAIRB_SESSION_DRAIN discarded_chunks={discarded_chunks} duration_sec=0.5",
            flush=True,
        )
        if args.role == "carrier":
            artifact_dir = args.dual_trajectory_artifact_dir
            if artifact_dir is None:
                artifact_dir = (
                    REPO_DIR
                    / "results"
                    / "pairb_dual_trajectory"
                    / datetime.now().strftime("%Y%m%d_%H%M%S")
                )
            trajectory_recorder = PairBDualTrajectoryRecorder(
                source.node,
                config,
                artifact_dir.resolve(),
                require_shared_origin=True,
            )
        print(f"LIVE_READY transport={transport.description}", flush=True)
        if args.role == "mini":
            result = mini_loop(args, config, worker, source, transport)
            if result != 0:
                send_abort_best_effort(
                    transport,
                    role=Role.MINI,
                    plan_id=config.plan_id,
                    reason=AbortReason.LOCAL_SAFETY,
                )
            return result
        result = carrier_loop(
            args,
            config,
            worker,
            source,
            transport,
            trajectory_recorder,
            field_origin,
        )
        if result != 0:
            send_abort_best_effort(
                transport,
                role=Role.CARRIER,
                plan_id=config.plan_id,
                reason=AbortReason.LOCAL_SAFETY,
            )
        return result
    except KeyboardInterrupt:
        print("OPERATOR_INTERRUPT", flush=True)
        send_abort_best_effort(
            transport,
            role=Role.MINI if args.role == "mini" else Role.CARRIER,
            plan_id=config.plan_id,
            reason=AbortReason.OPERATOR,
        )
        return 130
    except Exception as exc:
        print(f"SUPERVISOR_ABORT {type(exc).__name__}: {exc}", flush=True)
        send_abort_best_effort(
            transport,
            role=Role.MINI if args.role == "mini" else Role.CARRIER,
            plan_id=config.plan_id,
            reason=AbortReason.LOCAL_SAFETY,
        )
        return 7
    finally:
        if trajectory_recorder is not None:
            artifact_dir = trajectory_recorder.write_artifacts()
            if artifact_dir is not None:
                print(f"DUAL_TRAJECTORY_ARTIFACTS {artifact_dir}", flush=True)
        if transport is not None:
            try:
                transport.close()
            except Exception as exc:
                print(f"TRANSPORT_CLOSE_WARNING {type(exc).__name__}: {exc}", flush=True)
        if source is not None:
            try:
                source.close()
            except Exception as exc:
                print(f"SOURCE_CLOSE_WARNING {type(exc).__name__}: {exc}", flush=True)
        worker.close()


if __name__ == "__main__":
    raise SystemExit(main())
