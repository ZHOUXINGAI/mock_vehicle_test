#!/usr/bin/env python3

"""MAVLink PX4 Offboard smoke test for the real rover over Pixhawk USB."""

from __future__ import annotations

import argparse
import math
import signal
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

from pymavlink import mavutil


@dataclass(frozen=True)
class SmokeStep:
    name: str
    duration_sec: float
    forward_mps: float
    yaw_rate_radps: float


def parse_bool(value: str) -> bool:
    normalized = str(value).strip().lower()
    if normalized in {"true", "1", "yes", "y", "on"}:
        return True
    if normalized in {"false", "0", "no", "n", "off", ""}:
        return False
    raise argparse.ArgumentTypeError(f"invalid boolean value: {value!r}")


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Send a short PX4 MAVLink Offboard velocity smoke-test sequence."
    )
    parser.add_argument(
        "--device",
        default="/dev/serial/by-id/usb-Auterion_PX4_FMU_v6C.x_0-if00",
        help="Pixhawk MAVLink serial device.",
    )
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--source-system", type=int, default=245)
    parser.add_argument("--command-rate-hz", type=float, default=20.0)
    parser.add_argument("--warmup-sec", type=float, default=2.0)
    parser.add_argument("--stop-sec", type=float, default=1.0)
    parser.add_argument("--forward-sec", type=float, default=1.0)
    parser.add_argument("--backward-sec", type=float, default=1.0)
    parser.add_argument("--turn-sec", type=float, default=0.5)
    parser.add_argument("--final-stop-sec", type=float, default=2.0)
    parser.add_argument("--linear-speed-mps", type=float, default=0.12)
    parser.add_argument("--turn-yaw-rate-radps", type=float, default=0.25)
    parser.add_argument("--turn-sign", type=float, default=1.0)
    parser.add_argument("--max-linear-speed-mps", type=float, default=0.30)
    parser.add_argument("--max-yaw-rate-radps", type=float, default=0.70)
    parser.add_argument("--mode-change-on-start", type=parse_bool, default=False)
    parser.add_argument("--arm-on-start", type=parse_bool, default=False)
    parser.add_argument("--disarm-on-finish", type=parse_bool, default=False)
    parser.add_argument("--require-offboard-mode", type=parse_bool, default=True)
    parser.add_argument("--require-armed", type=parse_bool, default=True)
    parser.add_argument("--max-wait-for-ready-sec", type=float, default=60.0)
    parser.add_argument("--stop-burst-sec", type=float, default=0.8)
    parser.add_argument("--confirm-wheels-lifted", type=parse_bool, default=False)
    parser.add_argument("--confirm-rc-ready", type=parse_bool, default=False)
    parser.add_argument("--confirm-param-backup", type=parse_bool, default=False)
    return parser


class MavlinkOffboardSmoke:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.command_rate_hz = max(2.0, float(args.command_rate_hz))
        self.period_sec = 1.0 / self.command_rate_hz
        self.linear_speed_mps = abs(float(args.linear_speed_mps))
        self.turn_yaw_rate_radps = abs(float(args.turn_yaw_rate_radps))
        self.turn_sign = 1.0 if float(args.turn_sign) >= 0.0 else -1.0
        self.master: Optional[mavutil.mavfile] = None
        self.target_system = 1
        self.target_component = 1
        self.latest_heartbeat = None
        self.start_monotonic = time.monotonic()
        self.stop_requested = False

        self._validate()
        self.sequence = self._build_sequence()

    def _validate(self) -> None:
        missing = []
        if not self.args.confirm_wheels_lifted:
            missing.append("CONFIRM_WHEELS_LIFTED=true")
        if not self.args.confirm_rc_ready:
            missing.append("CONFIRM_RC_READY=true")
        if not self.args.confirm_param_backup:
            missing.append("CONFIRM_PARAM_BACKUP=true")
        if missing:
            raise RuntimeError(
                "Refusing to run real-hardware motion test until safety is confirmed: "
                + ", ".join(missing)
            )

        if self.linear_speed_mps > abs(float(self.args.max_linear_speed_mps)):
            raise ValueError("linear speed exceeds max-linear-speed-mps")
        if self.turn_yaw_rate_radps > abs(float(self.args.max_yaw_rate_radps)):
            raise ValueError("turn yaw rate exceeds max-yaw-rate-radps")

        device = Path(str(self.args.device))
        if not device.exists():
            raise FileNotFoundError(f"MAVLink device not found: {device}")

    def _build_sequence(self) -> list[SmokeStep]:
        speed = self.linear_speed_mps
        turn = self.turn_yaw_rate_radps * self.turn_sign
        return [
            SmokeStep("initial_stop", max(0.2, self.args.stop_sec), 0.0, 0.0),
            SmokeStep("forward", max(0.0, self.args.forward_sec), speed, 0.0),
            SmokeStep("stop_after_forward", max(0.2, self.args.stop_sec), 0.0, 0.0),
            SmokeStep("backward", max(0.0, self.args.backward_sec), -speed, 0.0),
            SmokeStep("stop_after_backward", max(0.2, self.args.stop_sec), 0.0, 0.0),
            SmokeStep("turn_left", max(0.0, self.args.turn_sec), 0.0, turn),
            SmokeStep("stop_after_left", max(0.2, self.args.stop_sec), 0.0, 0.0),
            SmokeStep("turn_right", max(0.0, self.args.turn_sec), 0.0, -turn),
            SmokeStep("final_stop", max(0.5, self.args.final_stop_sec), 0.0, 0.0),
        ]

    def run(self) -> int:
        print(f"Opening MAVLink device {self.args.device} @ {self.args.baud}")
        self.master = mavutil.mavlink_connection(
            str(self.args.device),
            baud=int(self.args.baud),
            source_system=int(self.args.source_system),
            autoreconnect=False,
        )
        print("Waiting for Pixhawk heartbeat...")
        self.latest_heartbeat = self.master.wait_heartbeat(timeout=15)
        if self.latest_heartbeat is None:
            raise TimeoutError("No heartbeat received from Pixhawk")
        self.target_system = int(self.master.target_system)
        self.target_component = int(self.master.target_component)
        print(
            "Heartbeat: "
            f"target_system={self.target_system} "
            f"target_component={self.target_component} "
            f"mode={self.mode_name()} armed={self.is_armed()}"
        )
        self.log_sequence()

        print(f"Streaming stop setpoints for {self.args.warmup_sec:.1f}s")
        self.stream_for(max(0.5, float(self.args.warmup_sec)), 0.0, 0.0)

        mode_requested = False
        arm_requested = False
        ready_start = time.monotonic()
        while not self.stop_requested:
            self.pump_messages()
            self.send_setpoint(0.0, 0.0)

            if self.args.mode_change_on_start and not mode_requested:
                self.request_offboard_mode()
                mode_requested = True
            if (
                self.args.arm_on_start
                and not arm_requested
                and (not self.args.require_offboard_mode or self.is_offboard_mode())
            ):
                self.request_arm(True)
                arm_requested = True

            if self.ready_to_start():
                break

            if time.monotonic() - ready_start > max(0.0, float(self.args.max_wait_for_ready_sec)):
                print("Timed out waiting for Offboard/armed state; sending stop and exiting")
                self.stop_burst()
                return 2

            print(
                "Waiting: "
                f"mode={self.mode_name()} armed={self.is_armed()} "
                f"need_offboard={self.args.require_offboard_mode} "
                f"need_armed={self.args.require_armed}"
            )
            time.sleep(1.0)

        print("Starting smoke sequence")
        for step in self.sequence:
            if self.stop_requested:
                break
            print(
                f"Step {step.name}: {step.duration_sec:.2f}s "
                f"vx={step.forward_mps:.3f} yaw_rate={step.yaw_rate_radps:.3f}"
            )
            self.stream_for(step.duration_sec, step.forward_mps, step.yaw_rate_radps)

        print("Sequence complete; sending final stop")
        self.stop_burst()
        if self.args.disarm_on_finish:
            self.request_arm(False)
        return 0

    def ready_to_start(self) -> bool:
        if self.args.require_offboard_mode and not self.is_offboard_mode():
            return False
        if self.args.require_armed and not self.is_armed():
            return False
        return True

    def stream_for(self, duration_sec: float, forward_mps: float, yaw_rate_radps: float) -> None:
        end_time = time.monotonic() + max(0.0, duration_sec)
        while time.monotonic() < end_time and not self.stop_requested:
            self.pump_messages()
            self.send_setpoint(forward_mps, yaw_rate_radps)
            time.sleep(self.period_sec)

    def stop_burst(self) -> None:
        duration = max(0.2, float(self.args.stop_burst_sec))
        self.stream_for(duration, 0.0, 0.0)

    def send_setpoint(self, forward_mps: float, yaw_rate_radps: float) -> None:
        assert self.master is not None
        type_mask = (
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
        )
        frame = getattr(
            mavutil.mavlink,
            "MAV_FRAME_BODY_FRD",
            mavutil.mavlink.MAV_FRAME_BODY_NED,
        )
        time_boot_ms = int((time.monotonic() - self.start_monotonic) * 1000)
        self.master.mav.set_position_target_local_ned_send(
            time_boot_ms,
            self.target_system,
            self.target_component,
            frame,
            type_mask,
            0.0,
            0.0,
            0.0,
            float(forward_mps),
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            math.nan,
            float(yaw_rate_radps),
        )

    def request_offboard_mode(self) -> None:
        assert self.master is not None
        mapping = self.master.mode_mapping() or {}
        if "OFFBOARD" in mapping:
            self.master.set_mode(mapping["OFFBOARD"])
        else:
            self.master.mav.command_long_send(
                self.target_system,
                self.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                0,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                6,
                0,
                0,
                0,
                0,
                0,
            )
        print("Requested OFFBOARD mode")

    def request_arm(self, arm: bool) -> None:
        assert self.master is not None
        self.master.mav.command_long_send(
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1.0 if arm else 0.0,
            0,
            0,
            0,
            0,
            0,
            0,
        )
        print("Requested ARM" if arm else "Requested DISARM")

    def pump_messages(self) -> None:
        assert self.master is not None
        while True:
            msg = self.master.recv_match(blocking=False)
            if msg is None:
                break
            if msg.get_type() == "HEARTBEAT":
                self.latest_heartbeat = msg

    def is_armed(self) -> bool:
        if self.latest_heartbeat is None:
            return False
        return bool(
            int(self.latest_heartbeat.base_mode)
            & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        )

    def is_offboard_mode(self) -> bool:
        return self.mode_name().upper() == "OFFBOARD"

    def mode_name(self) -> str:
        if self.latest_heartbeat is None:
            return "UNKNOWN"
        try:
            return str(mavutil.mode_string_v10(self.latest_heartbeat))
        except Exception:
            return "UNKNOWN"

    def log_sequence(self) -> None:
        print("Sequence:")
        for index, step in enumerate(self.sequence, start=1):
            print(
                f"  {index:02d} {step.name}: {step.duration_sec:.2f}s "
                f"vx={step.forward_mps:.3f} yaw_rate={step.yaw_rate_radps:.3f}"
            )


def main() -> int:
    parser = build_arg_parser()
    args = parser.parse_args()
    runner = MavlinkOffboardSmoke(args)

    def handle_stop(_signum: int, _frame: object) -> None:
        runner.stop_requested = True

    signal.signal(signal.SIGINT, handle_stop)
    signal.signal(signal.SIGTERM, handle_stop)

    try:
        return runner.run()
    except KeyboardInterrupt:
        runner.stop_requested = True
        runner.stop_burst()
        return 130
    finally:
        if runner.master is not None:
            try:
                runner.stop_burst()
            except Exception:
                pass
            runner.master.close()


if __name__ == "__main__":
    sys.exit(main())
