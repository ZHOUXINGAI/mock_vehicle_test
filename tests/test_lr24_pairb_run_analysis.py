#!/usr/bin/env python3

import csv
import json
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


REPO_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_DIR / "src"))

from lr24_pairb_run_analysis import (
    ArtifactError,
    CARRIER_CSV_FIELDS,
    MINI_CSV_FIELDS,
    analyze_pair,
    load_contract,
)


CONFIG = REPO_DIR / "config/lr24/pairb_v1.json"
CLI = REPO_DIR / "scripts/analyze_lr24_pairb_run.py"
UINT32_MASK = 0xFFFFFFFF
UINT32_HALF = 0x80000000


def sequence_gaps(values):
    gaps = 0
    for previous, current in zip(values, values[1:]):
        delta = (current - previous) & UINT32_MASK
        if 0 < delta < UINT32_HALF:
            gaps += delta - 1
    return gaps


def failure_codes(report):
    return {failure["code"] for failure in report["failures"]}


class PairBRunAnalysisTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        self.contract = load_contract(CONFIG)

    def tearDown(self):
        self.temporary.cleanup()

    @staticmethod
    def _times(provided, count, start, step):
        if provided is not None:
            if len(provided) != count:
                raise AssertionError("timestamp count does not match sequence count")
            return list(provided)
        return [start + index * step for index in range(count)]

    @staticmethod
    def _row(fields, role, event, mono_ms, seq, **updates):
        row = {field: "" for field in fields}
        row.update(
            {
                "role": role,
                "event": event,
                "mono_ms": str(mono_ms & UINT32_MASK),
                "seq": str(seq & UINT32_MASK),
            }
        )
        row.update({key: str(value) for key, value in updates.items()})
        return row

    @staticmethod
    def _write_csv(path, fields, rows):
        with path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=fields)
            writer.writeheader()
            writer.writerows(rows)

    def make_pair(
        self,
        *,
        carrier_endpoint="1.242->2.242",
        mini_endpoint="2.242->1.242",
        state_tx=tuple(range(10)),
        state_rx=None,
        command_tx=tuple(range(6)),
        command_rx=None,
        plan_tx=(0, 1),
        plan_rx=None,
        mini_state_times=None,
        carrier_state_times=None,
        carrier_command_times=None,
        mini_command_times=None,
        command_v=0.0,
        command_omega=0.0,
        rejected=0,
        aborts=0,
        field_origins=2,
    ):
        state_tx = tuple(state_tx)
        state_rx = state_tx if state_rx is None else tuple(state_rx)
        command_tx = tuple(command_tx)
        command_rx = command_tx if command_rx is None else tuple(command_rx)
        plan_tx = tuple(plan_tx)
        plan_rx = plan_tx if plan_rx is None else tuple(plan_rx)
        mini_state_times = self._times(
            mini_state_times, len(state_tx), 3_000_000_000, 100
        )
        carrier_state_times = self._times(
            carrier_state_times, len(state_rx), 500_000, 100
        )
        carrier_command_times = self._times(
            carrier_command_times, len(command_tx), 500_050, 500
        )
        mini_command_times = self._times(
            mini_command_times, len(command_rx), 9_000_000, 500
        )

        carrier_rows = [
            self._row(
                CARRIER_CSV_FIELDS, "carrier", "rx_state", mono, seq,
                stale_ms=0, x_m=0, y_m=0,
            )
            for seq, mono in zip(state_rx, carrier_state_times)
        ]
        carrier_rows += [
            self._row(
                CARRIER_CSV_FIELDS, "carrier", "tx_command", mono, seq,
                phase="HOLD", stale_ms=20 if state_rx else "",
                v_mps=command_v, omega_radps=command_omega,
            )
            for seq, mono in zip(command_tx, carrier_command_times)
        ]
        carrier_rows += [
            self._row(
                CARRIER_CSV_FIELDS, "carrier", "tx_corridor_plan",
                500_025 + index * 5_000, seq,
                phase="CORRIDOR_PLAN", x_m=-1.553, y_m=-4.224, v_mps=0.9,
            )
            for index, seq in enumerate(plan_tx)
        ]

        mini_rows = [
            self._row(
                MINI_CSV_FIELDS, "mini", "tx_state", mono, seq, x_m=0, y_m=0,
            )
            for seq, mono in zip(state_tx, mini_state_times)
        ]
        mini_rows += [
            self._row(
                MINI_CSV_FIELDS, "mini", "rx_command", mono, seq,
                phase="HOLD", v_mps=command_v, omega_radps=command_omega,
            )
            for seq, mono in zip(command_rx, mini_command_times)
        ]
        mini_rows += [
            self._row(
                MINI_CSV_FIELDS, "mini", "rx_corridor_plan",
                9_000_025 + index * 5_000, seq,
                phase="CORRIDOR_PLAN", x_m=-1.553, y_m=-4.224, v_mps=0.9,
            )
            for index, seq in enumerate(plan_rx)
        ]

        carrier_log = self.root / "lr24_pairb_carrier.log"
        carrier_csv = self.root / "lr24_pairb_carrier.csv"
        mini_log = self.root / "lr24_pairb_mini.log"
        mini_csv = self.root / "lr24_pairb_mini.csv"
        carrier_log.write_text(
            "carrier dry-run transport=mavlink-tunnel-serial:fixture@57600 "
            + carrier_endpoint
            + "\n"
            + (
                f"carrier summary states_rx={len(state_rx)} "
                f"state_seq_gaps={sequence_gaps(state_rx)} "
                f"commands_tx={len(command_tx)} "
                f"corridor_plans_tx={len(plan_tx)} "
                f"field_origins_tx={field_origins}\n"
            ),
            encoding="utf-8",
        )
        mini_log.write_text(
            "mini dry-run transport=mavros-router-tunnel:/pairb_tunnel "
            + mini_endpoint
            + "\n"
            + (
                f"mini summary states_tx={len(state_tx)} "
                f"commands_rx={len(command_rx)} "
                f"command_seq_gaps={sequence_gaps(command_rx)} "
                f"corridor_plans_rx={len(plan_rx)} "
                f"corridor_plan_seq_gaps={sequence_gaps(plan_rx)} "
                f"rejected={rejected} aborts_rx={aborts}\n"
            ),
            encoding="utf-8",
        )
        self._write_csv(carrier_csv, CARRIER_CSV_FIELDS, carrier_rows)
        self._write_csv(mini_csv, MINI_CSV_FIELDS, mini_rows)
        return carrier_log, carrier_csv, mini_log, mini_csv

    def analyze(self, paths, **options):
        return analyze_pair(
            carrier_log=paths[0],
            carrier_csv=paths[1],
            mini_log=paths[2],
            mini_csv=paths[3],
            contract=self.contract,
            **options,
        )

    def test_valid_targeted_pair_passes(self):
        report = self.analyze(self.make_pair())
        self.assertTrue(report["pass"])
        self.assertTrue(report["directed_acceptance"])

    def test_targeted_endpoints_allow_trailing_runtime_fields(self):
        paths = self.make_pair()
        for log_path in (paths[0], paths[2]):
            lines = log_path.read_text(encoding="utf-8").splitlines()
            lines[0] += " state_rate=10.0Hz simulate_orbit=True"
            log_path.write_text("\n".join(lines) + "\n", encoding="utf-8")

        report = self.analyze(paths)
        self.assertTrue(report["pass"])
        self.assertTrue(report["directed_acceptance"])

    def test_broadcast_is_rejected_as_targeted_and_labeled_diagnostic(self):
        paths = self.make_pair(
            carrier_endpoint="1.242->0.0", mini_endpoint="2.242->0.0"
        )
        targeted = self.analyze(paths)
        self.assertIn("carrier_endpoint_targeted", failure_codes(targeted))
        self.assertIn("mini_endpoint_targeted", failure_codes(targeted))
        broadcast = self.analyze(paths, mode="broadcast")
        self.assertTrue(broadcast["pass"])
        self.assertTrue(broadcast["diagnostic_only"])
        self.assertFalse(broadcast["directed_acceptance"])

    def test_nonzero_hold_command_fails(self):
        report = self.analyze(
            self.make_pair(command_v=0.1, command_omega=0.02)
        )
        self.assertIn("hold_zero_command", failure_codes(report))

    def test_dropped_state_sequence_fails(self):
        report = self.analyze(
            self.make_pair(
                state_tx=range(10), state_rx=(0, 1, 3, 4, 5, 6, 7, 8, 9)
            )
        )
        self.assertIn("mini_state_sequence", failure_codes(report))
        self.assertIn("mini_state_coverage", failure_codes(report))

    def test_state_stale_and_command_watchdog_timeouts_fail(self):
        report = self.analyze(
            self.make_pair(
                carrier_state_times=(
                    500_000, 500_100, 500_450, 500_550, 500_650,
                    500_750, 500_850, 500_950, 501_050, 501_150,
                ),
                mini_command_times=(
                    9_000_000, 9_000_500, 9_001_300,
                    9_001_800, 9_002_300, 9_002_800,
                ),
            )
        )
        self.assertIn("state_stale_timeout", failure_codes(report))
        self.assertIn("mini_watchdog_timeout", failure_codes(report))

    def test_reject_and_abort_fail(self):
        report = self.analyze(self.make_pair(rejected=2, aborts=1))
        self.assertIn("mini_gate_rejects", failure_codes(report))
        self.assertIn("mini_abort", failure_codes(report))

    def test_bad_csv_schema_is_input_error(self):
        paths = self.make_pair()
        paths[3].write_text("role,event\nmini,tx_state\n", encoding="utf-8")
        with self.assertRaises(ArtifactError):
            self.analyze(paths)

    def test_uint32_sequence_wrap_is_contiguous(self):
        wrapped = (0xFFFFFFFE, 0xFFFFFFFF, 0, 1)
        report = self.analyze(
            self.make_pair(
                state_tx=wrapped, command_tx=wrapped, plan_tx=wrapped
            )
        )
        self.assertTrue(report["pass"])

    def test_independent_monotonic_clocks_are_not_compared(self):
        report = self.analyze(
            self.make_pair(
                mini_state_times=tuple(
                    3_500_000_000 + index * 100 for index in range(10)
                ),
                carrier_state_times=tuple(
                    100 + index * 100 for index in range(10)
                ),
                carrier_command_times=tuple(
                    1_000 + index * 500 for index in range(6)
                ),
                mini_command_times=tuple(
                    2_000_000_000 + index * 500 for index in range(6)
                ),
            )
        )
        self.assertTrue(report["pass"])

    def test_historic_targeted_zero_receive_sample_fails(self):
        report = self.analyze(
            self.make_pair(
                state_tx=range(20),
                state_rx=(),
                command_tx=range(591),
                command_rx=range(20),
                plan_tx=range(60),
                plan_rx=range(4),
                field_origins=60,
            )
        )
        self.assertIn("mini_state_present", failure_codes(report))
        self.assertEqual(report["metrics"]["carrier"]["summary"]["states_rx"], 0)

    def _run_cli(self, paths):
        return subprocess.run(
            [
                sys.executable,
                str(CLI),
                "--carrier-log", str(paths[0]),
                "--carrier-csv", str(paths[1]),
                "--mini-log", str(paths[2]),
                "--mini-csv", str(paths[3]),
            ],
            text=True,
            capture_output=True,
            env={**os.environ, "PYTHONDONTWRITEBYTECODE": "1"},
            check=False,
        )

    def test_cli_exit_codes_and_required_json_shape(self):
        passed = self._run_cli(self.make_pair())
        self.assertEqual(passed.returncode, 0, passed.stderr)
        payload = json.loads(passed.stdout)
        for key in (
            "schema_version", "inputs", "mode", "endpoints",
            "metrics", "checks", "failures", "pass",
        ):
            self.assertIn(key, payload)

        failed = self._run_cli(
            self.make_pair(carrier_endpoint="1.242->0.0")
        )
        self.assertEqual(failed.returncode, 1, failed.stderr)

        damaged_paths = self.make_pair()
        damaged_paths[3].write_text(
            "role,event\nmini,tx_state\n", encoding="utf-8"
        )
        damaged = self._run_cli(damaged_paths)
        self.assertEqual(damaged.returncode, 2, damaged.stderr)
        self.assertEqual(
            json.loads(damaged.stdout)["failures"][0]["code"],
            "input_artifact_error",
        )


if __name__ == "__main__":
    unittest.main()
