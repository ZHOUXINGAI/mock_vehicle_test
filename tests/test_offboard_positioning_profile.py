#!/usr/bin/env python3

import os
from pathlib import Path
import signal
import subprocess
import tempfile
import textwrap
import time
import unittest


REPO = Path(__file__).resolve().parents[1]
INDOOR_WRAPPER = REPO / "scripts" / "run_indoor_fake_ev_profile.sh"
OUTDOOR_WRAPPER = REPO / "scripts" / "run_orin2_outdoor_forward_5m.sh"


class OffboardPositioningProfileTest(unittest.TestCase):
    def setUp(self):
        self.tempdir = tempfile.TemporaryDirectory()
        self.root = Path(self.tempdir.name)
        self.device = self.root / "pixhawk-by-id"
        self.device.touch()
        self.state_file = self.root / "ev_state"
        self.state_file.write_text("0\n", encoding="ascii")
        self.mock_bin = self.root / "bin"
        self.mock_bin.mkdir()
        mock_ps = self.mock_bin / "ps"
        mock_ps.write_text("#!/usr/bin/env bash\nexit 0\n", encoding="ascii")
        mock_ps.chmod(0o755)
        self.param_tool = self.root / "mock_param.py"
        self.param_tool.write_text(
            textwrap.dedent(
                """\
                import os
                from pathlib import Path
                import sys

                args = sys.argv[1:]
                command = "set" if "set" in args else "get"
                state_path = Path(os.environ["MOCK_EV_STATE"])
                ev = int(state_path.read_text(encoding="ascii").strip())
                if command == "set":
                    value = int(args[-1])
                    if value == 0 and os.environ.get("MOCK_FAIL_RESTORE") == "true":
                        raise SystemExit(9)
                    state_path.write_text(f"{value}\\n", encoding="ascii")
                    print(f"EKF2_EV_CTRL={value} type=int32 index=1/3")
                else:
                    print("MAV_SYS_ID=2 type=int32 index=0/3")
                    print(f"EKF2_EV_CTRL={ev} type=int32 index=1/3")
                    print("EKF2_GPS_CTRL=7 type=int32 index=2/3")
                """
            ),
            encoding="ascii",
        )

    def tearDown(self):
        self.tempdir.cleanup()

    def indoor_env(self, *, fail_restore=False):
        env = os.environ.copy()
        env.update(
            {
                "PATH": f"{self.mock_bin}:{env['PATH']}",
                "PIXHAWK_DEVICE": str(self.device),
                "EXPECTED_SYSTEM_ID": "2",
                "PX4_PARAM_TOOL": str(self.param_tool),
                "OFFBOARD_PROFILE_STATE_DIR": str(self.root / "runtime"),
                "MOCK_EV_STATE": str(self.state_file),
                "MOCK_FAIL_RESTORE": "true" if fail_restore else "false",
                "CONFIRM_WHEELS_LIFTED": "true",
                "CONFIRM_VEHICLE_DISARMED": "true",
                "CONFIRM_RC_KILL_READY": "true",
                "CONFIRM_RESTORE_GPS_BASELINE": "true",
            }
        )
        return env

    def run_indoor(self, child, *, fail_restore=False):
        return subprocess.run(
            [
                "bash",
                str(INDOOR_WRAPPER),
                "--execute",
                "--confirm",
                "INDOOR_FAKE_EV_WHEELS_LIFTED_RESTORE_GPS_ON_EXIT",
                "--",
                child,
            ],
            cwd=REPO,
            env=self.indoor_env(fail_restore=fail_restore),
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            timeout=20,
            check=False,
        )

    def test_normal_child_restores_outdoor_baseline_and_removes_marker(self):
        result = self.run_indoor("/bin/true")
        self.assertEqual(result.returncode, 0, result.stdout)
        self.assertEqual(self.state_file.read_text(encoding="ascii").strip(), "0")
        self.assertFalse((self.root / "runtime" / "indoor_fake_ev.active").exists())
        self.assertIn("OUTDOOR_GPS_BASELINE_RESTORED=true", result.stdout)

    def test_failed_child_still_restores_and_preserves_child_status(self):
        result = self.run_indoor("/bin/false")
        self.assertEqual(result.returncode, 1, result.stdout)
        self.assertEqual(self.state_file.read_text(encoding="ascii").strip(), "0")
        self.assertFalse((self.root / "runtime" / "indoor_fake_ev.active").exists())

    def test_failed_restore_retains_marker_and_returns_profile_error(self):
        result = self.run_indoor("/bin/true", fail_restore=True)
        self.assertEqual(result.returncode, 4, result.stdout)
        self.assertEqual(self.state_file.read_text(encoding="ascii").strip(), "15")
        self.assertTrue((self.root / "runtime" / "indoor_fake_ev.active").exists())
        self.assertIn("marker retained", result.stdout)

    def test_term_signal_restores_baseline_and_returns_nonzero(self):
        process = subprocess.Popen(
            [
                "bash",
                str(INDOOR_WRAPPER),
                "--execute",
                "--confirm",
                "INDOOR_FAKE_EV_WHEELS_LIFTED_RESTORE_GPS_ON_EXIT",
                "--",
                "/bin/sleep",
                "30",
            ],
            cwd=REPO,
            env=self.indoor_env(),
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
        )
        marker = self.root / "runtime" / "indoor_fake_ev.active"
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            if marker.exists() and self.state_file.read_text(encoding="ascii").strip() == "15":
                break
            time.sleep(0.05)
        else:
            process.kill()
            self.fail("indoor profile did not become active before timeout")
        process.send_signal(signal.SIGTERM)
        output, _ = process.communicate(timeout=15)
        self.assertEqual(process.returncode, 143, output)
        self.assertEqual(self.state_file.read_text(encoding="ascii").strip(), "0")
        self.assertFalse(marker.exists())
        self.assertIn("OUTDOOR_GPS_BASELINE_RESTORED=true", output)

    def test_outdoor_launcher_rejects_persistent_indoor_marker_before_hardware(self):
        state_dir = self.root / "runtime"
        state_dir.mkdir()
        marker = state_dir / "indoor_fake_ev.active"
        marker.write_text("profile=INDOOR_FAKE_EV\n", encoding="ascii")
        env = os.environ.copy()
        env["OFFBOARD_PROFILE_STATE_DIR"] = str(state_dir)
        for name in (
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
        ):
            env[name] = "true"
        result = subprocess.run(
            [
                "bash",
                str(OUTDOOR_WRAPPER),
                "--execute",
                "--confirm",
                "OUTDOOR_FORWARD_5M_AREA_CLEAR_RC_KILL_READY",
            ],
            cwd=REPO,
            env=env,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            timeout=10,
            check=False,
        )
        self.assertEqual(result.returncode, 2, result.stdout)
        self.assertIn("persistent INDOOR_FAKE_EV marker exists", result.stdout)


if __name__ == "__main__":
    unittest.main(verbosity=2)
