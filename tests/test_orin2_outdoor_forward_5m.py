import math
import unittest

from src.orin2_outdoor_forward_5m import (
    MissionConfig,
    Observation,
    manual_arm_ready,
    motion_fault,
    navigation_ready,
    safe_manual_prestate,
    track_metrics,
    validate_config,
)


def ready_observation(**changes):
    values = dict(
        state_present=True,
        state_age_sec=0.1,
        connected=True,
        armed=False,
        mode="MANUAL",
        manual_input=True,
        pose_present=True,
        pose_age_sec=0.1,
        x_m=0.0,
        y_m=0.0,
        yaw_rad=0.0,
        gps_present=True,
        gps_age_sec=0.1,
        gps_status=0,
        latitude_deg=22.0,
        longitude_deg=114.0,
        gps_raw_present=True,
        gps_raw_age_sec=0.1,
        gps_fix_type=3,
        satellites_visible=10,
    )
    values.update(changes)
    return Observation(**values)


class OutdoorForwardGeometryTests(unittest.TestCase):
    def test_track_metrics_at_zero_heading(self):
        along, cross, displacement, heading_error = track_metrics(
            1.0, 2.0, 0.0, 5.0, 2.5, math.radians(10.0)
        )
        self.assertAlmostEqual(along, 4.0)
        self.assertAlmostEqual(cross, 0.5)
        self.assertAlmostEqual(displacement, math.hypot(4.0, 0.5))
        self.assertAlmostEqual(math.degrees(heading_error), 10.0)

    def test_track_metrics_follow_rotated_heading(self):
        along, cross, _, _ = track_metrics(
            0.0, 0.0, math.pi / 2.0, 0.2, 3.0, math.pi / 2.0
        )
        self.assertAlmostEqual(along, 3.0)
        self.assertAlmostEqual(cross, -0.2)


class OutdoorForwardGateTests(unittest.TestCase):
    def test_safe_manual_prestate_requires_real_navigation(self):
        self.assertTrue(navigation_ready(ready_observation()))
        self.assertTrue(safe_manual_prestate(ready_observation()))
        self.assertFalse(safe_manual_prestate(ready_observation(gps_status=-1)))
        self.assertFalse(safe_manual_prestate(ready_observation(gps_fix_type=2)))
        self.assertFalse(safe_manual_prestate(ready_observation(satellites_visible=5)))
        self.assertFalse(safe_manual_prestate(ready_observation(pose_age_sec=2.0)))
        self.assertFalse(safe_manual_prestate(ready_observation(mode="AUTO.LOITER")))

    def test_manual_arm_gate_is_manual_and_armed(self):
        self.assertTrue(manual_arm_ready(ready_observation(armed=True)))
        self.assertFalse(manual_arm_ready(ready_observation(armed=False)))
        self.assertFalse(manual_arm_ready(ready_observation(armed=True, mode="OFFBOARD")))

    def test_motion_faults_fail_closed(self):
        active = ready_observation(armed=True, mode="OFFBOARD", manual_input=False)
        self.assertIsNone(motion_fault(active))
        self.assertEqual(motion_fault(ready_observation(armed=False, mode="OFFBOARD")), "unexpected_disarm")
        self.assertEqual(motion_fault(ready_observation(armed=True, mode="MANUAL")), "offboard_exit")
        self.assertEqual(motion_fault(ready_observation(armed=True, mode="OFFBOARD", gps_status=-1)), "gps_no_fix")
        self.assertEqual(motion_fault(ready_observation(armed=True, mode="OFFBOARD", gps_fix_type=2)), "gps_not_3d")
        self.assertEqual(motion_fault(ready_observation(armed=True, mode="OFFBOARD", satellites_visible=5)), "gps_satellites_low")
        self.assertEqual(motion_fault(ready_observation(armed=True, mode="OFFBOARD", pose_age_sec=2.0)), "local_pose_stale")

    def test_configuration_is_bounded(self):
        validate_config(MissionConfig())
        with self.assertRaises(ValueError):
            validate_config(MissionConfig(distance_m=50.0))
        with self.assertRaises(ValueError):
            validate_config(MissionConfig(speed_mps=0.3, max_speed_mps=0.3))
        with self.assertRaises(ValueError):
            validate_config(MissionConfig(stall_window_sec=1.0))


if __name__ == "__main__":
    unittest.main()
