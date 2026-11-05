"""Readiness, missing feedback and sinking detection without ROS/network access."""

import importlib.util
import math
from pathlib import Path
import sys
import unittest

SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "simulation_guard.py"
SPEC = importlib.util.spec_from_file_location("usv_simulation_guard", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class SimulationGuardTest(unittest.TestCase):
    def setUp(self):
        self.guard = MODULE.SimulationGuard(0.0)

    def sample(self, stamp, z=0.0, roll=0.0, pitch=0.0, now=None):
        now = stamp if now is None else now
        self.guard.update_clock(stamp, now)
        self.guard.update_odometry(stamp, z, roll, pitch, now)
        return self.guard.evaluate(now)

    def test_ready_requires_both_streams_and_settling(self):
        self.assertIsNone(self.sample(0.0))
        self.assertIsNone(self.sample(1.0, 0.01))
        self.assertEqual(self.sample(2.1, 0.0)[0], "READY")
        self.assertIsNone(self.sample(3.0))

    def test_sustained_sinking_is_a_failure(self):
        self.sample(0.0)
        self.assertIsNone(self.sample(0.5, -0.4))
        result = self.sample(1.6, -1.1)
        self.assertEqual(result[0], "ERROR")
        self.assertIn("drop=1.10 m", result[1])
        self.assertIn("SurfaceBuoyancy", result[1])

    def test_transient_and_level_offset_do_not_fail(self):
        self.sample(0.0, 3.0)
        self.sample(0.5, 2.6)
        self.sample(0.8, 3.0)
        self.assertEqual(self.sample(2.1, 3.0)[0], "READY")

    def test_tilt_failure(self):
        self.sample(0.0)
        self.sample(0.5, roll=0.9)
        result = self.sample(1.6, roll=0.9)
        self.assertEqual(result[0], "ERROR")
        self.assertIn("roll=51.6 deg", result[1])

    def test_missing_stream_times_out(self):
        self.guard.update_clock(120.0, 120.0)
        self.assertIsNone(self.guard.evaluate(120.0))
        self.assertIn("/sim/me120/odometry", self.guard.evaluate(120.1)[1])

    def test_paused_world_warns_without_false_failure(self):
        self.sample(0.0)
        self.sample(2.1)
        warning = self.guard.evaluate(20.0)
        self.assertEqual(warning[0], "WARNING")
        self.assertIsNone(self.guard.evaluate(21.0))
        self.assertEqual(self.sample(3.0, now=21.1)[0], "INFO")
        self.assertIsNone(self.guard.failure)

    def test_odom_loss_with_advancing_clock_fails(self):
        self.sample(0.0)
        self.guard.update_clock(20.0, 20.0)
        self.assertIn("Odometry stopped", self.guard.evaluate(20.0)[1])

    def test_clock_loss_with_odom_delivery_fails(self):
        self.sample(0.0)
        self.guard.update_odometry(20.0, 0.0, 0.0, 0.0, 20.0)
        self.assertIn("/clock bridge stopped", self.guard.evaluate(20.0)[1])

    def test_world_reset_restarts_observation(self):
        self.sample(0.0)
        self.sample(2.1)
        self.assertIsNone(self.sample(0.0, now=5.0))
        self.assertFalse(self.guard.ready)
        self.assertEqual(self.sample(2.1, now=7.1)[0], "READY")

    def test_invalid_values_and_quaternions(self):
        with self.assertRaises(ValueError):
            MODULE.SimulationGuard(0.0, startup_timeout=-1.0)
        with self.assertRaises(ValueError):
            MODULE.roll_pitch(0.0, 0.0, 0.0, 0.0)
        self.assertEqual(MODULE.roll_pitch(0.0, 0.0, 0.0, 1.0), (0.0, 0.0))
        self.guard.update_odometry(0.0, math.nan, 0.0, 0.0, 0.0)
        self.assertEqual(self.guard.evaluate(0.0)[0], "ERROR")


if __name__ == "__main__":
    unittest.main()
