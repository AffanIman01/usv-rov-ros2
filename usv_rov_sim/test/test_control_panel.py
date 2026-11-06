"""Simultaneous keys, smooth commands and immediate stops without ROS/display."""

import importlib.util
import math
from pathlib import Path
import sys
import unittest
from types import SimpleNamespace


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "control_panel.py"
SPEC = importlib.util.spec_from_file_location("usv_control_panel", SCRIPT)
PANEL = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = PANEL
SPEC.loader.exec_module(PANEL)


class DriveStateTest(unittest.TestCase):
    def setUp(self):
        self.state = PANEL.DriveState()
        self.now = 0.0
        self.state.advance(self.now)

    def tick(self, steps=1, dt=0.05):
        values = []
        for _ in range(steps):
            self.now += dt
            values.append(self.state.advance(self.now))
        return values

    def test_forward_and_steering_keys_combine_and_release_independently(self):
        self.state.press("w", "key:w")
        self.state.press("d", "key:d")
        self.assertEqual(self.state.target(), (0.6, -0.4))
        self.tick(30)
        self.assertAlmostEqual(self.state.surge, 0.6)
        self.assertAlmostEqual(self.state.yaw, -0.4)
        self.state.release("key:d")
        self.assertEqual(self.state.target(), (0.6, 0.0))
        self.tick(10)
        self.assertAlmostEqual(self.state.surge, 0.6)
        self.assertAlmostEqual(self.state.yaw, 0.0)
        self.state.press("a", "key:a")
        self.assertEqual(self.state.target(), (0.6, 0.4))

    def test_acceleration_and_release_deceleration_are_bounded(self):
        self.state.press("w", "key:w")
        self.state.press("a", "key:a")
        prior = (0.0, 0.0)
        for value in self.tick(30):
            self.assertLessEqual(value[0] - prior[0], 0.5 * 0.05 + 1e-10)
            self.assertLessEqual(value[1] - prior[1], 0.8 * 0.05 + 1e-10)
            prior = value
        self.state.release("key:w")
        self.state.release("key:a")
        for value in self.tick(20):
            self.assertLessEqual(prior[0] - value[0], 0.8 * 0.05 + 1e-10)
            self.assertLessEqual(prior[1] - value[1], 1.2 * 0.05 + 1e-10)
            self.assertGreaterEqual(value[0], 0.0)
            self.assertGreaterEqual(value[1], 0.0)
            prior = value
        self.assertEqual(prior, (0.0, 0.0))

    def test_reversals_go_through_zero(self):
        self.state.press("w", "key:w")
        self.tick(30)
        self.state.release("key:w")
        self.state.press("s", "key:s")
        values = [surge for surge, _ in self.tick(50)]
        self.assertIn(0.0, values)
        self.assertEqual(values[-1], -0.6)

    def test_opposing_keys_are_neutral_per_axis(self):
        for key in "wasd":
            self.state.press(key, "key:" + key)
        self.assertEqual(self.state.target(), (0.0, 0.0))
        self.state.release("key:a")
        self.assertEqual(self.state.target(), (0.0, -0.4))
        self.state.release("key:s")
        self.assertEqual(self.state.target(), (0.6, -0.4))

    def test_independent_mouse_and_keyboard_sources(self):
        self.state.press("w", "key:w")
        self.state.press("w", "mouse:w")
        self.state.release("mouse:w")
        self.assertEqual(self.state.target(), (0.6, 0.0))
        self.state.release("key:w")
        self.assertEqual(self.state.target(), (0.0, 0.0))

    def test_space_stops_immediately_and_repeat_cannot_rearm(self):
        self.state.press("w", "key:w")
        self.tick(30)
        self.state.press("space", "key:space")
        self.assertEqual((self.state.surge, self.state.yaw), (0.0, 0.0))
        self.assertFalse(self.state.press("w", "key:w"))
        self.assertFalse(self.state.press("d", "key:d"))
        self.tick(10)
        self.assertEqual((self.state.surge, self.state.yaw), (0.0, 0.0))
        self.state.release("key:space")
        self.assertFalse(self.state.press("w", "key:w"))
        self.state.release("key:w")
        self.assertTrue(self.state.press("w", "key:w"))
        self.tick(3)
        self.assertGreater(self.state.surge, 0.0)

    def test_focus_stop_forgets_unobservable_releases(self):
        self.state.press("w", "key:w")
        self.tick(10)
        self.state.stop(block_held=False)
        self.assertEqual(self.state.held, {})
        self.assertEqual(self.state.blocked, set())
        self.assertEqual((self.state.surge, self.state.yaw), (0.0, 0.0))
        self.tick(10)
        self.assertEqual(self.state.target(), (0.0, 0.0))
        self.assertTrue(self.state.press("w", "key:w"))

    def test_delayed_clock_reversal_or_invalid_tick_cuts_command(self):
        for next_time in (1.0, -1.0, math.nan):
            with self.subTest(next_time=next_time):
                state = PANEL.DriveState()
                state.advance(0.0)
                state.press("w", "key:w")
                state.advance(0.1)
                self.assertGreater(state.surge, 0.0)
                self.assertEqual(state.advance(next_time), (0.0, 0.0))
                self.assertTrue(state.interrupted)
                self.assertFalse(state.press("w", "key:w"))

    def test_limit_changes_remain_bounded(self):
        self.state.set_limits(speed=10, yaw=10)
        self.assertEqual((self.state.speed_limit, self.state.yaw_limit), (1.0, 0.6))
        self.state.set_limits(speed=-10, yaw=-10)
        self.assertEqual((self.state.speed_limit, self.state.yaw_limit), (0.1, 0.1))
        self.state.set_limits(speed=math.nan, yaw=math.inf)
        self.assertEqual((self.state.speed_limit, self.state.yaw_limit), (0.1, 0.1))

    def test_lift_is_independent_and_stops_with_drive(self):
        self.state.press("w", "key:w")
        self.state.press("d", "key:d")
        self.state.press("u", "key:u")
        self.assertEqual(self.state.target(), (0.6, -0.4))
        self.assertEqual(self.state.lift_velocity(), -1.0)
        self.state.press("j", "key:j")
        self.assertEqual(self.state.lift_velocity(), 0.0)
        self.state.release("key:u")
        self.assertEqual(self.state.lift_velocity(), 1.0)
        self.assertEqual(self.state.target(), (0.6, -0.4))
        self.state.press("space", "key:space")
        self.assertEqual(self.state.target(), (0.0, 0.0))
        self.assertEqual(self.state.lift_velocity(), 0.0)
        self.assertFalse(self.state.press("j", "key:j"))

    def test_lift_feedback_uses_studio_percentage_without_hiding_overshoot(self):
        feedback = PANEL.LiftFeedback()
        self.assertIn("Waiting", feedback.description(1.0))
        feedback.update(0.2717, 2.0)
        self.assertAlmostEqual(feedback.percent, 65.0)
        self.assertIn("65.0%", feedback.description(2.1))
        feedback.update(math.nan, 2.9)
        self.assertIn("stale", feedback.description(3.1))
        feedback.update(0.3, 4.0)
        self.assertIn("outside expected travel", feedback.description(4.1))

    def test_stale_or_invalid_feedback_is_not_shown_as_live(self):
        feedback = PANEL.Feedback()
        self.assertFalse(feedback.description(1.0)[1])
        feedback.update(0.5, -0.2, 2.0)
        self.assertTrue(feedback.description(2.1)[1])
        feedback.update(math.nan, 0, 3.0)
        self.assertIn("stale", feedback.description(3.1)[0])


class PanelStopTest(unittest.TestCase):
    def test_mouse_stop_publishes_drive_and_lift_zero_on_press(self):
        # Exercise the real press handler and real ROS-message construction,
        # with in-memory publisher sinks rather than a display or ROS graph.
        panel = PANEL.ControlPanel.__new__(PANEL.ControlPanel)
        panel.state = PANEL.DriveState(surge=0.5, yaw=-0.3)
        panel.state.press("w", "key:w")
        panel.state.press("j", "key:j")
        panel.ros = SimpleNamespace(ok=lambda: True)
        panel.twist_type = lambda: SimpleNamespace(linear=SimpleNamespace(x=0.0),
                                                   angular=SimpleNamespace(z=0.0))
        panel.float_type = lambda: SimpleNamespace(data=0.0)
        drive, lift, status = [], [], []
        panel.publisher = SimpleNamespace(publish=drive.append)
        panel.lift_publisher = SimpleNamespace(publish=lift.append)
        panel.status = SimpleNamespace(set=status.append)
        self.assertEqual(panel._stop_press("mouse:space"), "break")
        self.assertEqual(len(drive), 1)
        self.assertEqual((drive[0].linear.x, drive[0].angular.z, lift[0].data), (0.0, 0.0, 0.0))
        self.assertFalse(panel.state.press("j", "key:j"))
        self.assertFalse(panel.state.press("a", "key:a"))
        self.assertTrue(status)


    def test_focus_change_between_child_widgets_does_not_stop_motion(self):
        panel = PANEL.ControlPanel.__new__(PANEL.ControlPanel)
        panel.closed = False
        panel.focus_check = "pending"
        panel.state = PANEL.DriveState(surge=0.5, yaw=0.2)
        panel.state.press("w", "key:w")
        panel.state.press("u", "key:u")
        panel.root = SimpleNamespace()
        panel.root.focus_displayof = lambda: SimpleNamespace(winfo_toplevel=lambda: panel.root)
        panel._check_focus()
        self.assertEqual((panel.state.surge, panel.state.yaw), (0.5, 0.2))
        self.assertEqual(panel.state.lift_velocity(), -1.0)

    def test_leaving_window_cuts_drive_and_lift_without_stuck_keys(self):
        panel = PANEL.ControlPanel.__new__(PANEL.ControlPanel)
        panel.closed = False
        panel.focus_check = "pending"
        panel.state = PANEL.DriveState(surge=0.5, yaw=0.2)
        panel.state.press("w", "key:w")
        panel.state.press("j", "key:j")
        panel.root = SimpleNamespace(focus_displayof=lambda: None)
        panel.release_filter = SimpleNamespace(clear=lambda: None)
        commands, notices = [], []
        panel.publish = lambda *values: commands.append(values)
        panel.status = SimpleNamespace(set=notices.append)
        panel._check_focus()
        self.assertEqual(commands, [(0.0, 0.0)])
        self.assertEqual(panel.state.held, {})
        self.assertEqual(panel.state.lift_velocity(), 0.0)
        self.assertTrue(panel.state.press("w", "key:w"))
        self.assertTrue(notices)


class ReleaseFilterTest(unittest.TestCase):
    def test_auto_repeat_pairs_keep_both_keys_held(self):
        callbacks = {}
        state = PANEL.DriveState()
        next_token = [0]

        def schedule(callback):
            next_token[0] += 1
            callbacks[next_token[0]] = callback
            return next_token[0]

        def cancel(token):
            callbacks.pop(token)

        release = PANEL.KeyReleaseFilter(schedule, cancel, state.release)
        state.press("w", "key:w")
        state.press("d", "key:d")
        for _ in range(20):
            release.released("key:d")
            release.pressed("key:d")
            state.press("d", "key:d")
        self.assertEqual(callbacks, {})
        self.assertEqual(state.target(), (0.6, -0.4))
        release.released("key:d")
        callback = callbacks.pop(next(iter(callbacks)))
        callback()
        self.assertEqual(state.target(), (0.6, 0.0))
        release.released("key:w")
        release.clear()
        self.assertEqual(callbacks, {})


if __name__ == "__main__":
    unittest.main()
