"""Exercise real ROS launch exit events without starting ROS or Gazebo transport."""

import importlib.util
from pathlib import Path
import unittest

from launch import LaunchContext
from launch.actions import ExecuteProcess, LogInfo, Shutdown
from launch.event_handlers import OnProcessExit
from launch.events.process import ProcessExited


SOURCE = Path(__file__).resolve().parents[1] / "launch" / "sim.launch.py"
SPEC = importlib.util.spec_from_file_location("usv_sim_launch", SOURCE)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class LaunchExitTest(unittest.TestCase):
    def handle(self, component, code, stop_simulation, shutting_down=False):
        context = LaunchContext()
        context._set_is_shutdown(shutting_down)
        process = ExecuteProcess(cmd=["unused-test-process"])
        event = ProcessExited(action=process, name="test-process", cmd=[],
                              cwd=None, env=None, pid=123, returncode=code)
        handler = OnProcessExit(target_action=process, on_exit=MODULE._process_exit(
            component, stop_simulation=stop_simulation))
        self.assertTrue(handler.matches(event))
        actions = handler.handle(event, context)
        messages = ["".join(context.perform_substitution(s) for s in action.msg)
                    for action in actions if isinstance(action, LogInfo)]
        return actions, "\n".join(messages)

    def test_monitor_fault_leaves_scene_open(self):
        actions, message = self.handle("Simulation monitor", 1, False)
        self.assertFalse(any(isinstance(a, Shutdown) for a in actions))
        self.assertIn("Simulation monitor", message)
        self.assertIn("code 1", message)
        self.assertIn("Gazebo stays open", message)

    def test_simulator_and_bridge_failure_stop_remaining_processes(self):
        for component, code in (("Gazebo", -11), ("ROS/Gazebo bridge", 255)):
            with self.subTest(component=component):
                actions, message = self.handle(component, code, True)
                self.assertTrue(any(isinstance(a, Shutdown) for a in actions))
                self.assertIn(component, message)
                self.assertIn(f"code {code}", message)
                self.assertIn("ERROR", message)

    def test_normal_simulator_close_is_not_reported_as_error(self):
        actions, message = self.handle("Gazebo", 0, True)
        self.assertTrue(any(isinstance(a, Shutdown) for a in actions))
        self.assertNotIn("ERROR", message)

    def test_coordinated_shutdown_does_not_generate_new_failures(self):
        for stop in (True, False):
            actions, message = self.handle("test", -2, stop, shutting_down=True)
            self.assertEqual(actions, [])
            self.assertEqual(message, "")


if __name__ == "__main__":
    unittest.main()
