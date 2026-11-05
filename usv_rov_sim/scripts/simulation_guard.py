#!/usr/bin/env python3
"""Passive startup and flotation checks for the simulated ME120.

This node never publishes movement commands. Readiness confirms advancing
simulation time and stable odometry, not that a thruster command was accepted.
"""

import argparse
from dataclasses import dataclass
import math
import sys
import time


def roll_pitch(x, y, z, w):
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm < 1e-9:
        raise ValueError("Odometry contains an invalid orientation quaternion")
    x, y, z, w = (value / norm for value in (x, y, z, w))
    return (math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)),
            math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x)))))


@dataclass
class SimulationGuard:
    """Pure state machine; wall time handles delivery, sim time handles physics."""

    started_at: float
    startup_timeout: float = 120.0
    feedback_timeout: float = 10.0
    settle_time: float = 2.0
    max_drop: float = 0.35
    max_tilt: float = 0.8
    fault_duration: float = 1.0

    def __post_init__(self):
        values = (self.startup_timeout, self.feedback_timeout, self.settle_time,
                  self.max_drop, self.max_tilt, self.fault_duration)
        if not math.isfinite(self.started_at) or any(
                not math.isfinite(value) or value <= 0 for value in values):
            raise ValueError("Guard limits must be finite and positive")
        self.clock = self.first_clock = self.last_clock_at = None
        self.last_clock_advance_at = self.last_odom_at = None
        self.odom_stamp = self.first_odom_stamp = self.initial_z = None
        self.z = self.roll = self.pitch = 0.0
        self.bad_since = None
        self.ready = False
        self.failure = None
        self.last_notice = None

    def update_clock(self, stamp, now):
        if not math.isfinite(stamp):
            self.failure = "The /clock bridge delivered a non-finite timestamp."
            return
        if self.clock is not None and stamp < self.clock - 1e-6:
            # A user world reset starts a new flotation observation window.
            self.first_clock = stamp
            self.first_odom_stamp = self.initial_z = self.bad_since = None
            self.ready = False
            self.last_notice = None
            self.last_clock_advance_at = now
        if self.clock is None or stamp > self.clock + 1e-9:
            self.last_clock_advance_at = now
        if self.first_clock is None:
            self.first_clock = stamp
        self.clock, self.last_clock_at = stamp, now

    def update_odometry(self, stamp, z, roll, pitch, now):
        if any(not math.isfinite(value) for value in (stamp, z, roll, pitch)):
            self.failure = "The odometry bridge delivered a non-finite vessel pose."
            return
        if self.odom_stamp is not None and stamp < self.odom_stamp - 1e-6:
            self.first_odom_stamp = self.initial_z = self.bad_since = None
            self.ready = False
            self.last_notice = None
        if self.initial_z is None:
            self.initial_z, self.first_odom_stamp = z, stamp
        self.odom_stamp, self.last_odom_at = stamp, now
        self.z, self.roll, self.pitch = z, roll, pitch
        drop = self.initial_z - z
        bad = drop > self.max_drop or max(abs(roll), abs(pitch)) > self.max_tilt
        if not bad:
            self.bad_since = None
        elif self.bad_since is None:
            self.bad_since = stamp
        elif stamp - self.bad_since >= self.fault_duration:
            self.failure = (
                f"USV flotation check failed: drop={drop:.2f} m, "
                f"roll={math.degrees(roll):.1f} deg, pitch={math.degrees(pitch):.1f} deg. "
                "Check the Gazebo log above for SurfaceBuoyancy/plugin errors. "
                "Restart with tools/run_sim.sh after correcting the reported error."
            )

    def evaluate(self, now):
        """Return a new (level, message) notice, or None; errors remain sticky."""
        if self.failure:
            return "ERROR", self.failure
        missing = []
        if self.last_clock_at is None:
            missing.append("/clock")
        if self.last_odom_at is None:
            missing.append("/sim/me120/odometry")
        if missing:
            if now - self.started_at > self.startup_timeout:
                self.failure = (
                    "Startup timed out waiting for " + " and ".join(missing) + ". "
                    "Check Gazebo and ros_gz_bridge errors above; confirm both "
                    "terminals use the same ROS_DOMAIN_ID and this checkout."
                )
                return "ERROR", self.failure
            return None

        clock_live = now - self.last_clock_advance_at <= self.feedback_timeout
        odom_live = now - self.last_odom_at <= self.feedback_timeout
        clock_received = now - self.last_clock_at <= self.feedback_timeout
        if clock_live and not odom_live:
            self.failure = (
                "Odometry stopped while the Gazebo clock is advancing. "
                "Check the odometry publisher and ros_gz_bridge errors above."
            )
            return "ERROR", self.failure
        if not clock_received and odom_live:
            self.failure = (
                "The /clock bridge stopped while odometry is still arriving. "
                "Check the selected world's clock mapping in ros_gz_bridge."
            )
            return "ERROR", self.failure
        if not clock_live:
            notice = ("WARNING", "Simulation time is not advancing: Gazebo may be paused, "
                      "or its server/bridge has stopped. Press Play if paused; "
                      "check the Gazebo terminal otherwise.")
        elif (not self.ready and self.bad_since is None and
              self.first_odom_stamp is not None and
              abs(self.z - self.initial_z) < 0.15 and
              max(abs(self.roll), abs(self.pitch)) < 0.15 and
              self.clock - self.first_clock >= self.settle_time and
              self.odom_stamp - self.first_odom_stamp >= self.settle_time):
            self.ready = True
            notice = ("READY", "Clock and vessel odometry are live; flotation feedback is stable. "
                      "Use the keyboard terminal to test movement. This check does not send thrust.")
        elif self.last_notice and self.last_notice[0] == "WARNING":
            notice = ("INFO", "Simulation time is advancing again.")
        else:
            return None
        if notice == self.last_notice:
            return None
        self.last_notice = notice
        return notice


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--startup-timeout", type=float, default=120.0)
    parser.add_argument("--feedback-timeout", type=float, default=10.0)
    args, ros_args = parser.parse_known_args(argv)
    try:
        guard = SimulationGuard(time.monotonic(), args.startup_timeout, args.feedback_timeout)
    except ValueError as error:
        parser.error(str(error))

    node = ros = None
    initialized = False
    try:
        # Lazy ROS imports let the state-machine tests run without a ROS runtime.
        import rclpy as ros
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from nav_msgs.msg import Odometry
        from rosgraph_msgs.msg import Clock

        ros.init(args=ros_args)
        initialized = True
        node = Node("simulation_guard")

        def on_clock(message):
            guard.update_clock(message.clock.sec + message.clock.nanosec * 1e-9,
                               time.monotonic())

        def on_odometry(message):
            pose, stamp = message.pose.pose, message.header.stamp
            orientation = pose.orientation
            try:
                roll, pitch = roll_pitch(orientation.x, orientation.y,
                                         orientation.z, orientation.w)
            except ValueError as error:
                guard.failure = str(error)
                return
            guard.update_odometry(stamp.sec + stamp.nanosec * 1e-9,
                                  pose.position.z, roll, pitch, time.monotonic())

        node.create_subscription(Clock, "/clock", on_clock, qos_profile_sensor_data)
        node.create_subscription(Odometry, "/sim/me120/odometry", on_odometry,
                                 qos_profile_sensor_data)
        print("Simulation guard: waiting for advancing clock and vessel odometry.", flush=True)
        while ros.ok():
            ros.spin_once(node, timeout_sec=0.2)
            notice = guard.evaluate(time.monotonic())
            if notice:
                level, message = notice
                print(f"Simulation guard {level}: {message}",
                      file=sys.stderr if level == "ERROR" else sys.stdout, flush=True)
                if level == "ERROR":
                    return 1
        return 0
    except KeyboardInterrupt:
        return 0
    except Exception as error:
        if initialized and not ros.ok():
            return 0
        print(f"Simulation guard ERROR: {error}", file=sys.stderr, flush=True)
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        if ros is not None and ros.ok():
            ros.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
