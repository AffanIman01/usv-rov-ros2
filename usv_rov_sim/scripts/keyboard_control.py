#!/usr/bin/env python3
"""Momentary terminal controls for the simulated ME120; no hardware driver."""

from dataclasses import dataclass
import math
import os
import select
import sys
import termios
import time
import tty


TOPIC = "/sim/me120/cmd_vel"
PUBLISH_PERIOD = 0.05
HELP = """ME120 simulation keyboard control
Keep this terminal focused. Hold/repeat a movement key:
  W forward    S reverse    A turn left    D turn right
  Space stop   + / - speed  Q quit        Ctrl+C quit
Movement expires 0.35 s after the last movement key; terminal key repeat is required.
This publishes simulated velocity targets on /sim/me120/cmd_vel.
The status line shows the requested motion and received odometry separately.
"""


class TerminalKeyDecoder:
    """Discard ANSI escape sequences, including ones split across reads.

    Arrow and function keys contain ordinary letters after ESC/CSI or ESC/SS3.
    They must not become WASD commands. Other Alt-prefixed keys are discarded
    as well; only unmodified characters reach the command state.
    """

    def __init__(self):
        self._state = "normal"

    def feed(self, text):
        keys = []
        for character in text:
            if character == "\x1b":
                self._state = "escape"
            elif self._state == "normal":
                keys.append(character)
            elif self._state == "escape":
                self._state = "sequence" if character in ("[", "O") else "normal"
            elif "@" <= character <= "~":
                # CSI/SS3 final byte: all parameters and this byte are ignored.
                self._state = "normal"
            elif not (" " <= character <= "?"):
                # Malformed sequence: discard this character, then recover.
                self._state = "normal"
        return keys


@dataclass
class KeyboardCommand:
    """Pure input state, independent of ROS and terminal handling."""

    speed: float = 0.6
    yaw_speed: float = 0.4
    hold_timeout: float = 0.35
    _surge: float = 0.0
    _yaw: float = 0.0
    _last_motion: float | None = None

    def stop(self):
        self._surge = self._yaw = 0.0
        self._last_motion = None

    def apply_key(self, key, now):
        key = key.lower()
        motions = {"w": (1.0, 0.0), "s": (-1.0, 0.0),
                   "a": (0.0, 1.0), "d": (0.0, -1.0)}
        if key in motions:
            self._surge, self._yaw = motions[key]
            self._last_motion = now
            return "motion"
        if key in ("+", "=", "-"):
            delta = -0.1 if key == "-" else 0.1
            self.speed = round(max(0.1, min(1.0, self.speed + delta)), 2)
            self.yaw_speed = round(max(0.1, min(0.6, self.yaw_speed + delta)), 2)
            # Changing speed does not prolong an earlier motion command.
            return "speed"
        self.stop()
        return "quit" if key == "q" else "stop"

    def command(self, now):
        if (self._last_motion is None or
                now - self._last_motion >= self.hold_timeout or
                now < self._last_motion):
            return 0.0, 0.0
        return self._surge * self.speed, self._yaw * self.yaw_speed


@dataclass
class MotionFeedback:
    """Track fresh simulation feedback without treating it as a command ACK."""

    received_at: float | None = None
    surge: float = 0.0
    yaw: float = 0.0

    def update(self, surge, yaw, now):
        if all(math.isfinite(value) for value in (surge, yaw, now)):
            self.surge, self.yaw, self.received_at = surge, yaw, now

    def describe(self, now):
        if self.received_at is None:
            return "odometry: waiting"
        age = max(0.0, now - self.received_at)
        freshness = "live" if age < 1.0 else "STALE"
        return (f"measured {self.surge:+.2f} m/s, {self.yaw:+.2f} rad/s; "
                f"odometry: {freshness} ({age:.1f}s)")


def main(args=None):
    args = sys.argv[1:] if args is None else args
    if args in (["--help"], ["-h"]):
        print(HELP)
        return 0
    if not sys.stdin.isatty():
        print("Keyboard control needs an interactive terminal. Run it in a second terminal.",
              file=sys.stderr)
        return 2

    # Lazy imports keep the input logic testable without a ROS installation.
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry

    node = None
    publisher = None
    fd = sys.stdin.fileno()
    previous_terminal = termios.tcgetattr(fd)
    state = KeyboardCommand()
    decoder = TerminalKeyDecoder()
    feedback = MotionFeedback()
    exit_code = 0
    interactive_status = sys.stdout.isatty()

    def publish(surge, yaw):
        message = Twist()
        message.linear.x = surge
        message.angular.z = yaw
        publisher.publish(message)

    def on_odometry(message):
        feedback.update(message.twist.twist.linear.x,
                        message.twist.twist.angular.z, time.monotonic())

    try:
        rclpy.init(args=args)
        node = Node("me120_keyboard")
        publisher = node.create_publisher(Twist, TOPIC, 1)
        node.create_subscription(Odometry, "/sim/me120/odometry", on_odometry,
                                 qos_profile_sensor_data)
        tty.setcbreak(fd)  # Keep terminal signal handling, including Ctrl+C.
        print(HELP, flush=True)
        print(f"Speed: {state.speed:.1f} m/s; turn rate: {state.yaw_speed:.1f} rad/s", flush=True)
        print("If the command stays zero, click this terminal and hold W. "
              "Waiting for ROS subscribers means the bridge is not connected yet.", flush=True)
        next_publish = time.monotonic()
        next_status = next_publish
        previous_connection = None
        quit_requested = False
        while rclpy.ok() and not quit_requested:
            ready, _, _ = select.select([fd], [], [], max(0.0, next_publish - time.monotonic()))
            if ready:
                data = os.read(fd, 64)
                if not data:  # Closed terminal: leave the loop and stop.
                    break
                for key in decoder.feed(data.decode(errors="ignore")):
                    action = state.apply_key(key, time.monotonic())
                    if action == "speed":
                        if interactive_status:
                            print()
                        print(f"Speed: {state.speed:.1f} m/s; turn rate: {state.yaw_speed:.1f} rad/s",
                              flush=True)
                    elif action == "quit":
                        quit_requested = True
                        break
            now = time.monotonic()
            if now >= next_publish:
                publish(*state.command(now))
                rclpy.spin_once(node, timeout_sec=0.0)
                next_publish = now + PUBLISH_PERIOD
            if now >= next_status:
                listeners = publisher.get_subscription_count()
                connection = (f"ROS subscribers: {listeners}" if listeners else
                              "waiting for ROS subscribers")
                surge, yaw = state.command(now)
                line = (f"command {surge:+.2f} m/s, {yaw:+.2f} rad/s | "
                        f"{connection} | {feedback.describe(now)}")
                if interactive_status:
                    print(f"\r{line:<145}", end="", flush=True)
                elif connection != previous_connection:
                    print(line, flush=True)
                previous_connection = connection
                next_status = now + 0.25
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as error:
        print(f"Keyboard control stopped: {error}", file=sys.stderr)
        exit_code = 1
    finally:
        if interactive_status:
            print()
        # Send stop while the ROS context is usable. If it has already shut
        # down, the Gazebo controller's independent command watchdog stops it.
        if publisher is not None and rclpy.ok():
            try:
                for _ in range(3):
                    publish(0.0, 0.0)
                    time.sleep(0.02)
            except Exception:
                pass
        termios.tcsetattr(fd, termios.TCSADRAIN, previous_terminal)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
