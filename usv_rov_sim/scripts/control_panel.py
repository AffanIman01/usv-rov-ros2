#!/usr/bin/env python3
"""Focused-window, simultaneous-key controls for the simulated ME120."""

from dataclasses import dataclass, field
import math
import sys
import time


VELOCITY_TOPIC = "/sim/me120/cmd_vel"
ODOMETRY_TOPIC = "/sim/me120/odometry"
LIFT_VELOCITY_TOPIC = "/sim/me120/lift/velocity"
LIFT_POSITION_TOPIC = "/sim/me120/lift/position"
LIFT_REFERENCE_TRAVEL = 0.418
LIFT_MAX_PERCENT = 65.0
PUBLISH_PERIOD = 0.05
MAX_CONTROL_GAP = 0.4


def approach(current, target, amount):
    if current < target:
        return min(target, current + amount)
    return max(target, current - amount)


def ramp_axis(current, target, dt, acceleration, deceleration):
    # Reversals first decelerate to zero, then accelerate in the new direction.
    if current * target < 0:
        return approach(current, 0.0, deceleration * dt)
    rate = acceleration if abs(target) > abs(current) else deceleration
    return approach(current, target, rate * dt)


@dataclass
class DriveState:
    """Key/button state and command smoothing, independent of Tk and ROS."""

    speed_limit: float = 0.6
    yaw_limit: float = 0.4
    surge_acceleration: float = 0.5
    surge_deceleration: float = 0.8
    yaw_acceleration: float = 0.8
    yaw_deceleration: float = 1.2
    held: dict = field(default_factory=dict)
    blocked: set = field(default_factory=set)
    surge: float = 0.0
    yaw: float = 0.0
    last_tick: float | None = None
    interrupted: bool = False

    def press(self, control, source):
        if source in self.blocked or source in self.held:
            return False
        if control == "space":
            self.stop()
        elif "space" in self.held.values():
            self.blocked.add(source)
            return False
        self.held[source] = control
        return True

    def release(self, source):
        self.held.pop(source, None)
        self.blocked.discard(source)

    def stop(self, *, block_held=True):
        if block_held:
            self.blocked.update(self.held)
        else:
            self.blocked.clear()
        self.held.clear()
        self.surge = self.yaw = 0.0
        self.last_tick = None

    def set_limits(self, speed=None, yaw=None):
        if speed is not None and math.isfinite(speed):
            self.speed_limit = max(0.1, min(1.0, speed))
        if yaw is not None and math.isfinite(yaw):
            self.yaw_limit = max(0.1, min(0.6, yaw))

    def target(self):
        keys = set(self.held.values())
        return ((int("w" in keys) - int("s" in keys)) * self.speed_limit,
                (int("a" in keys) - int("d" in keys)) * self.yaw_limit)

    def lift_velocity(self):
        keys = set(self.held.values())
        return float("j" in keys) - float("u" in keys)

    def advance(self, now):
        self.interrupted = False
        if not math.isfinite(now):
            self.stop()
            self.interrupted = True
            return self.surge, self.yaw
        if self.last_tick is None:
            self.last_tick = now
            return self.surge, self.yaw
        elapsed = now - self.last_tick
        self.last_tick = now
        if elapsed < 0 or elapsed > MAX_CONTROL_GAP:
            self.stop()
            self.last_tick = now
            self.interrupted = True
            return self.surge, self.yaw
        dt = min(elapsed, 0.1)
        target_surge, target_yaw = self.target()
        self.surge = ramp_axis(self.surge, target_surge, dt,
                               self.surge_acceleration, self.surge_deceleration)
        self.yaw = ramp_axis(self.yaw, target_yaw, dt,
                             self.yaw_acceleration, self.yaw_deceleration)
        return self.surge, self.yaw


@dataclass
class Feedback:
    received_at: float | None = None
    surge: float = 0.0
    yaw: float = 0.0

    def update(self, surge, yaw, now):
        if all(math.isfinite(value) for value in (surge, yaw, now)):
            self.surge, self.yaw, self.received_at = surge, yaw, now

    def description(self, now):
        if self.received_at is None:
            return "Waiting for vessel odometry", False
        age = max(0.0, now - self.received_at)
        if age >= 1.0:
            return f"Feedback stale ({age:.1f}s). Check the simulation terminal.", False
        return f"Measured  {self.surge:+.2f} m/s   |   {self.yaw:+.2f} rad/s", True


@dataclass
class LiftFeedback:
    received_at: float | None = None
    position: float = 0.0

    def update(self, position, now):
        if math.isfinite(position) and math.isfinite(now):
            self.position, self.received_at = position, now

    @property
    def percent(self):
        return self.position / LIFT_REFERENCE_TRAVEL * 100.0

    def description(self, now):
        if self.received_at is None:
            return "Waiting for lift position"
        age = max(0.0, now - self.received_at)
        value = f"Lift {self.percent:.1f}% down · {self.position:.3f} m"
        if age >= 1.0:
            return value + f" · feedback stale ({age:.1f}s)"
        if self.percent < -0.5 or self.percent > LIFT_MAX_PERCENT + 0.5:
            return value + " · outside expected travel"
        return value


class KeyReleaseFilter:
    """Suppress X11's synthetic release/press pair during key auto-repeat.

    A real release is delivered when Tk becomes idle. A repeat press received
    first cancels that pending release, preserving the independently held keys.
    """

    def __init__(self, schedule, cancel, release):
        self.schedule, self.cancel, self.release = schedule, cancel, release
        self.pending = {}

    def pressed(self, source):
        token = self.pending.pop(source, None)
        if token is not None:
            self.cancel(token)

    def released(self, source):
        self.pressed(source)

        def deliver():
            self.pending.pop(source, None)
            self.release(source)

        self.pending[source] = self.schedule(deliver)

    def clear(self):
        for token in self.pending.values():
            self.cancel(token)
        self.pending.clear()


class ControlPanel:
    """Tk and ROS integration; only this window receives keyboard events."""

    def __init__(self, root, tk, ttk, ros, node, twist_type, float_type):
        self.root, self.tk, self.ttk = root, tk, ttk
        self.ros, self.node, self.twist_type = ros, node, twist_type
        self.float_type = float_type
        self.publisher = node.create_publisher(twist_type, VELOCITY_TOPIC, 1)
        self.lift_publisher = node.create_publisher(float_type, LIFT_VELOCITY_TOPIC, 1)
        self.state, self.feedback = DriveState(), Feedback()
        self.lift_feedback = LiftFeedback()
        self.closed = False
        self.next_publish = time.monotonic()
        self.release_filter = KeyReleaseFilter(root.after_idle, root.after_cancel,
                                               self.state.release)
        self.focus_check = None
        self.motion_buttons = {}
        self.lift_buttons = {}
        self.status = tk.StringVar(value="Click this window, then hold W + A or W + D.")
        self.command_text = tk.StringVar(value="Command  +0.00 m/s   |   +0.00 rad/s")
        self.feedback_text = tk.StringVar(value="Waiting for vessel odometry")
        self.connection_text = tk.StringVar(value="Waiting for ROS command subscribers")
        self.lift_text = tk.StringVar(value="Waiting for lift position")
        self.lift_progress = tk.DoubleVar(value=0.0)
        self.speed = tk.DoubleVar(value=self.state.speed_limit)
        self.turn = tk.DoubleVar(value=self.state.yaw_limit)
        self._build()
        root.bind("<KeyPress>", self._key_press)
        root.bind("<KeyRelease>", self._key_release)
        root.bind("<FocusOut>", self._focus_out)
        root.bind("<ButtonRelease-1>", self._mouse_release)
        root.protocol("WM_DELETE_WINDOW", self.close)
        root.after(50, root.focus_force)
        root.after(0, self.tick)

    def _build(self):
        root, ttk = self.root, self.ttk
        root.title("ME120 · Simulation controls")
        root.minsize(640, 570)
        root.configure(background="#f3f5f7")
        style = ttk.Style(root)
        if "clam" in style.theme_names():
            style.theme_use("clam")
        style.configure("TFrame", background="#f3f5f7")
        style.configure("TLabel", background="#f3f5f7", foreground="#163044", font=("Sans", 10))
        style.configure("Title.TLabel", font=("Sans", 18, "bold"))
        style.configure("Hint.TLabel", foreground="#536776")
        style.configure("Drive.TButton", font=("Sans", 12, "bold"), padding=(12, 16))
        style.configure("Held.Drive.TButton", background="#c2e3f3")
        style.configure("Stop.TButton", foreground="#9f2430", font=("Sans", 11, "bold"), padding=(12, 12))
        outer = ttk.Frame(root, padding=22)
        outer.pack(fill="both", expand=True)
        ttk.Label(outer, text="ME120 controls", style="Title.TLabel").pack(anchor="w")
        ttk.Label(outer, text="Hold W + A / D to steer while moving. Release to slow down.",
                  style="Hint.TLabel").pack(anchor="w", pady=(5, 18))
        controls = ttk.Frame(outer)
        controls.pack(fill="x")
        pad = ttk.Frame(controls)
        pad.grid(row=0, column=0, sticky="nw", padx=(0, 25))
        for key, label, row, col in (("w", "W  Forward", 0, 1),
                                     ("a", "A  Left", 1, 0),
                                     ("s", "S  Reverse", 1, 1),
                                     ("d", "D  Right", 1, 2)):
            button = ttk.Button(pad, text=label, style="Drive.TButton", takefocus=False)
            button.grid(row=row, column=col, padx=3, pady=3, sticky="nsew")
            button.bind("<ButtonPress-1>", lambda event, k=key: self._mouse_press(k))
            button.bind("<ButtonRelease-1>", lambda event, k=key: self.state.release("mouse:" + k))
            button.bind("<Leave>", lambda event, k=key: self.state.release("mouse:" + k))
            self.motion_buttons[key] = button
        stop_button = ttk.Button(pad, text="SPACE  ·  Stop drive and lift", style="Stop.TButton",
                                 command=lambda: self.stop("Stopped. Release held keys before driving again."),
                                 takefocus=False)
        stop_button.grid(row=2, column=0, columnspan=3, sticky="ew", pady=(10, 0))
        stop_button.bind("<ButtonPress-1>", lambda event: self._stop_press("mouse:space"))
        stop_button.bind("<ButtonRelease-1>", lambda event: self.state.release("mouse:space"))
        limits = ttk.Frame(controls)
        limits.grid(row=0, column=1, sticky="new")
        controls.columnconfigure(1, weight=1)
        ttk.Label(limits, text="Speed limit (m/s)").pack(anchor="w")
        self.tk.Scale(limits, from_=0.1, to=1.0, resolution=0.1, orient="horizontal",
                      variable=self.speed, background="#f3f5f7", highlightthickness=0,
                      command=lambda value: self.state.set_limits(speed=float(value)),
                      length=170).pack(fill="x", pady=(0, 12))
        ttk.Label(limits, text="Turn limit (rad/s)").pack(anchor="w")
        self.tk.Scale(limits, from_=0.1, to=0.6, resolution=0.1, orient="horizontal",
                      variable=self.turn, background="#f3f5f7", highlightthickness=0,
                      command=lambda value: self.state.set_limits(yaw=float(value)),
                      length=170).pack(fill="x")
        ttk.Separator(outer).pack(fill="x", pady=16)
        lift = ttk.Frame(outer)
        lift.pack(fill="x")
        lift_controls = ttk.Frame(lift)
        lift_controls.pack(side="left", padx=(0, 20))
        for key, label in (("u", "U  Raise lift"), ("j", "J  Lower lift")):
            button = ttk.Button(lift_controls, text=label, style="Drive.TButton", takefocus=False)
            button.pack(side="left", padx=(0, 6))
            button.bind("<ButtonPress-1>", lambda event, k=key: self._mouse_press(k))
            button.bind("<ButtonRelease-1>", lambda event, k=key: self.state.release("mouse:" + k))
            button.bind("<Leave>", lambda event, k=key: self.state.release("mouse:" + k))
            self.lift_buttons[key] = button
        lift_state = ttk.Frame(lift)
        lift_state.pack(side="left", fill="x", expand=True)
        ttk.Label(lift_state, textvariable=self.lift_text).pack(anchor="w")
        ttk.Progressbar(lift_state, maximum=LIFT_MAX_PERCENT, variable=self.lift_progress,
                        mode="determinate").pack(fill="x", pady=5)
        ttk.Label(lift_state, text="0% raised · 65% fully lowered", style="Hint.TLabel").pack(anchor="w")
        ttk.Separator(outer).pack(fill="x", pady=16)
        ttk.Label(outer, textvariable=self.command_text, font=("Monospace", 11)).pack(anchor="w", pady=3)
        ttk.Label(outer, textvariable=self.feedback_text).pack(anchor="w", pady=3)
        ttk.Label(outer, textvariable=self.connection_text, style="Hint.TLabel").pack(anchor="w", pady=3)
        ttk.Label(outer, textvariable=self.status, wraplength=600,
                  style="Hint.TLabel").pack(anchor="w", pady=(14, 0))
        ttk.Label(outer, text="Space stops drive and lift · Q / Esc closes · Switching windows stops motion",
                  style="Hint.TLabel", wraplength=600).pack(anchor="w", pady=(8, 0))

    def publish(self, surge, yaw, lift=0.0):
        if self.ros.ok():
            message = self.twist_type()
            message.linear.x, message.angular.z = surge, yaw
            self.publisher.publish(message)
            lift_message = self.float_type()
            lift_message.data = lift
            self.lift_publisher.publish(lift_message)

    def stop(self, reason, *, block_held=True):
        self.state.stop(block_held=block_held)
        self.publish(0.0, 0.0)
        self.status.set(reason)

    def _key_press(self, event):
        key = event.keysym.lower()
        source = "key:" + key
        self.release_filter.pressed(source)
        if key in ("w", "a", "s", "d", "u", "j"):
            if self.state.press(key, source):
                self.status.set("Lift moving · release U/J to hold, or Space to stop all motion."
                                if key in ("u", "j") else
                                "Driving · release keys to slow down, or Space to cut thrust.")
            return "break"
        if key == "space":
            return self._stop_press(source)
        if key in ("q", "escape"):
            self.close()
            return "break"
        if key in ("plus", "equal", "minus"):
            direction = -0.1 if key == "minus" else 0.1
            self.state.set_limits(self.state.speed_limit + direction,
                                  self.state.yaw_limit + direction)
            self.speed.set(self.state.speed_limit)
            self.turn.set(self.state.yaw_limit)
            return "break"
        return None

    def _key_release(self, event):
        key = event.keysym.lower()
        if key in ("w", "a", "s", "d", "u", "j", "space"):
            self.release_filter.released("key:" + key)
            return "break"
        return None

    def _stop_press(self, source):
        if self.state.press("space", source):
            self.publish(0.0, 0.0)
            self.status.set("Stopped. Release held keys before driving again.")
        return "break"

    def _mouse_press(self, key):
        self.root.focus_set()
        self.state.press(key, "mouse:" + key)
        self.status.set("Hold to move the lift; release to hold position."
                        if key in ("u", "j") else
                        "Hold a direction button, or combine W/S with A/D on the keyboard.")
        return "break"

    def _mouse_release(self, event):
        for key in ("w", "a", "s", "d", "u", "j", "space"):
            self.state.release("mouse:" + key)

    def _focus_out(self, event):
        if self.closed:
            return
        if self.focus_check is not None:
            self.root.after_cancel(self.focus_check)
        self.focus_check = self.root.after_idle(self._check_focus)

    def _check_focus(self):
        self.focus_check = None
        if self.closed:
            return
        focused = self.root.focus_displayof()
        if focused is None or focused.winfo_toplevel() != self.root:
            self.release_filter.clear()
            # Key releases outside this window are not observable. Forget held
            # state so returning to the window requires a new key event.
            self.stop("Window inactive · click here and press a movement key to drive.",
                      block_held=False)

    def tick(self):
        if self.closed:
            return
        try:
            if not self.ros.ok():
                self.close()
                return
            for _ in range(8):
                self.ros.spin_once(self.node, timeout_sec=0.0)
            now = time.monotonic()
            if now >= self.next_publish:
                surge, yaw = self.state.advance(now)
                self.publish(surge, yaw, self.state.lift_velocity())
                if self.state.interrupted:
                    self.status.set("Control timer paused · stopped. Release keys, then press again.")
                self.next_publish = now + PUBLISH_PERIOD
                self.command_text.set(f"Command   {surge:+.2f} m/s   |   {yaw:+.2f} rad/s")
                self.feedback_text.set(self.feedback.description(now)[0])
                self.lift_text.set(self.lift_feedback.description(now))
                self.lift_progress.set(max(0.0, min(LIFT_MAX_PERCENT, self.lift_feedback.percent)))
                drive_listeners = self.publisher.get_subscription_count()
                lift_listeners = self.lift_publisher.get_subscription_count()
                self.connection_text.set(
                    f"ROS subscribers · drive: {drive_listeners} · lift: {lift_listeners}")
                keys = set(self.state.held.values())
                for key, button in {**self.motion_buttons, **self.lift_buttons}.items():
                    button.configure(style="Held.Drive.TButton" if key in keys else "Drive.TButton")
            self.root.after(15, self.tick)
        except Exception as error:
            print(f"Control panel stopped: {error}", file=sys.stderr, flush=True)
            self.close()

    def close(self):
        if self.closed:
            return
        self.closed = True
        self.state.stop()
        try:
            # A closing window sends an immediate zero; the independent Gazebo
            # command watchdog also stops thrust if this process disappears.
            for _ in range(3):
                self.publish(0.0, 0.0)
                self.ros.spin_once(self.node, timeout_sec=0.0)
                time.sleep(0.01)
        except Exception:
            pass
        self.release_filter.clear()
        if self.focus_check is not None:
            self.root.after_cancel(self.focus_check)
            self.focus_check = None
        self.root.destroy()


def main(args=None):
    args = sys.argv[1:] if args is None else args
    if args in (["--help"], ["-h"]):
        print("ME120 control window: hold W/S + A/D together; U raises/J lowers lift; "
              "Space stops drive and lift; Q/Esc exits.")
        return 0
    try:
        import tkinter as tk
        from tkinter import ttk
    except ImportError:
        print("The control window needs Tk: sudo apt install python3-tk", file=sys.stderr)
        return 2
    try:
        root = tk.Tk()
    except tk.TclError as error:
        print(f"Cannot open the control window: {error}\nRun it inside the Ubuntu desktop session, "
              "or use tools/run_sim.sh --keyboard-terminal for the terminal fallback.", file=sys.stderr)
        return 2
    ros = node = panel = None
    try:
        import rclpy as ros
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from std_msgs.msg import Float64
        ros.init(args=args)
        node = Node("me120_control_panel")
        panel = ControlPanel(root, tk, ttk, ros, node, Twist, Float64)
        node.create_subscription(Odometry, ODOMETRY_TOPIC,
                                 lambda message: panel.feedback.update(
                                     message.twist.twist.linear.x,
                                     message.twist.twist.angular.z, time.monotonic()),
                                 qos_profile_sensor_data)
        node.create_subscription(Float64, LIFT_POSITION_TOPIC,
                                 lambda message: panel.lift_feedback.update(message.data,
                                                                            time.monotonic()),
                                 qos_profile_sensor_data)
        root.mainloop()
        return 0
    except KeyboardInterrupt:
        return 0
    except Exception as error:
        print(f"Control panel could not start: {error}", file=sys.stderr)
        return 1
    finally:
        if panel is not None:
            panel.close()
        else:
            root.destroy()
        if node is not None:
            node.destroy_node()
        if ros is not None and ros.ok():
            ros.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
