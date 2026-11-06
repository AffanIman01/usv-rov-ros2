#!/usr/bin/env python3
"""Bounded end-to-end test of the real ROS bridge and headless pool server.

Requires a sourced ROS 2 Jazzy installation and built usv_rov_sim package.
This starts and cleans up its own launch process group. It does not use the
network-free ECS harness or substitute forces for production system plugins.
"""

import argparse
from dataclasses import dataclass
import importlib.util
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid


def quaternion_rpy(x, y, z, w):
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm < 1e-9:
        raise ValueError("Odometry contains an invalid orientation quaternion")
    x, y, z, w = (value / norm for value in (x, y, z, w))
    return (
        math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)),
        math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x)))),
        math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)),
    )


@dataclass
class State:
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float
    unwrapped_yaw: float
    speed: float
    surge: float
    yaw_rate: float


def stop_process_group(process):
    """Signal only the session created by this test, including its children."""
    for sig, timeout in ((signal.SIGINT, 8), (signal.SIGTERM, 3), (signal.SIGKILL, 2)):
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            return
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            process.poll()  # Reap the launch parent, which may exit before children.
            try:
                os.killpg(process.pid, 0)
            except ProcessLookupError:
                return
            time.sleep(0.05)
    process.poll()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--log", type=Path, default=Path("/tmp/usv-ros-bridge-smoke.log"))
    parser.add_argument("--output", type=Path, default=Path("/tmp/usv-ros-bridge-smoke.json"))
    parser.add_argument("--startup-timeout", type=float, default=60.0)
    parser.add_argument("--timeout", type=float, default=150.0)
    parser.add_argument("--domain-id", type=int, default=20 + os.getpid() % 60)
    parser.add_argument("--world", choices=("pool", "sydney_regatta"), default="pool")
    args = parser.parse_args()
    if not (0 <= args.domain_id <= 101 and math.isfinite(args.timeout) and
            0 < args.startup_timeout <= args.timeout):
        parser.error("Require domain-id 0..101 and 0 < startup-timeout <= timeout")
    args.log.parent.mkdir(parents=True, exist_ok=True)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    os.environ["ROS_DOMAIN_ID"] = str(args.domain_id)
    os.environ["GZ_PARTITION"] = "usv_pool_smoke_" + uuid.uuid4().hex
    result = {
        "passed": False,
        "scope": "Real headless Gazebo server, production plugins and ROS/Gazebo bridge",
        "ros_domain_id": args.domain_id,
        "gz_partition": os.environ["GZ_PARTITION"],
        "checks": [],
        "errors": [],
        "launch_log": str(args.log.resolve()),
    }
    process = node = publisher = lift_publisher = rclpy = None
    started = time.monotonic()
    last_state = None
    initial_z = None
    sim_time = None
    initial_clock = None
    last_odom_wall = last_clock_wall = last_lift_wall = started
    lift_position = None
    lift_min = math.inf
    lift_max = -math.inf
    lift_messages = 0
    max_roll = max_pitch = max_z_drift = 0.0
    odom_messages = clock_messages = 0

    try:
        import rclpy as ros
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from rosgraph_msgs.msg import Clock
        from std_msgs.msg import Float64
        # Exercise the same independently held keys and ramps as the control window,
        # while sending the resulting commands through the real bridge.
        control_path = Path(__file__).resolve().parents[3] / "usv_rov_sim/scripts/control_panel.py"
        spec = importlib.util.spec_from_file_location("usv_smoke_drive", control_path)
        control_module = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = control_module
        spec.loader.exec_module(control_module)
        drive = control_module.DriveState(speed_limit=0.4, yaw_limit=0.3)
        rclpy = ros
        rclpy.init(args=[])
        node = rclpy.create_node("usv_pool_bridge_smoke")
        qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)

        def on_odometry(message):
            nonlocal last_state, initial_z, last_odom_wall, odom_messages
            nonlocal max_roll, max_pitch, max_z_drift
            p, q, velocity = message.pose.pose.position, message.pose.pose.orientation, message.twist.twist
            values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w,
                      velocity.linear.x, velocity.linear.y, velocity.linear.z,
                      velocity.angular.x, velocity.angular.y, velocity.angular.z)
            if not all(math.isfinite(value) for value in values):
                raise AssertionError("Odometry contains non-finite pose or velocity")
            roll, pitch, yaw = quaternion_rpy(q.x, q.y, q.z, q.w)
            unwrapped = yaw if last_state is None else last_state.unwrapped_yaw + math.atan2(
                math.sin(yaw - last_state.yaw), math.cos(yaw - last_state.yaw))
            last_state = State(p.x, p.y, p.z, roll, pitch, yaw, unwrapped,
                               math.sqrt(sum(v * v for v in values[7:10])),
                               velocity.linear.x, velocity.angular.z)
            if initial_z is None:
                initial_z = p.z
            max_roll, max_pitch = max(max_roll, abs(roll)), max(max_pitch, abs(pitch))
            max_z_drift = max(max_z_drift, abs(p.z - initial_z))
            if abs(roll) >= 0.15 or abs(pitch) >= 0.15 or abs(p.z - initial_z) >= 0.15:
                raise AssertionError("Vehicle lost stable flotation: roll/pitch or Z drift exceeded 0.15")
            last_odom_wall = time.monotonic()
            odom_messages += 1

        def on_clock(message):
            nonlocal sim_time, initial_clock, last_clock_wall, clock_messages
            value = message.clock.sec + message.clock.nanosec * 1e-9
            if sim_time is not None and value < sim_time - 1e-6:
                raise AssertionError("Simulation clock moved backwards unexpectedly")
            if initial_clock is None:
                initial_clock = value
            if sim_time is None or value > sim_time:
                last_clock_wall = time.monotonic()
            sim_time = value
            clock_messages += 1

        def on_lift(message):
            nonlocal lift_position, last_lift_wall, lift_messages, lift_min, lift_max
            value = float(message.data)
            if not math.isfinite(value) or not -0.01 <= value <= 0.2817:
                raise AssertionError(f"Lift position is invalid or outside its physical travel: {value}")
            lift_position, last_lift_wall = value, time.monotonic()
            lift_min, lift_max = min(lift_min, value), max(lift_max, value)
            lift_messages += 1

        node.create_subscription(Float64, "/sim/me120/lift/position", on_lift, qos)
        lift_publisher = node.create_publisher(Float64, "/sim/me120/lift/velocity", 1)
        node.create_subscription(Odometry, "/sim/me120/odometry", on_odometry, qos)
        node.create_subscription(Clock, "/clock", on_clock, qos)
        publisher = node.create_publisher(Twist, "/sim/me120/cmd_vel", 1)

        def publish(surge=0.0, yaw=0.0):
            message = Twist()
            message.linear.x, message.angular.z = surge, yaw
            publisher.publish(message)

        def publish_lift(velocity=0.0):
            message = Float64()
            message.data = float(velocity)
            lift_publisher.publish(message)

        def spin():
            now = time.monotonic()
            if now - started >= args.timeout:
                raise TimeoutError(f"Overall wall timeout of {args.timeout:g} seconds exceeded")
            if process.poll() is not None:
                raise RuntimeError(f"Pool launch exited unexpectedly with code {process.returncode}")
            if not rclpy.ok():
                raise RuntimeError("ROS context stopped unexpectedly")
            rclpy.spin_once(node, timeout_sec=0.01)

        def phase(name, duration, command, lift=0.0, min_wall=0.0):
            """Command at 20 Hz wall time while measuring duration in simulation time."""
            before = last_state
            start_sim = sim_time
            start_wall = time.monotonic()
            before_lift = lift_position
            lift_start_messages = lift_messages
            phase_lift_min = phase_lift_max = lift_position
            next_publish = time.monotonic()
            while sim_time - start_sim < duration or time.monotonic() - start_wall < min_wall:
                now = time.monotonic()
                if now >= next_publish:
                    if command is not None:
                        publish(*(command() if callable(command) else command))
                    if lift is not None:
                        publish_lift(lift)
                    next_publish = now + 0.05
                spin()
                now = time.monotonic()
                if max(now - last_odom_wall, now - last_clock_wall, now - last_lift_wall) > 5:
                    raise TimeoutError(f"Odometry, lift feedback or advancing clock disappeared during {name}")
                phase_lift_min = min(phase_lift_min, lift_position)
                phase_lift_max = max(phase_lift_max, lift_position)
            if lift_messages == lift_start_messages:
                raise AssertionError(f"No fresh lift position received during {name}")
            result.setdefault("phases", []).append({
                "name": name, "duration_sim_s": sim_time - start_sim,
                "command": "control_panel DriveState" if callable(command) else command,
                "lift_command": lift, "before": vars(before), "after": vars(last_state),
                "lift_before_m": before_lift, "lift_after_m": lift_position,
                "lift_min_m": phase_lift_min, "lift_max_m": phase_lift_max,
                "lift_messages": lift_messages - lift_start_messages,
            })
            print(f"{name}: sim={sim_time:.2f}s, speed={last_state.speed:.3f}m/s, "
                  f"yaw={last_state.unwrapped_yaw:.3f}rad, lift={lift_position:.3f}m", flush=True)
            return before, last_state

        def check(name, passed, **measurements):
            result["checks"].append({"name": name, "passed": bool(passed), **measurements})
            if not passed:
                raise AssertionError(f"{name} failed: {measurements}")

        with args.log.open("w", buffering=1) as launch_log:
            process = subprocess.Popen(
                ["ros2", "launch", "usv_rov_sim", "sim.launch.py", f"world:={args.world}", "gui:=false"],
                stdout=launch_log, stderr=subprocess.STDOUT, start_new_session=True,
            )
            while (last_state is None or sim_time is None or lift_position is None or
                   node.count_subscribers("/sim/me120/cmd_vel") == 0 or
                   node.count_subscribers("/sim/me120/lift/velocity") == 0):
                spin()
                if time.monotonic() - started >= args.startup_timeout:
                    raise TimeoutError("Startup timeout waiting for odometry, clock, lift feedback and both command bridges")
            phase("settle", 2.0, (0.0, 0.0))
            check("steady_float", last_state.speed < 0.1, speed=last_state.speed, z=last_state.z)
            check("lift_initially_raised", abs(lift_position) < 0.02, position_m=lift_position)
            phase("lift_partial_down", 2.0, (0.0, 0.0), lift=1.0)
            check("lift_moves_down", lift_position > 0.08, position_m=lift_position)
            phase("lift_hold", 1.5, (0.0, 0.0))
            hold = result["phases"][-1]
            check("lift_holds_on_release", hold["lift_max_m"] - hold["lift_min_m"] < 0.02,
                  drift_m=hold["lift_max_m"] - hold["lift_min_m"])
            phase("lift_lower_limit", 2.5, (0.0, 0.0), lift=1.0)
            check("lift_reaches_lower_limit", lift_position > 0.24, position_m=lift_position)
            phase("lift_raise", 4.0, (0.0, 0.0), lift=-1.0)
            check("lift_returns_raised", abs(lift_position) < 0.02, position_m=lift_position)
            phase("lift_timeout_accelerate", 0.8, (0.0, 0.0), lift=1.0)
            check("lift_timeout_precondition", lift_position > 0.01, position_m=lift_position)
            phase("lift_timeout_wait", 1.0, (0.0, 0.0), lift=None, min_wall=0.8)
            check("lift_timeout_holds_before_end_stop", 0.02 < lift_position < 0.23,
                  position_m=lift_position)
            phase("lift_timeout_hold", 1.5, (0.0, 0.0), lift=None)
            hold = result["phases"][-1]
            check("lift_holds_after_timeout", hold["lift_max_m"] - hold["lift_min_m"] < 0.02,
                  drift_m=hold["lift_max_m"] - hold["lift_min_m"])
            phase("lift_restore", 4.0, (0.0, 0.0), lift=-1.0)
            before, after = phase("forward", 6.0, (0.8, 0.0))
            distance = ((after.x - before.x) * math.cos(before.yaw) +
                        (after.y - before.y) * math.sin(before.yaw))
            check("forward_motion", distance > 1.0 and after.surge > 0.15,
                  forward_displacement_m=distance, final_surge_mps=after.surge)
            release_speed = after.speed
            _, after = phase("explicit_stop", 6.0, (0.0, 0.0))
            check("explicit_stop_decelerates", after.speed < max(0.1, 0.5 * release_speed),
                  release_speed_mps=release_speed, final_speed_mps=after.speed)
            before, after = phase("left_turn", 6.0, (0.0, 0.4))
            yaw_change = after.unwrapped_yaw - before.unwrapped_yaw
            check("left_turn_direction", yaw_change > 0.3,
                  yaw_change_rad=yaw_change, final_yaw_rate=after.yaw_rate)
            phase("settle_after_turn", 2.0, (0.0, 0.0))
            _, after = phase("watchdog_accelerate", 4.0, (0.7, 0.0))
            release_speed = after.speed
            check("watchdog_precondition", release_speed > 0.15, speed_mps=release_speed)
            wall_release = time.monotonic()
            _, after = phase("watchdog_no_publisher", 5.0, None, min_wall=0.85)
            check("watchdog_decelerates", time.monotonic() - wall_release > 0.8 and
                  after.speed < max(0.1, 0.5 * release_speed),
                  release_speed_mps=release_speed, final_speed_mps=after.speed,
                  wall_s_without_commands=time.monotonic() - wall_release)
            drive.press("w", "keyboard:w")
            drive.press("d", "keyboard:d")
            ramped_command = lambda: drive.advance(time.monotonic())
            before, after = phase("combined_forward_right_lift_down", 4.0, ramped_command, lift=1.0)
            check("combined_forward_right", after.unwrapped_yaw < before.unwrapped_yaw - 0.12 and
                  math.hypot(after.x - before.x, after.y - before.y) > 0.3,
                  yaw_change_rad=after.unwrapped_yaw - before.unwrapped_yaw,
                  distance_m=math.hypot(after.x - before.x, after.y - before.y))
            check("lift_lowers_while_maneuvering", lift_position > 0.24,
                  position_m=lift_position)
            drive.release("keyboard:d")
            phase("release_steering_keep_forward", 1.0, ramped_command)
            check("independent_steering_release", drive.target() == (0.4, 0.0) and
                  drive.surge > 0.1 and abs(drive.yaw) < 1e-6 and last_state.surge > 0.1,
                  command_surge=drive.surge, command_yaw=drive.yaw,
                  measured_surge_mps=last_state.surge)
            drive.press("a", "keyboard:a")
            before, after = phase("combined_forward_left_lift_up", 4.0, ramped_command, lift=-1.0)
            check("combined_forward_left", after.unwrapped_yaw > before.unwrapped_yaw + 0.12 and
                  math.hypot(after.x - before.x, after.y - before.y) > 0.3,
                  yaw_change_rad=after.unwrapped_yaw - before.unwrapped_yaw,
                  distance_m=math.hypot(after.x - before.x, after.y - before.y))
            check("lift_raises_while_maneuvering", abs(lift_position) < 0.02,
                  position_m=lift_position)
            drive.stop()
            phase("final_stop", 2.0, (0.0, 0.0))
            check("lift_remains_within_limits", lift_min >= -0.01 and lift_max <= 0.2817,
                  min_position_m=lift_min, max_position_m=lift_max, messages=lift_messages)
            check("clock_advances", sim_time - initial_clock > 25,
                  elapsed_sim_s=sim_time - initial_clock, messages=clock_messages)
            check("stable_attitude_and_height", max_roll < 0.15 and max_pitch < 0.15 and max_z_drift < 0.15,
                  max_abs_roll_rad=max_roll, max_abs_pitch_rad=max_pitch,
                  max_z_drift_m=max_z_drift, odometry_messages=odom_messages)
            result["passed"] = True
    except (Exception, KeyboardInterrupt) as error:
        result["errors"].append(f"{type(error).__name__}: {error}")
        print(result["errors"][-1], flush=True)
    finally:
        if publisher is not None and rclpy is not None and rclpy.ok():
            try:
                for _ in range(3):
                    publish()
                    if lift_publisher is not None:
                        publish_lift()
                    rclpy.spin_once(node, timeout_sec=0.02)
            except Exception:
                pass
        if process is not None:
            stop_process_group(process)
            result["launch_returncode_after_cleanup"] = process.poll()
        if node is not None:
            node.destroy_node()
        if rclpy is not None and rclpy.ok():
            rclpy.shutdown()
        result["wall_duration_s"] = time.monotonic() - started
        args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    print(f"Bridge smoke passed={result['passed']}; results={args.output}", flush=True)
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
