#!/usr/bin/env python3
"""Read the last simulator log without starting or changing the simulation."""

import argparse
from pathlib import Path
import re
import subprocess


REPO = Path(__file__).resolve().parents[1]
ANSI = re.compile(r"\x1b\[[0-?]*[ -/]*[@-~]")


def describe_log(text):
    """Report observed events without guessing why the simulator failed."""
    text = ANSI.sub("", text)
    events = []
    if "Simulation guard ERROR:" in text:
        events.append("The simulation monitor reported a fault; inspect its ERROR line below.")
    fault = text.find("Simulation guard ERROR:")
    shutdown = re.search(r"process\[simulation_guard[^\]]*\] was required", text)
    if fault >= 0 and shutdown and fault < shutdown.start():
        events.append("A monitor fault appears before the older launch configuration's whole-app shutdown request.")
    crashes = re.findall(r"process has died \[pid \d+, exit code (-?\d+), cmd '([^']+)'", text)
    for code, command in crashes:
        process = "Gazebo" if re.search(r"\bgz\s+sim\b", command) else (
            "ROS/Gazebo bridge" if "parameter_bridge" in command else "child process")
        events.append(f"{process} exited with code {code}; this alone does not identify the cause.")
    if re.search(r"segmentation fault|core dumped|\[BUG\]", text, re.IGNORECASE):
        events.append("The log contains a native crash marker. Preserve the original log for its stack trace.")
    if not events:
        events.append("No recognized failure marker in this log. Stale odometry alone cannot distinguish pause, feedback loss or a crash.")
    return events


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, default=REPO / ".pool_ws")
    parser.add_argument("--log", type=Path, help="Read a particular archived log instead")
    args = parser.parse_args()
    log = args.log or args.workspace / "last-launch.log"
    print("ME120 simulation diagnostic report (read-only)")
    try:
        result = subprocess.run(["git", "-C", str(REPO), "rev-parse", "--short=12", "HEAD"],
                                capture_output=True, text=True, timeout=5, check=False)
        print("Checkout:", result.stdout.strip() or "unknown")
    except (OSError, subprocess.TimeoutExpired):
        print("Checkout: unavailable")
    print("Log:", log)
    if not log.is_file():
        print("No launch log found. Run tools/run_sim.sh from this same checkout first.")
        return 1
    text = ANSI.sub("", log.read_text(errors="replace"))
    for event in describe_log(text):
        print("-", event)
    print("\nLast 100 log lines:")
    print("\n".join(text.splitlines()[-100:]))
    print("\nArchived runs:", args.workspace / "runs")
    print("This report does not restart Gazebo or publish commands.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
