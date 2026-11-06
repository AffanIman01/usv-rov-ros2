#!/usr/bin/env python3
"""Run native Gazebo physics/production buoyancy checks without a server."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("repo", type=Path)
parser.add_argument("--probe", required=True, type=Path)
parser.add_argument("--plugin-dir", required=True, type=Path)
parser.add_argument("--output", required=True, type=Path)
args = parser.parse_args()
args.output.mkdir(parents=True, exist_ok=True)
cases = [(name, False, 0) for name in (
    "idle", "roll", "pitch", "heave", "forward", "reverse", "left", "right",
    "coast", "wall", "mixed", "torque_pitch", "torque_roll")]
# A real negative control: the installed Harmonic graded-buoyancy plugin has
# the wrong pitch restoring sign for this hull, which these tests must detect.
cases.extend((("torque_pitch", True, 1), ("pitch", True, 1), ("torque_roll", True, 0)))
results = []
for case, builtin, expected_exit in cases:
    label = ("builtin_" if builtin else "") + case
    command = [str(args.probe.resolve()), str(args.repo.resolve()), case,
               str(args.plugin_dir.resolve())]
    if builtin:
        command.append("--builtin-buoyancy")
    try:
        process = subprocess.run(command, capture_output=True, text=True, timeout=60)
        output = process.stdout + process.stderr
        rows = [line[7:] for line in output.splitlines() if line.startswith("RESULT ")]
        measured = json.loads(rows[-1]) if rows else {}
        code = process.returncode
    except subprocess.TimeoutExpired as error:
        output = str(error)
        measured = {}
        code = 124
    (args.output / f"{label}.log").write_text(output)
    passed = code == expected_exit and bool(measured)
    if not builtin:
        passed = passed and measured.get("configured_before_world_parent") is True
    if expected_exit == 0:
        passed = passed and measured.get("passed", False)
    else:
        passed = passed and measured.get("passed") is False
    results.append({"case": label, "expected_exit": expected_exit,
                    "actual_exit": code, "check_passed": passed,
                    "measurement": measured})
    print(json.dumps(results[-1]), flush=True)
resources = ["usv_rov_sim/models/me120_dynamic/model.sdf",
             "usv_rov_sim/worlds/pool.sdf",
             "usv_rov_sim/include/usv_rov_sim/thruster_control.hpp",
             "usv_rov_sim/include/usv_rov_sim/surface_buoyancy.hpp",
             "usv_rov_sim/src/surface_buoyancy.cpp",
             "tools/runtime/marine/vehicle_probe.cpp",
             "tools/runtime/marine/run_checks.py"]
summary = {
    "passed": all(case["check_passed"] for case in results),
    "scope": "Actual Gazebo Physics plus production SurfaceBuoyancy shared plugin in an EntityComponentManager. SurfaceBuoyancy is configured synchronously through LoadSdfPlugins during SdfEntityCreator world construction, before the model receives its world parent, matching real server startup. Production ComputeThrust header drives native wrenches. Configured diagonal drag equation is applied by this harness; native Hydrodynamics transport initialization is blocked in this host.",
    "not_tested": ["Gazebo GUI", "ROS-to-Gazebo command bridge", "live transport subscriber", "native Hydrodynamics plugin stepping", "physical vehicle fidelity"],
    "inputs_sha256": {path: hashlib.sha256((args.repo / path).read_bytes()).hexdigest()
                      for path in resources},
    "surface_plugin_sha256": hashlib.sha256((args.plugin_dir / "libusv_rov_surface_buoyancy.so").read_bytes()).hexdigest(),
    "cases": results,
}
(args.output / "results.json").write_text(json.dumps(summary, indent=2) + "\n")
raise SystemExit(0 if summary["passed"] else 1)
