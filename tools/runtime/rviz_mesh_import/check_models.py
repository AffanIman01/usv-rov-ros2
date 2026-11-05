#!/usr/bin/env python3
"""Verify installed RViz's mesh loader, without creating a renderer or ROS graph."""
import argparse
import json
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

# Only these ME120 resources contain source double-sided materials. Other model
# meshes intentionally retain their original appearance and winding.
DOUBLE_SIDED_ME120 = {
    "antenna", "bridge_frame", "equipment_cabin", "hull_port", "hull_starboard",
    "mounted_e_reel", "mounted_starlink_mini", "payload_lifter_fixed",
    "payload_lifter_moving", "payload_mounting_ring", "thrusters",
}
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("package", type=Path)
parser.add_argument("--probe", type=Path, required=True)
parser.add_argument("--results", type=Path, required=True)
args = parser.parse_args()
results = []
paths = sorted((args.package / "meshes").glob("*/*.dae"))
for path in paths:
    expected = sum(int(t.attrib["count"]) for t in ET.parse(path).iter()
                   if t.tag.rsplit("}", 1)[-1] == "triangles")
    require_pairs = path.parent.name == "me120" and path.stem in DOUBLE_SIDED_ME120
    command = [str(args.probe.resolve()), str(path.resolve()), str(expected)]
    if require_pairs:
        command.append("--require-reversed-pairs")
    proc = subprocess.run(command, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    print(f"FILE {path.relative_to(args.package)}", flush=True)
    print(proc.stdout, end="", flush=True)
    rows = [line[7:] for line in proc.stdout.splitlines() if line.startswith("RESULT ")]
    measured = json.loads(rows[-1]) if rows else {"passed": False}
    results.append({"path": str(path.relative_to(args.package)), "expected_triangles": expected,
                    "require_reversed_pairs": require_pairs, "exit_code": proc.returncode,
                    **measured})
passed = len(results) == 39 and all(r["exit_code"] == 0 and r["passed"] for r in results)
summary = {"passed": passed, "files": len(results),
           "triangles": sum(r.get("triangles", 0) for r in results),
           "scope": "Actual installed rviz_rendering mesh import and CPU-buffer inspection; no GUI or pixel rendering.",
           "results": results}
args.results.write_text(json.dumps(summary, indent=2) + "\n")
print(f"SUMMARY passed={passed} files={len(results)} triangles={summary['triangles']}")
raise SystemExit(0 if passed else 1)
