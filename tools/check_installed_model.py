#!/usr/bin/env python3
"""Verify that ROS selects this checkout's URDF and visual mesh contents."""

import argparse
import hashlib
from pathlib import Path, PurePosixPath
import sys
import xml.etree.ElementTree as ET


PACKAGE = "usv_rov_description"
MODELS = (
    "me120", "m2_pro_max", "showcase", "e_reel", "starlink_mini",
    "compute_enclosure", "recovery_concept",
)
COLLADA = {"c": "http://www.collada.org/2005/11/COLLADASchema"}


def checked_path(root, relative, source=None):
    """Reject path traversal; allow colcon symlinks only to this source file."""
    relative = PurePosixPath(relative)
    if relative.is_absolute() or ".." in relative.parts:
        raise ValueError(f"Resource leaves the package: {relative}")
    path = root / relative
    resolved = path.resolve(strict=True)
    if not resolved.is_relative_to(root):
        if source is None or resolved != (source / relative).resolve(strict=True):
            raise ValueError(f"Resource resolves outside the selected package: {path}")
    if not resolved.is_file():
        raise ValueError(f"Resource is not a file: {path}")
    return path


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path,
                        default=Path(__file__).resolve().parents[1] / PACKAGE)
    parser.add_argument("--installed", type=Path,
                        help="Package share directory; defaults to the package selected by ROS.")
    parser.add_argument("--expected-share", type=Path,
                        help="Also require ROS to select this exact package share directory.")
    parser.add_argument("--model", choices=MODELS, default="me120")
    args = parser.parse_args()
    try:
        source = args.source.resolve(strict=True)
        if args.installed is None:
            from ament_index_python.packages import get_package_share_directory
            installed = Path(get_package_share_directory(PACKAGE)).resolve(strict=True)
        else:
            installed = args.installed.resolve(strict=True)

        print(f"Source package: {source}")
        print(f"ROS package share: {installed}")
        if args.expected_share is not None and installed != args.expected_share.resolve(strict=True):
            raise ValueError(f"Wrong overlay selected; expected {args.expected_share}")
        urdf_relative = f"urdf/{args.model}.urdf"
        source_urdf = checked_path(source, urdf_relative)
        root = ET.parse(source_urdf).getroot()
        resources = {urdf_relative}
        prefix = f"package://{PACKAGE}/"
        for mesh in root.iter("mesh"):
            uri = mesh.get("filename", "")
            if not uri.startswith(prefix):
                raise ValueError(f"Unexpected mesh URI: {uri}")
            resources.add(uri[len(prefix):])

        mismatches = []
        for relative in sorted(resources):
            expected = checked_path(source, relative)
            actual = checked_path(installed, relative, source=source)
            if digest(expected) != digest(actual):
                mismatches.append(relative)
        if mismatches:
            print("FAIL: the selected install differs from this checkout:", file=sys.stderr)
            for relative in mismatches:
                print(f"  {relative}", file=sys.stderr)
            print("Run bash tools/preview_model.sh from this repository to rebuild and select its overlay.",
                  file=sys.stderr)
            return 1

        cabin_relative = "meshes/me120/equipment_cabin.dae"
        if cabin_relative in resources:
            cabin = checked_path(installed, cabin_relative, source=source)
            tree = ET.parse(cabin)
            triangles = sum(int(item.get("count", 0))
                            for item in tree.findall(".//c:triangles", COLLADA))
            print(f"ME120 cabin: {triangles:,} triangles; SHA256 {digest(cabin)}")
            if triangles < 7552:
                print("FAIL: this checkout predates the ME120 double-sided surface correction. "
                      "Update it with git pull --ff-only, then rebuild.", file=sys.stderr)
                return 1
        print(f"PASS: {args.model} URDF and {len(resources) - 1} meshes match this checkout.")
        print("Restart RViz after an update; existing windows retain loaded meshes.")
        return 0
    except (OSError, ValueError, ET.ParseError, ImportError, LookupError) as error:
        print(f"FAIL: {error}", file=sys.stderr)
        print("Source ROS 2 Jazzy and your workspace, or provide --installed PACKAGE_SHARE.",
              file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
