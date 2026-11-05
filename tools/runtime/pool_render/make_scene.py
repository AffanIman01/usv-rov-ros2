#!/usr/bin/env python3
"""Export this pool's SDF visual placements to the native render probe's table.

This bounded reader supports the unrotated mesh/box/plane visuals used by the
phase-1 pool. It rejects unsupported rotations and resources rather than
silently producing a misleading verification image.
"""

import argparse
from pathlib import Path
import xml.etree.ElementTree as ET


def xyz(text):
    values = list(map(float, (text or "0 0 0 0 0 0").split()))
    if len(values) != 6 or any(abs(x) > 1e-8 for x in values[3:]):
        raise ValueError("The phase-1 render table requires zero pose rotations.")
    return values[:3]


def add(a, b):
    return [x + y for x, y in zip(a, b)]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("description_package", type=Path)
    parser.add_argument("simulation_package", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    description = args.description_package.resolve()
    simulation = args.simulation_package.resolve()
    world = ET.parse(simulation / "worlds/pool.sdf").find("world")
    if world is None:
        raise ValueError("World element missing.")
    rows = []

    def record(model, base):
        for link in model.findall("link"):
            link_position = add(base, xyz(link.findtext("pose")))
            for visual in link.findall("visual"):
                position = add(link_position, xyz(visual.findtext("pose")))
                geometry = visual.find("geometry")
                kind = geometry[0].tag
                if kind == "mesh":
                    uri = geometry.findtext("mesh/uri")
                    prefix = "model://usv_rov_description/"
                    if not uri.startswith(prefix):
                        raise ValueError(f"Unexpected mesh URI: {uri}")
                    path = (description / uri[len(prefix):]).resolve(strict=True)
                    if not path.is_relative_to(description):
                        raise ValueError(f"Mesh escapes the description package: {uri}")
                    if any(c.isspace() for c in str(path)):
                        raise ValueError("Use a description-package path without whitespace.")
                    filename = str(path)
                    size = [1, 1, 1]
                elif kind in ("box", "plane"):
                    filename = "-"
                    size = list(map(float, geometry.findtext(kind + "/size").split()))
                    if kind == "plane":
                        normal = list(map(float, geometry.findtext("plane/normal").split()))
                        if normal != [0, 0, 1]:
                            raise ValueError("Only upward-facing planes are supported.")
                        size = [*size, 1]
                else:
                    raise ValueError(f"Unsupported visual geometry: {kind}")
                ambient = list(map(float, visual.findtext("material/ambient", "1 1 1 1").split()))[:3]
                diffuse = list(map(float, visual.findtext("material/diffuse", "1 1 1 1").split()))[:3]
                transparency = float(visual.findtext("transparency", "0"))
                values = [kind, filename, *position, *size, *ambient, *diffuse, transparency]
                rows.append(" ".join(map(str, values)))

    for model in world.findall("model"):
        record(model, xyz(model.findtext("pose")))
    for include in world.findall("include"):
        uri = include.findtext("uri")
        if uri != "model://me120_dynamic":
            raise ValueError(f"Unexpected included model: {uri}")
        model = ET.parse(simulation / "models/me120_dynamic/model.sdf").find("model")
        record(model, xyz(include.findtext("pose")))
    args.output.write_text("\n".join(rows) + "\n")
    print(f"Wrote {len(rows)} visual records to {args.output}")


if __name__ == "__main__":
    main()
