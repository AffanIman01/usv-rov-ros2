#!/usr/bin/env python3
"""Check exported model structure without ROS, Gazebo, or third-party modules.

This verifies references, fixed kinematic trees, numeric geometry, Collada
indices/material bindings and static SDF declarations. It does not validate
rendering, physical fidelity, or ROS/Gazebo runtime compatibility.
"""

import argparse
import ast
from collections import Counter, defaultdict
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET


COLLADA = {"c": "http://www.collada.org/2005/11/COLLADASchema"}


class Validation:
    def __init__(self, package):
        self.package = package.resolve()
        self.errors = []
        self.meshes = {}
        self.counts = Counter()

    def require(self, condition, message):
        if not condition:
            self.errors.append(message)
        return condition

    def numbers(self, text, count=None, label="values", integer=False):
        try:
            values = [(int if integer else float)(x) for x in (text or "").split()]
        except ValueError:
            self.errors.append(f"{label}: invalid numeric value")
            return []
        self.require(all(math.isfinite(x) for x in values), f"{label}: non-finite value")
        if count is not None:
            self.require(len(values) == count, f"{label}: expected {count} values, found {len(values)}")
        return values

    def parse(self, path):
        try:
            return ET.parse(path).getroot()
        except (ET.ParseError, OSError) as error:
            self.errors.append(f"{path}: {error}")
            return None

    def resolve(self, uri, context):
        for prefix in ("package://usv_rov_description/", "model://usv_rov_description/"):
            if uri.startswith(prefix):
                candidate = (self.package / uri[len(prefix):]).resolve()
                break
        else:
            if uri.startswith("model://"):
                candidate = (self.package / "models" / uri[len("model://"):]).resolve()
            elif "://" not in uri:
                candidate = (context.parent / uri).resolve()
            else:
                self.errors.append(f"{context.name}: unsupported external resource {uri}")
                return None
        self.require(candidate.is_relative_to(self.package), f"{context.name}: resource escapes package: {uri}")
        self.require(candidate.exists(), f"{context.name}: missing resource {uri}")
        return candidate

    def check_urdf(self, path):
        root = self.parse(path)
        if root is None:
            return
        label = path.name
        self.require(root.tag == "robot", f"{label}: root must be robot")
        links = [x.get("name") for x in root.findall("link")]
        joints = root.findall("joint")
        names = [x.get("name") for x in joints]
        self.require(len(links) == len(set(links)), f"{label}: duplicate links")
        self.require(len(names) == len(set(names)), f"{label}: duplicate joint names")
        children = Counter()
        adjacency = defaultdict(list)
        for joint in joints:
            self.require(joint.get("type") == "fixed", f"{label}: non-fixed joint in visual transfer")
            parent = joint.find("parent")
            child = joint.find("child")
            if not self.require(parent is not None and child is not None, f"{label}: incomplete joint"):
                continue
            p, c = parent.get("link"), child.get("link")
            self.require(p in links and c in links, f"{label}: joint references unknown link")
            children[c] += 1
            adjacency[p].append(c)
        roots = set(links) - set(children)
        self.require(roots == {"base_link"}, f"{label}: expected sole base_link root, found {sorted(roots)}")
        self.require(all(n == 1 for n in children.values()), f"{label}: child with multiple parents")
        seen, visiting = set(), set()

        def visit(node):
            if not self.require(node not in visiting, f"{label}: kinematic cycle at {node}"):
                return
            if node in seen:
                return
            visiting.add(node)
            for child in adjacency[node]:
                visit(child)
            visiting.remove(node)
            seen.add(node)

        visit("base_link")
        self.require(seen == set(links), f"{label}: unreachable link(s)")
        for origin in root.iter("origin"):
            for attribute in ("xyz", "rpy"):
                if origin.get(attribute) is not None:
                    self.numbers(origin.get(attribute), 3, f"{label}: {attribute}")
        for mesh in root.iter("mesh"):
            self.resolve(mesh.get("filename", ""), path)
        self.require(not list(root.iter("inertial")), f"{label}: physical inertia unexpectedly supplied")
        self.counts["urdf_files"] += 1
        self.counts["urdf_links"] += len(links)

    def check_dae(self, path):
        root = self.parse(path)
        if root is None:
            return
        label = str(path.relative_to(self.package))
        ns = COLLADA
        ids = [x.get("id") for x in root.iter() if x.get("id")]
        self.require(len(ids) == len(set(ids)), f"{label}: duplicate XML IDs")
        by_id = {x.get("id"): x for x in root.iter() if x.get("id")}
        unit = root.find("c:asset/c:unit", ns)
        self.require(unit is not None and unit.get("meter") == "1", f"{label}: metre unit missing")
        self.require(root.findtext("c:asset/c:up_axis", namespaces=ns) == "Z_UP", f"{label}: expected Z_UP")
        for element in root.iter():
            for attr in ("url", "source", "target"):
                ref = element.get(attr, "")
                if ref.startswith("#"):
                    self.require(ref[1:] in by_id, f"{label}: unresolved reference {ref}")
        minimum, maximum = [math.inf] * 3, [-math.inf] * 3
        triangles_total = 0
        for mesh in root.findall("c:library_geometries/c:geometry/c:mesh", ns):
            sources = {}
            for source in mesh.findall("c:source", ns):
                array = source.find("c:float_array", ns)
                accessor = source.find("c:technique_common/c:accessor", ns)
                if not self.require(array is not None and accessor is not None, f"{label}: missing source data"):
                    continue
                values = self.numbers(array.text, int(array.get("count", 0)), f"{label}: float array")
                count, stride = int(accessor.get("count", 0)), int(accessor.get("stride", 1))
                self.require(len(values) == count * stride, f"{label}: accessor count mismatch")
                sources[source.get("id")] = (count, stride, values)
            vertices = {}
            for vertex in mesh.findall("c:vertices", ns):
                position = vertex.find("c:input[@semantic='POSITION']", ns)
                if not self.require(position is not None, f"{label}: vertices lack POSITION"):
                    continue
                source_id = position.get("source", "")[1:]
                vertices[vertex.get("id")] = source_id
                if source_id in sources:
                    _, stride, values = sources[source_id]
                    self.require(stride == 3, f"{label}: positions must have stride 3")
                    for i in range(0, len(values) - 2, stride):
                        for axis in range(3):
                            minimum[axis] = min(minimum[axis], values[i + axis])
                            maximum[axis] = max(maximum[axis], values[i + axis])
            for triangles in mesh.findall("c:triangles", ns):
                count = int(triangles.get("count", 0))
                inputs = triangles.findall("c:input", ns)
                stride = 1 + max((int(x.get("offset", 0)) for x in inputs), default=0)
                indices = self.numbers(triangles.findtext("c:p", namespaces=ns), count * 3 * stride, f"{label}: triangle indices", integer=True)
                for item in inputs:
                    offset = int(item.get("offset", 0))
                    source_id = item.get("source", "")[1:]
                    if item.get("semantic") == "VERTEX":
                        source_id = vertices.get(source_id)
                    if self.require(source_id in sources, f"{label}: input source missing"):
                        upper = sources[source_id][0]
                        self.require(all(0 <= i < upper for i in indices[offset::stride]), f"{label}: index outside source array")
                triangles_total += count
            geometry = next((g for g in root.findall("c:library_geometries/c:geometry", ns) if g.find("c:mesh", ns) is mesh), None)
            if geometry is not None:
                symbols = {t.get("material") for t in mesh.findall("c:triangles", ns)}
                instances = root.findall(f".//c:instance_geometry[@url='#{geometry.get('id')}']", ns)
                self.require(bool(instances), f"{label}: geometry has no scene instance")
                for instance in instances:
                    bindings = {b.get("symbol"): b.get("target") for b in instance.findall("c:bind_material/c:technique_common/c:instance_material", ns)}
                    self.require(symbols <= bindings.keys(), f"{label}: missing material symbol binding")
                    for target in bindings.values():
                        element = by_id.get(target.lstrip("#"))
                        self.require(element is not None and element.tag.endswith("}material"), f"{label}: material binding target invalid")
        self.require(triangles_total > 0, f"{label}: no triangles")
        self.require(all(math.isfinite(x) for x in minimum + maximum), f"{label}: no valid bounding box")
        self.require(any(b > a for a, b in zip(minimum, maximum)), f"{label}: collapsed geometry")
        self.meshes[path.resolve()] = {"triangles": triangles_total, "min": minimum, "max": maximum}
        self.counts["dae_files"] += 1
        self.counts["triangles"] += triangles_total

    def check_sdf(self, path):
        root = self.parse(path)
        if root is None:
            return
        label = str(path.relative_to(self.package))
        self.require(root.tag == "sdf", f"{label}: expected sdf root")
        for model in root.iter("model"):
            self.require(model.findtext("static") == "true", f"{label}: model is not explicitly static")
        for uri in root.iter("uri"):
            resolved = self.resolve(uri.text or "", path)
            if resolved is not None and resolved.is_dir():
                self.require((resolved / "model.sdf").is_file(), f"{label}: included model.sdf missing")
                self.require((resolved / "model.config").is_file(), f"{label}: included model.config missing")
        for pose in root.iter("pose"):
            self.numbers(pose.text, 6, f"{label}: pose")
        self.require(not list(root.iter("inertial")), f"{label}: physical inertia unexpectedly supplied")
        self.counts["sdf_files"] += 1

    def check_manifest(self):
        path = self.package / "config" / "model_manifest.json"
        try:
            manifest = json.loads(path.read_text())
        except (OSError, json.JSONDecodeError) as error:
            self.errors.append(f"manifest: {error}")
            return
        self.require(manifest.get("units") == "metres", "manifest: wrong units")
        for model in manifest.get("models", []):
            low, high, total = [math.inf] * 3, [-math.inf] * 3, 0
            for part in model["parts"]:
                mesh = self.meshes.get((self.package / "meshes" / model["name"] / (part["name"] + ".dae")).resolve())
                if not self.require(mesh is not None, f"manifest: missing part {model['name']}/{part['name']}"):
                    continue
                self.require(mesh["triangles"] == part["triangles"], f"manifest: triangle count differs for {part['name']}")
                total += mesh["triangles"]
                for axis in range(3):
                    low[axis] = min(low[axis], mesh["min"][axis] + part["origin"][axis])
                    high[axis] = max(high[axis], mesh["max"][axis] + part["origin"][axis])
            self.require(total == model["triangles"], f"manifest: model triangle count differs for {model['name']}")
            for actual, expected in ((low, model["bounds"]["min"]), (high, model["bounds"]["max"])):
                self.require(all(abs(a - b) < 1e-5 for a, b in zip(actual, expected)), f"manifest: bounds differ for {model['name']}")
        self.counts["manifest_models"] = len(manifest.get("models", []))

    def run(self):
        self.parse(self.package / "package.xml")
        for path in sorted((self.package / "meshes").rglob("*.dae")):
            self.check_dae(path)
        for path in sorted((self.package / "urdf").glob("*.urdf")):
            self.check_urdf(path)
        for path in sorted((self.package / "models").rglob("model.config")):
            root = self.parse(path)
            if root is not None:
                self.require((path.parent / (root.findtext("sdf") or "")).is_file(), f"{path}: missing configured SDF")
        for folder in ("models", "worlds"):
            for path in sorted((self.package / folder).rglob("*.sdf")):
                self.check_sdf(path)
        for path in sorted((self.package / "launch").glob("*.py")):
            try:
                ast.parse(path.read_text(), filename=str(path))
            except SyntaxError as error:
                self.errors.append(f"{path}: {error}")
        self.check_manifest()
        return {
            "passed": not self.errors,
            "checks": dict(self.counts),
            "errors": self.errors,
            "runtime_verified": False,
            "scope": "Structural checks only; no ROS, Gazebo, renderer or physical validation.",
        }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("package", nargs="?", type=Path, default=Path(__file__).resolve().parent.parent / "usv_rov_description")
    args = parser.parse_args()
    result = Validation(args.package).run()
    print(json.dumps(result, indent=2))
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
