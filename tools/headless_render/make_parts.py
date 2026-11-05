#!/usr/bin/env python3
"""Produce the native renderer's manifest-based ME120 mesh list."""
import argparse
import json
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument('package', type=Path)
parser.add_argument('output', type=Path)
args = parser.parse_args()
package = args.package.resolve()
manifest = json.loads((package / 'config/model_manifest.json').read_text())
model = next(m for m in manifest['models'] if m['name'] == 'me120')
rows = []
for part in model['parts']:
    mesh = package / 'meshes/me120' / (part['name'] + '.dae')
    if not mesh.is_file():
        raise FileNotFoundError(mesh)
    if any(c.isspace() for c in str(mesh)):
        raise ValueError('The native probe requires a package path without whitespace.')
    rows.append('\t'.join([str(mesh), *map(str, part['origin'])]))
args.output.write_text('\n'.join(rows) + '\n')
print(f'Wrote {len(rows)} ME120 parts to {args.output}')
