#!/usr/bin/env python3
"""Create native-loader expectations directly from exported COLLADA arrays.

The transfer writes each visual primitive in mesh-local coordinates with identity
scene nodes. This script rejects transform elements rather than silently ignoring
them, so the resulting min/max are valid expected loader-space bounds.
"""
import argparse
from pathlib import Path
import xml.etree.ElementTree as ET

NS={'c':'http://www.collada.org/2005/11/COLLADASchema'}


def expectations(path):
    root=ET.parse(path).getroot()
    for kind in ('matrix','translate','rotate','scale','lookat','skew'):
        if root.findall('.//c:visual_scene//c:'+kind,NS):
            raise ValueError(f'{path}: unexpected scene transform {kind}')
    unit=root.find('c:asset/c:unit',NS)
    axis=root.findtext('c:asset/c:up_axis',namespaces=NS)
    if unit is None or float(unit.get('meter','1'))!=1.0 or axis!='Z_UP':
        raise ValueError(f'{path}: expected metre / Z_UP metadata')
    lo=[float('inf')]*3;hi=[-float('inf')]*3;triangles=0
    for geo in root.findall('c:library_geometries/c:geometry',NS):
        sources={s.get('id'):s for s in geo.findall('c:mesh/c:source',NS)}
        vertices=geo.find('c:mesh/c:vertices',NS)
        pos_input=vertices.find('c:input[@semantic="POSITION"]',NS)
        source=sources[pos_input.get('source').lstrip('#')]
        array=source.find('c:float_array',NS)
        vals=list(map(float,array.text.split()))
        accessor=source.find('c:technique_common/c:accessor',NS)
        if int(accessor.get('stride','1'))!=3:
            raise ValueError(f'{path}: position stride is not 3')
        for j in range(0,len(vals),3):
            for a in range(3):
                lo[a]=min(lo[a],vals[j+a]);hi[a]=max(hi[a],vals[j+a])
        triangles+=sum(int(t.get('count')) for t in geo.findall('c:mesh/c:triangles',NS))
    return triangles,lo,hi


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('mesh_directory',type=Path)
    ap.add_argument('output',type=Path)
    args=ap.parse_args()
    rows=[]
    for path in sorted(args.mesh_directory.resolve().rglob('*.dae')):
        triangles,lo,hi=expectations(path)
        rows.append(str(path)+'\t'+str(triangles)+'\t'+'\t'.join(format(n,'.12g') for n in lo+hi))
    if len(rows)!=39:
        raise AssertionError(f'Expected 39 exported files, found {len(rows)}')
    args.output.write_text('\n'.join(rows)+'\n')
    print(f'{len(rows)} mesh bounds written to {args.output}')


if __name__=='__main__':
    main()
