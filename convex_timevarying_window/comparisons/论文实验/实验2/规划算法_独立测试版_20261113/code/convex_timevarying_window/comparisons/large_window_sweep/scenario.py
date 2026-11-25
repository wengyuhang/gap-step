"""Scale only the aperture; preserve anchors, motion, frame thickness and vehicle."""
from dataclasses import replace
from pathlib import Path
import hashlib
import sys
import xml.etree.ElementTree as ET
import numpy as np
from 复现.polynomial_dynamic_racing_2023.px4_fixed_plane import scenario as base
from convex_timevarying_window.receding_gate_racing import scenario as common

SCALE = 3.
Case = base.Case
ASSETS = Path(__file__).resolve().parents[3]/'convex_dynamic_seven_window_gazebo'


def build_case(case):
    track,cfg=base.build_case(case)
    for window in track.windows:
        a=window.aperture
        if a.kind=='circle':
            window.aperture=replace(a,radius=SCALE*a.radius)
        else:
            # Preserve the original absolute mapping inset, not a 3x safety margin.
            physical=SCALE*a.physical_vertices
            window.aperture=replace(a,physical_vertices=physical,
                safe_vertices=physical-(a.physical_vertices-a.safe_vertices))
    return track,cfg


def manifest(case):
    data=base.manifest(case);track,_=build_case(case)
    data.update(aperture_scale=SCALE,frame_tube_radius_m=.085,frame_outward_offset_m=.065,
        aperture_dimensions=[dict(name=w.name,kind=w.aperture.kind,radius=w.aperture.radius,
             vertices=None if w.aperture.kind=='circle' else w.aperture.physical_vertices.tolist())
             for w in track.windows],safety_margin_scaled=False)
    return data


def write_world(track,assets,outdir):
    path=base.write_world(track,assets,outdir)
    # Regenerate frame centerlines instead of scaling entire meshes (which would
    # triple tube thickness). Put new immutable assets under the existing mount.
    sys.path.insert(0,str(ASSETS))
    try:
        from convex_dynamic_seven_window_gazebo.export_world import tube_mesh,outward_offset_polygon
    finally:sys.path.pop(0)
    directory=assets/'results'/'large_window_meshes_scale3_v1';directory.mkdir(parents=True,exist_ok=True)
    tree=ET.parse(path);models=[m for m in tree.findall('.//world/model') if m.get('name','').startswith('gate_')]
    records=[]
    for index,(model,w) in enumerate(zip(models,track.windows),1):
        a=w.aperture;boundary=a.boundary_points(circle_samples=128)
        outer=(boundary*((a.radius+.065)/a.radius) if a.kind=='circle'
               else outward_offset_polygon(boundary,.065))
        for label,points,radius in [('core',boundary,.020),('sleeve',outer,.085)]:
            mesh=tube_mesh(points,radius,12);payload=mesh.export(file_type='stl')
            target=directory/f'gate_{index:02d}_{label}.stl'
            if target.exists():
                if target.read_bytes()!=payload:raise RuntimeError(f'Immutable mesh mismatch: {target}')
            else:target.write_bytes(payload)
            for uri in model.findall('.//mesh/uri'):
                if uri.text.endswith('_'+label+'.stl'):uri.text='/workspace/'+str(target.relative_to(assets))
            records.append(dict(gate=index,part=label,sha256=hashlib.sha256(payload).hexdigest(),
                                bounds=mesh.bounds.tolist(),tube_radius=radius))
    tree.write(path,encoding='unicode')
    import json
    (outdir/'mesh_manifest.json').write_text(json.dumps(records,indent=2))
    return path


def launch(track,assets,outdir):
    previous=common.write_variant_world;common.write_variant_world=write_world
    try:return common.launch_variant(track,assets,outdir)
    finally:common.write_variant_world=previous
