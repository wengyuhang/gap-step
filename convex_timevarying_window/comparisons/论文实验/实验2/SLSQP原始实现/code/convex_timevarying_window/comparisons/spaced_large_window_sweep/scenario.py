import json
import xml.etree.ElementTree as ET
import numpy as np
from convex_timevarying_window.comparisons.large_window_sweep import scenario as large
from convex_timevarying_window.receding_gate_racing import scenario as common

Case=large.Case
ORIGIN=np.array([-16.,4.,3.2])
SPACING=3.
LOWER=(-52.,-98.,.65)
UPPER=(116.,58.,12.)


def spread(point):
    p=np.array(point,dtype=float,copy=True)
    p[:2]=ORIGIN[:2]+SPACING*(p[:2]-ORIGIN[:2])
    return p


def build_case(case):
    track,cfg=large.build_case(case)
    for w in track.windows:w.center0=spread(w.center0)
    return track,cfg


def layout_check(max_amplitude=5.):
    track,_=build_case(Case(1.,max_amplitude,0.))
    radii=[w.aperture.radius if w.aperture.kind=='circle' else
           np.linalg.norm(w.aperture.physical_vertices,axis=1).max() for w in track.windows]
    pair=min(np.linalg.norm(a.center0-b.center0)-2*max_amplitude-radii[i]-radii[j]-.30
             for i,a in enumerate(track.windows) for j,b in enumerate(track.windows) if j>i)
    a,b=track.windows[-1].center0,track.start
    gaps=[]
    for i,w in enumerate(track.windows[:-1]):
        fraction=np.clip((w.center0-a)@(b-a)/np.dot(b-a,b-a),0,1)
        gaps.append(float(np.linalg.norm(w.center0-a-fraction*(b-a))-radii[i]-max_amplitude-.15-.38594472))
    return dict(max_amplitude_m=max_amplitude,minimum_gate_swept_sphere_gap_m=float(pair),
        straight_return_vs_other_gates_min_gap_m=min(gaps),
        limitation='Only layout envelopes and straight center-to-start segment; not a flight safety proof')


def manifest(case):
    data=large.manifest(case);track,_=build_case(case)
    data.update(anchors=[w.center0.tolist() for w in track.windows],original_positions_unchanged=False,
        original_planes_unchanged=False,orientations_unchanged=True,horizontal_spacing_scale=SPACING,
        spacing_origin=ORIGIN.tolist(),altitudes_unchanged=True,workspace_lower=LOWER,
        workspace_upper=UPPER,max_flight_time_s=180,layout_check=layout_check())
    return data


def write_world(track,assets,outdir):
    path=large.write_world(track,assets,outdir);tree=ET.parse(path)
    # The gate writer already writes the new anchors. Expand the surrounding
    # static room horizontally too, without scaling gate thickness or flight height.
    for model in tree.findall('.//world/model'):
        name=model.get('name','')
        if name.startswith('gate_') or name=='start_finish_marker':continue
        pose=model.find('pose')
        if pose is not None:
            values=np.fromstring(pose.text,sep=' ');values[:3]=spread(values[:3]);pose.text=' '.join(map(str,values))
        for node in model.findall('.//box/size')+model.findall('.//plane/size'):
            values=np.fromstring(node.text,sep=' ');values[:2]*=SPACING;node.text=' '.join(map(str,values))
    tree.write(path,encoding='unicode')
    (outdir/'layout_check.json').write_text(json.dumps(layout_check(),indent=2))
    return path


def launch(track,assets,outdir):
    previous=common.write_variant_world;common.write_variant_world=write_world
    try:return common.launch_variant(track,assets,outdir)
    finally:common.write_variant_world=previous
