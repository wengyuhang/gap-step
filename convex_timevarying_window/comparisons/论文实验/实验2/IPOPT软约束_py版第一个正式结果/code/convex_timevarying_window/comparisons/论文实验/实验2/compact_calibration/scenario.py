"""Only contract horizontal gate anchors, never the physical apertures."""
from dataclasses import replace
import numpy as np
from ..interlaced_calibration import scenario as parent
from convex_dynamic_seven_window_gazebo.x500_togt.experiment import X500_FRAME_RADIUS
from ..static_calibration.scenario import Window

ROOT=parent.ROOT;ASSETS=parent.ASSETS;Case=parent.Case;LAYOUTS=parent.LAYOUTS;START=parent.START.copy()
HORIZONTAL_SCALE=.25
FIRST_GATE_SIDE_SHIFT_M=-4.
TRACKING_RESERVE_M=.20
WINDOW_MARGIN=2*(X500_FRAME_RADIUS+TRACKING_RESERVE_M)
# Static layout design only: full 3-D centre-crossing directions from the
# retained v3 height reference. No optimization or adaptation during flight.
HEIGHT_NORMALS=np.array([
    [.960164507902231,.1662204205658545,.22462165868806439],
    [.6174778763962084,-.037215045290482175,-.7857073962775558],
    [-.6412180472198005,-.7501154995426471,.16175955386162102],
    [-.6989580035333182,.7117304207969951,.06998226495941731],
    [-.9854774728783428,-.1491636587447268,.08114402848763538],
    [.9875611436115059,.15019360022353578,.046528164380592464],
    [-.63403213105967,-.7711568166983169,-.057623092971932674],
])
prepare_assets=parent.prepare_assets
contact_topics=parent.contact_topics;contact_groups=parent.contact_groups;parse_all_contacts=parent.parse_all_contacts
write_world=parent.write_world;launch=parent.launch

def build_case(case):
    track,_=parent.build_case(case)
    for w in track.windows:
        w.center0=w.center0.copy()
        w.center0[:2]=START[:2]+HORIZONTAL_SCALE*(w.center0[:2]-START[:2])
    track.windows[0].center0+=FIRST_GATE_SIDE_SHIFT_M*track.windows[0].R[:,1]
    for w in track.windows:
        ap=w.aperture;ap.margin=WINDOW_MARGIN
        if ap.kind=='polygon':
            vertices=ap.physical_vertices
            if len(vertices)==4:
                ap.safe_vertices=np.sign(vertices)*(np.max(abs(vertices),axis=0)-WINDOW_MARGIN/2)
            else:
                radius=float(np.linalg.norm(vertices[0]));ap.safe_vertices=vertices*((radius-WINDOW_MARGIN)/radius)
    if case.layout=='height':
        windows=[]
        for i,(w,n) in enumerate(zip(track.windows,HEIGHT_NORMALS)):
            n=n/np.linalg.norm(n);up=np.array([0.,0.,1.]);a=up-(up@n)*n;a/=np.linalg.norm(a)
            R=np.column_stack((a,np.cross(n,a),n))
            windows.append(Window(i,w.center0,R,.5,0.,w.aperture))
        track=replace(track,windows=tuple(windows))
    return replace(track,name='compact_'+track.name),None

def manifest(case):
    result=parent.manifest(case);track,_=build_case(case)
    points=np.array([w.center0 for w in track.windows])
    for row,w in zip(result['windows'],track.windows):
        row['anchor']=w.center0.tolist()
        row['R']=w.R.tolist()
        row['safe_vertices']=None if w.aperture.kind=='circle' else w.aperture.safe_vertices.tolist()
        row['safe_radius']=w.aperture.radius-WINDOW_MARGIN/2 if w.aperture.kind=='circle' else None
    result.update(purpose='user-requested compact interlaced static comparison',
        layout_rule='horizontal anchors contracted to 1/4 around unchanged start; W1 shifted -4m in local horizontal direction; heights, planes and physical apertures unchanged; uniform x500 body plus tracking reserve inset',
        horizontal_scale_relative_to_previous_interlaced=HORIZONTAL_SCALE,
        first_gate_extra_side_shift_m=FIRST_GATE_SIDE_SHIFT_M,
        window_margin_m=WINDOW_MARGIN,x500_body_radius_m=X500_FRAME_RADIUS,tracking_reserve_m=TRACKING_RESERVE_M,
        tracking_reserve_role='uniform design allowance based on observed 0.146m precontact tracking error; not a proven tracking bound or a safety certificate',
        scale_origin=START.tolist(),physical_apertures_scaled=False,historical_anchor_set_preserved=False,
        togt_original_course_reproduction=False,
        inter_gate_distances_m=np.linalg.norm(np.diff(points,axis=0),axis=1).tolist(),
        reference_polyline_length_m=float(np.linalg.norm(np.diff(np.vstack([track.start,points,track.goal]),axis=0),axis=1).sum()))
    if case.layout=='height':
        result.update(layout_rule=result['layout_rule']+'; height variant planes aligned with full 3-D centre traversal directions, not only horizontal yaw',
            height_plane_normals=HEIGHT_NORMALS.tolist(),
            orientation_calibration='retained compact probe_v3 centre reference; fixed full 3-D normals, incoming chord projection >=.01',
            orientation_basis='results/compact_layout_probe_v3_20261106/height_dynamic_togt_center/result.json')
        result.pop('orientation_yaw_deg',None)
    return result
