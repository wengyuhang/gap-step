"""Historical interlaced anchor set with W4/W5 exchanged, never an oval."""
from dataclasses import asdict
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from nonconvex_timevarying_window.sc_dynatogt.environment import SCWindowTrack
from convex_timevarying_window.single_gate_stop_racing.scenario import build_case as recent_build,Case as RecentCase
from convex_dynamic_seven_window_gazebo.x500_togt.experiment import apertures
from ..static_calibration import scenario as common

ROOT=common.ROOT;HERE=Path(__file__).resolve().parent;ASSETS=common.ASSETS
Case=common.Case;LAYOUTS=common.LAYOUTS;START=common.START.copy()
prepare_assets=common.prepare_assets
contact_topics=common.contact_topics;contact_groups=common.contact_groups;parse_all_contacts=common.parse_all_contacts

# Static scene calibration, not a change to the planner: point a vertical
# gate toward its nominal crossing velocity, while preserving a positive
# projection on the incoming chord. Source: retained swapped-layout probes.
ANCHOR_ORDER=(0,1,2,4,3,5,6)
ORIENTATION_YAW_DEG={
    'original':[-8.598604408024174,8.829926574685324,-176.008319584411,153.1345084587107,179.2202829195115,15.513453309093764,-129.8892601472362],
    'slalom':[-8.98474435612855,8.542911775849177,-174.9106769942214,160.96898636251942,175.9214009316972,15.13468052504739,-132.15200623302115],
    'height':[-14.686231736345333,17.8211133618127,-179.42703265514285,157.3804104014106,175.402339841984,11.994069501145136,-129.8892601472362],
}


def build_case(case):
    if case.level!='static':raise ValueError('this module first calibrates static layouts; dynamic adaptation is not yet validated')
    old,_=recent_build(RecentCase(1.,0.,0.));points=np.array([w.center0 for w in old.windows])[list(ANCHOR_ORDER)]
    if case.layout=='slalom':
        points+=np.array([[1.,-2.,0.],[-2.,2.,.4],[2.,-2.,.3],[-2.,-1.,-.3],[1.,1.,.6],[-1.,2.,.5],[1.,-1.,-.2]])
    elif case.layout=='height':points[:,2]+=np.array([0.,1.8,.7,.8,1.6,2.,-.6])
    elif case.layout!='original':raise ValueError(case.layout)
    rotations=[]
    for angle in ORIENTATION_YAW_DEG[case.layout]:
        a=np.radians(angle);normal=np.array([np.cos(a),np.sin(a),0.]);up=np.array([0.,0.,1.])
        rotations.append(np.column_stack((up,np.cross(normal,up),normal)))
    windows=[common.Window(i,p,R,.5,0.,ap) for i,(p,R,ap) in enumerate(zip(points,rotations,apertures()))]
    return SCWindowTrack('historical_interlaced_'+case.layout,START.copy(),START.copy(),tuple(windows),tuple(range(7))),None


def manifest(case):
    # Keep the complete geometry/safety metadata, replace only layout metadata.
    result=common.manifest(case);track,_=build_case(case)
    for row,w in zip(result['windows'],track.windows):
        row['anchor']=w.center0.tolist();row['R']=w.R.tolist()
    result.update(purpose='user-corrected interlaced static-layout calibration',
        layout_rule='historical interlaced anchor set with W4/W5 positions exchanged; 2x spacing and -5m W1 shift; small position/height variants; calibrated vertical planes',
        anchor_source='convex_timevarying_window/single_gate_stop_racing/scenario.py:build_case(Case(1,0,0))',
        historical_gate_orientations_preserved=False,orientation_yaw_deg=ORIENTATION_YAW_DEG[case.layout],
        orientation_calibration='swapped-layout nominal centre velocity with incoming-chord cosine at least .01; geometry only, no objective/controller modification',
        historical_anchor_order=[i+1 for i in ANCHOR_ORDER],historical_anchor_set_preserved=case.layout=='original',original_layout_exact_positions=False,
        motion='static only; this adapter does not claim validated dynamic motion',
        reference_polyline_length_m=float(np.linalg.norm(np.diff(np.vstack([track.start,*[w.center0 for w in track.windows],track.goal]),axis=0),axis=1).sum()))
    return result


def write_world(track,assets,out):return common.write_world(track,assets,out)


def launch(track,assets,out):return common.launch(track,assets,out)
