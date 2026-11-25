import copy
from dataclasses import asdict
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from convex_dynamic_seven_window_gazebo.x500_togt.experiment import build_track
from convex_timevarying_window.receding_gate_racing import scenario as common
from nonconvex_timevarying_window.sc_dynatogt.environment import SCWindowTrack
from 复现.polynomial_dynamic_racing_2023.px4_fixed_plane.scenario import Case,PlanarMotion


def build_case(case):
    original,cfg=build_track();first=copy.deepcopy(original.windows[0])
    first.center0=np.array([-2.42,-3.52,6.48]);first.angles0_rpy=np.array([0.,-np.pi/2,0.])
    second=copy.deepcopy(first);second.center0=np.array([18.,8.,6.48])
    for i,w in enumerate((first,second)):
        w.motion=PlanarMotion(np.zeros(3),1/case.frequency,case.phase+i*np.pi)
        axis=w.state_at(0.)[1][:,1]
        w.motion=PlanarMotion(case.amplitude*axis,1/case.frequency,case.phase+i*np.pi)
    return SCWindowTrack('two_staggered_translation',original.start.copy(),np.array([24.,8.,6.48]),
                         (first,second),(0,1)),cfg


def manifest(case):
    track,_=build_case(case)
    return dict(case=asdict(case),gate_count=2,start=track.start.tolist(),goal=track.goal.tolist(),
        anchors=[w.center0.tolist() for w in track.windows],
        anchor_distance_m=float(np.linalg.norm(track.windows[1].center0-track.windows[0].center0)),
        plane_spacing_m=20.42,lateral_offset_m=11.52,aperture_m=[2.9,2.3],
        motion='fixed parallel planes; horizontal sine; phase offset pi',
        planner='joint two-gate hard_v2; three pieces, two free interior P/V knots')


def write_world(track,assets,out):
    tree=ET.parse(assets/'convex_seven_dynamic_px4_togt.sdf');world=tree.find('.//world')
    template=copy.deepcopy(world.find("model[@name='gate_01_W1_rectangle']"))
    for m in list(world.findall('model')):
        if m.get('name','').startswith('gate_'):world.remove(m)
    for i,w in enumerate(track.windows):
        # Preserve the shared logger's scoped contact-topic names. Both physical
        # meshes are rectangles; the second legacy model suffix is only a name.
        m=copy.deepcopy(template);m.set('name',('gate_01_W1_rectangle','gate_02_W2_circle')[i])
        m.find('.//sensor/topic').text=f'/convex_seven/gate_{i+1:02d}/contacts'
        p=m.find("plugin[@name='convex_dynamic_course::PeriodicGateMotion']")
        values=dict(center0=w.center0,base_rpy=w.angles0_rpy,translation_amplitude=w.motion.translation_amplitude,
                    rotation_amplitude=np.zeros(3))
        for k,v in values.items():p.find(k).text=' '.join(map(str,v))
        for k,v in dict(translation_period=w.motion.translation_period,rotation_period=1.,phase=w.motion.phase).items():
            p.find(k).text=str(v)
        phase=p.find('translation_phase_offsets')
        if phase is None:phase=ET.SubElement(p,'translation_phase_offsets')
        phase.text='0 0 0'
        m.find('pose').text=' '.join(map(str,np.r_[w.state_at(0.)[0],w.angles0_rpy]))
        world.append(m)
    path=(out/'world.sdf').resolve();tree.write(path,encoding='unicode');return path


def launch(track,assets,out):
    previous=common.write_variant_world;common.write_variant_world=write_world
    try:return common.launch_variant(track,assets,out)
    finally:common.write_variant_world=previous
