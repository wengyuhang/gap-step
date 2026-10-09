"""Original W1 geometry, fixed plane and in-plane sinusoidal translation."""
from dataclasses import asdict
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from nonconvex_timevarying_window.sc_dynatogt.environment import SCWindowTrack
from 复现.polynomial_dynamic_racing_2023.px4_fixed_plane import scenario as seven
from convex_timevarying_window.receding_gate_racing import scenario as common

Case=seven.Case
ASSETS=Path('/home/jack/wyh/时变窗口/convex_dynamic_seven_window_gazebo')


def build_case(case):
    original,cfg=seven.build_case(case)
    gate=original.windows[0]
    start=original.start.copy()
    normal=gate.state_at(0.)[1][:,2]
    direction=np.sign((start-gate.center0)@normal)
    goal=gate.center0-6.*direction*normal
    track=SCWindowTrack('single_original_W1_translation_x500',start,goal,(gate,),(0,))
    return track,cfg


def manifest(case):
    track,_=build_case(case);w=track.windows[0]
    return dict(case=asdict(case),start=track.start.tolist(),goal=track.goal.tolist(),
                gate_anchor=w.center0.tolist(),base_rpy=w.angles0_rpy.tolist(),
                aperture_vertices=w.aperture.physical_vertices.tolist(),
                amplitude_m=case.amplitude,frequency_hz=case.frequency,
                peak_speed_mps=2*np.pi*case.frequency*case.amplitude,
                peak_acceleration_mps2=(2*np.pi*case.frequency)**2*case.amplitude,
                motion='fixed plane, horizontal in-plane sine',gate_count=1)


def write_world(track,assets,outdir):
    w=track.windows[0];m=w.motion
    case=Case(1/m.translation_period,float(np.linalg.norm(m.translation_amplitude)),m.phase)
    full,_=seven.build_case(case)
    path=seven.write_world(full,assets,outdir)
    tree=ET.parse(path);world=tree.find('.//world')
    for model in list(world.findall('model')):
        name=model.get('name','')
        if name.startswith('gate_') and not name.startswith('gate_01_'):world.remove(model)
    gate_models=[x for x in world.findall('model') if x.get('name','').startswith('gate_')]
    assert len(gate_models)==1
    tree.write(path,encoding='unicode')
    return path


def launch(track,assets,outdir):
    previous=common.write_variant_world
    common.write_variant_world=write_world
    try:return common.launch_variant(track,ASSETS,outdir)
    finally:common.write_variant_world=previous
