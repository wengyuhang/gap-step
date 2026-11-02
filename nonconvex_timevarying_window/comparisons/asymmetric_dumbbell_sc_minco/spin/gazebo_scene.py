"""Exact finite-segment capsule frame and R0*Rz spin; isolated scene adapter."""
from pathlib import Path
import os
import subprocess
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
from convex_timevarying_window.receding_gate_racing import scenario as common

ASSETS = Path(__file__).resolve().parents[4]/'convex_dynamic_seven_window_gazebo'
FRAME_TOPIC = '/world/convex_seven_dynamic_px4_togt/model/gate_01_spinning_dumbbell/link/frame/sensor/frame_contact/contact'
GROUND_TOPIC = '/world/convex_seven_dynamic_px4_togt/model/ground/link/ground_link/sensor/ground_contact/contact'
CONTAINER = 'convex_seven_togt_experiment'
PARTITION = CONTAINER+'_partition'


def field(parent, name, value):
    ET.SubElement(parent, name).text = str(value)


def write_world(track, assets, outdir):
    tree = ET.parse(assets/'convex_seven_dynamic_px4_togt.sdf')
    world = tree.find('world')
    world.find('physics/max_step_size').text = '.001'
    for model in list(world.findall('model')):
        if model.get('name', '').startswith('gate_'):
            world.remove(model)
    w = track.windows[0]; m = w.motion
    model = ET.SubElement(world, 'model', name='gate_01_spinning_dumbbell')
    field(model, 'static', 'true')
    field(model, 'pose', ' '.join(map(str, np.r_[w.center0, w.angles0_rpy])))
    link = ET.SubElement(model, 'link', name='frame')
    collision_names = []
    vertices = w.aperture.physical_vertices
    # Cylinder plus a sphere at each vertex = radius-.085 capsule union.
    # This matches the planner's distance to finite polygon segments.
    for i, (a, b) in enumerate(zip(vertices, np.roll(vertices, -1, axis=0))):
        delta = b-a; length = np.linalg.norm(delta)
        poses = [('edge', np.r_[(a+b)/2, 0., 0., np.pi/2, np.arctan2(delta[1], delta[0])]),
                 ('vertex', np.r_[a, 0., 0., 0., 0.])]
        for kind, pose in poses:
            name = f'{kind}_{i:02d}'
            collision_names.append(name)
            for tag in ('collision', 'visual'):
                item = ET.SubElement(link, tag, name=name)
                field(item, 'pose', ' '.join(map(str, pose)))
                geom = ET.SubElement(item, 'geometry')
                shape = ET.SubElement(geom, 'cylinder' if kind=='edge' else 'sphere')
                field(shape, 'radius', '.085')
                if kind=='edge': field(shape, 'length', length)
                if tag=='visual':
                    material=ET.SubElement(item,'material')
                    field(material,'diffuse','0.05 0.55 0.95 1')
                    field(material,'ambient','0.05 0.4 0.7 1')
    sensor=ET.SubElement(link,'sensor',name='frame_contact',type='contact')
    field(sensor,'always_on','true');field(sensor,'update_rate',1000);field(sensor,'topic',FRAME_TOPIC)
    contact=ET.SubElement(sensor,'contact')
    for name in collision_names: field(contact,'collision',name)
    ground=world.find("model[@name='ground']/link")
    sensor=ET.SubElement(ground,'sensor',name='ground_contact',type='contact')
    field(sensor,'always_on','true');field(sensor,'update_rate',1000);field(sensor,'topic',GROUND_TOPIC)
    field(ET.SubElement(sensor,'contact'),'collision','ground_collision')
    plugin=ET.SubElement(model,'plugin',name='NonconvexSpinMotion',
                         filename='/workspace/build/nonconvex_spin/libNonconvexSpinMotion.so')
    for key, value in dict(center0=' '.join(map(str,w.center0)),
            base_rpy=' '.join(map(str,w.angles0_rpy)),
            translation_amplitude=' '.join(map(str,m.translation_amplitude)),
            frequency=1/m.translation_period,spin_hz=m.spin_hz,phase=m.phase,
            spin_phase=m.spin_phase,motion_start_time=30).items(): field(plugin,key,value)
    world.find("model[@name='start_finish_marker']/pose").text=' '.join(map(str,np.r_[track.goal,0,0,0]))
    path=(outdir/'world.sdf').resolve();tree.write(path,encoding='unicode')
    return path


def launch(track, assets, outdir):
    previous=common.write_variant_world
    common.write_variant_world=write_world
    try: common.launch_variant(track,ASSETS,outdir)
    finally: common.write_variant_world=previous
    if os.environ.get('GZ_SHOW_GUI')=='1':
        camera=ASSETS.parent/'convex_timevarying_window/single_gate_stop_racing/camera_view.py'
        subprocess.run(['docker','cp',str(camera),CONTAINER+':/tmp/camera_view.py'],check=True)
        subprocess.run(['docker','exec','-d',CONTAINER,'bash','-lc',
                        'exec python3 /tmp/camera_view.py >/tmp/camera_view.log 2>&1'],check=True)


def manifest(track):
    w=track.windows[0];m=w.motion
    return dict(start=track.start.tolist(),goal=track.goal.tolist(),gate_anchor=w.center0.tolist(),
        vertices=w.aperture.physical_vertices.tolist(),frequency_hz=1/m.translation_period,
        amplitude_m=float(np.linalg.norm(m.translation_amplitude)),spin_hz=m.spin_hz,
        motion='c=c0+A sin(2 pi f t); R=R0 Rz(2 pi spin_hz t)',race_epoch_sim_s=30.,
        frame='union of radius .085 m finite segment capsules',physics_step_s=.001)
