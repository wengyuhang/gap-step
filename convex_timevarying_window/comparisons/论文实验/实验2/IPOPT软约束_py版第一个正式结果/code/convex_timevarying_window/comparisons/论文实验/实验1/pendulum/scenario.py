"""Result-local circle world, original pendulum ODE, same physical x500."""
from dataclasses import asdict
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import trimesh
from convex_timevarying_window.receding_gate_racing import scenario as common
from .motion import build,Case,DIAMETER,SCALE,PIVOT,R0
from .planner import Config

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[4]
ASSETS=HERE/'assets'
PHYSICS_STEP=.001
GATE_TOPIC='/world/convex_seven_dynamic_px4_togt/model/gate_01_pendulum/link/frame_link/sensor/frame_contact/contact'
GROUND_TOPIC='/world/convex_seven_dynamic_px4_togt/model/ground/link/ground_link/sensor/ground_contact/contact'

def build_case(case):return build(case),Config()
def manifest(case):
    track=build(case);m=track.windows[0].motion
    return dict(case=asdict(case),start=track.start.tolist(),goal=track.goal.tolist(),pivot=PIVOT.tolist(),length_m=2.,
        motion='author damped pendulum ODE, shared cubic table; circular gate roll retained',diameter_m=DIAMETER,
        scale=SCALE,matching='gate/propeller diameter, NOT verified full-body ratio',physics_step_s=PHYSICS_STEP,
        initial_angular_velocity_rad_s=0.,damping=.1,mass=2.,gravity=9.81,epoch_sim_s=30.,
        maximum_table_speed_mps=float(np.max(abs(m.theta(m.times,1)))*2))

def write_world(track,assets,out):
    tree=ET.parse(ROOT/'convex_dynamic_seven_window_gazebo/convex_seven_dynamic_px4_togt.sdf');world=tree.find('world')
    world.find('physics/max_step_size').text=str(PHYSICS_STEP)
    for model in list(world.findall('model')):
        if model.get('name','').startswith('gate_'):world.remove(model)
    ground=world.find("model[@name='ground']/link")
    for material in ground.findall('visual/material/pbr'):ground.find('visual/material').remove(material)
    sensor=ET.SubElement(ground,'sensor',name='ground_contact',type='contact');ET.SubElement(sensor,'always_on').text='true';ET.SubElement(sensor,'update_rate').text='1000'
    ET.SubElement(ET.SubElement(sensor,'contact'),'collision').text='ground_collision'
    w=track.windows[0];cfg=Config();ASSETS.mkdir(exist_ok=True)
    # Docker nested read-only bind mount requires an existing target file.
    (ASSETS/'convex_seven_dynamic_px4_togt.sdf').touch(exist_ok=True)
    # Exact circular torus is approximated by 256x16 triangles; inner chord error <0.1mm.
    mesh=ASSETS/'circle_frame.stl'
    if not mesh.exists():trimesh.creation.torus(major_radius=w.aperture.radius+cfg.frame_offset,minor_radius=cfg.frame_thickness,major_sections=256,minor_sections=16).export(mesh)
    table=out/'pendulum_table.txt';w.motion.write_table(table)
    c,R,*_=w.state_at(0.)
    from scipy.spatial.transform import Rotation
    model=ET.SubElement(world,'model',name='gate_01_pendulum');ET.SubElement(model,'static').text='true';ET.SubElement(model,'pose').text=' '.join(map(str,np.r_[c,Rotation.from_matrix(R).as_euler('xyz')]))
    link=ET.SubElement(model,'link',name='frame_link')
    for kind in ('collision','visual'):
        obj=ET.SubElement(link,kind,name='frame_'+kind);geo=ET.SubElement(ET.SubElement(obj,'geometry'),'mesh');ET.SubElement(geo,'uri').text='/workspace/circle_frame.stl'
        if kind=='visual':ET.SubElement(ET.SubElement(obj,'material'),'diffuse').text='.1 .5 .9 1'
    sensor=ET.SubElement(link,'sensor',name='frame_contact',type='contact');ET.SubElement(sensor,'always_on').text='true';ET.SubElement(sensor,'update_rate').text='1000';ET.SubElement(ET.SubElement(sensor,'contact'),'collision').text='frame_collision'
    plugin=ET.SubElement(model,'plugin',filename='/workspace/build/libPendulumGateMotion.so',name='paper_experiment::PendulumGateMotion')
    for key,value in {'pivot':' '.join(map(str,PIVOT)),'length':'2','epoch':'30','table':'/workspace/pendulum_table.txt'}.items():ET.SubElement(plugin,key).text=value
    # Assets contain one immutable ODE table per frozen angle; coordinator runs serially.
    import shutil
    shutil.copyfile(table,ASSETS/'pendulum_table.txt')
    target=out/'world.sdf';tree.write(target,encoding='unicode');return target

def launch(track,assets,out):
    old=common.write_variant_world;common.write_variant_world=write_world
    try:return common.launch_variant(track,ASSETS,out)
    finally:common.write_variant_world=old
