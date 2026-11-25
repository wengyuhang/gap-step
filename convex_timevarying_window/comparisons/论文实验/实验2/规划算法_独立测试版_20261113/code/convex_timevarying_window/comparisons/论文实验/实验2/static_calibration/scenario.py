"""New layouts, unchanged historical aperture sizes and TOGT safe regions."""
import copy
import json
import shutil
import xml.etree.ElementTree as ET
from dataclasses import asdict
from pathlib import Path
import numpy as np
from convex_dynamic_seven_window_gazebo.x500_togt.experiment import apertures, WINDOW_MARGIN
from convex_timevarying_window.geometry import ConvexAperture
from convex_timevarying_window.comparisons.single_gate_swing.contacts import parse_contacts as legacy_contacts
from convex_timevarying_window.receding_gate_racing import scenario as launch_common
from convex_timevarying_window.single_gate_stop_racing.presentation import clear_view
from nonconvex_timevarying_window.sc_dynatogt.environment import SCWindowTrack
from .. import scenario as mixed

HERE=Path(__file__).resolve().parent; ROOT=mixed.ROOT; ASSETS=HERE/'assets'
Case=mixed.Case; LAYOUTS=mixed.LAYOUTS; START=mixed.START.copy()
SHAPES=('rectangle','circle','pentagon','circle','hexagon','circle','rectangle')
HISTORIC=ROOT/'convex_dynamic_seven_window_gazebo'


class Aperture:
    """Mutable geometry view; exact historical TOGT shape-specific safe map."""
    def __init__(self, original):
        self.kind=original.kind;self.radius=original.radius;self.margin=original.margin
        self.physical_vertices=None if original.physical_vertices is None else original.physical_vertices.copy()
        self.safe_vertices=None if original.safe_vertices is None else original.safe_vertices.copy()
    dimension=ConvexAperture.dimension
    initial_d=ConvexAperture.initial_d
    point_and_jacobian=ConvexAperture.point_and_jacobian
    contains=ConvexAperture.contains
    boundary_distance=ConvexAperture.boundary_distance
    boundary_points=ConvexAperture.boundary_points


class Window(mixed.Window):
    def __init__(self,index,anchor,R,frequency,amplitude,aperture):
        super().__init__(index,anchor,R,frequency,amplitude)
        self.name=f'{index+1:02d}_{SHAPES[index]}'
        self.aperture=Aperture(aperture)
    def point_and_jacobians(self,d,t):
        q,J=self.aperture.point_and_jacobian(d);c,R,dc,dR=self.state_at(t)
        return c+R[:,:2]@q,q,R[:,:2]@J,dc+dR[:,:2]@q
    def to_point(self,d,t):return self.point_and_jacobians(d,t)[0]


def anchors(layout):
    u=np.arange(1,8)/8;theta=np.pi-2*np.pi*u
    if layout=='original':
        rx,ry=27.,21.;z=np.full(7,4.2);y=ry*np.sin(theta)
    elif layout=='slalom':
        rx,ry=33.,20.;z=np.full(7,4.2);y=ry*np.sin(theta)+4*np.sin(2*theta)
    elif layout=='height':
        rx,ry=29.,23.;z=5.2+1.4*np.sin(2*theta);y=ry*np.sin(theta)
    else:raise ValueError(layout)
    return np.column_stack((START[0]+rx+rx*np.cos(theta),START[1]+y,z))


def build_case(case):
    f,a=mixed.LEVELS[case.level];points=anchors(case.layout);windows=[]
    for i,(p,ap) in enumerate(zip(points,apertures())):
        previous=START if i==0 else points[i-1];following=START if i==6 else points[i+1]
        incoming=p-previous;outgoing=following-p;incoming[2]=outgoing[2]=0.
        normal=incoming/np.linalg.norm(incoming)+outgoing/np.linalg.norm(outgoing)
        windows.append(Window(i,p,mixed.make_rotation(normal),f,a,ap))
    return SCWindowTrack('calibrated_'+case.layout+'_'+case.level,START.copy(),START.copy(),tuple(windows),tuple(range(7))),None


def manifest(case):
    tr,_=build_case(case)
    return dict(case=asdict(case),purpose='layout calibration, not held-out evaluation',
        start=tr.start.tolist(),goal=tr.goal.tolist(),return_to_start=True,gate_count=7,
        physical_apertures_source='convex_dynamic_seven_window_gazebo/x500_togt/experiment.py:apertures',
        shape_order=SHAPES,window_margin_m=WINDOW_MARGIN,
        margin_rule='rectangle full dimensions minus margin; circle radius minus margin/2; regular polygon circumradius minus margin',
        rectangular_opening_m=[[2.9,2.3],[2.7,2.16]],circular_radii_m=[1.35,1.25,1.4],
        pentagon_circumradius_m=1.5,hexagon_side_m=1.45,physics_step_s=.001,contact_hz=1000,
        motion='static for calibration; future common translation/swing definitions retained',
        frequency_hz=mixed.LEVELS[case.level][0],amplitude_m=mixed.LEVELS[case.level][1],phase=0.,epoch_sim_s=30.,
        layout_rule='ellipse / gentle S-distorted ellipse / height-varying ellipse; normals from adjacent geometric chords, not flown trajectories',
        reference_polyline_length_m=float(np.linalg.norm(np.diff(np.vstack([tr.start,points(tr),tr.goal]),axis=0),axis=1).sum()),
        windows=[dict(name=w.name,anchor=w.center0.tolist(),R=w.R.tolist(),shape=shape,
                      physical_vertices=None if w.aperture.kind=='circle' else w.aperture.physical_vertices.tolist(),
                      safe_vertices=None if w.aperture.kind=='circle' else w.aperture.safe_vertices.tolist(),
                      radius=w.aperture.radius,safe_radius=w.aperture.radius-WINDOW_MARGIN/2 if w.aperture.kind=='circle' else None,
                      motion=w.motion.kind) for w,shape in zip(tr.windows,SHAPES)])


def points(track):return np.array([w.center0 for w in track.windows])


def prepare_assets():
    ASSETS.mkdir(exist_ok=True);(ASSETS/'build').mkdir(exist_ok=True)
    for name in ('meshes','materials'):
        target=ASSETS/name
        if not target.exists():shutil.copytree(HISTORIC/name,target)
    target=ASSETS/'build/libMixedGateMotion.so'
    if not target.exists():shutil.copyfile(mixed.ASSETS/'build/libMixedGateMotion.so',target)


def contact_topics(track):
    return tuple('/world/convex_seven_dynamic_px4_togt/model/gate_'+w.name+'/link/frame/sensor/frame_contact/contact' for w in track.windows)+(mixed.GROUND_TOPIC,)


def contact_groups(track):return tuple((t,) for t in contact_topics(track))


def parse_all_contacts(path,track,end_sim_s):
    """Protobuf omits nsec=0; count those messages as well, retaining 8 groups."""
    import re
    rows=[]
    for i in range(1,9):
        path_i=Path(path)/f'gate_{i:02d}_contacts.pbtxt'
        text=path_i.read_text(errors='replace') if path_i.exists() else ''
        # Parse a normalized in-memory view; never rewrite native logs.
        text=re.sub(r'(stamp\s*\{\s*sec:\s*\d+)(\s*\})',r'\1\n nsec: 0\2',text)
        class View:
            name=path_i.name
            def exists(self):return path_i.exists()
            def read_text(self,**unused):return text
        row=legacy_contacts(View(),end_sim_s)
        rows.append(dict(raw_logs=[row['raw_log']],sensor_records=[row],**{k:v for k,v in row.items() if k!='raw_log'}))
    return rows


def write_world(track,assets,out):
    tree=ET.parse(HISTORIC/'convex_seven_dynamic_px4_togt.sdf');world=tree.find('world')
    world.find('physics/max_step_size').text='.001'
    models=[m for m in world.findall('model') if m.get('name','').startswith('gate_')]
    if len(models)!=7:raise ValueError('historic course must have seven meshes')
    for model,w in zip(models,track.windows):
        model.set('name','gate_'+w.name)
        model.find('pose').text=' '.join(map(str,np.r_[w.center0,w.angles0_rpy]))
        for old in list(model.findall('plugin')):model.remove(old)
        link=model.find('link')
        for sensor in list(link.findall('sensor')):link.remove(sensor)
        sensor=ET.SubElement(link,'sensor',name='frame_contact',type='contact')
        ET.SubElement(sensor,'always_on').text='true';ET.SubElement(sensor,'update_rate').text='1000'
        ET.SubElement(ET.SubElement(sensor,'contact'),'collision').text='frame_collision'
        plugin=ET.SubElement(model,'plugin',filename='/workspace/build/libMixedGateMotion.so',name='paper_experiment::MixedGateMotion')
        data=dict(center0=' '.join(map(str,w.center0)),base_rpy=' '.join(map(str,w.angles0_rpy)),axis0=' '.join(map(str,w.R[:,0])),axis1=' '.join(map(str,w.R[:,1])),frequency=w.motion.frequency,amplitude=w.motion.amplitude,length=mixed.SWING_LENGTH,kind=w.motion.kind,epoch=30.)
        for key,value in data.items():ET.SubElement(plugin,key).text=str(value)
    ground=world.find("model[@name='ground']/link")
    for sensor in list(ground.findall('sensor')):ground.remove(sensor)
    sensor=ET.SubElement(ground,'sensor',name='ground_contact',type='contact')
    ET.SubElement(sensor,'always_on').text='true';ET.SubElement(sensor,'update_rate').text='1000'
    ET.SubElement(ET.SubElement(sensor,'contact'),'collision').text='ground_collision'
    target=out/'world.sdf';tree.write(target,encoding='unicode');clear_view(target);return target


def launch(track,assets,out):
    prepare_assets();previous=launch_common.write_variant_world;launch_common.write_variant_world=write_world
    try:return launch_common.launch_variant(track,ASSETS,out)
    finally:launch_common.write_variant_world=previous
