"""Shared analytic fixed-plane translation and external-pivot swing course."""
from dataclasses import dataclass,asdict
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import casadi as ca
from scipy.spatial.transform import Rotation
from convex_timevarying_window.dynamic_passage_minco.safety import sine_range
from nonconvex_timevarying_window.sc_dynatogt.environment import SCWindowTrack
from convex_timevarying_window.single_gate_stop_racing.scenario import build_case as old_build,Case as OldCase
from convex_timevarying_window.receding_gate_racing import scenario as common

HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3];ASSETS=HERE/'assets'
START=np.array([-16.,4.,3.2]);PHYSICS_STEP=.001;FRAME_OFFSET=.085;FRAME_THICKNESS=.085
SWING_LENGTH=4./np.sin(np.pi/3)
LEVELS={'static':(.5,0.),'medium':(1.,2.),'high':(3.,4.)}
LAYOUTS=('original','slalom','height')
GROUND_TOPIC='/world/convex_seven_dynamic_px4_togt/model/ground/link/ground_link/sensor/ground_contact/contact'

def interval_scale(lo,hi,axis):
    a,b=np.asarray(axis)*lo,np.asarray(axis)*hi
    return np.minimum(a,b),np.maximum(a,b)

@dataclass(frozen=True)
class Case:
    layout:str
    level:str
    def __post_init__(self):
        if self.layout not in LAYOUTS or self.level not in LEVELS:raise ValueError('unknown frozen condition')

@dataclass(frozen=True)
class Motion:
    kind:str
    rotation_basis:np.ndarray
    frequency:float
    amplitude:float
    phase:float=0.
    length:float=SWING_LENGTH
    translation_enabled:bool=True
    rotation_enabled:bool=False
    scale_enabled:bool=False
    rotation_period:float=1.
    @property
    def translation_period(self):return 1/self.frequency
    @property
    def translation_amplitude(self):return self.rotation_basis[:,0]*self.amplitude
    @property
    def rotation_amplitude(self):return np.zeros(3)
    @property
    def angle(self):return float(np.arcsin(self.amplitude/self.length))
    def rotation(self,t):return np.zeros(3),np.zeros(3)
    def scale(self,t):return 1.,0.
    def local(self,t):
        z=2*np.pi*self.frequency*np.asarray(t)+self.phase;s=np.sin(z);c=np.cos(z);omega=2*np.pi*self.frequency
        if self.kind=='translation':
            p=np.stack((self.amplitude*s,np.zeros_like(s),np.zeros_like(s)),axis=-1)
            v=np.stack((self.amplitude*omega*c,np.zeros_like(s),np.zeros_like(s)),axis=-1)
        else:
            theta=self.angle*s;rate=self.angle*omega*c
            p=np.stack((self.length*np.sin(theta),self.length*(1-np.cos(theta)),np.zeros_like(s)),axis=-1)
            v=np.stack((self.length*np.cos(theta)*rate,self.length*np.sin(theta)*rate,np.zeros_like(s)),axis=-1)
        return p,v
    def translation(self,t):
        p,v=self.local(t);return p@self.rotation_basis.T,v@self.rotation_basis.T
    def local_bounds(self,a,b):
        lo,hi=sine_range(2*np.pi*self.frequency*a+self.phase,2*np.pi*self.frequency*b+self.phase)
        if self.kind=='translation':return np.array([self.amplitude*lo,0.,0.]),np.array([self.amplitude*hi,0.,0.])
        tl,th=self.angle*lo,self.angle*hi
        sl,sh=sine_range(tl,th);cl,ch=sine_range(tl+np.pi/2,th+np.pi/2)
        return np.array([self.length*sl,self.length*(1-ch),0.]),np.array([self.length*sh,self.length*(1-cl),0.])
    def translation_bounds(self,a,b):
        lo,hi=self.local_bounds(a,b);center=(lo+hi)/2;radius=(hi-lo)/2
        return self.rotation_basis@center-abs(self.rotation_basis)@radius,self.rotation_basis@center+abs(self.rotation_basis)@radius

class Aperture:
    def __init__(self,kind):
        self.kind=kind
        if kind=='circle':self.radius=.9*(.2792307692307692/.127)/2
        else:self.physical_vertices=np.array([[-1.45,-1.15],[1.45,-1.15],[1.45,1.15],[-1.45,1.15]])
    def contains(self,q):
        q=np.asarray(q);return bool(np.linalg.norm(q)<=self.radius+1e-10) if self.kind=='circle' else bool(np.all(abs(q)<=[1.45+1e-10,1.15+1e-10]))
    def boundary_distance(self,q):
        q=np.asarray(q)
        if self.kind=='circle':return abs(self.radius-np.linalg.norm(q,axis=-1))
        delta=abs(q)-[1.45,1.15]
        return np.where(np.any(delta>0,axis=-1),np.linalg.norm(np.maximum(delta,0),axis=-1),-np.max(delta,axis=-1))
    def boundary_points(self,count=128):
        if self.kind=='polygon':return self.physical_vertices.copy()
        t=2*np.pi*np.arange(count)/count;return self.radius*np.c_[np.cos(t),np.sin(t)]

class Window:
    def __init__(self,index,anchor,R,frequency,amplitude):
        self.name=f'{index+1:02d}_'+('translation' if index%2==0 else 'swing')
        self.center0=np.asarray(anchor,float);self.R=np.asarray(R,float)
        self.angles0_rpy=Rotation.from_matrix(R).as_euler('xyz')
        self.aperture=Aperture('polygon' if index%2==0 else 'circle')
        self.motion=Motion('translation' if index%2==0 else 'swing',self.R,frequency,amplitude)
    def state_at(self,t):
        d,v=self.motion.translation(t);return self.center0+d,self.R,v,np.zeros((3,3))
    def world_to_local(self,p,t):
        c,R,*_=self.state_at(t);q=R.T@(np.asarray(p)-c);return q[:2],float(q[2])
    def boundary_at(self,t,circle_samples=128):
        c,R,*_=self.state_at(t);return c+self.aperture.boundary_points(circle_samples)@R[:,:2].T

def pose(window,t):
    m=window.motion;z=2*np.pi*m.frequency*t+m.phase
    if m.kind=='translation':d=ca.vertcat(m.amplitude*ca.sin(z),0,0)
    else:
        theta=m.angle*ca.sin(z);d=ca.vertcat(m.length*ca.sin(theta),m.length*(1-ca.cos(theta)),0)
    return ca.DM(window.center0)+ca.DM(window.R)@d,ca.DM(window.R)

def centers(w,times):return w.center0+w.motion.translation(np.asarray(times))[0]

def local_coordinates(window,positions,times):return (np.asarray(positions)-centers(window,times))@window.R

def make_rotation(normal):
    normal=np.asarray(normal,float).copy();normal[2]=0;normal/=np.linalg.norm(normal)
    return np.column_stack(([-normal[1],normal[0],0.],[0.,0.,1.],normal))

def build_case(case):
    f,a=LEVELS[case.level]
    if case.layout=='original':
        old,_=old_build(OldCase(1.,0.,0.));anchors=np.array([w.center0 for w in old.windows])
    elif case.layout=='slalom':anchors=np.array([[-2,4,4],[14,18,4],[30,4,4],[46,18,4],[46,-12,4],[22,-26,4],[-2,-12,4]],float)
    else:anchors=np.array([[-2,3,4],[20,-12,8],[45,-8,4],[60,15,9],[35,35,4],[10,32,7],[-8,20,4]],float)
    windows=[]
    for i,p in enumerate(anchors):
        before=START if i==0 else anchors[i-1];after=START if i==6 else anchors[i+1]
        incoming=p-before;outgoing=after-p;incoming[2]=outgoing[2]=0
        direction=incoming/np.linalg.norm(incoming)+outgoing/np.linalg.norm(outgoing)
        if np.linalg.norm(direction)<1e-6:direction=incoming
        windows.append(Window(i,p,make_rotation(direction),f,a))
    track=SCWindowTrack('mixed_'+case.layout+'_'+case.level,START.copy(),START.copy(),tuple(windows),tuple(range(7)))
    return track,None

def contact_groups(track):
    groups=[]
    for w in track.windows:
        names=('frame_contact',) if w.aperture.kind=='circle' else ('frame_contact','bottom_contact','left_contact','right_contact')
        groups.append(tuple('/world/convex_seven_dynamic_px4_togt/model/gate_'+w.name+'/link/frame/sensor/'+name+'/contact' for name in names))
    return tuple(groups)+((GROUND_TOPIC,),)

def contact_topics(track):return tuple(t for g in contact_groups(track) for t in g)

def parse_all_contacts(path,track,end_sim_s):
    from convex_timevarying_window.comparisons.single_gate_swing.contacts import parse_contacts
    result=[];index=1
    for group in contact_groups(track):
        records=[parse_contacts(path/f'gate_{i:02d}_contacts.pbtxt',end_sim_s) for i in range(index,index+len(group))];index+=len(group)
        stamps=[r['first_contact_sim_time_s'] for r in records if r['first_contact_sim_time_s'] is not None]
        result.append(dict(raw_logs=[r['raw_log'] for r in records],sensor_records=records,contact_pair_count=sum(r['contact_pair_count'] for r in records),
            first_contact_sim_time_s=min(stamps) if stamps else None,collision_pairs=sorted({p for r in records for p in r['collision_pairs']}),interval_sim_s=[30.,end_sim_s]))
    return result

def manifest(case):
    tr,_=build_case(case)
    return dict(case=asdict(case),start=tr.start.tolist(),goal=tr.goal.tolist(),return_to_start=True,
        gate_count=7,translation_gates=[1,3,5,7],swing_gates=[2,4,6],phase=0.,frequency_hz=LEVELS[case.level][0],amplitude_m=LEVELS[case.level][1],
        swing='fixed plane; centre circles about external pivot; theta=asin(A/L)*sin(2*pi*f*t), L fixed',
        swing_length_m=SWING_LENGTH,swing_angle_deg=float(np.degrees(tr.windows[1].motion.angle)),
        static_reference='swing theta=0 at bottom; physical opening and pivot length unchanged',
        frame_offset_m=FRAME_OFFSET,frame_thickness_m=FRAME_THICKNESS,
        rectangular_opening_m=[2.9,2.3],circular_opening_diameter_m=2*tr.windows[1].aperture.radius,
        physics_step_s=PHYSICS_STEP,contact_hz=1000,epoch_sim_s=30.,
        reference_polyline_length_m=float(np.linalg.norm(np.diff(np.vstack([tr.start,*[w.center0 for w in tr.windows],tr.goal]),axis=0),axis=1).sum()),
        windows=[dict(name=w.name,anchor=w.center0.tolist(),R=w.R.tolist(),motion=w.motion.kind,pivot=(w.center0+SWING_LENGTH*w.R[:,1]).tolist() if w.motion.kind=='swing' else None) for w in tr.windows])

def write_world(track,assets,out):
    tree=ET.parse(ROOT/'convex_dynamic_seven_window_gazebo/convex_seven_dynamic_px4_togt.sdf');world=tree.find('world')
    world.find('physics/max_step_size').text=str(PHYSICS_STEP)
    for model in list(world.findall('model')):
        if model.get('name','').startswith('gate_') or model.get('name','').startswith(('wall','roof','ceiling','light_panel')):world.remove(model)
    ground=world.find("model[@name='ground']/link")
    sensor=ET.SubElement(ground,'sensor',name='ground_contact',type='contact');ET.SubElement(sensor,'always_on').text='true';ET.SubElement(sensor,'update_rate').text='1000';ET.SubElement(ET.SubElement(sensor,'contact'),'collision').text='ground_collision'
    for w in track.windows:
        model=ET.SubElement(world,'model',name='gate_'+w.name);ET.SubElement(model,'static').text='true';ET.SubElement(model,'pose').text=' '.join(map(str,np.r_[w.center0,w.angles0_rpy]));link=ET.SubElement(model,'link',name='frame')
        parts=[]
        if w.aperture.kind=='circle':parts=[('circle',None,None)]
        else:
            x,y=1.45+FRAME_OFFSET,1.15+FRAME_OFFSET
            parts=[('top',[0,y,0],[2*x,2*FRAME_THICKNESS,2*FRAME_THICKNESS]),('bottom',[0,-y,0],[2*x,2*FRAME_THICKNESS,2*FRAME_THICKNESS]),('left',[-x,0,0],[2*FRAME_THICKNESS,2*y,2*FRAME_THICKNESS]),('right',[x,0,0],[2*FRAME_THICKNESS,2*y,2*FRAME_THICKNESS])]
        names=[]
        for name,offset,size in parts:
            for kind in ('collision','visual'):
                obj=ET.SubElement(link,kind,name=name+'_'+kind)
                if offset is not None:ET.SubElement(obj,'pose').text=' '.join(map(str,offset+[0,0,0]))
                geo=ET.SubElement(obj,'geometry')
                if size is None:ET.SubElement(ET.SubElement(geo,'mesh'),'uri').text='/workspace/circle_frame.stl'
                else:ET.SubElement(ET.SubElement(geo,'box'),'size').text=' '.join(map(str,size))
                if kind=='visual':ET.SubElement(ET.SubElement(obj,'material'),'diffuse').text='.1 .5 .9 1' if w.motion.kind=='swing' else '.9 .5 .1 1'
                else:names.append(name+'_collision')
        for i,name in enumerate(names):
            sensor_name='frame_contact' if i==0 else name.removesuffix('_collision')+'_contact'
            sensor=ET.SubElement(link,'sensor',name=sensor_name,type='contact');ET.SubElement(sensor,'always_on').text='true';ET.SubElement(sensor,'update_rate').text='1000'
            ET.SubElement(ET.SubElement(sensor,'contact'),'collision').text=name
        plugin=ET.SubElement(model,'plugin',filename='/workspace/build/libMixedGateMotion.so',name='paper_experiment::MixedGateMotion')
        data=dict(center0=' '.join(map(str,w.center0)),base_rpy=' '.join(map(str,w.angles0_rpy)),axis0=' '.join(map(str,w.R[:,0])),axis1=' '.join(map(str,w.R[:,1])),frequency=w.motion.frequency,amplitude=w.motion.amplitude,length=SWING_LENGTH,kind=w.motion.kind,epoch=30.)
        for key,value in data.items():ET.SubElement(plugin,key).text=str(value)
    target=out/'world.sdf';tree.write(target,encoding='unicode');return target

def prepare_assets():
    import trimesh
    ASSETS.mkdir(exist_ok=True);(ASSETS/'convex_seven_dynamic_px4_togt.sdf').touch(exist_ok=True)
    mesh=ASSETS/'circle_frame.stl'
    if not mesh.exists():trimesh.creation.torus(major_radius=Aperture('circle').radius+FRAME_OFFSET,minor_radius=FRAME_THICKNESS,major_sections=256,minor_sections=16).export(mesh)

def launch(track,assets,out):
    old=common.write_variant_world;common.write_variant_world=write_world
    try:return common.launch_variant(track,ASSETS,out)
    finally:common.write_variant_world=old
