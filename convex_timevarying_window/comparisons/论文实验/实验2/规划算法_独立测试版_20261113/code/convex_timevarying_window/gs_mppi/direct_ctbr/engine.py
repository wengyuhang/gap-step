"""One 17-state CTBR model for CPU execution and GPU prediction."""
import ctypes
from dataclasses import replace
import hashlib
from pathlib import Path
import subprocess
import numpy as np
from convex_timevarying_window.dynamic_gate_mppi_v3.config import Config
from convex_timevarying_window.dynamic_gate_mppi_v3.geometry import Geometry
from convex_timevarying_window.dynamic_gate_mppi_v3.rollout import MASS
from convex_timevarying_window.three_phase_mppi.engine import cuda_source

HERE=Path(__file__).parent
PHYSICS=Path('convex_timevarying_window/dynamic_gate_mppi_v3/native/rollout.cpp')

class Engine:
    def __init__(self,track,gpu=True,racing_guidance=False,coupled_racing=False):
        self.track=track;self.config=replace(Config(),axis_workspace_bounds=True)
        self.geometry=Geometry(track,self.config);self.gates=len(track.order)
        self.geom=np.zeros((self.gates,32));self.signs=[];previous=track.start
        for i,index in enumerate(track.order):
            w=track.windows[index];c,r,*_=w.state_at(0);self.signs.append(-np.sign(r[:,2]@(previous-c)));previous=c
            frame=self.geometry.frames[index]
            if frame.kind=='circle':self.geom[i,1]=frame.radius
            else:self.geom[i,:3]=[1,0,len(frame.physical_vertices)];self.geom[i,3:3+2*len(frame.physical_vertices)]=frame.physical_vertices.ravel()
        self.signs=np.asarray(self.signs);self.speeds=np.ascontiguousarray(self.geometry.frame_speeds[list(track.order)])
        c=self.config;self.cfg=np.array([c.radius,c.error_margin,c.frame_thickness,c.speed,c.acceleration,np.cos(np.deg2rad(c.tilt)),*c.lower,*c.upper,1.,*track.goal,float(racing_guidance),float(coupled_racing)])
        physical=PHYSICS.read_text().split('extern "C" void v3_integrate')[0]
        code='#define V3_CTBR 1\n'+physical+(HERE/'kernel.cpp').read_text()
        build=HERE/'build';build.mkdir(exist_ok=True);key=hashlib.sha256(code.encode()).hexdigest()[:16];lib=build/f'{key}.so'
        if not lib.exists():subprocess.run(['g++','-O3','-std=c++17','-shared','-fPIC','-x','c++','-','-o',str(lib)],input=code,text=True,check=True)
        self.lib=ctypes.CDLL(str(lib));p=ctypes.c_void_p;i=ctypes.c_int;d=ctypes.c_double
        self.lib.direct_integrate.argtypes=[p,p,d];self.lib.direct_integrate.restype=None
        self.lib.direct_seed.argtypes=[i,i]+[p]*6+[i,i,d,p];self.lib.direct_seed.restype=None
        self.lib.direct_rollout.argtypes=[i]*3+[p]*9+[i]*4+[d]+[p]*3;self.lib.direct_rollout.restype=None
        self.cp=None
        if gpu:
            import cupy as cp
            self.cp=cp;self.module=cp.RawModule(code='#define DIRECT_GPU 1\n'+cuda_source(code),options=('--std=c++17',));self.kernel=self.module.get_function('direct_rollout')
            self.constants=[cp.asarray(x,dtype=cp.float32) for x in (self.geom,self.speeds,self.signs,self.cfg)]
    def poses(self,times):
        c,r=self.geometry.poses(times);order=list(self.track.order)
        return np.ascontiguousarray(np.concatenate((c[:,order],r[:,order].reshape(len(times),self.gates,9)),axis=-1))
    def integrate(self,state,action,dt=.005):
        x=np.array(state,dtype=float,copy=True);u=np.ascontiguousarray(action,dtype=float)
        self.lib.direct_integrate(x.ctypes.data,u.ctypes.data,dt);return x
    def seed(self,state,reference,poses,first,mode,dt):
        steps=len(reference)-1;out=np.empty((steps,4))
        reference=np.asarray(reference)[:,:9]
        arrays=[np.ascontiguousarray(x,dtype=float) for x in (state,reference,poses,self.signs,self.geom,self.cfg)]
        self.lib.direct_seed(steps,self.gates,*[x.ctypes.data for x in arrays],first,mode,dt,out.ctypes.data);return out
    def rollout(self,state,controls,reference,poses,first=0,phase=0,mode=0,dt=.02,gpu=True,projection_iterations=None,soft_future=False,previous_command=None):
        batch,steps,_=controls.shape
        if np.shape(reference)!=(steps+1,9):raise ValueError('reference must contain one PVA row per prediction state')
        if previous_command is None:
            previous_command=np.array([MASS*9.8066,0.,0.,0.])
        if gpu:
            if projection_iterations is not None:raise ValueError('per-call projection budget is CPU only')
            cp=self.cp;arrays=[cp.asarray(x,dtype=cp.float32) for x in (state,controls,previous_command,reference,poses)]
            out=cp.empty((batch,8),dtype=cp.float32)
            self.kernel(((batch+127)//128,),(128,),tuple([np.int32(x) for x in (batch,steps,self.gates)]+arrays+self.constants+
                [np.int32(first),np.int32(phase),np.int32(mode),np.int32(soft_future),np.float32(dt),out,np.uint64(0),np.uint64(0)]));return out
        assert batch==1
        config=self.cfg.copy()
        if projection_iterations is not None:config[12]=projection_iterations
        arrays=[np.ascontiguousarray(x,dtype=float) for x in (state,controls,previous_command,reference,poses,self.geom,self.speeds,self.signs,config)]
        out=np.empty(8);paths=np.empty((steps+1,17));actions=np.empty((steps,4))
        self.lib.direct_rollout(batch,steps,self.gates,*[x.ctypes.data for x in arrays],first,phase,mode,int(soft_future),dt,out.ctypes.data,paths.ctypes.data,actions.ctypes.data)
        count=int(out[6]);return out,paths[:count+1],actions[:count]
