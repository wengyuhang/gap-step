"""Identical CPU/CUDA time-varying barrier algebra and direct acceleration model."""
import ctypes
import hashlib
from pathlib import Path
import re
import subprocess
import numpy as np
from convex_timevarying_window.dynamic_gate_mppi_v3.geometry import Geometry
from convex_timevarying_window.three_phase_mppi.controller import config
from convex_timevarying_window.three_phase_mppi.engine import cuda_source

HERE=Path(__file__).parent

class Engine:
    def __init__(self,track,gpu=True):
        self.track=track;self.config=config();self.geometry=Geometry(track,self.config)
        self.gates=len(track.order);self.features=[];self.signs=[];previous=track.start
        for gate,index in enumerate(track.order):
            window=track.windows[index];frame=self.geometry.frames[index]
            c,r,*_=window.state_at(0);self.signs.append(-np.sign(r[:,2]@(previous-c)));previous=c
            if frame.kind=='circle':
                angles=np.arange(32)*2*np.pi/32;points=frame.radius*np.column_stack((np.cos(angles),np.sin(angles)))
                reserve=frame.radius*(1-np.cos(np.pi/32))
            else:points=frame.physical_vertices.copy();reserve=0.
            if np.sum(points[:,0]*np.roll(points[:,1],-1)-points[:,1]*np.roll(points[:,0],-1))<0:points=points[::-1]
            for a,b in zip(points,np.roll(points,-1,axis=0)):
                self.features.append([gate,*a,0.,*b,0.,reserve])
        self.features=np.array(self.features,dtype=float);self.signs=np.array(self.signs)
        slab=self.config.radius+self.config.error_margin+self.config.frame_thickness
        # Extra numerical reserve for the 5 ms execution interval. It does not
        # replace the independent interval audit or prove sampled-data invariance.
        padding=.5*(12.+max(self.geometry.frame_speeds))*.005+.002
        self.cfg=np.array([slab+padding,10.,10.,4.,1.,6.5,12.,*self.config.lower,*self.config.upper,
                           self.config.frame_offset,slab,padding])
        code=(HERE/'kernel.cpp').read_text();key=hashlib.sha256(code.encode()).hexdigest()[:16]
        build=HERE/'build';build.mkdir(exist_ok=True);lib=build/f'{key}.so'
        if not lib.exists():subprocess.run(['g++','-O3','-std=c++17','-x','c++','-shared','-fPIC','-','-o',str(lib)],input=code,text=True,check=True)
        self.lib=ctypes.CDLL(str(lib));ptr=ctypes.c_void_p;integer=ctypes.c_int
        self.lib.gs_barrier.argtypes=[ptr,ptr,integer,ptr,integer,ptr,ptr];self.lib.gs_barrier.restype=None
        self.lib.gs_filter.argtypes=[ptr]*5;self.lib.gs_filter.restype=integer
        self.lib.gs_rollout.argtypes=[integer]*4+[ptr]*7+[integer]*3+[ctypes.c_double]+[ptr]*3;self.lib.gs_rollout.restype=None
        self.cp=None
        if gpu:
            import cupy as cp
            self.cp=cp;code=cuda_source(code)
            code=re.sub(r'^int filter\(',r'__device__ int filter(',code,flags=re.M)
            self.module=cp.RawModule(code='#define GS_GPU 1\n'+code,options=('--std=c++17',))
            self.kernel=self.module.get_function('gs_rollout')
    def poses(self,times):
        times=np.asarray(times);epsilon=1e-4
        c,r=self.geometry.poses(times);cm,rm=self.geometry.poses(times-epsilon);cp,rp=self.geometry.poses(times+epsilon)
        values=[c,r.reshape(len(times),-1,9),(cp-cm)/(2*epsilon),((rp-rm)/(2*epsilon)).reshape(len(times),-1,9),
                (cp-2*c+cm)/epsilon**2,((rp-2*r+rm)/epsilon**2).reshape(len(times),-1,9)]
        return np.ascontiguousarray(np.concatenate(values,axis=-1)[:,list(self.track.order)])
    def barrier(self,state,now):
        arrays=[np.ascontiguousarray(state,dtype=float),self.poses([now]),self.features,self.cfg];out=np.empty(6)
        self.lib.gs_barrier(arrays[0].ctypes.data,arrays[1].ctypes.data,self.gates,arrays[2].ctypes.data,len(self.features),arrays[3].ctypes.data,out.ctypes.data)
        return out
    def filter(self,state,now,desired):
        b=self.barrier(state,now);d=np.ascontiguousarray(desired,dtype=float);a=np.empty(3);info=np.empty(2)
        status=self.lib.gs_filter(d.ctypes.data,b.ctypes.data,self.cfg.ctypes.data,a.ctypes.data,info.ctypes.data)
        return a,dict(status=status,B=float(b[0]),h_min=float(b[5]),residual=float(info[0]),mu=float(info[1]))
    def rollout(self,state,controls,reference,poses,first=0,phase=0,mode=1,dt=.02,gpu=True):
        batch,steps,_=controls.shape
        if gpu:
            cp=self.cp;arrays=[cp.asarray(v,dtype=cp.float32) for v in (state,controls,reference,poses,self.features,self.cfg,self.signs)]
            out=cp.empty((batch,6),dtype=cp.float32)
            self.kernel(((batch+127)//128,),(128,),tuple([np.int32(v) for v in (batch,steps,self.gates,len(self.features))]+arrays+
                [np.int32(first),np.int32(phase),np.int32(mode),np.float32(dt),out,np.uint64(0),np.uint64(0)]))
            return out
        assert batch==1
        arrays=[np.ascontiguousarray(v,dtype=float) for v in (state,controls,reference,poses,self.features,self.cfg,self.signs)]
        out=np.empty((1,6));states=np.empty((steps+1,6));actions=np.empty((steps,3))
        self.lib.gs_rollout(batch,steps,self.gates,len(self.features),*[v.ctypes.data for v in arrays],first,phase,mode,dt,out.ctypes.data,states.ctypes.data,actions.ctypes.data)
        return out[0],states,actions

def advance(state,action,dt):
    out=np.array(state,copy=True,dtype=float);out[:3]+=dt*out[3:]+.5*dt*dt*action;out[3:]+=dt*action;return out
