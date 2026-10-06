"""Same new screening kernel on CUDA/CPU; shared physics, no old controller."""
import ctypes
import hashlib
from pathlib import Path
import re
import subprocess
import numpy as np
from convex_timevarying_window.dynamic_gate_mppi_v3.geometry import Geometry
from convex_timevarying_window.dynamic_gate_mppi_v3 import rollout as physics

HERE=Path(__file__).parent

def source():
    old=(Path(physics.__file__).parent/'native/rollout.cpp').read_text()
    physical=old.split('extern "C" void v3_integrate')[0]
    feedback=old[old.index('namespace v3 {\nvoid feedback'):old.index('extern "C" void v3_feedback')]
    return '#define V3_CTBR 1\n'+physical+feedback+(HERE/'kernel.cpp').read_text()

def cuda_source(code):
    code=re.sub(r'#include[^\n]*','',code)
    code=re.sub(r'^(double|void|bool) (\w+)\(',r'__device__ \1 \2(',code,flags=re.M)
    code=code.replace('constexpr double I[','__device__ const double I[').replace('constexpr double mix[','__device__ const double mix[')
    code=code.replace('extern "C" void','extern "C" __global__ void')
    code=code.replace('for(int b=0;b<batch;b++){','{int b=blockIdx.x*blockDim.x+threadIdx.x;if(b>=batch)return;')
    code=code.replace('std::clamp','clamp_value').replace('std::max','fmax').replace('std::min','fmin').replace('std::','')
    code='__device__ double clamp_value(double x,double a,double b){return fmin(b,fmax(a,x));}\n'+code
    code=code.replace('double','float')
    code=re.sub(r'(?<![\w.])((?:\d+\.\d*|\.\d+)(?:[eE][+-]?\d+)?|\d+[eE][+-]?\d+)(?![\w.])',r'\1f',code)
    for name in ('fmin','fmax','sqrt','hypot','ceil'):code=re.sub(r'\b'+name+r'\(',name+'f(',code)
    return code

class Engine:
    def __init__(self,track,cfg):
        import cupy as cp
        self.cp=cp;self.track=track;self.config=cfg;self.geometry=Geometry(track,cfg)
        self.signs=[];previous=track.start
        for wi in track.order:
            c,r,*_=track.windows[wi].state_at(0.)
            self.signs.append(-np.sign(r[:,2]@(previous-c)));previous=c
        self.signs=np.asarray(self.signs,dtype=float)
        self.geom=np.zeros((len(track.order),32))
        for i,wi in enumerate(track.order):
            f=self.geometry.frames[wi]
            if f.kind=='circle':self.geom[i,1]=f.radius
            else:self.geom[i,:3]=[1,0,len(f.physical_vertices)];self.geom[i,3:3+2*len(f.physical_vertices)]=f.physical_vertices.ravel()
        self.speeds=np.ascontiguousarray(self.geometry.frame_speeds[list(track.order)])
        self.cfg=np.array([cfg.radius,cfg.error_margin,cfg.frame_thickness,cfg.speed,cfg.acceleration,np.cos(np.deg2rad(cfg.tilt)),*cfg.lower,*cfg.upper,cfg.feedback_geometry_reserve])
        code=source();key=hashlib.sha256(code.encode()).hexdigest()[:16]
        build=HERE/'build';build.mkdir(exist_ok=True)
        lib=build/f'{key}.so'
        if not lib.exists():
            subprocess.run(['g++','-O3','-std=c++17','-x','c++','-shared','-fPIC','-','-o',str(lib)],input=code,text=True,check=True)
        self.lib=ctypes.CDLL(str(lib));ptr=ctypes.c_void_p
        self.lib.phase3_batch.argtypes=[ctypes.c_int]*3+[ptr]*9+[ctypes.c_int]*3+[ptr]*3
        self.lib.phase3_batch.restype=None
        self.lib.phase3_command.argtypes=[ptr]*5;self.lib.phase3_command.restype=None
        self.lib.phase3_integrate.argtypes=[ptr,ptr,ctypes.c_double];self.lib.phase3_integrate.restype=None
        self.module=cp.RawModule(code=cuda_source(code),options=('--std=c++17',))
        self.kernel=self.module.get_function('phase3_batch')
        self.constants=[cp.asarray(x,dtype=cp.float32) for x in (self.geom,self.speeds,self.signs,self.cfg)]

    def prepare(self,now,steps):
        c,r=self.geometry.poses(now+np.arange(steps+1)*.01);order=list(self.track.order)
        return np.ascontiguousarray(np.concatenate((c[:,order],r[:,order].reshape(steps+1,len(order),9)),axis=-1))

    def evaluate(self,state,previous,policies,reference,poses,first,phase,mode,gpu=True):
        cp=self.cp;steps=len(poses)-1
        if gpu:
            n=len(policies);arrays=[cp.asarray(x,dtype=cp.float32) for x in (state,previous,policies,reference,poses)]
            out=cp.empty((n,10),dtype=cp.float32)
            self.kernel(((n+127)//128,),(128,),tuple([np.int32(n),np.int32(steps),np.int32(len(self.signs)),*arrays,*self.constants,np.int32(first),np.int32(phase),np.int32(mode),out,np.uint64(0),np.uint64(0)]))
            return out
        arrays=[np.ascontiguousarray(x,dtype=float) for x in (state,previous,policies,reference,poses,self.geom,self.speeds,self.signs,self.cfg)]
        out=np.empty((1,10));states=np.empty((steps+1,17));commands=np.empty((steps,4))
        self.lib.phase3_batch(1,steps,len(self.signs),*[x.ctypes.data for x in arrays],first,phase,mode,out.ctypes.data,states.ctypes.data,commands.ctypes.data)
        end=int(out[0,5]);return out[0],states[:end+1],commands[:end]

    def command(self,state,pva,previous):
        arrays=[np.ascontiguousarray(x,dtype=float) for x in (state,pva,self.cfg,np.array(previous,copy=True))];u=np.empty(4)
        self.lib.phase3_command(*[x.ctypes.data for x in arrays],u.ctypes.data);return u

    def integrate(self,state,u,dt=.01):
        state=np.array(state,copy=True,dtype=float);u=np.ascontiguousarray(u,dtype=float)
        self.lib.phase3_integrate(state.ctypes.data,u.ctypes.data,dt);return state
