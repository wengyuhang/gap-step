"""Independent full-rigid-body GP-MPPI, plus documented dynamic/PX4 adaptations."""
from dataclasses import dataclass, asdict
from pathlib import Path
import ctypes
import subprocess
import time
import numpy as np
from convex_timevarying_window.conditional_dual_constraint_cem.safety_penalty import polygon_halfspaces


@dataclass(frozen=True)
class GPConfig:
    samples:int=8192
    horizon:int=20
    dt:float=.03
    temperature:float=.01
    noise_std:float=3.
    derivative_limit:float=10.
    near_weight:float=1.
    near_range:float=1.5
    constraint_weight:float=1000.
    input_mode:str='rotor_rate'


class GPMppi:
    def __init__(self,track,seed=0,cfg=GPConfig()):
        self.track=track;self.cfg=cfg;self.rng=np.random.default_rng(seed)
        root=Path(__file__).parent; source=root/'gp_rollout.cpp'; lib=root/'build'/'gp_rollout.so'
        lib.parent.mkdir(exist_ok=True)
        if not lib.exists() or source.stat().st_mtime>lib.stat().st_mtime:
            subprocess.run(['g++','-O3','-std=c++17','-fopenmp','-shared','-fPIC',str(source),'-o',str(lib)],check=True)
        self.lib=ctypes.CDLL(str(lib.resolve()))
        arr=np.ctypeslib.ndpointer(dtype=np.float64,flags='C_CONTIGUOUS')
        ints=np.ctypeslib.ndpointer(dtype=np.int32,flags='C_CONTIGUOUS')
        self.lib.rollout.argtypes=[ctypes.c_int,ctypes.c_int,ctypes.c_double,arr,arr,arr,arr,arr,arr,ints,
                                  ctypes.c_int,ctypes.c_int,ctypes.c_double,ctypes.c_double,ctypes.c_double,arr,ctypes.c_void_p,ctypes.c_void_p]
        self.lib.integrate.argtypes=[arr,arr,ctypes.c_double]
        self.lib.integrate_ctbr.argtypes=[arr,arr,ctypes.c_double]
        self.u=np.zeros((cfg.horizon,4));self.previous=None;self.previous_ids=np.zeros(cfg.horizon+1,dtype=np.int32)
        if cfg.input_mode=='ctbr':self.u[:,0]=2.0643076923076924*9.8066
        self.last_time=None;self.geometry=np.zeros((7,32));self.signs=[]
        for i,w in enumerate(track.windows):
            self.signs.append(float(np.sign(w.state_at(0)[1][:,2]@((track.start if i==0 else track.windows[i-1].center0)-w.center0)) or 1))
            if w.aperture.kind=='circle':self.geometry[i,1]=w.aperture.radius+.065
            else:
                hs=polygon_halfspaces(w.aperture.physical_vertices)
                vertices=np.array([np.linalg.solve(np.stack([hs.normals[j-1],hs.normals[j]]),np.array([hs.offsets[j-1],hs.offsets[j]])+.065) for j in range(len(hs.offsets))])
                self.geometry[i,0]=1;self.geometry[i,2]=len(vertices);self.geometry[i,3:3+vertices.size]=vertices.ravel()
        self.signs=np.array(self.signs)

    def poses(self,now):
        p=np.zeros((self.cfg.horizon+1,8,12))
        for k in range(len(p)):
            for j,w in enumerate(self.track.windows):
                c,r,*_=w.state_at(now+k*self.cfg.dt);p[k,j]=np.r_[c,r.ravel()]
            p[k,7]=np.r_[self.track.goal,np.eye(3).ravel()]
        return p

    def evaluate(self,state,controls,poses,first,save=False):
        n=len(controls);cost=np.zeros(n)
        trajectory=np.zeros((n,self.cfg.horizon+1,17)) if save else None
        ids=np.zeros((n,self.cfg.horizon+1),dtype=np.int32) if save else None
        self.lib.rollout(n,self.cfg.horizon,self.cfg.dt,np.ascontiguousarray(state),np.ascontiguousarray(controls),
            poses,self.geometry,self.signs,self.previous,self.previous_ids,first,int(self.cfg.input_mode=='ctbr'),self.cfg.near_weight,
            self.cfg.near_range,self.cfg.constraint_weight,cost,
            trajectory.ctypes.data if save else None,ids.ctypes.data if save else None)
        return cost,trajectory,ids

    def solve(self,state,now,first):
        tick=time.perf_counter();cfg=self.cfg
        if self.last_time is not None:
            shift=max(0,min(cfg.horizon,int(round((now-self.last_time)/cfg.dt))))
            if shift:
                self.u=np.r_[self.u[shift:],np.repeat(self.u[-1:],shift,axis=0)]
                self.previous=np.r_[self.previous[shift:],np.repeat(self.previous[-1:],shift,axis=0)]
                self.previous_ids=np.r_[self.previous_ids[shift:],np.repeat(self.previous_ids[-1:],shift)].astype(np.int32)
        else:
            self.previous=np.tile(state,(cfg.horizon+1,1));self.previous_ids.fill(first)
        pose=self.poses(now)
        noise=cfg.noise_std if cfg.input_mode=='rotor_rate' else np.array([2.,.4,.4,.15])
        controls=self.u[None]+self.rng.normal(0,1,(cfg.samples,cfg.horizon,4))*noise
        if cfg.input_mode=='rotor_rate':controls=np.clip(controls,-cfg.derivative_limit,cfg.derivative_limit)
        else:controls=np.clip(controls,[4.,-3.49,-3.49,-3.49],[4*8.54858*.9,3.49,3.49,3.49])
        controls[0]=self.u
        cost,_,_=self.evaluate(state,controls,pose,first)
        weights=np.exp(-(cost-cost.min())/cfg.temperature);weights/=weights.sum()
        self.u=np.einsum('m,mkj->kj',weights,controls)
        _,pred,ids=self.evaluate(state,self.u[None],pose,first,save=True)
        self.previous=pred[0];self.previous_ids=ids[0];self.last_time=now
        return self.u[0].copy(),self.previous.copy(),1000*(time.perf_counter()-tick)


if __name__=='__main__':
    from convex_dynamic_seven_window_gazebo.x500_togt.experiment import build_track
    track,_=build_track();m=GPMppi(track)
    state=np.r_[track.start,[0.,0.,0.],[1.,0.,0.,0.],[0.,0.,0.],np.full(4,2.0643076923076924*9.8066/4)]
    original=state.copy();m.lib.integrate(state,np.zeros(4),.03)
    np.testing.assert_allclose(state,original,atol=1e-12)
    for i in range(4):
        _,s,ms=m.solve(state,i*.03,0);print('solve',ms,'predicted displacement',s[-1,:3]-state[:3])
