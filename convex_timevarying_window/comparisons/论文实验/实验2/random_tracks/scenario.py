"""Frozen random layouts with identical numerical and symbolic motion."""
from dataclasses import dataclass, replace
from pathlib import Path
import json
import numpy as np
import casadi as ca
from scipy.spatial.transform import Rotation
from convex_timevarying_window.comparisons.论文实验.实验2.higher_swing import scenario as original
from convex_timevarying_window.comparisons.论文实验.实验2 import scenario as mixed

HERE=Path(__file__).resolve().parent
ROOT=original.ROOT
ASSETS=original.ASSETS
VARIANT='random_tracks_v2'
SWING_LENGTH=2.2
LEVELS=tuple(f'{seed}_{difficulty}' for seed in [2026111301,2026111302,2026111303] for difficulty in ['low','medium','high'])

@dataclass(frozen=True)
class Case:
    level:str
    layout:str='random_v2'
    def __post_init__(self):
        if self.level not in LEVELS:raise ValueError(self.level)

@dataclass(frozen=True)
class Motion(mixed.Motion):
    direction_local:tuple=(1.,0.,0.)
    @property
    def translation_amplitude(self):
        return self.rotation_basis@np.asarray(self.direction_local)*self.amplitude
    def local(self,t):
        z=2*np.pi*self.frequency*np.asarray(t)+self.phase
        if self.kind=='translation':
            direction=np.asarray(self.direction_local)
            return self.amplitude*np.sin(z)[...,None]*direction,self.amplitude*2*np.pi*self.frequency*np.cos(z)[...,None]*direction
        p,v=super().local(t)
        return p[..., [1,0,2]]*[1,-1,1],v[..., [1,0,2]]*[1,-1,1]
    def local_bounds(self,a,b):
        if self.kind=='translation':
            lo,hi=mixed.sine_range(2*np.pi*self.frequency*a+self.phase,2*np.pi*self.frequency*b+self.phase)
            values=self.amplitude*np.asarray(self.direction_local)[:,None]*np.array([lo,hi])
            return values.min(axis=1),values.max(axis=1)
        lo,hi=super().local_bounds(a,b)
        return np.array([lo[1],-hi[0],lo[2]]),np.array([hi[1],-lo[0],hi[2]])

def manifest(case):
    return json.loads((HERE/'results/v2'/f'{case.level}.json').read_text())

def build_case(case):
    data=manifest(case)
    track,_=original.build_case(original.Case(level='static'))
    for w,row in zip(track.windows,data['windows']):
        w.center0=np.array(row['center']);w.R=np.array(row['R'])
        w.angles0_rpy=Rotation.from_matrix(w.R).as_euler('xyz')
        local=w.R.T@np.array(row['direction']);local[2]=0.
        w.motion=Motion(row['kind'],w.R,row['frequency_hz'],row['amplitude_m'],phase=row['phase'],length=2.2,direction_local=tuple(local))
    return replace(track,name=case.level,start=np.array(data['start']),goal=np.array(data['goal'])),None

def pose(w,t):
    m=w.motion;z=2*np.pi*m.frequency*t+m.phase
    if m.kind=='translation':d=m.amplitude*ca.sin(z)*ca.DM(m.direction_local)
    else:
        theta=m.angle*ca.sin(z);d=ca.vertcat(m.length*(1-ca.cos(theta)),-m.length*ca.sin(theta),0.)
    return ca.DM(w.center0)+ca.DM(w.R)@d,ca.DM(w.R)
