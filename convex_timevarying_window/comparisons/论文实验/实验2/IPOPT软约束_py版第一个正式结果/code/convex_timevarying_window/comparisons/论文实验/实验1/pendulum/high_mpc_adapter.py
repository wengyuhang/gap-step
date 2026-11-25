"""Author weights/cache on x500: measured states, physical input bounds, CTBR."""
from dataclasses import dataclass
from pathlib import Path
import json
import sys
import time
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
from scipy.spatial.transform import Rotation
from .motion import PIVOT,GOAL
from . import scenario
from convex_timevarying_window.togt_style_mpc.model import MASS,G,FMIN,FMAX,RATE_MAX

@dataclass(frozen=True)
class Config:
    update_period: float=.02
    horizon_s: float=2.
    discretization_s: float=.04

def project_solver_roundoff(command):
    command=np.asarray(command,float)
    if command.shape!=(4,) or not np.isfinite(command).all():raise ValueError('invalid author command')
    lower,upper=np.r_[4*FMIN,-RATE_MAX],np.r_[4*FMAX,RATE_MAX]
    violation=float(max(np.max(lower-command),np.max(command-upper),0.))
    if violation>1e-5:raise ValueError('author solution exceeds input bounds beyond numerical tolerance')
    return np.clip(command,lower,upper),violation

class MPC:
    window=None
    def __init__(self):
        from convex_timevarying_window.togt_style_mpc.shooting_controller import MPC as Preparation
        self.preparation=Preparation();self.cfg=Config();self.logs=[];self.last_time=None;self.previous=np.array([MASS*G,0.,0.,0.]);self.controls=None;self.session=None
        repo=scenario.ROOT/'复现/high_mpc_author';sys.path.insert(0,str(repo))
        import casadi as ca
        import tensorflow as tf
        from high_mpc.mpc.high_mpc import High_MPC
        from high_mpc.policy.deep_high_policy import Actor
        from high_mpc.simulation.pendulum_v1 import Pendulum_v1
        original=repo/'high_mpc/mpc/saved/high_mpc.so'
        compat=scenario.HERE.parent/'results/author_reproduction_v4_20261105'
        checked=json.loads((compat/'result.json').read_text())
        if not checked['metadata']['source_unchanged']:raise RuntimeError('unverified author cache shim')
        self.cache_compat=checked['metadata']['cached_solver_compatibility']
        old=ca.nlpsol
        def load(name,backend,model,options):
            return old(name,backend,str(compat/'author_cache_metadata_compat.so') if isinstance(model,str) and model==str(original) else model,options)
        with patch.object(ca,'nlpsol',load):self.author=High_MPC(T=2.,dt=.04,so_path=str(original))
        self.actor=Actor(9,1);self.actor.load_weights(str(repo/'high_mpc/Dataset/act_net/weights_999.h5'))
        self.coordinate_shift=PIVOT-np.array([2.,0.,3.])
        self.prediction=Pendulum_v1(np.array([2.,0.,3.]),sigma=10,T=2.,dt=.04)
        self.goal=(GOAL-self.coordinate_shift).tolist()+[1.,0.,0.,0.,0.,0.,0.]
        # Only bound arrays change; cached objective/dynamics/derivatives and actor retain author values.
        for k in range(self.author._N):
            i=10+14*k;self.author.lbw[i:i+4]=np.r_[4*FMIN/MASS,-RATE_MAX].tolist();self.author.ubw[i:i+4]=np.r_[4*FMAX/MASS,RATE_MAX].tolist()
        self.metadata=dict(implementation='author_high_mpc_x500_state_adapter',author_commit=checked['metadata']['author_commit'],
            original_weights=True,original_cached_objective_and_dynamics=True,physical_input_bounds_adapted=True,
            thrust_specific_box_mps2=[4*FMIN/MASS,4*FMAX/MASS],body_rate_box_rad_s=RATE_MAX.tolist(),
            high_level_policy_retrained=False,position_coordinate_shift_to_author_frame=self.coordinate_shift.tolist(),comparison_scope='zero-shot x500 adaptation, not original-paper performance',
            preparation_controller_only_before_task='shared execution-aligned hover MPC',reference_retiming=False,numerical_input_box_tolerance=1e-5)
    def bind_session(self,session):self.session=session;self.preparation.bind_session(session)
    def command(self,x,now,reference):
        began=time.perf_counter()
        if now<0:
            command=self.preparation.command(x,now,reference);self.previous=command;self.last_time=now
            self.logs.append(dict(time=float(now),cycle_ms=1000*(time.perf_counter()-began),preparation_only=True,command=command.tolist()))
            return command
        motion=self.window.motion;theta=float(motion.theta(now));omega=float(motion.theta(now,1));d,v=motion.translation(now)
        quad=np.r_[x[:3]-self.coordinate_shift,x[6:10],x[3:6]]
        euler=Rotation.from_quat(np.r_[x[7:10],x[6]]).as_euler('xyz')
        observation=np.r_[x[:3]-(PIVOT+d),euler-[theta,0.,0.],x[3:6]-v]
        selected=float(self.actor(observation.reshape(1,9)).numpy()[0,0])
        prediction,_=self.prediction.plan(np.array([theta,omega]),selected)
        action,_=self.author.solve(quad.tolist()+prediction+self.goal);action=np.asarray(action).ravel()
        raw_command=np.r_[MASS*action[0],action[1:]]
        command,roundoff=project_solver_roundoff(raw_command)
        stats=self.author.solver.stats()
        self.previous=command;self.last_time=now
        self.logs.append(dict(time=float(now),cycle_ms=1000*(time.perf_counter()-began),solver_status=str(stats.get('return_status')),
            solver_success=bool(stats.get('success',False)),selected_traversal_time_s=selected,command=command.tolist(),raw_solver_command=raw_command.tolist(),input_box_roundoff=roundoff,observation=observation.tolist()))
        return command.copy()

class Worker:
    def __init__(self,track,cfg):
        self.geometry=SimpleNamespace(cfg=cfg,windows=list(track.windows),signs=[-1.]);self.busy=False;self.metrics=[]
        self.startup=dict(no_trajectory_planner=True,reason='author High-MPC directly emits CTBR')
    def poll(self):return None
    def close(self):pass

class Scheduler:
    def __init__(self,error_high,error_low):self.error_high=error_high;self.error_low=error_low;self.retry_delay=0.;self.log=[]
    def observe(self,*args):pass
    def due(self,*args):return False
