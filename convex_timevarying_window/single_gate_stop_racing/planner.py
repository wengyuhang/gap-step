"""Single-window MINCO with TOGT shape coordinates and measured gate identity."""
from dataclasses import dataclass
import tempfile
import casadi as ca
import numpy as np
from nonconvex_timevarying_window.sc_dynatogt.environment import SCWindowTrack
from convex_timevarying_window.dynamic_passage_velocity_minco.hard_planner import Planner as Hard,Config as HardConfig
from convex_timevarying_window.dynamic_passage_velocity_minco.trajectory import minco_coefficients,Plan
from convex_timevarying_window.dynamic_passage_minco.planner import Stats
from convex_timevarying_window.togt_minco_mpc.trajectory import basis
from convex_timevarying_window.comparisons.original_fixed_plane_ours.adapter import symbolic_pose
from .geometry import symbolic_constraints,passage
from . import point_mapping
from .solver import solve


@dataclass(frozen=True)
class Config(HardConfig):
    full_task_error_recovery: bool=False
    arrival_seed_speed: float=6.5
    workspace_lower: tuple=(-28.,-30.,.65)
    workspace_upper: tuple=(28.,22.,12.)
    feasibility_restore: bool=True


def route_signs(track):
    return [float(np.sign(((track.start if i==0 else track.windows[i-1].center0)-w.center0)
                         @w.state_at(0.)[1][:,2]) or 1.) for i,w in enumerate(track.windows)]


def parking_points(track,cfg):
    signs=route_signs(track);points=[]
    for i,w in enumerate(track.windows):
        if i==len(track.windows)-1:points.append(track.goal.copy());continue
        out=-signs[i]*w.state_at(0.)[1][:,2]
        delta=track.windows[i+1].center0-w.center0
        tangent=.5*(delta-out*np.dot(delta,out))
        depth=max(cfg.radius+cfg.frame_thickness+1.,min(6.,.5*np.linalg.norm(delta)))
        p=w.center0+tangent+out*depth
        p=np.clip(p,cfg.workspace_lower,cfg.workspace_upper)
        if np.dot(p-w.center0,out)<=cfg.radius+cfg.frame_thickness:
            raise ValueError('No exit-side parking point inside workspace')
        points.append(p)
    return points


class Single(Hard):
    def __init__(self,track,cfg,sign):
        self.track,self.cfg=track,cfg;self.windows=list(track.windows)
        self.rotation=self.windows[0].state_at(0.)[1];self.signs=[sign];self.admitted=None
        self._jit_dir=tempfile.TemporaryDirectory(prefix='single_stop_jit_')
        self.failed_seed=None
        self._build();self.solvers=[(Stats(),)]
        from .local_bounds import warmup
        warmup()

    def _build(self):
        cfg=self.cfg;w=self.windows[0]
        self.point_dim=point_mapping.dimension(w.aperture) if cfg.free_points else 0
        self.nvar=5+self.point_dim
        x=ca.MX.sym('x',self.nvar);par=ca.MX.sym('head',13+2*cfg.witness_slots)
        times=ca.exp(x[:2]);cross=par[0]+times[0]
        uv=point_mapping.symbolic(w.aperture,x[2:2+self.point_dim]) if self.point_dim else ca.DM.zeros(2)
        center,R=symbolic_pose(w,cross);point=center+R[:,:2]@uv
        C=minco_coefficients(ca.reshape(par[1:13],3,4),
            ca.horzcat(ca.DM(self.track.goal),ca.DM.zeros(3,3)),point,x[-3:],times)
        def state(piece,u,d=0):return C[piece*8:piece*8+8,:].T@basis(u,d)/times[piece]**d
        constraints=self.thrust_constraints(state,times)
        radius=cfg.radius+cfg.frame_thickness+cfg.passage_margin+cfg.witness_slack
        for piece in range(2):
            for j in range(cfg.witness_slots):
                offset=ca.fmin(par[13+piece*cfg.witness_slots+j],times[piece])
                u=1-offset/times[0] if piece==0 else offset/times[1]
                t=cross-offset if piece==0 else cross+offset
                cc,rr=symbolic_pose(w,t);q=rr.T@(state(piece,u)-cc)
                constraints.extend(symbolic_constraints(w.aperture,q,radius))
        constraints.append(-self.signs[0]*ca.dot(R[:,2],state(0,1.,1))-.1)
        g=ca.vertcat(*constraints);cost=ca.sum1(times)/1000.
        options={'cse':True}
        if cfg.compile_objective:
            options.update(jit=True,compiler='shell',jit_options={'flags':[cfg.compiler_optimization],
                'directory':self._jit_dir.name+'/'})
        self.function=ca.Function('single_stop_eval',[x,par],
            [cost,ca.gradient(cost,x),g,ca.jacobian(g,x),C,ca.MX(0.)],options)
        self.lower=np.r_[np.log([.3,.3]),np.full(self.nvar-2,-np.inf)]
        self.upper=np.r_[np.log([15.,15.]),np.full(self.nvar-2,np.inf)]

    solve=solve

    def local_passage(self,plan,window,piece):
        return passage(plan,window,self.cfg.radius+self.cfg.frame_thickness+self.cfg.passage_margin,piece)


class Planner:
    def __init__(self,track,cfg=Config()):
        self.track,self.cfg=track,cfg;self.windows=list(track.windows);self.signs=route_signs(track)
        self.parking=parking_points(track,cfg);self.admitted=None;self.locals=[]
        for i,w in enumerate(self.windows):
            local=SCWindowTrack('local_'+w.name,track.start.copy(),self.parking[i],(w,),(0,))
            self.locals.append(Single(local,cfg,self.signs[i]))
        self.solvers=[p.solvers[0] for p in self.locals]

    def commit(self,plan):self.admitted=plan
    def prefix_clearance(self,plan,at):return None
    def solve(self,state,now,first):
        if first>=len(self.windows):raise RuntimeError('No replanning after last body exit')
        local=self.locals[first]
        local.commit(self.admitted if self.admitted is not None and self.admitted.first_gate==first else None)
        plan=local.solve(state,now,0);plan.first_gate=first
        plan.optimizer_stats['parking_point']=self.parking[first].tolist()
        plan.optimizer_stats['target_window_count']=1
        return plan
