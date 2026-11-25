"""Same local MINCO, native oriented-box passage instead of enclosing sphere."""
from dataclasses import dataclass
from pathlib import Path
import casadi as ca
import numpy as np
from nonconvex_timevarying_window.sc_dynatogt.environment import SCWindowTrack
from ..planner import Config as ParentConfig
from .. import point_mapping
from convex_timevarying_window.dynamic_passage_velocity_minco.trajectory import minco_coefficients
from convex_timevarying_window.togt_minco_mpc.trajectory import basis
from convex_timevarying_window.comparisons.original_fixed_plane_ours.adapter import symbolic_pose
from .planner import Single as ParentSingle,Planner as ParentPlanner
from . import box_geometry,box_solver,native_frame_geometry


@dataclass(frozen=True)
class Config(ParentConfig):
    witness_slots:int=24
    passage_seed_offsets:tuple=(0.,.01,.02,.03,.04,.05,.075,.1)
    native_half:tuple=(.492,.492,.148)
    native_offset:tuple=(0.,0.,-.033)
    geometry_model:str='native_oriented_box_slab'
    frame_outward_offset:float=.065
    native_solver:str='al_lbfgs'
    log_cost_scaling:bool=True
    # Experimental ablation only; do not silently replace the inherited PV knot.
    interior_acceleration_free:bool=False


class Single(ParentSingle):
    cache_sources=(__file__,box_geometry.__file__,native_frame_geometry.__file__)
    solve=box_solver.solve

    def _build_expression(self):
        cfg=self.cfg;w=self.windows[0]
        self.point_dim=point_mapping.dimension(w.aperture) if cfg.free_points else 0
        self.nvar=5+self.point_dim+(3 if cfg.interior_acceleration_free else 0)
        x=ca.MX.sym('x',self.nvar);par=ca.MX.sym('head',13+2*cfg.witness_slots)
        times=ca.exp(x[:2]);cross=par[0]+times[0]
        uv=point_mapping.symbolic(w.aperture,x[2:2+self.point_dim]) if self.point_dim else ca.DM.zeros(2)
        center,R=symbolic_pose(w,cross);point=center+R[:,:2]@uv
        k=2+self.point_dim
        C=minco_coefficients(ca.reshape(par[1:13],3,4),
            ca.horzcat(ca.DM(self.track.goal),ca.DM.zeros(3,3)),point,x[k:k+3],times,
            x[k+3:k+6] if cfg.interior_acceleration_free else None)
        def state(piece,u,d=0):return C[piece*8:piece*8+8,:].T@basis(u,d)/times[piece]**d
        constraints=self.thrust_constraints(state,times)
        orientation=box_geometry.orientation_map(self.native_model.air_density)
        # Reuse one small SX geometry kernel at all witness times. Inlining
        # hundreds of clip branches in one C function makes GCC optimization
        # quadratic and needlessly expensive; the mathematical map is identical.
        local_q=ca.SX.sym('local_q',3);local_R=ca.SX.sym('local_R',3,3)
        body_constraints=ca.Function('box_frame_constraints',[local_q,local_R],
            [ca.vertcat(*native_frame_geometry.symbolic(w.aperture,local_q,local_R,
                np.array(cfg.native_half)+cfg.passage_margin,np.array(cfg.native_offset),
                cfg.frame_thickness+cfg.witness_slack,cfg.frame_outward_offset))])
        for piece in range(2):
            for j in range(cfg.witness_slots):
                offset=ca.fmin(par[13+piece*cfg.witness_slots+j],times[piece])
                u=1-offset/times[0] if piece==0 else offset/times[1]
                t=cross-offset if piece==0 else cross+offset
                cc,rr=symbolic_pose(w,t);q=rr.T@(state(piece,u)-cc)
                rb=rr.T@orientation(state(piece,u,1),state(piece,u,2))
                constraints.extend(ca.vertsplit(body_constraints(q,rb)))
        constraints.append(-self.signs[0]*ca.dot(R[:,2],state(0,1.,1))-.1)
        g=ca.vertcat(*constraints);cost=ca.sum1(times)/1000.
        self.function=ca.Function('single_stop_eval',[x,par],
            [cost,ca.gradient(cost,x),g,ca.jacobian(g,x),C,ca.MX(0.)],{'cse':True})
        self.lower=np.r_[np.log([.3,.3]),np.full(self.nvar-2,-np.inf)]
        self.upper=np.r_[np.log([15.,15.]),np.full(self.nvar-2,np.inf)]
        self.lower[2:k]=-1.;self.upper[2:k]=1.

    def _plan(self,x,now,out):
        plan=super()._plan(x,now,out)
        plan.native_box_half=self.cfg.native_half;plan.native_box_offset=self.cfg.native_offset
        plan.native_box_margin=self.cfg.passage_margin;plan.native_frame_radius=self.cfg.frame_thickness
        plan.native_air_density=self.native_model.air_density
        plan.native_aperture_shrink=self.cfg.frame_thickness-self.cfg.frame_outward_offset
        return plan


class Planner(ParentPlanner):
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        # A virtual stop must leave braking distance for the native vehicle;
        # the inherited fixed 6 m normal-depth cap was an x500 assumption.
        for i,w in enumerate(self.windows[:-1]):
            out=-self.signs[i]*w.state_at(0.)[1][:,2]
            delta=self.windows[i+1].center0-w.center0
            tangent=.5*(delta-out*np.dot(delta,out))
            depth=max(self.cfg.radius+self.cfg.frame_thickness+1.,.5*np.linalg.norm(delta))
            self.parking[i]=np.clip(w.center0+tangent+out*depth,
                self.cfg.workspace_lower,self.cfg.workspace_upper)

    def prepare(self,index):
        if self.locals[index] is None:
            w=self.windows[index]
            local=SCWindowTrack('native_box_'+w.name,self.track.start.copy(),self.parking[index],(w,),(0,))
            self.locals[index]=Single(local,self.cfg,self.signs[index],self.native_model)
        return self.locals[index]
