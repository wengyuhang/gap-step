"""Wang N=2/M=4 independent core, unchanged objective/initialization, new scene queries."""
from dataclasses import dataclass
import numpy as np
from convex_timevarying_window.wang_paper_aligned_mpc.planner import Config as BaseConfig,Planner as BasePlanner
from .scenario import pose,local_coordinates,FRAME_OFFSET
from convex_timevarying_window.comparisons.original_fixed_plane_ours.adapter import symbolic_clearance_sq as unused

@dataclass(frozen=True)
class Config(BaseConfig):
    free_points:bool=False
    frame_offset:float=FRAME_OFFSET
    workspace_lower:tuple=(-100.,-120.,.65)
    workspace_upper:tuple=(150.,120.,20.)
    experiment_variant:str='wang'
    full_task_error_recovery:bool=False
    abort_missed_gate:bool=True


def install_queries():
    import casadi as ca
    from convex_timevarying_window.wang_paper_aligned_mpc import planner,events
    from convex_timevarying_window.receding_gate_racing.planner import symbolic_clearance_sq as generic
    def clearance(window,position,t):
        from convex_timevarying_window.receding_gate_racing import planner as p
        old=p.symbolic_pose;p.symbolic_pose=pose
        try:return generic(window,position,t)
        finally:p.symbolic_pose=old
    def frames(frames,positions,times,radius,thickness):
        result=np.full(len(times),np.inf)
        for w in frames:
            q=local_coordinates(w,positions,times);d=w.aperture.boundary_distance(q[:,:2])
            result=np.minimum(result,np.hypot(d,q[:,2])-radius-thickness)
        return result
    planner.symbolic_pose=pose;planner.symbolic_clearance_sq=clearance;planner.frame_clearances=frames;events.local_coordinates=local_coordinates

class Planner(BasePlanner):
    def __init__(self,track,cfg=Config()):install_queries();super().__init__(track,cfg)
    def solve(self,state,now,first):
        plan=super().solve(state,now,first)
        if not plan.event_valid or plan.violation>=.003:
            raise RuntimeError('wang_not_admitted:'+plan.event_reason)
        if not np.isfinite(getattr(plan,'clearance',-np.inf)) or plan.clearance<self.cfg.tracking_margin:
            raise RuntimeError('wang_not_admitted:sampled_geometry_or_dynamics')
        plan.optimizer_stats=dict(self.solvers[first][0].stats(),candidate_policy='original_single_global_or_admitted_seed',formal_certificate=False)
        return plan
