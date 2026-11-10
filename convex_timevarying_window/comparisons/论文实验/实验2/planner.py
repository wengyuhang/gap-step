"""Full-course PVAJ optimizer: all seven crossing times/points/states and return."""
from dataclasses import replace
import time
import casadi as ca
import numpy as np
from convex_timevarying_window.global_gate_minco_probe import planner as original
from convex_timevarying_window.single_gate_stop_racing import point_mapping
from .scenario import pose,FRAME_OFFSET
from .safety import symbolic_constraints,passage

class Planner(original.Planner):
    def __init__(self,track,free_points=True):
        self.free_points=free_points
        previous=(original.symbolic_pose,original.symbolic_constraints,original.workspace_bounds,
            point_mapping.dimension,point_mapping.symbolic)
        original.symbolic_pose=pose;original.symbolic_constraints=symbolic_constraints
        original.workspace_bounds=lambda:((-100.,-120.,.65),(150.,120.,20.))
        if not free_points:
            point_mapping.dimension=lambda ap:0
            point_mapping.symbolic=lambda ap,x:ca.DM.zeros(2)
        try:super().__init__(track,full_state=True,planning_thrust_fraction=.9)
        finally:
            original.symbolic_pose,original.symbolic_constraints,original.workspace_bounds,point_mapping.dimension,point_mapping.symbolic=previous
        self.cfg=replace(self.cfg,frame_offset=FRAME_OFFSET,free_points=free_points,max_solve_ms=None)
    def seeds(self):
        nodes=np.vstack([self.track.start,*[w.center0 for w in self.track.windows],self.track.goal])
        T=np.maximum(.8,np.linalg.norm(np.diff(nodes,axis=0),axis=1)/6.5);T[0]+=.5
        d=np.linalg.norm(nodes[-1]-nodes[-2]);T[-1]=max(.8,np.sqrt(6*d/self.cfg.acceleration),(60*d/self.cfg.jerk)**(1/3))
        points=np.concatenate([point_mapping.initial(w.aperture) for w in self.track.windows]) if self.free_points else np.empty(0)
        velocity=np.array([-.8*self.cfg.speed*s*w.R[:,2] for s,w in zip(self.signs,self.track.windows)]).ravel()
        scales=getattr(self,'temporal_scales',(1.,.8,1.2,1.5))
        return [self.lift_pv(np.r_[np.log(T*scale),points,velocity]) for scale in scales]
    def verify(self,plan,deadline=np.inf):
        radius=self.cfg.radius+self.cfg.frame_thickness+self.cfg.passage_margin;results=[]
        for gate,w in enumerate(self.track.windows):
            for piece in range(self.n):
                result=passage(plan,w,radius,piece,deadline);results.append(dict(gate=gate,piece=piece,**result))
                if time.perf_counter()>=deadline:return results,False
        return results,all(r['passed'] for r in results)
    def solve_all(self,seconds_per_seed=180.,progress=lambda r:None):
        began=time.perf_counter();candidates=[];best=None
        for index,seed in enumerate(self.seeds()):
            def record(r):progress(dict(temporal_seed=index,**r))
            plan,result=super().solve(seconds_per_seed,record,initial_seeds=[seed])
            row=dict(seed=index,initial_variables=seed.tolist(),found=plan is not None,
                nominal_time_s=float(plan.durations.sum()) if plan else None,statistics=result)
            candidates.append(row);progress(dict(stage='seed_finished',seed=index,found=row['found'],nominal_time_s=row['nominal_time_s'],elapsed_s=time.perf_counter()-began))
            if plan is not None and (best is None or plan.durations.sum()<best[0].durations.sum()):best=plan,index
        return (best[0] if best else None),dict(found=best is not None,solve_s=time.perf_counter()-began,
            temporal_scales=[1.,.8,1.2,1.5],selected_seed=best[1] if best else None,candidates=candidates,
            all_four_attempted=True,all_seven_joint=True,intermediate_stops=False,replanning=False,
            independent_internal_pvaj=True,formal_certificate=False,global_optimality_claim=False)
