"""Amplitude continuation of the same constrained global racing problem.

Auxiliary problems only initialize the solver. A trajectory is returned only
after the exact full-amplitude target problem converges and passes verification.
"""
import copy,time
from dataclasses import replace
import numpy as np
from convex_timevarying_window.global_gate_minco_probe import planner as core
from .planner import Planner


def amplitude_track(target,factor):
    if not 0.<=factor<=1.:raise ValueError('continuation factor outside [0,1]')
    track=copy.deepcopy(target)
    for window,original in zip(track.windows,target.windows):
        window.motion=replace(original.motion,amplitude=factor*original.motion.amplitude)
    return track


def solve_once(planner,seed,seconds,progress):
    previous=core.minimize
    def literal(fun,x,*args,**kwargs):
        kwargs['options']=dict(kwargs['options'],maxiter=planner.max_iterations)
        return previous(lambda y:planner.literal_time_objective(fun,y),x,*args,**kwargs)
    core.minimize=literal
    try:return core.Planner.solve(planner,seconds,progress,initial_seeds=[seed])
    finally:core.minimize=previous


class ContinuationPlanner:
    def __init__(self,track,free_points=True,*,pose_function,max_iterations=2000):
        self.track=track;self.free_points=free_points;self.pose_function=pose_function;self.max_iterations=max_iterations

    def solve(self,total_seconds=720.,per_stage_seconds=180.,progress=lambda row:None):
        if total_seconds<=0 or per_stage_seconds<=0:raise ValueError('positive budgets required')
        began=time.perf_counter();deadline=began+total_seconds;stages=[];build_total=0.
        plan=None;lam=0.;step=.25
        def stage(value,seed):
            nonlocal build_total
            start=time.perf_counter()
            p=Planner(amplitude_track(self.track,value),self.free_points,pose_function=self.pose_function,max_iterations=self.max_iterations)
            build_total+=time.perf_counter()-start
            x=p.seeds()[int(seed)] if np.isscalar(seed) else np.array(seed)
            current,stats=solve_once(p,x,min(per_stage_seconds,max(.01,deadline-time.perf_counter())),
                lambda r:progress(dict(lambda_target=value,**r)))
            stages.append(dict(lambda_target=value,found=current is not None,statistics=stats,
                time_s=float(current.durations.sum()) if current else None))
            progress(dict(stage='continuation_stage_finished',lambda_target=value,found=current is not None,
                time_s=float(current.durations.sum()) if current else None,stages=len(stages)))
            return current
        for seed in range(4):
            if time.perf_counter()>=deadline:break
            plan=stage(0.,seed)
            if plan is not None:break
        if plan is not None:
            while lam<1. and time.perf_counter()<deadline and step>=1/64:
                value=min(1.,lam+step);candidate=stage(value,plan.x)
                if candidate is not None:plan=candidate;lam=value;step=min(.25,step*2)
                else:step/=2
        found=bool(plan is not None and lam==1.)
        result=dict(found=found,solve_s=time.perf_counter()-began,build_s=build_total,
            lambda_reached=lam,step=step,stages=stages,optimizer_convergence_required=True,
            original_togt_point_map_preserved=True,initialization='adaptive amplitude continuation',
            total_budget_s=total_seconds,per_stage_budget_s=per_stage_seconds,
            numerical_objective='literal total time',max_iterations=self.max_iterations,
            auxiliary_result_is_not_target_success=True)
        return (plan if found else None),result
