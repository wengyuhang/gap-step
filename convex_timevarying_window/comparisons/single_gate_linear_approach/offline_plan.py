"""Same optimizer, identical nominal PVAJ and t=0, no live scene clock."""
from dataclasses import asdict
import time
import numpy as np
from .scenario import Case,build_case,manifest
from .planner import Planner,Config
from convex_timevarying_window.comparisons.single_gate_constant_velocity.run import save


def compute(speed,variant,out,offset=30.):
    out.mkdir(parents=True,exist_ok=False)
    case=Case(speed,offset);track,_=build_case(case)
    # JIT changes evaluation speed, not the mathematical objective/constraints.
    cfg=Config(free_points=variant=='ours',full_task_error_recovery=False,compile_objective=False)
    started=time.perf_counter();planner=Planner(track,cfg);build_s=time.perf_counter()-started
    head=np.r_[track.start,np.zeros(9)]
    result=dict(case=asdict(case),variant=variant,config=asdict(cfg),head_pvaj=head.tolist(),
        made_at=0.,build_s=build_s,found=False,scene=manifest(case),new_planning_during_flight=False)
    started=time.perf_counter()
    try:
        plan=planner.solve(head,0.,0)
        result.update(found=True,durations=plan.durations.tolist(),coefficients=plan.coefficients.tolist(),
            variables=plan.x.tolist(),nominal_task_time_s=float(sum(plan.durations)))
    except Exception as exc:result['failure']=repr(exc)
    result.update(solve_s=time.perf_counter()-started,optimizer_stats=planner.solvers[0][0].stats())
    save(out/'plan.json',result)
    return result
