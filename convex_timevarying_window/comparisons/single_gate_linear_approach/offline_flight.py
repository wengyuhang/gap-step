"""Execute a previously accepted affine-window plan, without any online solve."""
import argparse
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace
import sys
import numpy as np
from convex_timevarying_window.dynamic_passage_velocity_minco.trajectory import Plan
from convex_timevarying_window.global_gate_minco_probe.flight import Reference
from convex_timevarying_window.comparisons.single_gate_constant_velocity.run import save
from .planner import Config


def main():
    p=argparse.ArgumentParser();p.add_argument('--plan',type=Path,required=True)
    p.add_argument('--outdir',type=Path,required=True);a=p.parse_args()
    src=a.plan.resolve();r=json.loads(src.read_text())
    if not r['found']:raise ValueError('No admitted offline plan; not a flight result')
    plan=Plan(r['made_at'],0,np.array(r['durations']),np.array(r['coefficients']),np.array(r['variables']),
        r['solve_s']*1000,0.,'offline_frozen',1)
    from . import flight
    from .audit import audit as original_audit
    from convex_timevarying_window.simple_two_gate_soft_only_mpc import flight as shared
    class Worker:
        def __init__(self,track,cfg):
            self.geometry=SimpleNamespace(cfg=cfg,windows=list(track.windows),
                signs=[float(np.sign((track.start-w.center0)@w.state_at(0.)[1][:,2])) for w in track.windows])
            self.startup=dict(offline_precomputed_plan=True,source=str(src),new_planning=False)
            self.busy=False;self.metrics=[]
        def poll(self):return None
        def close(self):pass
    class Scheduler:
        def __init__(self,**kwargs):
            self.log=[];self.error_high=kwargs['error_high'];self.error_low=kwargs['error_low'];self.retry_delay=.2
        def observe(self,*args):pass
        def due(self,*args):return False
    def audit(path):
        raw=json.loads((path/'result.json').read_text())
        raw.update(offline_plan=r,offline_plan_source=str(src),offline_plan_sha256=hashlib.sha256(src.read_bytes()).hexdigest(),
            new_planning_during_flight=False,planning_scheduler='none_offline_frozen',
            offline_planning_s=r['solve_s'],nominal_task_time_s=r['nominal_task_time_s'])
        assert not raw['solves'],'Unexpected online planning'
        save(path/'result.json',raw)
        result=original_audit(path)
        if result['verdict']=='online_no_accepted_plan':result['verdict']='not_completed'
        for e in result['crossings']:
            e.update(planned_point_local_m=r['variables'][2:4],
                planned_crossing_time_s=r['made_at']+r['durations'][0])
        trace=np.load(path/'mechanism_trace.npz')
        reference=plan.sample_many(trace['times'])
        errors=np.linalg.norm(trace['positions']-reference,axis=1)
        result.update(offline_plan_found=True,offline_planning_s=r['solve_s'],online_planning_requests=0,
            nominal_task_time_s=r['nominal_task_time_s'],true_tracking_error_max_m=float(errors.max()),
            true_tracking_error_rms_m=float(np.sqrt(np.mean(errors**2))),
            planned_crossing_time_s=r['made_at']+r['durations'][0],
            plan_source_sha256=hashlib.sha256(src.read_bytes()).hexdigest())
        np.savez_compressed(path/'tracking_audit.npz',times=trace['times'],reference=reference,
            positions=trace['positions'],errors=errors)
        save(path/'comparison_audit.json',result)
        return result
    flight.Worker=Worker;flight.Scheduler=Scheduler;flight.audit=audit
    flight.Config=lambda **kwargs:Config(compile_objective=False,**kwargs)
    shared.Reference=lambda start,count:Reference(start,plan)
    sys.argv=[flight.__name__,'--speed',str(r['case']['speed']),'--offset',str(r['case'].get('offset',30.)),
        '--method',r['variant'],'--outdir',str(a.outdir)]
    flight.main()


if __name__=='__main__':main()
