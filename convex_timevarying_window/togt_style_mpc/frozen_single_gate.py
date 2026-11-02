"""Re-fly an existing single-gate reference; never re-optimize or retime it."""
import argparse
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace
import sys
import numpy as np
from convex_timevarying_window.dynamic_passage_velocity_minco.trajectory import Plan
from convex_timevarying_window.global_gate_minco_probe.flight import Reference


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--baseline-dir',type=Path,required=True)
    parser.add_argument('--outdir',type=Path,required=True)
    parser.add_argument('--solver',choices=('SQP_RTI','SQP','L-BFGS-B'),default='L-BFGS-B')
    parser.add_argument('--input-weight',type=float,default=1.)
    args=parser.parse_args();base=args.baseline_dir.resolve();out=args.outdir.resolve()
    raw=json.loads((base/'result.json').read_text())
    s=next(s for s in raw['solves'] if s['accepted'])
    plan=Plan(s['made_at'],0,np.array(s['durations']),np.array(s['coefficients']),np.array(s['variables']),0,0,'frozen',1)
    from convex_timevarying_window.comparisons.single_gate_swing import flight,scenario
    from convex_timevarying_window.simple_two_gate_soft_only_mpc import flight as shared
    from convex_timevarying_window.togt_style_mpc import aligned_controller
    original_mpc=aligned_controller.MPC
    if args.solver=='L-BFGS-B':
        from .shooting_controller import MPC as Shooting,Config as ShootingConfig
        flight.TrackingMPC=lambda:Shooting(ShootingConfig(control_weight=args.input_weight))
    else:
        flight.TrackingMPC=lambda:original_mpc(aligned_controller.Config(solver_type=args.solver,control_weight=args.input_weight))
    from convex_timevarying_window.comparisons.single_gate_constant_velocity.run import sources
    root=Path(__file__).resolve().parents[2];hashes=sources(root)
    class Worker:
        def __init__(self,track,cfg):
            self.geometry=SimpleNamespace(cfg=cfg,windows=list(track.windows),
                signs=[float(np.sign((track.start-w.center0)@w.state_at(0.)[1][:,2])) for w in track.windows])
            self.startup=dict(frozen_reference=True,source=str(base),new_planning=False)
            self.busy=False;self.metrics=[]
        def poll(self):return None
        def close(self):pass
    class Scheduler:
        def __init__(self,**kwargs):self.log=[];self.error_high=kwargs['error_high'];self.error_low=kwargs['error_low'];self.retry_delay=.2
        def observe(self,*args):pass
        def due(self,*args):return False
    flight.Worker=Worker;flight.Scheduler=Scheduler
    shared.Reference=lambda start,count:Reference(start,plan)
    case=raw['case']
    sys.argv=[flight.__name__,'--frequency',str(case['frequency']),'--amplitude',str(case['amplitude']),
        '--method',raw['comparison_variant'],'--outdir',str(out)]
    try:flight.main()
    finally:
        if (out/'result.json').exists():
            result=json.loads((out/'result.json').read_text())
            result.update(frozen_reference_source=str(base),frozen_reference= s,new_planning=False)
            (out/'result.json').write_text(json.dumps(result,indent=2)+'\n')
            if not (out/'mechanism_trace.npz').exists():
                (out/'verification.json').write_text(json.dumps(dict(evidence_insufficient=True,
                    reason='no post-start truth trace',failure=result['failure'],baseline_dir=str(base)),indent=2)+'\n')
                raise RuntimeError('fixed_reference_trial_missing_truth: '+str(result['failure']))
            trace=np.load(out/'mechanism_trace.npz');times=trace['times'];ref=plan.sample_many(times)
            errors=np.linalg.norm(trace['positions']-ref,axis=1)
            old=np.load(base/'mechanism_trace.npz');old_error=np.linalg.norm(old['positions']-plan.sample_many(old['times']),axis=1)
            audit=json.loads((out/'comparison_audit.json').read_text())
            if audit['verdict']=='online_no_accepted_plan':audit['verdict']='not_completed'
            h=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
            verification=dict(source_unchanged_during_flight=sources(root)==hashes,
                source_sha256=hashes,
                world_identical=h(out/'world.sdf')==h(base/'world.sdf'),
                parameters_identical=result['parameters']==raw['parameters'],
                reference_identical=result['frozen_reference']==s,
                baseline_dir=str(base),baseline_error_max_m=float(old_error.max()),
                true_tracking_error_max_m=float(errors.max()),true_tracking_error_rms_m=float(np.sqrt(np.mean(errors**2))),
                baseline_verdict=json.loads((base/'comparison_audit.json').read_text())['verdict'],
                verdict=audit['verdict'],flight_time_s=audit['flight_time_s'],contacts=result['contacts'],
                mpc_max_ms=max([c['cycle_ms'] for c in result['mpc_cycles'] if c.get('time',-1)>=0],default=None))
            np.savez_compressed(out/'tracking_audit.npz',times=times,reference=ref,positions=trace['positions'],errors=errors)
            (out/'verification.json').write_text(json.dumps(verification,indent=2)+'\n')
            (out/'comparison_audit.json').write_text(json.dumps(audit,indent=2)+'\n')
            print('FIXED_REFERENCE_VERIFICATION',json.dumps({k:v for k,v in verification.items() if k!='source_sha256'}),flush=True)


if __name__=='__main__':main()
