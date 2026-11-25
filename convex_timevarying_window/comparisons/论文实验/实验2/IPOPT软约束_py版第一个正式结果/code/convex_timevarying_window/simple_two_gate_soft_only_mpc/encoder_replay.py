"""Reproduce CTBR encoder rejection using the exact saved feedback stream."""
import json
from pathlib import Path
import numpy as np
from .elastic_mpc import MPC, Config
from convex_timevarying_window.checked_soft_mpc_racing.mpc import hover_state, LOWER, UPPER
from convex_timevarying_window.dynamic_gate_mppi_v3.ctbr import encode_ctbr
from convex_timevarying_window.event_free_terminal_mpc.core import Plan


def main():
    path=Path(__file__).parent/'results/half_step_elastic_v2_20261008/baseline_center_f1_a2_p000_attempt1'
    raw=json.loads((path/'result.json').read_text());rows=np.load(path/'execution.npz')['rows']
    failure=np.load(path/'sender_failure.npz');plans=[]
    for s in raw['solves']:
        if s['accepted']:
            plans.append((s['time'],Plan(s['made_at'],s['candidate_first_gate'],np.array(s['durations']),
                np.array(s['coefficients']),np.array(s['variables']),0.,s['violation'],s['status'],len(s['durations'])//2-1)))
    epoch=-100.;start=np.array([-16.,4.,3.2])
    def reference(times):
        values=[]
        for t in times:
            active=[p for at,p in plans if at<=epoch+1e-8 and p.made_at<=t]
            p=active[-1] if active else None
            values.append(np.r_[start,np.zeros(6)] if p is None else np.concatenate([
                p.sample(t,d) if d==0 or t<p.end_time else np.zeros(3) for d in range(3)]))
        return np.array(values)
    m=MPC(Config(**raw['mpc_config']));m.command(hover_state(start),-1.,reference)
    m.logs.clear();m.controls=None;m.last_time=None
    errors=[];result=None
    for cycle in raw['mpc_cycles']:
        epoch=cycle['time'];index=int(np.argmin(abs(rows[:,0]-epoch)))
        recorded=abs(rows[index,0]-epoch)<1e-6
        if not recorded and abs(float(failure['time'])-epoch)>1e-6:continue
        state=rows[index,1:18] if recorded else failure['state']
        u=m.command(state,epoch,reference)
        try:encode_ctbr(u)
        except ValueError as exc:
            result=dict(time=epoch,recorded_failure_time=float(failure['time']),command=u.tolist(),
                upper_excess=(u-UPPER).tolist(),lower_excess=(LOWER-u).tolist(),error=str(exc),
                prefix_max_control_difference=max(errors,default=0.),prefix_steps=len(errors))
            break
        if recorded:errors.append(float(np.max(abs(u-rows[index,18:22]))))
    if result is None:result=dict(reproduced=False,prefix_steps=len(errors),prefix_max_control_difference=max(errors,default=0.))
    out=Path(__file__).parent/'results/mpc_diagnosis_20261008/encoder_replay.json'
    out.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))


if __name__=='__main__':main()
