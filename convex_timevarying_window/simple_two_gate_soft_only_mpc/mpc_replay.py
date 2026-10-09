"""Replay saved controller inputs; capture the old hard QP, not a new flight."""
import argparse
import json
from pathlib import Path
import numpy as np
from scipy.optimize import linprog
from convex_timevarying_window.checked_soft_mpc_isolated.fast.mpc import MPC as OldMPC
from convex_timevarying_window.checked_soft_mpc_racing.mpc import Config, hover_state, NU
from convex_timevarying_window.event_free_terminal_mpc.core import Plan


class Capture(OldMPC):
    def condense(self, x, states, controls, refs, ff):
        result = super().condense(x, states, controls, refs, ff)
        self.problem = result
        self.problem_context = dict(x=x, states=states, controls=controls, refs=refs, ff=ff)
        return result


def replay(path, output):
    raw = json.loads((path/'result.json').read_text())
    rows = np.load(path/'execution.npz')['rows']
    failure = np.load(path/'sender_failure.npz')
    plans = [(s['time'], Plan(s['made_at'], s['candidate_first_gate'], np.array(s['durations']),
              np.array(s['coefficients']), np.array(s['variables']), 0., s['violation'], s['status'],
              len(s['durations'])//2-1)) for s in raw['solves'] if s['accepted']]
    mpc = Capture(Config(**raw['mpc_config']))
    start = np.array([-16.,4.,3.2]); epoch = -100.
    def reference(times):
        installed = [p for at,p in plans if at <= epoch+1e-8]
        values=[]
        for t in times:
            active = [p for p in installed if p.made_at <= t]
            p = active[-1] if active else None
            values.append(np.r_[start,np.zeros(6)] if p is None else np.concatenate([
                p.sample(t,d) if d==0 or t<p.end_time else np.zeros(3) for d in range(3)]))
        return np.array(values)
    mpc.command(hover_state(start), -1., reference)
    mpc.logs.clear();mpc.controls=None;mpc.last_time=None
    differences=[]; result={}
    for cycle in raw['mpc_cycles']:
        epoch=cycle['time']
        if cycle.get('failure'):
            state=failure['state']
        else:
            index=int(np.argmin(abs(rows[:,0]-epoch)))
            if abs(rows[index,0]-epoch)>1e-6: continue
            state=rows[index,1:18]
        try:
            command=mpc.command(state,epoch,reference)
            if not cycle.get('failure'):differences.append(float(np.max(abs(command-rows[index,18:22]))))
        except RuntimeError as exc:
            P,q,A,lower,upper=mpc.problem
            n=mpc.cfg.steps*NU
            T=A[n:].toarray();bound=lower[n:]
            lp=linprog(np.zeros(n),A_ub=-T,b_ub=-bound,bounds=list(zip(lower[:n],upper[:n])),method='highs')
            best_possible=np.sum(np.where(T>=0,T*upper[:n],T*lower[:n]),axis=1)
            result=dict(replay_failure=str(exc),replay_time=epoch,recorded_failure_time=float(failure['time']),
                        prefix_max_control_difference=max(differences,default=0.),prefix_steps=len(differences),
                        linear_program_status=int(lp.status),linear_program_message=lp.message,
                        impossible_tilt_nodes=np.flatnonzero(best_possible<bound-1e-8).tolist(),
                        first_tilt_constraint_lower=float(bound[0]),first_tilt_max_under_input_box=float(best_possible[0]))
            output.mkdir(parents=True,exist_ok=True)
            np.savez_compressed(output/'hard_qp.npz',P=P.toarray(),q=q,A=A.toarray(),lower=lower,upper=upper,
                                state=state,previous=mpc.previous,**mpc.problem_context)
            break
    if not result:result=dict(reproduced=False,prefix_steps=len(differences),message='No failure reproduced')
    output.mkdir(parents=True,exist_ok=True)
    (output/'diagnosis.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('path',type=Path);p.add_argument('output',type=Path);a=p.parse_args()
    replay(a.path,a.output)
