"""Verify bounded fresh commands on saved failed states, without claiming a flight."""
import json
from pathlib import Path
import numpy as np
from convex_timevarying_window.simple_two_gate_soft_only_mpc.elastic_mpc import MPC
from convex_timevarying_window.checked_soft_mpc_racing.mpc import LOWER, UPPER
from convex_timevarying_window.event_free_terminal_mpc.core import Plan


def main():
    root=Path(__file__).parent/'results'
    cases={'center':'baseline_center_f1_a2_p000_attempt1','ours':'joint_ours_f1.5_a2.5_p000_attempt1'}
    output=[]
    for name,directory in cases.items():
        path=root/'half_step_v1_20261008'/directory
        raw=json.loads((path/'result.json').read_text())
        snapshot=np.load(root/'mpc_diagnosis_20261008'/name/'hard_qp.npz')
        failure=np.load(path/'sender_failure.npz');at=float(failure['time'])
        plans=[Plan(s['made_at'],s['candidate_first_gate'],np.array(s['durations']),np.array(s['coefficients']),
                    np.array(s['variables']),0.,s['violation'],s['status'],len(s['durations'])//2-1)
               for s in raw['solves'] if s['accepted'] and s['time']<=at]
        def reference(times):
            result=[]
            for t in times:
                p=[p for p in plans if p.made_at<=t][-1]
                result.append(np.concatenate([p.sample(t,d) if d==0 or t<p.end_time else np.zeros(3) for d in range(3)]))
            return np.array(result)
        m=MPC();m.controls=snapshot['controls'];m.previous=snapshot['previous'];m.last_time=at
        u=m.command(failure['state'],at,reference)
        result=dict(case=name,time=at,command=u.tolist(),previous=snapshot['previous'].tolist(),
                    control_change=float(np.linalg.norm(u-snapshot['previous'])),
                    finite=bool(np.isfinite(u).all()),inside_physical_input_box=bool(np.all(u>=LOWER)&np.all(u<=UPPER)),
                    log=m.logs[-1],evidence='single saved-state recovery, not a flight')
        assert result['finite'] and result['inside_physical_input_box']
        assert any(a>0 for a in m.logs[-1]['nonlinear_steps'])
        output.append(result)
    (root/'mpc_diagnosis_20261008'/'elastic_recovery.json').write_text(json.dumps(output,indent=2)+'\n')
    print(json.dumps(output,indent=2))


if __name__=='__main__':main()
