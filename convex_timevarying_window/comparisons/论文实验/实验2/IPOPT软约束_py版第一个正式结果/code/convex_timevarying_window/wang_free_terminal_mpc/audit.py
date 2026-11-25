"""Check actual optimized endpoint PVAJ against saved polynomial coefficients."""
import json
import numpy as np
from 复现.polynomial_dynamic_racing_2023.minco import basis


def mechanism_audit(path):
    raw=json.loads((path/'result.json').read_text()); failures=[]; checked=0; warm=0; error=0.
    for s in raw['solves']:
        if 'variables' not in s: continue
        x=np.array(s['variables']);seed=np.array(s['initial_variables'])
        terminal=s['terminal_goal']; cfg=raw['config']
        if len(x)!=(28 if terminal else 37): failures.append('decision_dimension');continue
        if s['pieces_per_leg']!=4 or len(s['durations'])!=8: failures.append('N_M')
        if np.max(np.abs(seed[26:28]))>1e-10: failures.append('noncenter_seed')
        warm+=s['seed_provenance'][-1].get('terminal_state_source')=='admitted'
        if 'coefficients' not in s: continue
        C=np.array(s['coefficients'])[-1];h=s['durations'][-1]
        measured=np.array([basis(1.,d)@C/h**d for d in range(4)])
        target=(np.array([[-16.,4.,3.2],[0.,0.,0.],[0.,0.,0.],cfg['jerk']*x[23:26]]) if terminal else
                np.array([10*x[28:31],cfg['speed']*x[31:34],cfg['acceleration']*x[34:37],cfg['jerk']*x[23:26]]))
        e=float(np.max(np.abs(measured-target)));error=max(error,e);checked+=1
        if e>1e-6: failures.append('terminal_PVAJ_not_applied')
    if not checked: failures.append('no_logged_polynomial_to_verify')
    result=dict(passed=not failures,failures=failures,polynomials_checked=checked,
                max_terminal_pvaj_error=error,warm_terminal_seeds=warm)
    (path/'mechanism_audit.json').write_text(json.dumps(result,indent=2)+'\n')
    return result
