"""Offline only: what would the removed admission check have said?"""
import json
import numpy as np
from .adapter import install
from .planner import Planner, Config
from convex_timevarying_window.togt_minco_mpc.planner import Planner as Geometry
from convex_timevarying_window.wang_free_state_mpc.trajectory import Plan
from 复现.polynomial_dynamic_racing_2023.px4_fixed_plane.scenario import Case,build_case


def audit(path):
    install()
    raw=json.loads((path/'result.json').read_text());cfg=Config(**raw['config'])
    track,_=build_case(Case(**raw['case']));p=Planner(track,cfg,build_indices=[])
    rows=[];errors=[]
    for s in raw['solves']:
        if not s['accepted']:continue
        if s.get('post_trajectory_validation_enabled') is not False or s.get('prefix_clearance') is not None:
            errors.append(dict(time=s['time'],reason='online_clearance_not_explicitly_unchecked'))
        if 'coefficients' not in s:
            errors.append(dict(time=s['time'],reason='missing_polynomial'));continue
        first=s['candidate_first_gate']
        plan=Plan(s['made_at'],first,np.array(s['durations']),np.array(s['coefficients']),np.array(s['variables']),
                  0.,s['violation'],s['status'],min(cfg.horizon,len(track.windows)-first))
        clearance=Geometry.validate(p,plan)
        rows.append(dict(time=s['time'],first_gate=first+1,
            original_clearance_m=float(clearance) if np.isfinite(clearance) else None,
            original_dynamic_or_terminal_check_failed=not np.isfinite(clearance),
            would_be_rejected=bool(clearance<cfg.tracking_margin)))
    result=dict(evidence='Offline saved-candidate evaluation; never used by controller',
        implementation_passed=not errors,errors=errors,accepted_polynomials=len(rows),
        original_check_would_reject=sum(r['would_be_rejected'] for r in rows),candidates=rows)
    (path/'removed_check_audit.json').write_text(json.dumps(result,indent=2)+'\n')
    return {k:v for k,v in result.items() if k!='candidates'}
