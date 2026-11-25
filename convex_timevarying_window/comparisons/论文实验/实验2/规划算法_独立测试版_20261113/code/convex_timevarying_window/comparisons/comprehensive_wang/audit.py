"""Independent race-prefix audit; return completion is a SEPARATE outcome."""
import json
import numpy as np
from 复现.polynomial_dynamic_racing_2023.px4_fixed_plane.audit import audit as base_audit,geometry,truth
from 复现.polynomial_dynamic_racing_2023.px4_fixed_plane.scenario import Case,build_case
from convex_timevarying_window.comparisons.original_fixed_plane_ours.run import strict_truth


def audit(path):
    raw=json.loads((path/'result.json').read_text())
    row={k:raw.get(k) for k in ('completed','failure','flight_time_s','last_time_s','collision_detected')}
    row.update(base_audit(path));row.update(strict_truth(path,row))
    row['race_passed']=False;row['race_time_s']=None
    row['body_exit_count']=len({e['gate'] for e in row['body_exits']})
    times,pos=truth(path,raw['last_time_s']);speed=np.linalg.norm(np.gradient(pos,times,axis=0),axis=1)
    row['low_speed_time_s']=float(np.sum(speed<1.)*.004)
    at=next((e['time'] for e in row['body_exits'] if e['gate']==6),None)
    if at is not None:
        ids=times<=at+1e-9;track,_=build_case(Case(**raw['case']));prefix=geometry(track,times[ids],pos[ids])
        events=[e for e in prefix['events'] if e['inside']]
        ordered=[(e['gate'],e['direction']) for e in events]==[(i,'forward') for i in range(7)]
        enter={e['gate']:e['time'] for e in events};exits={e['gate']:e['time'] for e in prefix['body_exits']}
        exit_order=all(exits.get(i,np.inf)<enter.get(i+1,-np.inf) for i in range(6))
        collision=any(c['first_contact_sim_time_s'] is not None and c['first_contact_sim_time_s']-30<=at for c in raw['contacts'])
        planners=[s['cycle_ms'] for s in raw['solves'] if s['time']<=at]
        controls=[s for s in raw['mpc_cycles'] if 0<=s['time']<=at]
        realtime=bool(planners and controls and max(planners)<100 and max(c['cycle_ms'] for c in controls)<100
                      and not any(c.get('failure') for c in controls))
        workspace=float(min(np.min(pos[ids]-[-28,-30,.65]),np.min([28,22,12]-pos[ids])))
        passed=ordered and prefix['complete_body_exits'] and exit_order and not collision and realtime and workspace>=0 and prefix['min_sampled_sphere_frame_clearance_m']>=0
        row.update(race_passed=bool(passed),race_time_s=float(at) if passed else None,
            raw_seventh_exit_s=float(at),race_prefix_clearance_m=prefix['min_sampled_sphere_frame_clearance_m'],
            race_prefix_workspace_m=workspace,race_prefix_realtime_passed=realtime,race_prefix_collision=collision,
            race_prefix_ordered=ordered,race_prefix_exit_order=exit_order,
            race_low_speed_time_s=float(np.sum(speed[ids]<1.)*.004))
    return row
