"""Audit enlarged physical geometry, never the old small apertures."""
import json
import numpy as np
from 复现.polynomial_dynamic_racing_2023.px4_fixed_plane.audit import truth,geometry
from convex_timevarying_window.comparisons.checked_soft_mpc_racing.run import control_audit,timing
from convex_timevarying_window.comparisons.original_fixed_plane_ours.run import strict_truth
from .scenario import build_case,Case


def audit(path):
    r=json.loads((path/'result.json').read_text())
    scene=json.loads((path/'scene.json').read_text());assert scene['aperture_scale']==3.
    track,_=build_case(Case(**r['case']));ts,pos=truth(path,r['last_time_s'])
    g=geometry(track,ts,pos);execution=np.load(path/'execution.npz')['rows']
    g.update({k:r.get(k) for k in ('completed','failure','flight_time_s','last_time_s','collision_detected')})
    g.update(control_audit(r,execution));g['planner_cycle_ms']=timing([s['cycle_ms'] for s in r['solves']])
    g.update(body_exit_count=len({e['gate'] for e in g['body_exits']}),
        physical_success=bool(r['completed'] and g['ordered_once'] and g['complete_body_exits'] and not r['collision_detected']),
        sampled_envelope_passed=g['min_sampled_sphere_frame_clearance_m']>=0)
    g.update(strict_truth(path,g))
    speed=np.linalg.norm(np.gradient(pos,ts,axis=0),axis=1)
    np.savez_compressed(path/'truth_audit.npz',times=ts,positions=pos,speed=speed)
    return g
