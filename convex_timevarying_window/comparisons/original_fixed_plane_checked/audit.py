"""Keep the strict truth geometry, but audit original PVA rather than fake MPC logs."""
import json
import numpy as np
from 复现.polynomial_dynamic_racing_2023.px4_fixed_plane.scenario import Case, build_case
from 复现.polynomial_dynamic_racing_2023.px4_fixed_plane.audit import truth, geometry
from convex_timevarying_window.comparisons.checked_soft_mpc_racing.run import timing
from convex_timevarying_window.comparisons.original_fixed_plane_ours.run import strict_truth


def audit(path):
    raw=json.loads((path/'result.json').read_text());track,_=build_case(Case(**raw['case']))
    times,positions=truth(path,raw['last_time_s']);g=geometry(track,times,positions)
    speed=np.linalg.norm(np.gradient(positions,times,axis=0),axis=1)
    g.update(last_time_s=raw['last_time_s'],physical_success=bool(raw['completed'] and g['ordered_once'] and
        g['complete_body_exits'] and not raw['collision_detected']),
        sampled_envelope_passed=bool(g['min_sampled_sphere_frame_clearance_m']>=0),
        planner_cycle_ms=timing([s['cycle_ms'] for s in raw['solves']]),
        sender_max_ms=raw['sender_max_ms'],controller_deadline_passed=raw['sender_max_ms'] is not None and raw['sender_max_ms']<100,
        max_truth_speed_mps=float(speed.max()),truth_path_length_m=float(np.linalg.norm(np.diff(positions,axis=0),axis=1).sum()),
        truth_used_for_control=False,evidence='Original PVA transport; 4ms truth sampling, not certification')
    g.update(strict_truth(path,g))
    np.savez_compressed(path/'truth_audit.npz',times=times,positions=positions,speed=speed)
    (path/'audit.json').write_text(json.dumps(g,indent=2)+'\n')
    return g
