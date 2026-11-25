"""A flying lookahead endpoint is arrival evidence, not a completed body exit."""
import numpy as np
from convex_timevarying_window.wang_free_state_mpc.events import (
    Progress, planning_block_reason, plane_events, local_coordinates)


def plan_event_check(plan, windows, signs, cfg):
    tt = np.unique(np.r_[np.linspace(plan.made_at, plan.end_time,
        max(2, int(np.ceil((plan.end_time-plan.made_at)/.01))+1)), plan.crossing_times])
    positions = plan.sample_many(tt)
    expected = dict(zip(range(plan.first_gate, plan.first_gate+plan.gate_count), plan.crossing_times))
    slab = cfg.radius+cfg.frame_thickness
    for wi, w in enumerate(windows):
        events, z = plane_events(w, signs[wi], positions, tt, gate=wi, nearby_margin=slab)
        inside = [e for e in events if e['inside']]
        if wi not in expected:
            if inside: return False, f'unplanned_gate_event:{wi}'
            continue
        if wi == plan.first_gate and z[0] <= 0:
            return False, f'head_already_beyond_target:{wi}'
        flying_endpoint = not plan.terminal_goal and wi == plan.first_gate+plan.gate_count-1
        if flying_endpoint:
            local = local_coordinates(w, positions[-1:], tt[-1:])[0]
            if abs(local[2]) > 1e-6 or not w.aperture.contains(local[:2]):
                return False, f'endpoint_misses_opening:{wi}'
            if inside or any(e['direction']=='backward' for e in events):
                return False, f'early_endpoint_passage:{wi}'
            # Analytic relative plane derivative, including known gate motion.
            center, R, dc, dR = w.state_at(plan.end_time)
            relative = -signs[wi]*(R[:, 2]@(plan.sample(plan.end_time, 1)-dc)
                + dR[:, 2]@(positions[-1]-center))
            earlier = np.flatnonzero(tt < plan.end_time-1e-6)
            if relative < .1-1e-6 or not len(earlier) or z[earlier[-1]] <= 0:
                return False, f'endpoint_not_forward:{wi}'
            continue
        if len(inside)!=1 or inside[0]['direction']!='forward':
            return False, f'non_single_passage:{wi}'
        if abs(inside[0]['time']-expected[wi]) > .02:
            return False, f'early_or_late_passage:{wi}'
        if any(e['direction']=='backward' and e['time']>expected[wi] for e in events):
            return False, f'return_to_gate:{wi}'
        if not np.any((tt>expected[wi]) & (z<=-slab)):
            return False, f'body_not_exited:{wi}'
    return True, 'single_passage_or_flying_lookahead_endpoint'
