"""Independent post-solve nominal evidence; never changes solve acceptance."""
import argparse
import json
from pathlib import Path
import time
import numpy as np
from convex_timevarying_window.single_gate_stop_racing.scenario import Case, build_case
from convex_timevarying_window.single_gate_stop_racing.planner import route_signs
from convex_timevarying_window.dynamic_passage_velocity_minco.trajectory import Plan
from convex_timevarying_window.dynamic_passage_velocity_minco.hard_dynamics import flat_actuators
from convex_timevarying_window.dynamic_passage_velocity_minco.allocation_mpc import KF


def audit(out):
    began = time.perf_counter()
    result = json.loads((out/'result.json').read_text())
    protocol = json.loads((out/'protocol.json').read_text())
    config = json.loads((out/'build.json').read_text())['config']
    track, _ = build_case(Case(**protocol['case']))
    plan = Plan(0., 0, np.array(result['durations_s']), np.array(result['coefficients']),
                np.array(result['variables']), result['solve_s']*1000, 0., 'archived_global', 7)
    total = float(plan.durations.sum())
    times = np.unique(np.r_[np.linspace(0., total, int(np.ceil(total/.001))+1), np.cumsum(plan.durations)])
    derivatives = [plan.sample_many(times, d) for d in range(5)]
    position = derivatives[0]
    required = config['radius']+config['frame_thickness']+config['passage_margin']
    frames = []
    events = []
    starts = np.r_[0., np.cumsum(plan.durations)[:-1]]
    signs = route_signs(track)
    for gate, w in enumerate(track.windows):
        R = w.state_at(0.)[1]
        centers = w.center0+np.sin(2*np.pi*times[:, None]/w.motion.translation_period+w.motion.phase)*w.motion.translation_amplitude
        q = (position-centers)@R
        distance = np.hypot(w.aperture.boundary_distance(q[:, :2]), q[:, 2])
        index = int(np.argmin(distance))
        frames.append(dict(gate=gate+1, minimum_distance_m=float(distance[index]),
            required_distance_m=required, minimum_margin_m=float(distance[index]-required),
            time_s=float(times[index])))
        found = []
        for piece, C in enumerate(plan.coefficients):
            normal_poly = C@R[:, 2]
            normal_poly[0] -= w.center0@R[:, 2]
            for root in np.polynomial.polynomial.polyroots(normal_poly):
                if abs(root.imag) > 1e-6 or not -1e-7 <= root.real <= 1+1e-7:
                    continue
                at = float(starts[piece]+np.clip(root.real, 0., 1.)*plan.durations[piece])
                local, _ = w.world_to_local(plan.sample(at), at)
                if w.aperture.contains(local) and all(abs(v['time_s']-at) > 1e-5 for v in found):
                    found.append(dict(time_s=at, local_point=local.tolist(),
                        forward=bool(-signs[gate]*plan.sample(at, 1)@R[:, 2] > .1)))
        events.append(dict(gate=gate+1, events=sorted(found, key=lambda v: v['time_s']),
            expected_time_s=float(plan.crossing_times[gate])))
    # Batched evaluation of the original flatness map; no optimization Jacobian.
    mapped = flat_actuators.map(512)
    all_rates, all_rotors = [], []
    for start in range(0, len(times), 512):
        stop = min(len(times), start+512)
        arguments = [np.pad(v[start:stop].T, ((0, 0), (0, 512-(stop-start))), mode='edge')
                     for v in derivatives[1:5]]
        values = mapped(*arguments)
        all_rates.append(np.asarray(values[1])[:, :stop-start].T)
        all_rotors.append(np.asarray(values[2])[:, :stop-start].T)
    rates, rotors = np.vstack(all_rates), np.vstack(all_rotors)
    low, high = KF*150**2, KF*1000**2
    planning_high = high*protocol.get('planning_thrust_fraction', 1.)
    limits = np.array([3.8, 3.8, 3.49])
    terminal = np.column_stack([plan.sample(total, d) for d in range(4)])
    terminal[:, 0] -= track.goal
    exactly_once = all(len(v['events']) == 1 and v['events'][0]['forward'] and
                       abs(v['events'][0]['time_s']-v['expected_time_s']) < 1e-4 for v in events)
    audit = dict(step_max_s=float(np.max(np.diff(times))), samples=len(times), frames=frames,
        minimum_frame_margin_m=min(v['minimum_margin_m'] for v in frames),
        ordered_single_forward_crossings=exactly_once, opening_events=events,
        terminal_pvaj_error_abs_max=float(np.abs(terminal).max()),
        rotor_min_N=float(rotors.min()), rotor_max_N=float(rotors.max()),
        rotor_lower_N=low, rotor_upper_N=high,
        planning_rotor_upper_N=planning_high,
        rotor_planning_high_excess_percent=max(0.,float(rotors.max()/planning_high-1))*100,
        rotor_high_excess_percent=max(0., float(rotors.max()/high-1))*100,
        rotor_low_violation_N=max(0., float(low-rotors.min())),
        rate_abs_max_rad_s=np.max(np.abs(rates), axis=0).tolist(),
        rate_limit_rad_s=limits.tolist(),
        workspace_violation_m=max(0., float(np.max(np.array(config['workspace_lower'])-position)),
                                   float(np.max(position-np.array(config['workspace_upper'])))),
        audit_s=time.perf_counter()-began,
        evidence='independent 1ms nominal sampling and polynomial opening events; not continuous dynamics or closed-loop certificate',
        included_in_first_qualified_solve_time=False)
    (out/'independent_audit.json').write_text(json.dumps(audit, indent=2, ensure_ascii=False)+'\n')
    return audit


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('outdir', type=Path)
    args = parser.parse_args()
    print(json.dumps(audit(args.outdir.resolve()), indent=2))
