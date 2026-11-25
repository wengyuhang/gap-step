"""Post-flight audit; never part of the online controller or planner."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from convex_timevarying_window.comparisons.single_gate_translation.audit import audit
from 复现.polynomial_dynamic_racing_2023.px4_fixed_plane.audit import truth
from convex_timevarying_window.togt_point_only_mpc.trajectory import Plan


def summarize(directory):
    raw = json.loads((directory/'result.json').read_text())
    physical = audit(directory)
    goal = np.array(json.loads((directory/'scene.json').read_text())['goal'])
    ts, positions = truth(directory, raw['last_time_s'])
    at = physical['crossings'][0]['time_s'] if physical['ordered_once'] else None
    tail = positions[ts >= at] if at is not None else np.empty((0, 3))
    row = dict(directory=str(directory), audit=physical,
        planning_max_cycle_ms=max((s['cycle_ms'] for s in raw['solves']), default=None),
        mpc_max_cycle_ms=max((s['cycle_ms'] for s in raw['mpc_cycles']), default=None),
        planning_requests=len(raw['solves']),
        failed_planning_requests=sum(not s['accepted'] for s in raw['solves']),
        actual_tail_time_s=None if at is None or raw['flight_time_s'] is None else raw['flight_time_s']-at,
        actual_tail_path_length_m=None if not len(tail) else float(np.linalg.norm(np.diff(tail, axis=0), axis=1).sum()),
        actual_tail_normal_overshoot_m=None if not len(tail) else max(0., float(tail[:, 0].max()-goal[0])))
    accepted = [s for s in raw['solves'] if s['accepted']]
    if len(accepted) == 1:
        s = accepted[0]
        plan = Plan(s['made_at'], 0, np.array(s['durations']), np.array(s['coefficients']),
                    np.array(s['variables']), s['cycle_ms'], 0., s['status'], 1)
        times = np.linspace(plan.crossing_times[0], plan.end_time, 8001)
        ps, vs = plan.sample_many(times), plan.sample_many(times, 1)
        row['nominal'] = dict(durations_s=s['durations'], end_time_s=plan.end_time,
            original_objective=s['optimizer_stats']['final_objective'],
            dynamic_violation=s['violation'],
            tail_path_length_m=float(np.linalg.norm(np.diff(ps, axis=0), axis=1).sum()),
            tail_normal_overshoot_m=max(0., float(ps[:, 0].max()-goal[0])),
            tail_min_normal_speed_mps=float(vs[:, 0].min()))
    return raw, row


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--flight', type=Path, required=True)
    parser.add_argument('--baseline', type=Path, required=True)
    args = parser.parse_args(); flight, baseline = args.flight.resolve(), args.baseline.resolve()
    new_raw, new = summarize(flight); old_raw, old = summarize(baseline)
    matched = {k:new_raw[k] == old_raw[k] for k in ('case', 'parameters', 'mpc_config')}
    matched['world'] = (hashlib.sha256((flight/'world.sdf').read_bytes()).hexdigest()
                        == hashlib.sha256((baseline/'world.sdf').read_bytes()).hexdigest())
    matched['terminal_rule'] = True  # Same reused executor, not inferred from final position.
    source_hashes = json.loads((flight/'passage_sources.json').read_text())
    root = Path(__file__).resolve().parents[2]
    matched['frozen_sources'] = all(hashlib.sha256((root/k).read_bytes()).hexdigest() == v
                                    for k, v in source_hashes.items())
    both_passed = bool(new['audit']['passed'] and old['audit']['passed'] and all(matched.values()))
    report = dict(new=new, baseline=old, matched=matched, both_passed=both_passed,
        full_single_gate_time_reduction_pct=(100*(1-new_raw['flight_time_s']/old_raw['flight_time_s'])
                                              if both_passed else None),
        caveats=['One physical flight per version; different measured starting states.',
                 'Not a paired statistical study, seven-window result, or safety certificate.',
                 'Same terminal stop rule and 0.55 m / 1 m/s completion tolerance.',
                 'Forward-tail condition restricts the feasible curve family; original dynamics remain soft.'])
    (flight/'physical_audit.json').write_text(json.dumps(new['audit'], indent=2)+'\n')
    (flight/'comparison.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__': main()
