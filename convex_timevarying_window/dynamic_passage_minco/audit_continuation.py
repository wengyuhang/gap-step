"""Audit v6 continuation and historical feasible flights without changing raw data."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from convex_timevarying_window.comparisons.single_gate_translation.scenario import Case, build_case
from convex_timevarying_window.togt_point_only_mpc.trajectory import Plan
from .safety import passage


def read_audit(path, raw, track):
    saved = path/'comparison_audit.json'
    if saved.exists(): return json.loads(saved.read_text())
    # Older direct probes have no saved audit. Read the ULog without writing
    # derived traces into that historical result directory.
    from 复现.polynomial_dynamic_racing_2023.px4_fixed_plane.audit import truth
    times, positions = truth(path, raw['last_time_s'])
    w = track.windows[0]; rotation = w.state_at(0.)[1]
    centers = np.array([w.state_at(float(t))[0] for t in times])
    local = (positions-centers)@rotation
    sign = np.sign((track.start-w.center0)@rotation[:, 2]); normal = sign*local[:, 2]
    crossings = []
    for k in np.flatnonzero(normal[:-1]*normal[1:] < 0):
        alpha = abs(normal[k])/(abs(normal[k])+abs(normal[k+1]))
        q = (1-alpha)*local[k, :2]+alpha*local[k+1, :2]
        if w.aperture.contains(q):
            crossings.append(dict(time_s=float(times[k]+alpha*(times[k+1]-times[k])),
                point_local_m=q.tolist(), direction='forward' if normal[k] > 0 else 'backward'))
    valid = len(crossings) == 1 and crossings[0]['direction'] == 'forward'
    exited = valid and np.any((times > crossings[0]['time_s']) & (normal < -.47094472))
    return dict(passed=bool(raw['completed'] and valid and exited and not raw['collision_detected']),
        collision_detected=raw['collision_detected'],
        world_sha256=hashlib.sha256((path/'world.sdf').read_bytes()).hexdigest(),
        mpc_config=raw['mpc_config'], px4_parameters=raw['parameters'], crossings=crossings)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--outdir', type=Path, required=True)
    args = parser.parse_args(); out = args.outdir.resolve()
    root = Path(__file__).resolve().parents[2]
    base = root/'convex_timevarying_window/togt_point_only_mpc/results'
    paths = [base/'no_inset_probe_20261009/f3_a4',
             base/'contact_sweep_s4_no_inset_v2_20261009/single_joint_center_f3_a4_p000',
             out/'passage_ours_f3_a4_p000']
    track, _ = build_case(Case(3., 4., 0.)); rows = []
    for p in paths:
        raw = json.loads((p/'result.json').read_text())
        audit = read_audit(p, raw, track)
        accepted = [s for s in raw['solves'] if s['accepted']]
        seed = accepted[0]
        plan = Plan(seed['made_at'], 0, np.array(seed['durations']),
                    np.array(seed['coefficients']), np.array(seed['variables']), 0., 0., 'audit', 1)
        verdict = passage(plan, track.windows[0], .38594472+.085+.10)
        time = audit['crossings'][0]['time_s']
        rows.append(dict(path=str(p), physical_passed=audit['passed'],
            collision_detected=audit['collision_detected'], flight_time_s=raw['flight_time_s'],
            world_sha256=audit['world_sha256'], mpc_config=audit['mpc_config'],
            px4_parameters=audit['px4_parameters'], nominal_proxy_verdict=verdict,
            crossing_time_s=time, crossing_point_local_m=audit['crossings'][0]['point_local_m'],
            gate_speed_at_crossing_mps=float(2*np.pi*3*4*np.cos(2*np.pi*3*time)),
            requests=len(raw['solves']),
            max_planning_end_to_end_ms=max(s['cycle_ms'] for s in raw['solves']),
            max_mpc_cycle_ms=max(s['cycle_ms'] for s in raw['mpc_cycles'])))
    for key in ('world_sha256', 'mpc_config', 'px4_parameters'):
        assert all(r[key] == rows[0][key] for r in rows)
    manifest = json.loads((out/'protocol.json').read_text())['source_sha256']
    assert all(hashlib.sha256((out/'source_snapshot'/rel).read_bytes()).hexdigest() == value
               for rel, value in manifest.items())
    assert all(hashlib.sha256((root/rel).read_bytes()).hexdigest() == value
               for rel, value in manifest.items())
    previous = root/'convex_timevarying_window/dynamic_passage_minco/results'
    v4 = json.loads((previous/'development_probe_v4_20261009/results.json').read_text())[3]
    v5 = json.loads((previous/'development_probe_v5_20261009/results.json').read_text())[3]
    v6 = json.loads((previous/'development_probe_v6_20261009/results.json').read_text())[3]
    evidence = dict(rows=rows, historical_flights_not_reflown=True,
        same_world_mpc_px4=True, frozen_sources_and_snapshot_verified=True,
        physical_collision_rule_unchanged=True, safety_radius_and_margin_unchanged=True,
        v4_first_seed_rounds=v4['optimizer_stats']['attempts'][0]['rounds'],
        v5_last_round=v5['optimizer_stats']['attempts'][-1]['rounds'][-1],
        v6_solve_ms=v6['solve_ms'], v6_rounds=v6['optimizer_stats']['attempts'][0]['rounds'],
        interpretation='Existing physical feasible flights are preserved. Their conservative proxy rejection '
                       'is not a physical infeasibility proof. v6 obtains a proxy-feasible and physical '
                       'contact-free new trajectory, without using historical solutions as seeds.')
    (out/'continuation_audit.json').write_text(json.dumps(evidence, indent=2, ensure_ascii=False)+'\n')
    print(json.dumps([{k: r[k] for k in ('path', 'physical_passed', 'collision_detected', 'flight_time_s',
               'gate_speed_at_crossing_mps', 'max_planning_end_to_end_ms', 'max_mpc_cycle_ms')} for r in rows], indent=2))


if __name__ == '__main__': main()
