"""Separated right-seed feasibility control and residual audit, NOT migration."""
import argparse
from dataclasses import asdict
import hashlib
from pathlib import Path
import time
import json
import numpy as np

from convex_timevarying_window.single_gate_stop_racing.planner import Config
from nonconvex_timevarying_window.sc_dynatogt.sc_mapping import SCDiskMap
from .planner import Planner, inverse_along_corridor
from .scenario import Case, build_case, room
from .run import save_json


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--primary', type=Path, required=True)
    ap.add_argument('--outdir', type=Path, required=True)
    args = ap.parse_args()
    if args.outdir.exists():
        raise FileExistsError('Preserve previous results')
    args.outdir.mkdir(parents=True)
    primary = json.loads((args.primary / 'f3_a2/result.json').read_text())
    cfg = Config(max_solve_ms=5000., compile_objective=False)
    case = Case(**primary['case'])
    track = build_case(case)
    mapping = SCDiskMap.load(args.primary / 'sc_map.npz')
    p = Planner(track, cfg, mapping)
    state = np.r_[track.start, np.zeros(9)]
    offsets = np.tile(np.r_[cfg.passage_seed_offsets,
                           np.zeros(cfg.witness_slots-len(cfg.passage_seed_offsets))], 2)
    par = np.r_[0., state, offsets]
    audit = []
    geometry_count = 2 * cfg.witness_slots * len(track.windows[0].aperture.physical_vertices)
    for i, attempt in enumerate(primary['stats']['attempts']):
        for record in attempt['rounds']:
            raw = p.function(record['variables'], par)
            g = np.asarray(raw[2]).ravel()
            dynamic_count = len(g) - geometry_count - 1
            index = int(g.argmin())
            classification = ('dynamic_or_workspace' if index < dynamic_count else
                              'finite_frame_safety' if index < len(g)-1 else 'forward_crossing')
            audit.append(dict(seed=i, minimum_residual=float(g.min()),
                              minimum_dynamic_residual=float(g[:dynamic_count].min()),
                              minimum_frame_residual=float(g[dynamic_count:-1].min()),
                              worst_constraint=classification,
                              q=mapping.map_unconstrained(np.array(record['variables'])[2:4]).tolist()))
    # A different spatial seed is ONLY used for this explicitly labelled
    # control. It must never be counted as success of the left-seeded run.
    p.initial_d = inverse_along_corridor(mapping, np.array([1.6, 0.]))
    initial_q = mapping.map_unconstrained(p.initial_d)
    p.evaluations, p.map_ms = [], 0.
    result = dict(role='right_seed_feasibility_control_NOT_left_to_right_migration',
                  initial_q=initial_q, case=asdict(case), config=asdict(cfg),
                  accepted=False, flight_executed=False,
                  primary_left_candidate_residual_audit=audit)
    try:
        plan = p.solve(state)
        q = mapping.map_unconstrained(plan.x[2:4])
        result.update(accepted=True, final=dict(q=q, room=room(q),
                      crossing_time_s=float(plan.crossing_times[0]),
                      nominal_total_time_s=float(plan.durations.sum())))
        np.savez(args.outdir / 'plan.npz', coefficients=plan.coefficients,
                 durations=plan.durations, x=plan.x)
    except RuntimeError as exc:
        result['failure'] = str(exc)
    result['stats'] = p.solvers[0][0].stats()
    result['evaluations'] = p.evaluations
    source_paths = list(Path(__file__).parent.glob('*.py'))
    result['source_sha256'] = {str(path): hashlib.sha256(path.read_bytes()).hexdigest()
                               for path in source_paths}
    save_json(args.outdir / 'result.json', result)
    print(json.dumps(dict(accepted=result['accepted'], final=
                          {k: v.tolist() if isinstance(v, np.ndarray) else v
                           for k, v in result.get('final', {}).items()},
                          audit=audit, solve_ms=result['stats']['optimization_ms']),
                     ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
