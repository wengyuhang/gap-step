"""Solve one independent complete-course free-point or center problem."""
import argparse
from dataclasses import asdict
import json
from pathlib import Path
import time

import numpy as np

from .planner import Planner
from ..basic_group.whole_course_difficulty import scenario
from ..independent_repair.run import attitude_audit


def save(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2)+'\n')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--level', choices=scenario.LEVELS, required=True)
    parser.add_argument('--method', choices=('ours', 'center'), required=True)
    parser.add_argument('--solver', choices=('ipopt', 'ipopt_legacy', 'slsqp'), default='slsqp')
    parser.add_argument('--dynamics', choices=('togt', 'x500'), default='togt')
    parser.add_argument('--dynamic-limit-fraction', type=float, default=1.)
    parser.add_argument('--time-scales', type=float, nargs='+')
    parser.add_argument('--outdir', type=Path, required=True)
    args = parser.parse_args()
    out = args.outdir.resolve()
    out.mkdir(parents=True, exist_ok=False)
    track, _ = scenario.build_case(scenario.Case(level=args.level))
    save(out/'scene.json', scenario.manifest(scenario.Case(level=args.level)))
    began = time.perf_counter()
    planner = Planner(track, args.method == 'ours', pose_function=scenario.pose,
        solver=args.solver, dynamics_model=args.dynamics, dynamic_limit_fraction=args.dynamic_limit_fraction,
        temporal_scales=args.time_scales)
    build_s = time.perf_counter()-began
    save(out/'build.json', dict(build_s=build_s, variables=planner.nvar, config=asdict(planner.cfg)))
    with (out/'progress.jsonl').open('w') as log:
        def progress(row):
            log.write(json.dumps(row, ensure_ascii=False)+'\n')
            log.flush()
            if row.get('stage') == 'seed_finished':
                print('SEED', json.dumps(row, ensure_ascii=False), flush=True)
        plan, result = planner.solve_all(np.inf, progress)
    result.update(build_s=build_s, case=dict(layout='togt_whole', level=args.level),
        method=args.method, config=asdict(planner.cfg), wall_time_limit_s_per_seed=None)
    if plan is not None:
        objective, _, integral = planner.objective(plan.x)
        result.update(durations=plan.durations.tolist(), coefficients=plan.coefficients.tolist(),
            variables=plan.x.tolist(), made_at=0., nominal_time_s=float(plan.durations.sum()),
            dynamic_soft_integral=integral, objective=objective,
            crossing_times_s=plan.crossing_times.tolist(),
            crossing_points=np.asarray(planner.point_function(plan.x)).T.tolist())
        np.savez_compressed(out/'trajectory.npz', durations=plan.durations,
            coefficients=plan.coefficients, variables=plan.x)
        save(out/'attitude_audit.json', attitude_audit(plan))
    save(out/'result.json', result)
    print('PLAN_RESULT', args.level, args.method, result['found'], result.get('nominal_time_s'), flush=True)


if __name__ == '__main__':
    main()
