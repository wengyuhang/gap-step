"""Export value-only frozen SLSQP constraints for the C++ initializer."""
import argparse
import hashlib
import importlib
import json
from pathlib import Path
import sys
import time

import casadi as ca
import numpy as np

PACKAGE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PACKAGE / 'code'))
base = importlib.import_module('convex_timevarying_window.comparisons.论文实验.实验2')
base.__path__.append(str(PACKAGE / 'adapters'))
from convex_timevarying_window.comparisons.论文实验.实验2.higher_swing import scenario
from convex_timevarying_window.comparisons.论文实验.实验2.independent_free_solve.planner import Planner


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--cases', type=Path, required=True)
    parser.add_argument('--level', required=True)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=False)
    begin = time.perf_counter()
    track, _ = scenario.build_case(scenario.Case(level=args.level))
    planner = Planner(track, True, pose_function=scenario.pose)
    manifest = json.loads((args.cases / 'inputs' / f'{args.level}_ours' / 'manifest.json').read_text())
    n, m = planner.n, planner.m
    c = ca.MX.sym('coefficients', 24*n)
    times = ca.MX.sym('durations', n)
    starts = ca.vertcat(0, ca.cumsum(times)[:n-1])
    rows = []
    for piece in range(n):
        rows.append(planner.physical(ca.vertcat(c[24*piece:24*(piece+1)], times[piece]))[0])
    physical_rows = sum(x.numel() for x in rows)
    for (gate, piece), witness in sorted(planner.witnesses().items()):
        offsets = np.resize(witness['values'], planner.cfg.witness_slots)
        offset = ca.DM(offsets)
        u = ca.fmin(offset, times[piece])/times[piece]
        if witness['kind'] == 'before':
            u = 1-u
        rows.append(planner.safety[gate](ca.vertcat(c[24*piece:24*(piece+1)], times[piece], starts[piece]), u)[0])
    for gate, window in enumerate(track.windows):
        block = ca.reshape(c[24*gate:24*(gate+1)], 8, 3)
        tangent = block.T @ ca.DM([0, 1, 2, 3, 4, 5, 6, 7])/times[gate]
        normal = -planner.signs[gate]*window.R[:, 2]
        rows.append(ca.dot(ca.DM(normal), tangent)-.1)
    model = ca.Function('frozen_initial_constraints', [c, times], [ca.vertcat(*rows)]).expand()
    model.save(str(args.out / 'constraints.casadi'))
    validation = []
    previous = PACKAGE/'outputs/original_eight_ten_v1_20261112/run/groups'/args.level/'ours'
    for seed in range(8):
        initial = np.array(json.loads((previous/f'seed_{seed}'/'INITIAL_VALIDATION.json').read_text())['initial_variables'])
        expected, _, coefficients = planner.evaluate(initial, planner.witnesses())
        actual = np.array(model(coefficients.transpose(0, 2, 1).ravel(), np.exp(initial[:n]))).ravel()
        error = float(np.max(abs(expected-actual)))
        np.testing.assert_allclose(actual, expected, rtol=1e-10, atol=1e-9)
        validation.append(dict(seed=seed, maximum_residual_error=error))
    metadata = dict(level=args.level, n=n, m=m, start=track.start.tolist(), goal=track.goal.tolist(),
                    windows=manifest['windows'], original_initials=manifest['initials'],
                    velocity_start=manifest['velocity_start'], physical_rows=physical_rows,
                    constraint_rows=int(model.numel_out(0)), lower=np.exp(planner.lower[:n]).tolist(),
                    upper=np.exp(planner.upper[:n]).tolist(), tolerance=1e-5,
                    physical_thrust_fraction=planner.cfg.physical_thrust_fraction,
                    formal_initial_safety=False, export_wall_s=time.perf_counter()-begin,
                    residual_validation=validation, frozen_source=str(PACKAGE/'code'))
    (args.out/'model.json').write_text(json.dumps(metadata, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(dict(level=args.level, export_s=metadata['export_wall_s'], residual_rows=metadata['constraint_rows'],
                          validation_max=max(r['maximum_residual_error'] for r in validation))))


if __name__ == '__main__':
    main()
