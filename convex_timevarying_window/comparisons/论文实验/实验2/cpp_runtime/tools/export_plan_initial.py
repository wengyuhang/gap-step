"""Use a solved seven-window trajectory as the sole C++ SIP initial guess."""
import argparse
import json
import math
from pathlib import Path
import shutil

import casadi as ca
import numpy as np

from convex_timevarying_window.single_gate_stop_racing import point_mapping
from convex_timevarying_window.comparisons.论文实验.实验2.higher_swing import scenario, run


def export(graph: Path, result: Path, output: Path) -> None:
    source = json.loads(result.read_text())
    if not source.get('found'):
        raise ValueError('The source plan has no solved trajectory')
    shutil.copytree(graph, output)
    manifest_path = output / 'manifest.json'
    manifest = json.loads(manifest_path.read_text())
    durations = np.asarray(source['durations'], dtype=float)
    coefficients = np.asarray(source['coefficients'], dtype=float)
    if durations.shape != (manifest['n'],) or coefficients.shape != (manifest['n'], 8, 3):
        raise ValueError('Source plan and graph have different trajectory structure')
    if not np.isfinite(coefficients).all() or np.any(durations <= 0.):
        raise ValueError('Invalid source trajectory')
    m = manifest['m']
    derivatives = []
    for order in (1, 2, 3):
        factor = np.array([math.factorial(k) / math.factorial(k-order) if k >= order else 0.
                           for k in range(8)])
        at_knots = (coefficients[:m] * factor[None, :, None]).sum(axis=1)
        derivatives.append(at_knots / durations[:m, None]**order)
    pvaj = np.concatenate([value.ravel() for value in derivatives])
    old = manifest['initials'][0]
    z = np.zeros(manifest['nvar'])
    if manifest['method'] == 'ours':
        track, _ = scenario.build_case(scenario.Case(level=manifest['level']))
        planner = run.make_planner(track, True)
        center_parameters = np.concatenate([
            point_mapping.inverse(aperture, np.zeros(2))
            for aperture in planner.point_apertures])
        z[manifest['n']:manifest['velocity_start']] = center_parameters
    parameters = np.asarray(old['parameters'], dtype=float)
    parameters[0] = durations.sum()
    parameters[1:1+manifest['n']] = np.cumsum(durations)
    scale = np.asarray(manifest['state_scale'])
    baseline = np.asarray(ca.Function.load(str(output/'baseline.casadi'))(z, parameters)).ravel()
    z[manifest['velocity_start']:] = (pvaj - baseline) / scale
    extended = np.r_[z, np.zeros(len(manifest['slack_scale']))]
    trajectory = ca.Function.load(str(output/'trajectory.casadi'))
    physical, rebuilt_times, rebuilt_coefficients, points, *_ = trajectory(extended, parameters)
    source_coefficients = coefficients.transpose(0, 2, 1).ravel()
    curve_error = float(np.max(np.abs(np.asarray(rebuilt_coefficients).ravel() - source_coefficients)))
    time_error = float(np.max(np.abs(np.asarray(rebuilt_times).ravel() - durations)))
    center_error = float(np.max(np.abs(np.asarray(points).T - np.asarray(source['crossing_points']))))
    if curve_error > 1e-6 or time_error > 1e-9 or center_error > 1e-8:
        raise ValueError(f'Trajectory was not reproduced: coeff={curve_error}, time={time_error}, point={center_error}')
    residual = np.asarray(ca.Function.load(str(output/'global_uniform_dynamics.casadi'))(
        rebuilt_coefficients, rebuilt_times)).ravel()
    manifest['initials'] = [dict(index=0, original=np.asarray(physical).ravel().tolist(),
                                 parameters=parameters.tolist(), finite=extended.tolist())]
    manifest['initialization'] = dict(source=str(result.resolve()),
        source_method=source['method'], source_nominal_time_s=source['nominal_time_s'],
        curve_max_coefficient_error=curve_error, time_max_error=time_error,
        point_max_error=center_error, number_of_starts=1)
    manifest_path.write_text(json.dumps(manifest, indent=2) + '\n')
    (output/'initial_diagnostic.json').write_text(json.dumps(dict(
        source=str(result.resolve()), source_method=source['method'],
        total_time_s=float(durations.sum()), curve_max_coefficient_error=curve_error,
        sampled_dynamic_positive_rows=int((residual > 0.).sum()),
        sampled_dynamic_max_residual=float(residual.max())), indent=2) + '\n')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('graph', type=Path)
    parser.add_argument('result', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    export(args.graph, args.result, args.output)
