"""Paired single-initialization ablation; never reads a stored right solution."""
import argparse
from dataclasses import asdict
from datetime import datetime
from zoneinfo import ZoneInfo
import hashlib
import json
from pathlib import Path
import time

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from shapely.geometry import Polygon

from convex_timevarying_window.single_gate_stop_racing.planner import Config
from nonconvex_timevarying_window.sc_dynatogt.sc_mapping import SCDiskMap
from ..analyze import audit_plan
from ..run import save_json
from ..scenario import Case, VERTICES, LEFT_INITIAL_POINT, build_case, room
from .solver import Planner


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--outdir', type=Path, required=True)
    ap.add_argument('--jit', action='store_true')
    args = ap.parse_args()
    if args.outdir.exists():
        raise FileExistsError('Preserve prior results; choose another directory')
    args.outdir.mkdir(parents=True)
    root = Path(__file__).resolve().parents[4]
    source_paths = list(Path(__file__).parent.glob('*.py')) + list(Path(__file__).parent.parent.glob('*.py'))
    source_paths += [root / path for path in (
        'convex_timevarying_window/single_gate_stop_racing/planner.py',
        'convex_timevarying_window/single_gate_stop_racing/geometry.py',
        'convex_timevarying_window/single_gate_stop_racing/local_bounds.py',
        'convex_timevarying_window/dynamic_passage_velocity_minco/hard_dynamics.py',
        'convex_timevarying_window/dynamic_passage_velocity_minco/trajectory.py',
        'nonconvex_timevarying_window/sc_dynatogt/sc_mapping.py')]
    hashes = {str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest() for p in source_paths}
    cfg, case = Config(compile_objective=args.jit, max_solve_ms=5000.), Case(3., 2.)
    began = time.perf_counter()
    mapping = SCDiskMap.fit(VERTICES)
    fit_ms = 1000*(time.perf_counter()-began)
    mapping.save(args.outdir/'sc_map.npz')
    save_json(args.outdir/'manifest.json', dict(
        created_at=datetime.now(ZoneInfo('America/Indiana/Winamac')).isoformat(),
        case=asdict(case), config=asdict(cfg), source_sha256=hashes, offline_sc_fit_ms=fit_ms,
        protocol='One identical left seed. Only the nonmonotone disk poll is toggled.',
        acceptance='Original finite constraints (1e-5 tolerance) plus local geometry; no complete dynamics certificate',
        jit=cfg.compile_objective, flight_executed=False))
    results = []
    for name, escape in [('single_local', False), ('single_poll', True)]:
        folder = args.outdir/name
        folder.mkdir()
        track = build_case(case)
        began = time.perf_counter()
        p = Planner(track, cfg, mapping)
        build_ms = 1000*(time.perf_counter()-began)
        initial_q = mapping.map_unconstrained(p.initial_d)
        np.testing.assert_allclose(initial_q, LEFT_INITIAL_POINT, atol=1e-7)
        result = dict(method=name, accepted=False, case=asdict(case),
                      initial_q=initial_q.tolist(), offline_build_ms=build_ms, flight_executed=False)
        try:
            plan = p.solve(np.r_[track.start, np.zeros(9)], escape=escape)
            q = mapping.map_unconstrained(plan.x[2:4])
            result.update(accepted=True, final=dict(q=q.tolist(), room=room(q),
                          nominal_total_time_s=float(plan.durations.sum()),
                          crossing_time_s=float(plan.crossing_times[0])))
            np.savez(folder/'plan.npz', coefficients=plan.coefficients, durations=plan.durations, x=plan.x)
            result['posthoc_audit'] = audit_plan(folder, case, cfg.radius+cfg.frame_thickness+cfg.passage_margin)
        except RuntimeError as exc:
            result['failure'] = str(exc)
        result['stats'] = p.solvers[0][0].stats()
        result['evaluations'] = p.evaluations
        result['points_outside_physical_polygon'] = sum(
            not track.windows[0].aperture.contains(v['q'], tolerance=1e-7) for v in p.evaluations)
        save_json(folder/'result.json', result)
        results.append(result)
        save_json(args.outdir/'summary.json', results)
        print(json.dumps(dict(method=name, accepted=result['accepted'], final=result.get('final'),
                              solve_ms=result['stats']['optimization_ms'], hops=result['stats']['hop_count']),
                         ensure_ascii=False), flush=True)
    np.testing.assert_array_equal(results[0]['stats']['initial_variables'], results[1]['stats']['initial_variables'])
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.8), sharex=True, sharey=True)
    outline = np.vstack([VERTICES, VERTICES[0]])
    erosion = Polygon(VERTICES).buffer(-(cfg.radius+cfg.frame_thickness+cfg.passage_margin))
    for ax, result in zip(axes, results):
        ax.fill(*outline.T, color='#eeeeee')
        ax.plot(*outline.T, color='black', lw=1.4)
        for part in erosion.geoms:
            ax.fill(*np.asarray(part.exterior.coords).T, color='#b5d9bd')
        q = np.array([v['q'] for v in result['stats']['trace']])
        ax.plot(*q.T, '.-', color='#bc6c25', lw=.7, ms=3, label='Optimizer iterates, not flight')
        ax.scatter(*LEFT_INITIAL_POINT, marker='*', s=160, color='#2166ac', zorder=5, label='Only initial point')
        if result['accepted']:
            ax.scatter(*result['final']['q'], s=65, color='#187337', zorder=6, label='Accepted nominal knot')
        ax.set(title=f"{result['method']}: {'accepted' if result['accepted'] else 'failed'}",
               xlabel='Gate-local horizontal coordinate (m)', ylabel='Gate-local vertical coordinate (m)')
        ax.set_aspect('equal')
        ax.grid(alpha=.2)
        ax.legend(fontsize=8, loc='lower right')
    fig.suptitle('3 Hz / 2 m; identical single left seed and original hard constraints')
    fig.tight_layout()
    fig.savefig(args.outdir/'single_initial_comparison.png', dpi=180)
    plt.close(fig)
    changed = [name for name, value in hashes.items()
               if hashlib.sha256((root/name).read_bytes()).hexdigest() != value]
    if changed:
        raise RuntimeError('Sources changed during experiment: '+str(changed))
    print('FINISHED '+str(args.outdir), flush=True)


if __name__ == '__main__':
    main()
