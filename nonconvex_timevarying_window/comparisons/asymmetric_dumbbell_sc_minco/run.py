"""Offline planning experiment, no Gazebo or flight process interaction."""
import argparse
from dataclasses import asdict, replace
from datetime import datetime
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
from .planner import Planner, recenter_at_left
from .scenario import Case, LEFT_INITIAL_POINT, VERTICES, build_case, room


def clean(obj):
    if isinstance(obj, dict):
        return {str(k): clean(v) for k, v in obj.items()}
    if isinstance(obj, (tuple, list)):
        return [clean(v) for v in obj]
    if isinstance(obj, np.ndarray):
        return clean(obj.tolist())
    if isinstance(obj, np.generic):
        return clean(obj.item())
    if isinstance(obj, float) and not np.isfinite(obj):
        return None
    return obj


def save_json(path, data):
    path.write_text(json.dumps(clean(data), ensure_ascii=False, indent=2, allow_nan=False) + '\n')


def draw(outdir, results, radius):
    polygon = Polygon(VERTICES)
    erosion = polygon.buffer(-radius)
    fig, axes = plt.subplots(1, len(results), figsize=(6 * len(results), 5), squeeze=False)
    for ax, result in zip(axes[0], results):
        outline = np.vstack([VERTICES, VERTICES[0]])
        ax.fill(outline[:, 0], outline[:, 1], color='#eeeeee')
        ax.plot(outline[:, 0], outline[:, 1], color='black', lw=2, label='Physical frame centreline')
        components = list(erosion.geoms) if hasattr(erosion, 'geoms') else [erosion]
        for i, part in enumerate(components):
            q = np.asarray(part.exterior.coords)
            ax.fill(q[:, 0], q[:, 1], color='#a9d6ab', alpha=.65,
                    label='Body + margin feasible at plane' if i == 0 else None)
        for seed in range(4):
            trace = [r for r in result['stats'].get('trace', []) if r['seed'] == seed]
            if not trace:
                continue
            points = np.array([r['q'] for r in trace])
            ax.plot(points[:, 0], points[:, 1], '.-', lw=.7, alpha=.65,
                    label=f'Local optimizer seed {seed + 1}')
        ax.scatter(*LEFT_INITIAL_POINT, marker='*', s=180, c='#2166ac', zorder=5, label='Every initial knot')
        final = result.get('final', result.get('last_candidate'))
        if final:
            q = np.array(final['q'])
            ax.scatter(*q, c='#b2182b', s=70, marker='o' if result['accepted'] else 'x',
                       zorder=6, label='Accepted' if result['accepted'] else 'Last iterate (rejected)')
        ax.axvspan(-.4, 0., color='#ffbc66', alpha=.15)
        c = result['case']
        ax.set(title=f"{c['frequency']:g} Hz / {c['amplitude']:g} m; "
                     f"budget {result['budget_ms']:g} ms\n"
                     f"{'ACCEPTED' if result['accepted'] else 'NO ACCEPTED PLAN'}",
               xlabel='Gate-local horizontal coordinate (m)', ylabel='Gate-local vertical coordinate (m)')
        ax.set_aspect('equal')
        ax.set_xlim(-1.85, 3.45)
        ax.set_ylim(-1.85, 1.85)
        ax.legend(fontsize=7, loc='lower right')
        ax.grid(alpha=.2)
    fig.tight_layout()
    fig.savefig(outdir / 'passage_points.png', dpi=180)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--outdir', type=Path, required=True)
    parser.add_argument('--frequency', type=float, default=3.)
    parser.add_argument('--amplitude', type=float, default=2.)
    parser.add_argument('--budget-ms', type=float, default=5000.)
    parser.add_argument('--no-static', action='store_true')
    parser.add_argument('--normalization', choices=('polylabel', 'left'), default='polylabel')
    parser.add_argument('--no-jit', action='store_true', help='Offline diagnosis only; timing not online-comparable')
    args = parser.parse_args()
    if args.outdir.exists():
        raise FileExistsError('Preserve previous results; use a new output directory')
    args.outdir.mkdir(parents=True)
    root = Path(__file__).resolve().parents[3]
    sources = list(Path(__file__).parent.glob('*.py')) + [
        root / 'convex_timevarying_window/single_gate_stop_racing/solver.py',
        root / 'convex_timevarying_window/single_gate_stop_racing/geometry.py',
        root / 'convex_timevarying_window/single_gate_stop_racing/local_bounds.py',
        root / 'convex_timevarying_window/dynamic_passage_velocity_minco/hard_dynamics.py',
        root / 'convex_timevarying_window/dynamic_passage_velocity_minco/trajectory.py',
        root / 'nonconvex_timevarying_window/sc_dynatogt/sc_mapping.py',
    ]
    hashes = {str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}
    cfg = Config(max_solve_ms=args.budget_ms, compile_objective=not args.no_jit)
    rho = cfg.radius + cfg.frame_thickness + cfg.passage_margin
    erased = Polygon(VERTICES).buffer(-rho)
    manifest = dict(created_at=datetime.now().isoformat(), config=asdict(cfg),
                    vertices=VERTICES, left_initial_point=LEFT_INITIAL_POINT,
                    plane='fixed y=0; horizontal x translation only',
                    start=[0., -6., 3.2], goal=[0., 6., 3.2],
                    body_radius_m=cfg.radius, frame_radius_m=cfg.frame_thickness,
                    safety_margin_m=cfg.passage_margin, check_radius_m=rho,
                    left_half_width_m=.6, left_spare_after_check_margin_m=.6-rho,
                    corridor_width_m=.4, safe_cross_section_components=len(erased.geoms),
                    note='Erosion is plotted/diagnosed only; SC maps the ORIGINAL polygon.',
                    flight_executed=False, source_sha256=hashes,
                    sc_normalization=args.normalization)
    save_json(args.outdir / 'manifest.json', manifest)
    began = time.perf_counter()
    mapping = SCDiskMap.fit(VERTICES)
    if args.normalization == 'left':
        mapping = recenter_at_left(mapping)
    fit_ms = 1000 * (time.perf_counter() - began)
    mapping.save(args.outdir / 'sc_map.npz')
    save_json(args.outdir / 'sc_diagnostics.json', dict(offline_fit_ms=fit_ms,
              **asdict(mapping.diagnostics)))
    print(json.dumps(dict(stage='SC_fit', offline_ms=fit_ms)), flush=True)
    cases = ([Case(args.frequency, 0.)] if not args.no_static else []) + [
        Case(args.frequency, args.amplitude)]
    results = []
    for case in cases:
        folder = args.outdir / case.name
        folder.mkdir()
        track = build_case(case)
        began = time.perf_counter()
        planner = Planner(track, cfg, mapping)
        build_ms = 1000 * (time.perf_counter() - began)
        initial_q = mapping.map_unconstrained(planner.initial_d)
        if room(initial_q) != 'left' or np.linalg.norm(initial_q - LEFT_INITIAL_POINT) > 1e-7:
            raise RuntimeError('The requested left-room initialization was not met')
        print(json.dumps(dict(stage='built', case=case.name, offline_build_ms=build_ms,
                              initial_q=initial_q.tolist())), flush=True)
        result = dict(case=asdict(case), budget_ms=args.budget_ms, accepted=False,
                      sc_normalization=args.normalization, jit=cfg.compile_objective,
                      offline_build_ms=build_ms, initial_q=initial_q.tolist(),
                      initial_d=planner.initial_d.tolist(),
                      sc_initial_jacobian_condition=float(np.linalg.cond(
                          mapping.jacobian_unconstrained(planner.initial_d))), flight_executed=False)
        try:
            plan = planner.solve(np.r_[track.start, np.zeros(9)])
            q = mapping.map_unconstrained(plan.x[2:4])
            at = float(plan.crossing_times[0])
            velocity = plan.sample(at, 1)
            result.update(accepted=True, final=dict(q=q.tolist(), room=room(q),
                          crossing_time_s=at, nominal_total_time_s=float(plan.durations.sum()),
                          world_point=plan.sample(at).tolist(), velocity_mps=velocity.tolist(),
                          window_velocity_mps=track.windows[0].state_at(at)[2].tolist(),
                          boundary_distance_at_plane_m=float(track.windows[0].aperture.boundary_distance(q))))
            np.savez(folder / 'plan.npz', coefficients=plan.coefficients, durations=plan.durations, x=plan.x)
        except RuntimeError as exc:
            result['failure'] = str(exc)
        result['stats'] = planner.solvers[0][0].stats()
        result['evaluations'] = planner.evaluations
        if result['stats'].get('trace'):
            last = result['stats']['trace'][-1]
            result['last_candidate'] = dict(q=last['q'], room=last['room'])
        result['evaluated_room_counts'] = {r: sum(room(v['q']) == r for v in planner.evaluations)
                                           for r in ('left', 'corridor', 'right')}
        result['evaluated_points_outside_physical_polygon'] = sum(
            not track.windows[0].aperture.contains(v['q'], tolerance=1e-7)
            for v in planner.evaluations)
        save_json(folder / 'result.json', result)
        results.append(result)
        save_json(args.outdir / 'summary.json', results)
        print(json.dumps(clean({k: result[k] for k in ('case', 'accepted', 'evaluated_room_counts')}
                               | dict(final=result.get('final'), last=result.get('last_candidate'),
                                      solve_ms=result['stats'].get('optimization_ms'))), ensure_ascii=False), flush=True)
    draw(args.outdir, results, rho)
    if any(hashlib.sha256((root / p).read_bytes()).hexdigest() != h for p, h in hashes.items()):
        raise RuntimeError('Source changed during experiment; do not treat as frozen evidence')
    print('FINISHED ' + str(args.outdir), flush=True)


if __name__ == '__main__':
    main()
