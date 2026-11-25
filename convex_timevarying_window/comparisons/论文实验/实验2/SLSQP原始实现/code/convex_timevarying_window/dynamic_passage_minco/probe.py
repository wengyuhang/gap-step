"""Deterministic fixed-input solver verification before physical flights."""
import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import shutil
import time
import numpy as np
from convex_timevarying_window.comparisons.single_gate_translation.scenario import Case, build_case
from convex_timevarying_window.comparisons.original_fixed_plane_ours.adapter import frame_clearances
from .planner import Config, Planner


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--outdir', type=Path, required=True)
    parser.add_argument('--max-solve-ms', type=float, default=90.)
    parser.add_argument('--no-jit', action='store_true')
    args = parser.parse_args(); out = args.outdir.resolve(); out.mkdir(parents=True, exist_ok=False)
    rows = []
    for frequency, amplitude in ((3.5, 2.), (3., 4.)):
        track, _ = build_case(Case(frequency, amplitude, 0.))
        for free in (False, True):
            cfg = Config(free_points=free, compile_objective=not args.no_jit,
                         max_solve_ms=args.max_solve_ms)
            began = time.perf_counter(); planner = Planner(track, cfg)
            row = dict(frequency=frequency, amplitude=amplitude, free_points=free,
                       config=asdict(cfg), build_s=time.perf_counter()-began)
            try:
                plan = planner.solve(np.r_[track.start, np.zeros(9)], .162, 0)
                # Independent dense OFFLINE diagnostic, never in online planning.
                times = np.arange(plan.made_at, plan.end_time, .0005)
                positions = plan.sample_many(times)
                from copy import copy
                frame = copy(track.windows[0]); aperture = copy(frame.aperture)
                vertices = aperture.physical_vertices
                object.__setattr__(aperture, 'physical_vertices',
                    np.sign(vertices)*(np.abs(vertices)+cfg.frame_offset))
                frame.aperture = aperture
                clearance = frame_clearances([frame], positions, times, cfg.radius, cfg.frame_thickness)
                row.update(found=True, solve_ms=plan.solve_ms, made_at=plan.made_at,
                           durations=plan.durations.tolist(), variables=plan.x.tolist(),
                           coefficients=plan.coefficients.tolist(),
                           dense_physical_proxy_clearance_m=float(clearance.min()),
                           optimizer_stats=plan.optimizer_stats)
            except Exception as exc:
                row.update(found=False, error=repr(exc), optimizer_stats=planner.solvers[0][0].stats())
            rows.append(row)
            (out/'results.json').write_text(json.dumps(rows, indent=2)+'\n')
            print(json.dumps({k: row.get(k) for k in ('frequency', 'amplitude', 'free_points', 'found',
                       'solve_ms', 'durations', 'variables', 'dense_physical_proxy_clearance_m', 'error')}), flush=True)
    root = Path(__file__).resolve().parents[2]
    hashes = {}
    for source in Path(__file__).parent.glob('*.py'):
        destination = out/'source_snapshot'/source.relative_to(root)
        destination.parent.mkdir(parents=True, exist_ok=True); shutil.copyfile(source, destination)
        hashes[str(source.relative_to(root))] = hashlib.sha256(source.read_bytes()).hexdigest()
    (out/'sources.json').write_text(json.dumps(hashes, indent=2)+'\n')


if __name__ == '__main__': main()
