"""Length-guided initialization followed by the original time-only NLP."""
import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import shutil
import time
import numpy as np
from .planner import Planner
from .phase_descent import load_plan
from .run import save
from .audit import audit
from convex_timevarying_window.single_gate_stop_racing.scenario import Case, build_case, manifest
from convex_timevarying_window.single_gate_stop_racing.run import sources


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--baseline', required=True, type=Path)
    parser.add_argument('--outdir', required=True, type=Path)
    parser.add_argument('--seconds', type=float, default=180.)
    args = parser.parse_args(); out = args.outdir.resolve(); out.mkdir(parents=True, exist_ok=False)
    src = args.baseline.resolve(); old = json.loads((src/'result.json').read_text())
    if not old['found']: raise ValueError('Require accepted initial trajectory')
    case = Case(**json.loads((src/'protocol.json').read_text())['case']); track, _ = build_case(case)
    root = Path(__file__).resolve().parents[2]; frozen = sources(root)
    for rel in frozen:
        dst = out/'source_snapshot'/rel; dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(root/rel, dst)
    save(out/'protocol.json', dict(case=asdict(case), scene=manifest(case), baseline=str(src),
        baseline_sha256=hashlib.sha256((src/'result.json').read_bytes()).hexdigest(),
        weights=[1., 0.], initialization_objective='T + L/12mps', final_objective='T',
        independent_gate_velocity=True, arrival_phase_boxes=False, controller_enabled=False,
        seconds_per_stage=args.seconds, source_sha256=frozen))
    began = time.perf_counter(); planner = Planner(track); build_s = time.perf_counter()-began
    save(out/'build.json', dict(build_s=build_s, config=asdict(planner.cfg), variables=planner.nvar))
    current = load_plan(old); history = []; polished = False
    with (out/'progress.jsonl').open('w') as stream:
        for step, weight in enumerate((1., 0.)):
            def progress(row):
                item = dict(step=step, distance_weight=weight, **row)
                stream.write(json.dumps(item)+'\n'); stream.flush()
                print('PROGRESS', json.dumps({k:v for k,v in item.items() if k!='checks'}), flush=True)
            candidate, result = planner.solve(args.seconds, progress, initial_seeds=[current.x], distance_weight=weight)
            history.append(dict(step=step, distance_weight=weight, **result))
            save(out/f'attempt_{step:02d}.json', history[-1])
            if candidate is None: break
            current = candidate; polished = weight == 0.
            np.savez_compressed(out/f'trajectory_{step:02d}.npz', durations=current.durations,
                coefficients=current.coefficients, variables=current.x, made_at=0.)
            print('ACCEPTED', step, 'time', float(current.durations.sum()), 'length', float(planner.arc(current.x)[0]), flush=True)
    result = dict(found=True, improved=bool(current.durations.sum()<sum(old['durations_s'])),
        time_only_polished=polished, nominal_flight_time_s=float(current.durations.sum()),
        solve_s=sum(r['solve_s'] for r in history), total_wall_s=time.perf_counter()-began,
        build_s=build_s, baseline_solve_s=old['solve_s'], baseline_nominal_time_s=sum(old['durations_s']),
        controller_enabled=False, durations_s=current.durations.tolist(), coefficients=current.coefficients.tolist(),
        crossing_times_s=current.crossing_times.tolist(), variables=current.x.tolist(),
        crossing_points=np.asarray(planner.point_function(current.x)).T.tolist(), scene=manifest(case), history=history)
    save(out/'result.json', result)
    np.savez_compressed(out/'trajectory.npz', durations=current.durations, coefficients=current.coefficients, variables=current.x, made_at=0.)
    save(out/'verification.json', dict(sources_unchanged=sources(root)==frozen))
    audit(out)
    print('RESULT', result['improved'], polished, result['nominal_flight_time_s'], result['solve_s'], flush=True)


if __name__ == '__main__': main()
