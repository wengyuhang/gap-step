"""Periodic-opportunity descent, keeping the same P/V MINCO NLP and checks."""
import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import shutil
import time
import numpy as np
from .planner import Planner
from .run import save
from .audit import audit
from convex_timevarying_window.dynamic_passage_velocity_minco.trajectory import Plan
from convex_timevarying_window.single_gate_stop_racing.scenario import Case, build_case, manifest
from convex_timevarying_window.single_gate_stop_racing.run import sources


def load_plan(result):
    return Plan(0., 0, np.array(result['durations_s']), np.array(result['coefficients']),
                np.array(result['variables']), result['solve_s']*1000, 0., 'archived_global', 7)


def earlier_period_seed(x, track):
    """Advance gate i by i periods. No constraint or window motion is changed."""
    n = len(track.windows)+1
    T = np.exp(x[:n])
    arrivals = np.cumsum(T)[:-1]
    periods = np.array([w.motion.translation_period for w in track.windows])
    proposed = arrivals-np.arange(1, n)*periods
    new_T = np.r_[np.diff(np.r_[0., proposed]), T[-1]]
    if not np.isfinite(new_T).all() or np.min(new_T) < .3:
        return None
    for w, old, new in zip(track.windows, arrivals, proposed):
        # Only use this neighborhood where the actual queried pose repeats.
        for dt in (-.02, 0., .02):
            before, after = w.state_at(old+dt), w.state_at(new+dt)
            if not all(np.allclose(a, b, atol=1e-9, rtol=1e-9)
                       for a, b in zip(before, after)):
                raise ValueError('Periodic-opportunity descent requires repeating window pose')
    seed = x.copy()
    seed[:n] = np.log(new_T)
    return seed


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--baseline', required=True, type=Path)
    parser.add_argument('--outdir', required=True, type=Path)
    parser.add_argument('--seconds', type=float, default=180.)
    parser.add_argument('--steps', type=int, default=12)
    parser.add_argument('--distance-weight', type=float, default=0.)
    args = parser.parse_args()
    out = args.outdir.resolve(); out.mkdir(parents=True, exist_ok=False)
    baseline = args.baseline.resolve()
    previous = json.loads((baseline/'result.json').read_text())
    if not previous['found']:
        raise ValueError('No accepted baseline')
    old_protocol = json.loads((baseline/'protocol.json').read_text())
    case = Case(**old_protocol['case']); track, _ = build_case(case)
    root = Path.cwd(); frozen = sources(root)
    for rel in frozen:
        dst = out/'source_snapshot'/rel; dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(root/rel, dst)
    save(out/'protocol.json', dict(case=asdict(case), scene=manifest(case),
        baseline=str(baseline), baseline_sha256=hashlib.sha256((baseline/'result.json').read_bytes()).hexdigest(),
        independent_gate_velocity=True, arrival_phase_boxes=False,
        objective='T + distance_weight*L/12mps', distance_weight=args.distance_weight,
        change='earlier periodic opportunity warm starts; optional geometric objective',
        controller_enabled=False, seconds_per_attempt=args.seconds, max_steps=args.steps,
        source_sha256=frozen))
    began = time.perf_counter(); planner = Planner(track); build_s = time.perf_counter()-began
    save(out/'build.json', dict(build_s=build_s, config=asdict(planner.cfg), variables=planner.nvar))
    current = load_plan(previous); history = []
    checks, ok = planner.verify(current)
    g = planner.evaluate(current.x, planner.witnesses())[0]
    if not ok or g.min() < -1e-5:
        raise ValueError('Archived baseline does not pass current unchanged checks')
    with (out/'progress.jsonl').open('w') as stream:
        for step in range(args.steps):
            seed = earlier_period_seed(current.x, track)
            if seed is None: break
            print('STEP', step, 'incumbent_s', current.durations.sum(), 'seed_s', np.exp(seed[:planner.n]).sum(), flush=True)
            def progress(row):
                item = dict(step=step, **row); stream.write(json.dumps(item)+'\n'); stream.flush()
                print('PROGRESS', json.dumps({k:v for k,v in item.items() if k!='checks'}), flush=True)
            candidate, result = planner.solve(args.seconds, progress, initial_seeds=[seed],
                                             distance_weight=args.distance_weight)
            accepted = bool(candidate is not None and candidate.durations.sum() < current.durations.sum()-1e-4)
            history.append(dict(step=step, accepted=accepted, incumbent_s=float(current.durations.sum()), **result))
            save(out/f'attempt_{step:02d}.json', history[-1])
            if not accepted: break
            current = candidate
            np.savez_compressed(out/f'trajectory_{step:02d}.npz', durations=current.durations,
                                coefficients=current.coefficients, variables=current.x, made_at=0.)
    elapsed = time.perf_counter()-began
    result = dict(found=True, improved=any(r['accepted'] for r in history),
        nominal_flight_time_s=float(current.durations.sum()), solve_s=sum(r['solve_s'] for r in history),
        total_wall_s=elapsed, build_s=build_s, baseline_solve_s=previous['solve_s'],
        baseline_nominal_time_s=previous['nominal_flight_time_s'], controller_enabled=False,
        durations_s=current.durations.tolist(), crossing_times_s=current.crossing_times.tolist(),
        coefficients=current.coefficients.tolist(), variables=current.x.tolist(),
        crossing_points=np.asarray(planner.point_function(current.x)).T.tolist(),
        scene=manifest(case), history=history)
    save(out/'result.json', result)
    np.savez_compressed(out/'trajectory.npz', durations=current.durations,
                        coefficients=current.coefficients, variables=current.x, made_at=0.)
    save(out/'verification.json', dict(sources_unchanged=sources(root)==frozen))
    audited = audit(out)
    print('RESULT', result['improved'], result['nominal_flight_time_s'], result['solve_s'],
          'ordered_once', audited['ordered_single_forward_crossings'], flush=True)


if __name__ == '__main__': main()
