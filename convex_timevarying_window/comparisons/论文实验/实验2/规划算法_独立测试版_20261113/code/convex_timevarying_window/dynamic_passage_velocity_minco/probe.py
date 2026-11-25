"""Read a recorded head; compare parameterizations without changing the world."""
import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import shutil
import time
import numpy as np
from convex_timevarying_window.comparisons.single_gate_translation.scenario import Case, build_case
from convex_timevarying_window.dynamic_passage_minco.safety import passage
from .planner import Config, Planner


def metrics(plan, track):
    crossing = float(plan.crossing_times[0])
    times = np.linspace(crossing, plan.end_time, 8001)
    positions = plan.sample_many(times)
    normal = track.goal-track.windows[0].center0
    normal /= np.linalg.norm(normal)
    normal_velocity = plan.sample_many(times, 1)@normal
    return dict(made_at=plan.made_at, durations=plan.durations.tolist(),
        variables=plan.x.tolist(), coefficients=plan.coefficients.tolist(),
        nominal_total_time_s=float(plan.durations.sum()), nominal_end_time_s=plan.end_time,
        nominal_crossing_time_s=crossing, crossing_velocity_mps=plan.sample(crossing, 1).tolist(),
        tail_path_length_m=float(np.linalg.norm(np.diff(positions, axis=0), axis=1).sum()),
        tail_normal_overshoot_m=max(0., float(np.max((positions-track.goal)@normal))),
        tail_min_normal_velocity_mps=float(normal_velocity.min()),
        endpoint_error_m=float(np.linalg.norm(positions[-1]-track.goal)),
        endpoint_velocity_mps=plan.sample(plan.end_time, 1).tolist(),
        optimizer_stats=plan.optimizer_stats, solve_ms=plan.solve_ms)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--head-result', type=Path, required=True)
    parser.add_argument('--outdir', type=Path, required=True)
    parser.add_argument('--no-jit', action='store_true')
    parser.add_argument('--max-solve-ms', type=float, default=90.)
    parser.add_argument('--center', action='store_true')
    parser.add_argument('--initial-tail-scale', type=float, default=3.2)
    parser.add_argument('--free-acceleration', action='store_true')
    parser.add_argument('--kinematic-arrival-seed', action='store_true')
    parser.add_argument('--forward-tail', action='store_true')
    parser.add_argument('--log-cost-scaling', action='store_true')
    args = parser.parse_args(); out = args.outdir.resolve(); out.mkdir(parents=True, exist_ok=False)
    raw = json.loads(args.head_result.read_text()); record = raw['solves'][0]
    track, _ = build_case(Case(**raw['case']))
    cfg = Config(free_points=not args.center, compile_objective=not args.no_jit,
                 max_solve_ms=args.max_solve_ms, initial_tail_scale=args.initial_tail_scale,
                 free_crossing_acceleration=args.free_acceleration,
                 kinematic_arrival_seed=args.kinematic_arrival_seed,
                 forward_tail=args.forward_tail, log_cost_scaling=args.log_cost_scaling)
    began = time.perf_counter(); planner = Planner(track, cfg)
    row = dict(head_result=str(args.head_result.resolve()), case=raw['case'], config=asdict(cfg),
        build_s=time.perf_counter()-began, head=record['request_head'], same_goal_stopping_condition=True)
    try:
        plan = planner.solve(np.array(record['request_head']['pvaj']), record['made_at'], 0)
        row.update(found=True, **metrics(plan, track))
        row['independent_local_passage'] = passage(plan, track.windows[0],
            cfg.radius+cfg.frame_thickness+cfg.passage_margin, cfg.frame_offset)
    except Exception as exc:
        row.update(found=False, error=repr(exc), optimizer_stats=planner.solvers[0][0].stats())
    root = Path(__file__).resolve().parents[2]
    sources = {}
    for p in Path(__file__).parent.glob('*.py'):
        target = out/'source_snapshot'/p.relative_to(root)
        target.parent.mkdir(parents=True, exist_ok=True); shutil.copyfile(p, target)
        sources[str(p.relative_to(root))] = hashlib.sha256(p.read_bytes()).hexdigest()
    row['source_sha256'] = sources
    (out/'result.json').write_text(json.dumps(row, indent=2)+'\n')
    print(json.dumps({k: row.get(k) for k in ['found','build_s','solve_ms','nominal_total_time_s',
        'durations','variables','crossing_velocity_mps','tail_path_length_m','tail_normal_overshoot_m',
        'tail_min_normal_velocity_mps','error']}, indent=2), flush=True)


if __name__ == '__main__': main()
