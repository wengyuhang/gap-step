"""Freeze and measure a full-course nominal solve; never starts Gazebo."""
import time
PROCESS_START = time.perf_counter()
import argparse
from dataclasses import asdict
import json
from pathlib import Path
import shutil
import numpy as np
from convex_timevarying_window.single_gate_stop_racing.scenario import Case, build_case, manifest
from convex_timevarying_window.single_gate_stop_racing.run import sources
from .planner import Planner


def save(path, value):
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False)+'\n')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--frequency', type=float, default=4.)
    parser.add_argument('--amplitude', type=float, default=4.)
    parser.add_argument('--seconds', type=float, default=180.)
    parser.add_argument('--planning-thrust-fraction', type=float, default=1.,
                        help='Planning single-rotor upper bound / physical upper bound; tracker is unchanged')
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument('--position-only', action='store_true', help='C6 position-only MINCO; no independent interior velocity')
    modes.add_argument('--full-state', action='store_true', help='C3 shared free internal PVAJ; endpoint PVAJ unchanged')
    parser.add_argument('--seed-speed', type=float, default=6.5,
                        help='Time initialization only; default preserves historical initialization')
    parser.add_argument('--outdir', type=Path, required=True)
    args = parser.parse_args()
    out = args.outdir.resolve()
    out.mkdir(parents=True, exist_ok=False)
    root = Path.cwd()
    frozen = sources(root)
    case = Case(args.frequency, args.amplitude, 0.)
    track, _ = build_case(case)
    save(out/'protocol.json', dict(case=asdict(case), source_sha256=frozen,
        scene=manifest(case), nominal_start_pvaj=np.r_[track.start, np.zeros(9)].tolist(),
        endpoint_pvaj=np.r_[track.goal, np.zeros(9)].tolist(),
        phase_clock_fixed_at_zero=True, moving_start_time_during_solver=False,
        all_seven_joint=True, intermediate_stop_points=False, controller_enabled=False,
        independent_gate_velocity=not args.position_only, arrival_phase_boxes=False,
        independent_gate_acceleration_jerk=args.full_state,
        continuity_order=3 if args.full_state else (6 if args.position_only else 5),
        initial_curve_rule='exact PV seed embedding' if args.full_state else 'original seeds',
        seed_speed_override=args.seed_speed, seed_speed_rule='distance divided by seed speed',
        planning_thrust_fraction=args.planning_thrust_fraction,
        tracking_actuator_limits_unchanged=True,
        objective='total nominal flight time', global_solver_budget_s=args.seconds,
        qualified='optimizer converged + original nodal actuator/workspace limits + local enclosure of all frames',
        dynamics_continuously_certified=False, closed_loop_safe=False,
        compilation='CasADi VM analytic chain rule; no native C compilation'))
    for rel in frozen:
        target = out/'source_snapshot'/rel
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(root/rel, target)
    began = time.perf_counter()
    print('BUILD_START', flush=True)
    planner = Planner(track, position_only=args.position_only, full_state=args.full_state, seed_speed=args.seed_speed,
                      planning_thrust_fraction=args.planning_thrust_fraction)
    build_s = time.perf_counter()-began
    save(out/'build.json', dict(build_s=build_s, variables=planner.nvar, pieces=planner.n,
        point_variables=planner.dims, config=asdict(planner.cfg), seed_speed=planner.seed_speed))
    print('BUILD_DONE', build_s, 'variables', planner.nvar, flush=True)
    with (out/'progress.jsonl').open('w') as stream:
        def progress(record):
            stream.write(json.dumps(record, ensure_ascii=False)+'\n')
            stream.flush()
            print('CANDIDATE', json.dumps(record, ensure_ascii=False), flush=True)
        plan, result = planner.solve(args.seconds, progress)
    result.update(build_s=build_s, build_plus_solve_s=build_s+result['solve_s'],
        process_to_result_s=time.perf_counter()-PROCESS_START, controller_enabled=False,
        native_compile_s=0., scene=manifest(case))
    if plan is not None:
        result.update(nominal_flight_time_s=float(plan.durations.sum()),
            durations_s=plan.durations.tolist(), crossing_times_s=plan.crossing_times.tolist(),
            variables=plan.x.tolist(), coefficients=plan.coefficients.tolist(),
            crossing_points=np.asarray(planner.point_function(plan.x)).T.tolist())
        np.savez_compressed(out/'trajectory.npz', durations=plan.durations, coefficients=plan.coefficients,
                            variables=plan.x, made_at=0.)
    save(out/'result.json', result)
    unchanged = sources(root) == frozen
    save(out/'verification.json', dict(sources_unchanged=unchanged))
    print('RESULT', json.dumps({k: result.get(k) for k in ('found', 'build_s', 'solve_s',
        'build_plus_solve_s', 'nominal_flight_time_s', 'first_nodes_s', 'first_local_checks_s',
        'best_node_violation', 'failure')}), flush=True)
    if not unchanged:
        raise RuntimeError('Sources changed during experiment')


if __name__ == '__main__':
    main()
