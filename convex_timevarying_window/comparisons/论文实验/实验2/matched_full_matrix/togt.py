"""Original dynamic TOGT objective with the current shared crossing-point inset."""
import copy
from dataclasses import asdict
import json
import time

import numpy as np

from convex_timevarying_window.single_gate_stop_racing import point_mapping
from nonconvex_timevarying_window.sc_dynatogt.optimizer import _minimize_togt_lbfgs
from nonconvex_timevarying_window.sc_dynatogt.time_mapping import k_from_durations
from ..static_calibration.planner import Objective
from ..soft_dynamics import run
from ..soft_dynamics.best_center_release import make_planner
from ..independent_repair.run import attitude_audit, frozen_plan


class MatchedObjective(Objective):
    def __init__(self, track, center, parameters):
        self.center_parameters = parameters
        super().__init__(track, center=center, dynamic_limit_fraction=.95)

    def split(self, x):
        times, points = super().split(x)
        if self.center:
            return times, self.center_parameters
        return times, points


def matched_objective(track, center, initializer):
    """Change only the mapping's safe domain on a private copy of the track."""
    aligned = copy.deepcopy(track)
    for window, inset in zip(aligned.windows, initializer.point_apertures):
        aperture = window.aperture
        if aperture.kind == 'circle':
            aperture.margin = 2.*(aperture.radius-inset.radius)
        else:
            aperture.safe_vertices = inset.physical_vertices.copy()
    parameters = tuple(point_mapping.inverse(aperture, np.zeros(2))
        for aperture in initializer.point_apertures)
    return MatchedObjective(aligned, center, parameters)


def initial_vector(objective, initializer, index):
    original = initializer.seeds()[index]
    times = np.exp(original[:initializer.n])
    if objective.center:
        return k_from_durations(times)
    return np.r_[k_from_durations(times), original[initializer.n:initializer.velocity_start]]


def worker(out, job_id):
    job = run.read(out/'jobs'/f'{job_id}.json')
    folder = out/'plans'/job_id
    folder.mkdir(parents=True, exist_ok=False)
    began = time.perf_counter()
    track, _ = run.scenario.build_case(run.scenario.Case(level=job['level']))
    initializer = make_planner(track, True)
    objective = matched_objective(track, job['method'].endswith('_center'), initializer)
    initial = initial_vector(objective, initializer, job['seed'])
    forward = objective.forward(initial)
    normalized = forward.trajectory.coefficients*forward.durations[:, None, None]**np.arange(8)[None, :, None]
    run.save(folder/'initial.json', dict(job=job, variables=initial.tolist(),
        durations_s=forward.durations.tolist(), points=forward.waypoints.tolist(),
        coefficients=normalized.tolist(), point_variables=objective.dimension-objective.temporal_dimension,
        optimized_initial_solution_used=False, source='matched center-point phase-sampled MINCO cold curve'))
    build_s = time.perf_counter()-began
    began = time.perf_counter()
    opt = _minimize_togt_lbfgs(objective.scipy_value_and_gradient, initial, objective.config)
    cost, gradient = objective.value_and_gradient(opt.x)
    solve_s = time.perf_counter()-began
    found = bool(opt.success and np.isfinite(cost))
    candidate = dict(seed=job['seed'], initial_variables=initial.tolist(), variables=opt.x.tolist(),
        success=found, status=int(opt.status), message=str(opt.message), objective=float(cost),
        iterations=int(opt.nit), evaluations=int(opt.nfev), gradient_inf_norm=float(abs(gradient).max()),
        elapsed_s=solve_s)
    result = dict(found=found, case=dict(layout='togt_whole', level=job['level']),
        method=job['method'], job=job, build_s=build_s, solve_s=solve_s, candidates=[candidate],
        selected_seed=job['seed'] if found else None, all_seven_joint=True, replanning=False,
        intermediate_stops=False, dynamic_limit_fraction=.95, dynamics_model='togt',
        dynamic_limits=asdict(objective.config.dynamic_limits), config=asdict(initializer.cfg),
        fixed_quadrature_intervals=32, tilt_penalty_weight=0., number_initial_starts=1,
        point_variables=objective.dimension-objective.temporal_dimension,
        crossing_points_mapped_into_inset=True, inset_definition='same frame-centerline erosion as current method',
        optimized_initial_solution_used=False, external_solution_used=False, center_solution_used=False,
        initial_sampling='matched independent gate travel estimates and motion phases, Sobol seed 0',
        optimizer='original SciPy L-BFGS-B with TOGT past-32 stopping',
        safety_objective_added=False, safety_filter_added=False, made_at=0.)
    if found:
        forward = objective.forward(opt.x)
        coefficients = forward.trajectory.coefficients*forward.durations[:, None, None]**np.arange(8)[None, :, None]
        result.update(variables=opt.x.tolist(), durations=forward.durations.tolist(),
            coefficients=coefficients.tolist(), nominal_time_s=float(forward.durations.sum()),
            objective=float(cost), dynamic_soft_integral=float(cost-forward.durations.sum()),
            crossing_points=forward.waypoints.tolist(), crossing_times_s=forward.traversal_times.tolist(),
            safe_point_membership=[window.aperture.contains(q, safe=True)
                for window, q in zip(objective.track.windows, forward.local_points)])
        run.save(folder/'attitude_audit.json', attitude_audit(frozen_plan(result)))
    run.save(folder/'result.json', result)
    print('TOGT_RESULT', job_id, found, result.get('nominal_time_s'), flush=True)
