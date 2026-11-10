"""Scoped single-gate adapters for the frozen four planning variants."""
import copy
from dataclasses import asdict
import json
from pathlib import Path
import time

import casadi as ca
import numpy as np
from convex_timevarying_window.dynamic_passage_velocity_minco.trajectory import Plan
from convex_timevarying_window.single_gate_stop_racing import point_mapping
from convex_timevarying_window.tilt_heading_minco import dynamics
from convex_timevarying_window.togt_minco_mpc.trajectory import basis
from convex_timevarying_window.togt.native_objective import NativeJointTOGTObjective
from nonconvex_timevarying_window.sc_dynatogt.optimizer import _minimize_togt_lbfgs
from nonconvex_timevarying_window.sc_dynatogt.time_mapping import k_from_durations
from ..实验2.soft_dynamics.phase_initialization import Planner as FrozenPlanner
from ..实验2.soft_dynamics.best_center_release import release_initial
from ..实验2.static_calibration.planner import Objective
from ..实验2.basic_group.trajectory_only.collision import evaluate, opening_events
from . import scenario

LABELS = dict(ours='本文自由点', instant='瞬时安全消融', center='本文中心点', togt_center='动态TOGT中心点')


def save(path, value):
    path = Path(path)
    temporary = path.with_suffix(path.suffix+'.writing')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2)+'\n')
    temporary.replace(path)


def read(path):
    return json.loads(Path(path).read_text())


def frozen_plan(raw):
    return Plan(0., 0, np.asarray(raw['durations']), np.asarray(raw['coefficients']),
        np.asarray(raw.get('variables', [])), 0., 0., 'frozen', 1)


class InstantPlanner(FrozenPlanner):
    """Same knot inset and model, with no off-plane frame safety exchange."""
    def verify(self, plan, deadline=np.inf):
        checks = []
        for piece in range(self.n):
            for u in np.linspace(0., 1., self.cfg.nodes_per_piece+1):
                velocity, acceleration = [plan.coefficients[piece].T@np.asarray(basis(float(u), d)).ravel()
                    /plan.durations[piece]**d for d in (1, 2)]
                health = dynamics.domain(velocity, acceleration)
                if not health['passed']:
                    return [dict(gate=-1, piece=piece, passed=False,
                        outcome='attitude_chart_unresolved', witness_u=None)], False
                if time.perf_counter() >= deadline:
                    return [], False
        events, ordered = opening_events(plan.coefficients, plan.durations, self.track)
        checks.append(dict(gate=0, piece=0, passed=bool(ordered),
            outcome='single_forward_crossing' if ordered else 'task_invalid',
            witness_u=None, events=events))
        return checks, bool(ordered)


def make_planner(track, method):
    cls = InstantPlanner if method == 'instant' else FrozenPlanner
    return cls(track, method != 'center', solver='ipopt', dynamics_model='togt',
        dynamic_limit_fraction=.95, extra_clearance_m=.05, initial_count=8,
        sampling_seed=0, pose_function=scenario.pose)


class CenterObjective(Objective):
    """The existing native TOGT objective, with a generic strict-center split."""
    def __init__(self, track, initializer):
        aligned = copy.deepcopy(track)
        for window, inset in zip(aligned.windows, initializer.point_apertures):
            window.aperture.safe_vertices = inset.physical_vertices.copy()
        self.center_parameters = tuple(point_mapping.inverse(aperture, np.zeros(2))
            for aperture in initializer.point_apertures)
        super().__init__(aligned, center=False, dynamic_limit_fraction=.95)
        self.center = True
        self.dimension = self.temporal_dimension

    def split(self, x):
        x = np.asarray(x, float)
        if x.shape != (self.temporal_dimension,) or not np.isfinite(x).all():
            raise ValueError('Expected only the two positive-time coordinates')
        return x, self.center_parameters

    def value_and_gradient(self, x):
        cost, gradient = NativeJointTOGTObjective.value_and_gradient(self, x)
        return cost, gradient[:self.temporal_dimension]


def worker(out, job):
    folder = out/'plans'/job['id']
    folder.mkdir(parents=True, exist_ok=False)
    began = time.perf_counter()
    track, _ = scenario.build_case(scenario.Case(job['level']))
    planner = make_planner(track, 'center' if job['method'] == 'togt_center' else job['method'])
    source_center = job.get('center_source')
    if source_center:
        initial, equivalence = release_initial(planner, read(source_center))
        save(folder/'initial_equivalence.json', equivalence)
    else:
        initial = planner.seeds()[job['seed']]
    coefficients = np.asarray(planner.curve_values(initial)).ravel().reshape(planner.n, 3, 8).transpose(0, 2, 1)
    save(folder/'initial.json', dict(job=job, variables=initial.tolist(),
        durations_s=np.exp(initial[:planner.n]).tolist(),
        points=np.asarray(planner.point_function(initial)).T.tolist(), coefficients=coefficients.tolist(),
        nvar=planner.nvar, point_variables=planner.velocity_start-planner.n,
        optimized_initial_solution_used=bool(source_center)))
    build_s = time.perf_counter()-began
    if job['method'] == 'togt_center':
        objective = CenterObjective(track, planner)
        x0 = k_from_durations(np.exp(initial[:planner.n]))
        begun = time.perf_counter()
        opt = _minimize_togt_lbfgs(objective.scipy_value_and_gradient, x0, objective.config)
        value, gradient = objective.value_and_gradient(opt.x)
        result = dict(found=bool(opt.success and np.isfinite(value)), solve_s=time.perf_counter()-begun,
            candidates=[dict(success=bool(opt.success), status=int(opt.status), message=str(opt.message),
                iterations=int(opt.nit), evaluations=int(opt.nfev), objective=float(value),
                gradient_inf_norm=float(abs(gradient).max()), initial_variables=x0.tolist(),
                variables=opt.x.tolist())], optimizer_calls=[], point_variables=0)
        if result['found']:
            forward = objective.forward(opt.x)
            c = forward.trajectory.coefficients*forward.durations[:, None, None]**np.arange(8)[None, :, None]
            result.update(variables=opt.x.tolist(), durations=forward.durations.tolist(),
                coefficients=c.tolist(), nominal_time_s=float(forward.durations.sum()),
                objective=float(value), dynamic_soft_integral=float(value-forward.durations.sum()),
                crossing_points=forward.waypoints.tolist(), crossing_times_s=forward.traversal_times.tolist())
    else:
        planner.seeds = lambda: [initial.copy()]
        factory = ca.nlpsol
        def ordinary_initialization(name, plugin, nlp, options):
            options = dict(options)
            for key in ('ipopt.warm_start_bound_push', 'ipopt.warm_start_mult_bound_push',
                    'ipopt.warm_start_slack_bound_push'):
                options.pop(key, None)
            return factory(name, plugin, nlp, options)
        ca.nlpsol = ordinary_initialization
        try:
            with (folder/'progress.jsonl').open('w') as log:
                def progress(row):
                    log.write(json.dumps(row, ensure_ascii=False)+'\n')
                    log.flush()
                plan, result = planner.solve_all(np.inf, progress)
        finally:
            ca.nlpsol = factory
        if plan is not None:
            value, _, penalty = planner.objective(plan.x)
            result.update(variables=plan.x.tolist(), durations=plan.durations.tolist(),
                coefficients=plan.coefficients.tolist(), nominal_time_s=float(plan.durations.sum()),
                objective=value, dynamic_soft_integral=penalty,
                crossing_points=np.asarray(planner.point_function(plan.x)).T.tolist(),
                crossing_times_s=plan.crossing_times.tolist())
    result.update(job=job, method=job['method'], case=dict(level=job['level']), build_s=build_s,
        config=asdict(planner.cfg), gate_count=1, all_seven_joint=False, all_gates_joint=True,
        made_at=0., dynamic_limit_fraction=.95, tilt_penalty_weight=0., fixed_quadrature_intervals=32,
        point_variables=0 if job['method'] in ('center', 'togt_center') else planner.velocity_start-planner.n,
        optimized_initial_solution_used=bool(source_center), center_source=source_center,
        safety_coverage='instantaneous_knot_only' if job['method'] in ('instant', 'togt_center') else 'full_passage_exchange')
    if result['found']:
        geometry = evaluate(result['coefficients'], result['durations'], track, scenario.ASSETS/'meshes')
        save(folder/'geometry.json', geometry)
        result['geometry_verdict'] = geometry['verdict']
        result['trajectory_distance_m'] = geometry['trajectory_distance_m']
        # The ablated/reference methods are selected without a post hoc safety filter.
        if job['method'] in ('ours', 'center') and not geometry['passed']:
            result['found'] = False
            result['postcheck_rejected'] = True
    save(folder/'result.json', result)
    print('RESULT', job['id'], result['found'], result.get('nominal_time_s'), flush=True)
