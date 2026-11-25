"""Original full-PVAJ geometry with the exact native TOGT dynamic penalty."""
from dataclasses import replace
import time

import casadi as ca
import numpy as np
from scipy.optimize import OptimizeResult

from nonconvex_timevarying_window.sc_dynatogt.minco import BoundaryState, MincoSnap
from convex_timevarying_window.global_gate_minco_probe import planner as core
from convex_timevarying_window.tilt_heading_minco import dynamics
from convex_timevarying_window.single_gate_stop_racing import point_mapping
from ..independent_free_solve.planner import Planner as OriginalPlanner
from .native import Penalty
from .interior_point import minimize as interior_minimize
from .fast_ipopt import minimize as fast_minimize


def workspace_constraints(state, times, nodes=16, workspace_lower=None, workspace_upper=None, **kwargs):
    rows = []
    for piece in range(int(times.numel())):
        for u in np.linspace(0., 1., nodes+1):
            position = state(piece, float(u), 0)
            rows.extend(ca.vertsplit((position-ca.DM(workspace_lower))/30.))
            rows.extend(ca.vertsplit((ca.DM(workspace_upper)-position)/30.))
    return rows


class Planner(OriginalPlanner):
    def __init__(self, *args, solver='slsqp', dynamics_model='togt', dynamic_limit_fraction=1.,
            temporal_scales=None, **kwargs):
        if solver not in ('ipopt', 'ipopt_legacy', 'slsqp'):
            raise ValueError(f'Unknown soft-dynamics solver {solver!r}')
        self.solver = solver
        self.temporal_scales = tuple((1., .8, 1.2, 1.5) if temporal_scales is None else temporal_scales)
        if not self.temporal_scales or not np.isfinite(self.temporal_scales).all() or min(self.temporal_scales) <= 0.:
            raise ValueError('Temporal scales must be a nonempty sequence of positive finite values')
        if dynamics_model not in ('togt', 'x500'):
            raise ValueError(f'Unknown dynamics model {dynamics_model!r}')
        self.dynamics_model = dynamics_model
        if not np.isfinite(dynamic_limit_fraction) or not (0.0225 < dynamic_limit_fraction <= 1.):
            raise ValueError('Dynamic limit fraction must be in (0.0225, 1]')
        self.dynamic_limit_fraction = float(dynamic_limit_fraction)
        old = dynamics.constraints
        dynamics.constraints = workspace_constraints
        try:
            super().__init__(*args, **kwargs)
        finally:
            dynamics.constraints = old
        self.cfg = replace(self.cfg, physical_thrust_fraction=self.dynamic_limit_fraction)
        if dynamics_model == 'x500' or self.dynamic_limit_fraction != 1.:
            from .kernels import PolynomialPenalty
            self.dynamic_penalty = PolynomialPenalty(dynamics_model, self.dynamic_limit_fraction)
        else:
            self.dynamic_penalty = Penalty()
        self.cached_x, self.cached_objective = None, None

    def witnesses(self):
        if self.solver != 'ipopt':
            return super().witnesses()
        # Start the exchange at each actual knot. The same full-domain oracle
        # adds any missing passage or nonlocal frame constraints.
        return {(gate, gate+side): dict(kind='before' if side == 0 else 'after',
            values=[0.] if side == 0 else []) for gate in range(self.m) for side in (0, 1)}

    def seeds(self):
        seeds = super().seeds()
        for seed in seeds:
            durations = np.exp(seed[:self.n])
            arrivals = np.cumsum(durations)[:-1]
            previous = 0.
            for gate, window in enumerate(self.track.windows):
                motion = window.motion
                arrivals[gate] = max(arrivals[gate], previous+.3)
                if motion.amplitude > 0. and motion.frequency > 0.:
                    omega = 2.*np.pi*motion.frequency
                    phase = omega*arrivals[gate]+motion.phase
                    turning = round((phase-np.pi/2.)/np.pi)
                    earliest = int(np.ceil((omega*(previous+.3)+motion.phase-np.pi/2.)/np.pi))
                    arrivals[gate] = (np.pi/2.+max(turning, earliest)*np.pi-motion.phase)/omega
                previous = arrivals[gate]
            durations[:-1] = np.diff(np.r_[0., arrivals])
            seed[:self.n] = np.log(durations)
            if self.free_points:
                radius = self.cfg.radius+self.cfg.frame_thickness+self.cfg.passage_margin+self.cfg.witness_slack
                for window, at, selection in zip(self.track.windows, np.cumsum(durations)[:-1], self.slices):
                    center, rotation, *_ = window.state_at(at)
                    target = rotation[:, :2].T@(window.center0-center)
                    aperture = window.aperture
                    if aperture.kind == 'circle':
                        ratio = min(1., max(0., aperture.radius-radius)/max(np.linalg.norm(target), 1e-12))
                    else:
                        vertices = aperture.physical_vertices
                        edges = np.roll(vertices, -1, axis=0)-vertices
                        normals = np.c_[edges[:, 1], -edges[:, 0]]/np.linalg.norm(edges, axis=1)[:, None]
                        limits = np.sum(normals*vertices, axis=1)-radius
                        projected = normals@target
                        active = projected > 0.
                        ratio = min(1., float(np.min(limits[active]/projected[active]))) if active.any() else 1.
                    seed[selection] = point_mapping.inverse(aperture, max(0., ratio)*target)
            points = np.asarray(self.point_function(seed)).T
            curve = MincoSnap(BoundaryState(self.track.start), BoundaryState(self.track.goal),
                points, durations)
            arrivals = np.cumsum(durations)[:-1]
            seed[self.velocity_start:] = np.concatenate([
                curve.evaluate(arrivals, derivative).ravel() for derivative in (1, 2, 3)])
        return seeds

    def objective(self, x):
        if self.cached_x is None or not np.array_equal(x, self.cached_x):
            durations = np.exp(x[:self.n])
            raw, jacobian = self.curve(x)
            coefficients = np.asarray(raw).ravel().reshape(self.n, 3, 8).transpose(0, 2, 1)
            penalty, partial_coefficients, partial_times = self.dynamic_penalty.evaluate(coefficients, durations)
            partial = partial_coefficients.transpose(0, 2, 1).ravel()
            gradient = partial@np.asarray(jacobian)
            gradient[:self.n] += (1.+partial_times)*durations
            self.cached_x = x.copy()
            self.cached_objective = float(durations.sum()+penalty), gradient, penalty
        return self.cached_objective

    def constrained_minimize(self, fun, initial, **kwargs):
        if self.solver == 'ipopt':
            return fast_minimize(self, fun, initial, **kwargs)
        return interior_minimize(self, fun, initial, **kwargs)

    def solve_all(self, seconds_per_seed=np.inf, progress=lambda row: None):
        started = time.perf_counter()
        candidates, best = [], None
        old = core.minimize, core.least_squares
        core.minimize = self.constrained_minimize
        core.least_squares = lambda fun, x, **kwargs: OptimizeResult(x=x.copy())
        self.optimizer_calls = []
        try:
            seeds = self.seeds()
            for index, initial in enumerate(seeds):
                self._fast_warm = None
                self._fast_problem = None
                self._fast_time_origin = np.cumsum(np.exp(initial[:self.n]))
                self._fast_normalization = float(np.exp(initial[:self.n]).sum())
                plan, statistics = core.Planner.solve(self, seconds_per_seed,
                    lambda row: progress(dict(temporal_seed=index, **row)), initial_seeds=[initial])
                row = dict(seed=index, initial_variables=initial.tolist(), found=plan is not None,
                    objective=None, nominal_time_s=None, dynamic_soft_integral=None, statistics=statistics)
                if plan is not None:
                    cost, _, penalty = self.objective(plan.x)
                    row.update(objective=cost, nominal_time_s=float(plan.durations.sum()), dynamic_soft_integral=penalty,
                        variables=plan.x.tolist(), durations=plan.durations.tolist(), coefficients=plan.coefficients.tolist())
                    if best is None or cost < best[0]:
                        best = cost, index, plan
                candidates.append(row)
                progress(dict(stage='seed_finished', seed=index, found=row['found'], objective=row['objective'],
                    nominal_time_s=row['nominal_time_s'], elapsed_s=time.perf_counter()-started))
        finally:
            core.minimize, core.least_squares = old
        plan = best[2] if best else None
        return plan, dict(found=plan is not None, solve_s=time.perf_counter()-started,
            candidates=candidates, selected_seed=best[1] if best else None, all_four_attempted=len(seeds) == 4,
            temporal_scales=list(self.temporal_scales), number_initial_starts=len(seeds),
            all_starts_attempted=len(seeds) == len(self.temporal_scales),
            all_seven_joint=True, replanning=False, intermediate_stops=False, independent_internal_pvaj=True,
            objective=best[0] if best else None,
            nonlinear_solver='direct soft objective SLSQP' if self.solver == 'slsqp' else 'equivalent soft-violation epigraph IPOPT',
            numerical_solver=self.solver,
            dynamics='x500 motor and body-rate soft integral' if self.dynamics_model == 'x500' else 'native x500 TOGT soft penalty; no nodal motor or body-rate hard bounds',
            dynamics_model=self.dynamics_model,
            dynamic_limit_fraction=self.dynamic_limit_fraction,
            dynamic_model=('shared x500 aerodynamic north-heading reference with rotor drag and rolling moments'
                if self.dynamics_model == 'x500' else 'released gravity-only x500 QuadManifold with fixed tilt-yaw zero'),
            geometric_constraints='original enclosing-sphere exchange and full frame verification',
            selection='lowest complete time plus configured dynamic integral among qualified converged cold starts',
            attitude_model='north_tilt_heading_pi_over_2 verification and execution interface',
            center_solution_used=False, cross_condition_warm_start=False, optimizer_calls=self.optimizer_calls,
            initialization='four independent motion-turning-time, geometry-only point and analytic MINCO PVAJ curves',
            external_solution_used=False, primal_dual_tolerance=1e-8 if self.solver != 'slsqp' else None,
            slsqp_ftol=1e-8 if self.solver == 'slsqp' else None,
            auxiliary_variables=0 if self.solver == 'slsqp' else self.n*33*6,
            hessian_approximation='limited-memory' if self.solver == 'ipopt' else 'exact' if self.solver == 'ipopt_legacy' else 'SLSQP BFGS',
            initial_safety_nodes='one plane witness per gate' if self.solver == 'ipopt' else 'original 20 offsets on both sides',
            solver_coordinates=('TOGT positive durations and algebraic MINCO PVAJ plus free deviations'
                if self.solver == 'slsqp' else 'phase-scaled arrival times and full reduced-MINCO PVAJ deviations'
                if self.solver == 'ipopt' else 'absolute arrivals and algebraic MINCO PVAJ plus free deviations'),
            formal_certificate=False, global_optimality_claim=False)
