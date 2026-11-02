"""Instrumented copy of single_gate_stop_racing.solver, all point seeds LEFT.

Same four time seeds, restoration, SLSQP, exchange, acceptance and constraints.
The only optimization change is replacing convex initial point coordinates
with the offline inverse of the explicitly requested left-room point.
"""
import time
import numpy as np
from scipy.optimize import minimize, least_squares
from convex_timevarying_window.dynamic_passage_minco.planner import BudgetExpired
from convex_timevarying_window.single_gate_stop_racing.solver import add_witness
from convex_timevarying_window.single_gate_stop_racing.geometry import passage
from .scenario import room


def solve(self, state, now, first):
    if first != 0:
        raise ValueError('Only one window is present')
    began = time.perf_counter()
    deadline = began + self.cfg.max_solve_ms / 1000.
    cfg = self.cfg
    state = np.asarray(state)
    if len(state) == 9:
        state = np.r_[state, np.zeros(3)]
    w = self.windows[0]
    d1 = np.linalg.norm(w.center0 - state[:3])
    d2 = np.linalg.norm(self.track.goal - w.center0)
    initial1 = max(.8, d1 / cfg.arrival_seed_speed + .5)
    initial2 = max(.8, np.sqrt(6 * d2 / cfg.acceleration), (60 * d2 / cfg.jerk) ** (1 / 3))
    omega = 2 * np.pi / w.motion.translation_period
    nearest = round((omega * (now + initial1) + w.motion.phase - np.pi / 2) / np.pi)
    velocity0 = -.8 * cfg.speed * self.signs[0] * self.rotation[:, 2]
    base = np.asarray(cfg.passage_seed_offsets)
    offsets0 = np.tile(np.r_[base, np.zeros(cfg.witness_slots - len(base))], (2, 1))
    cold = [np.r_[np.log([max(.3, (np.pi / 2 + (nearest + k) * np.pi - w.motion.phase)
                                 / omega - now), initial2 * cfg.initial_tail_scale]),
                  self.initial_d, velocity0] for k in (0, -1, -2, 1)]
    par0 = np.r_[now, state, offsets0.ravel()]

    def score(x):
        out = self.function(np.clip(x, self.lower, self.upper), par0)
        return float(np.max(np.maximum(-np.asarray(out[2]), 0.))), float(out[0])

    cold.sort(key=score)
    attempts, trace = [], []
    iterations, check_ms, best = 0, 0., None
    for seed_index, seed in enumerate(cold):
        if time.perf_counter() >= deadline - .003:
            break
        x = np.clip(seed, self.lower, self.upper)
        offsets, used = offsets0.copy(), [len(base)] * 2
        low, high = self.lower.copy(), self.upper.copy()
        records = []

        def record_point(z, phase, outer):
            q = self.mapping.map_unconstrained(z[2:4])
            trace.append(dict(seed=seed_index, round=outer, phase=phase, q=q.tolist(),
                              room=room(q), x=z.tolist(),
                              boundary_distance_m=float(w.aperture.boundary_distance(q))))

        initial_room = room(self.mapping.map_unconstrained(x[2:4]))
        record_point(x, 'initial_' + initial_room, -1)
        for outer in range(cfg.exchange_rounds):
            if time.perf_counter() >= deadline - .003:
                break
            par = np.r_[now, state, offsets.ravel()]
            origin, last = x.copy(), x.copy()
            scale = np.r_[1 / max(1., omega * np.exp(x[0])), 1.,
                          np.ones(self.point_dim), np.full(3, cfg.speed)]
            cached_x, cached = None, None
            restore_best, restore_x = np.inf, np.zeros(self.nvar)

            def evaluate(y):
                nonlocal cached_x, cached
                if time.perf_counter() >= deadline - .003:
                    raise BudgetExpired()
                z = origin + scale * y
                if cached_x is None or not np.array_equal(z, cached_x):
                    raw = self.function(z, par)
                    cached_x = z.copy()
                    cached = (float(raw[0]), np.asarray(raw[1]).ravel(),
                              np.asarray(raw[2]).ravel(), np.asarray(raw[3]), raw)
                return cached

            def objective(y):
                a = evaluate(y)
                if cfg.log_cost_scaling:
                    cost = 1000 * a[0]
                    return np.log1p(cost), a[1] * scale * (1000 / (1 + cost))
                return a[0], a[1] * scale

            def callback(y):
                nonlocal last
                last = origin + scale * y
                record_point(last, 'slsqp_iterate', outer)
                if time.perf_counter() >= deadline - .003:
                    raise BudgetExpired()

            in_restore, restoration_evaluations = True, 0
            try:
                def residual(y):
                    nonlocal restore_best, restore_x
                    g = evaluate(y)[2]
                    r = np.minimum(g, 0.)
                    value = float(r @ r)
                    if value < restore_best:
                        restore_best, restore_x = value, y.copy()
                        record_point(origin + scale * y, 'restoration_best', outer)
                    return r

                def residual_jac(y):
                    a = evaluate(y)
                    return a[3] * scale * (a[2] < 0.)[:, None]

                initial = np.zeros(self.nvar)
                if cfg.feasibility_restore and np.min(evaluate(initial)[2]) < -1e-5:
                    restored = least_squares(residual, initial, jac=residual_jac,
                        bounds=((low - origin) / scale, (high - origin) / scale),
                        method='trf', max_nfev=40, ftol=1e-8, xtol=1e-8, gtol=1e-8)
                    initial = restored.x
                    last = origin + scale * initial
                    restoration_evaluations = int(restored.nfev)
                in_restore = False
                if time.perf_counter() >= deadline - .003:
                    raise BudgetExpired()
                result = minimize(objective, initial, method='SLSQP', jac=True,
                    bounds=list(zip((low - origin) / scale, (high - origin) / scale)),
                    constraints={'type': 'ineq', 'fun': lambda y: evaluate(y)[2],
                                 'jac': lambda y: evaluate(y)[3] * scale},
                    callback=callback,
                    options={'maxiter': cfg.constrained_iterations, 'ftol': 1e-7, 'disp': False})
                x = origin + scale * result.x
                iterations += int(result.nit)
                status, converged = str(result.message), bool(result.success)
            except BudgetExpired:
                if in_restore and restore_best < np.inf:
                    last = origin + scale * restore_x
                x, status, converged = last, 'wall_budget', False
            if time.perf_counter() >= deadline:
                break
            out = self.function(x, par)
            g = np.asarray(out[2]).ravel()
            violation = max(0., float(-g.min()))
            record_point(x, 'round_end', outer)
            record = dict(round=outer, status=status, variables=x.tolist(),
                          objective_s=1000 * float(out[0]), minimum_witness_residual=float(g.min()),
                          restoration_evaluations=restoration_evaluations)
            records.append(record)
            if not converged or violation > 1e-5:
                break
            plan = self._plan(x, now, out)
            check_start = time.perf_counter()
            parts = [self.check_passage(plan, w, cfg.radius + cfg.frame_thickness + cfg.passage_margin,
                                       piece, deadline) for piece in (0, 1)]
            check_ms += 1000 * (time.perf_counter() - check_start)
            verified = dict(passed=all(p['passed'] for p in parts), parts=parts,
                minimum_sample_distance_m=min(p['minimum_sample_distance_m'] for p in parts),
                evidence='floating-point local Bernstein/sine enclosures, not an Arb certificate')
            record['passage'] = verified
            if verified['passed'] and time.perf_counter() < deadline:
                best = plan, verified
                break
            added = False
            for piece, p in enumerate(parts):
                if p['passed'] or p['witness_u'] is None:
                    continue
                offset = ((1 - p['witness_u']) if piece == 0 else p['witness_u']) * plan.durations[piece]
                added = add_witness(offsets, used, piece, offset) or added
            if not added:
                break
        attempts.append(dict(seed=seed.tolist(),
                             kind=room(self.mapping.map_unconstrained(seed[2:4])) + '_point_time_seed',
                             rounds=records))
        if best is not None:
            break
    elapsed = 1000 * (time.perf_counter() - began)
    stats = dict(iter_count=iterations, optimization_ms=elapsed, local_check_ms=check_ms,
                 attempts=attempts, trace=trace, map_ms=self.map_ms,
                 return_status='safe_passage_found' if best else 'no_safe_passage_under_budget',
                 converged=best is not None, formal_certificate=False,
                 candidate_policy='first_converged_locally_safe',
                 point_parameterization='SC_physical_polygon_no_inset',
                 every_trajectory_seed_in_left_room=all(
                     room(self.mapping.map_unconstrained(s[2:4])) == 'left' for s in cold),
                 failed_candidate_is_executable=False)
    self.solvers[0][0].value = stats
    if best is None:
        raise RuntimeError('no_safe_passage_under_budget')
    plan, verified = best
    stats.update(final_objective_s=float(plan.durations.sum()), passage=verified)
    plan.optimizer_stats, plan.solve_ms = stats, elapsed
    plan.event_valid, plan.event_reason = True, 'local_passage_numerically_bounded'
    plan.terminal_gate = None
    return plan
