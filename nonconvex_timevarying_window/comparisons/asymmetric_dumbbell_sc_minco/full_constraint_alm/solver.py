"""Original dynamic hard NLP; only change how a single initial guess is solved."""
import time
import numpy as np
from scipy.optimize import minimize

from convex_timevarying_window.dynamic_passage_minco.planner import BudgetExpired
from convex_timevarying_window.single_gate_stop_racing.solver import add_witness
from ..planner import Planner as BasePlanner


def initial_variables(planner, state, now):
    cfg, w = planner.cfg, planner.windows[0]
    head = max(.8, np.linalg.norm(w.center0-state[:3])/cfg.arrival_seed_speed+.5)
    distance = np.linalg.norm(planner.track.goal-w.center0)
    tail = max(.8, np.sqrt(6*distance/cfg.acceleration), (60*distance/cfg.jerk)**(1/3))
    omega = 2*np.pi/w.motion.translation_period
    nearest = round((omega*(now+head)+w.motion.phase-np.pi/2)/np.pi)
    duration = max(.3, (np.pi/2+nearest*np.pi-w.motion.phase)/omega-now)
    velocity = -.8*cfg.speed*planner.signs[0]*planner.rotation[:, 2]
    return np.clip(np.r_[np.log([duration, tail*cfg.initial_tail_scale]),
                          planner.initial_d, velocity], planner.lower, planner.upper)


def augmented_value_gradient(cost, grad, g, jac, multiplier, rho):
    active = np.maximum(0., multiplier-rho*g)
    value = cost+(active@active-multiplier@multiplier)/(2*rho)
    return float(value), grad-jac.T@active


class Planner(BasePlanner):
    def solve(self, state, now=0., first=0, *, rho0=1., outer_iterations=20, inner_iterations=100):
        if first != 0:
            raise ValueError('Single gate only')
        cfg, w = self.cfg, self.windows[0]
        state = np.asarray(state, float)
        if len(state) == 9:
            state = np.r_[state, np.zeros(3)]
        began = time.perf_counter()
        deadline = began+cfg.max_solve_ms/1000
        x = initial_variables(self, state, now)
        initial = x.copy()
        base = np.asarray(cfg.passage_seed_offsets)
        offsets = np.tile(np.r_[base, np.zeros(cfg.witness_slots-len(base))], (2, 1))
        used = [len(base), len(base)]
        trace, rounds = [], []
        selected, status, local_check_ms = None, 'outer_iteration_limit', 0.

        def record(z, phase, outer):
            trace.append(dict(phase=phase, outer=outer, variables=z.copy(),
                              q=self.mapping.map_unconstrained(z[2:4])))

        def guard():
            if time.perf_counter() >= deadline-.003:
                raise BudgetExpired()

        record(x, 'single_left_initial', -1)
        try:
            for exchange in range(cfg.exchange_rounds):
                par = np.r_[now, state, offsets.ravel()]
                omega = 2*np.pi/w.motion.translation_period
                scale = np.r_[1/max(1., omega*np.exp(x[0])), 1., np.ones(2), np.full(3, cfg.speed)]
                origin = x.copy()
                bounds = list(zip((self.lower-origin)/scale, (self.upper-origin)/scale))
                cache_x, cache_value = None, None
                last = x.copy()

                def evaluate(y):
                    nonlocal cache_x, cache_value, last
                    guard()
                    z = origin+scale*y
                    last = z.copy()
                    if cache_x is None or not np.array_equal(z, cache_x):
                        out = self.function(z, par)
                        cost, grad = float(out[0]), np.asarray(out[1]).ravel()*scale
                        if cfg.log_cost_scaling:
                            cost, grad = np.log1p(1000*cost), 1000*grad/(1+1000*cost)
                        cache_value = (cost, grad, np.asarray(out[2]).ravel(), np.asarray(out[3])*scale)
                        cache_x = z.copy()
                    return cache_value

                y = np.zeros(7)
                g = evaluate(y)[2]
                multiplier, rho, previous_error = np.zeros_like(g), float(rho0), np.inf
                needs_exchange = False
                for outer in range(outer_iterations):
                    def merit(z):
                        f, df, g, dg = evaluate(z)
                        return augmented_value_gradient(f, df, g, dg, multiplier, rho)
                    result = minimize(merit, y, method='L-BFGS-B', jac=True, bounds=bounds,
                                      callback=lambda yy: record(origin+scale*yy, 'alm_lbfgs', outer),
                                      options=dict(maxiter=inner_iterations, maxls=40, ftol=1e-11, gtol=1e-7))
                    y = result.x.copy()
                    x = origin+scale*y
                    _, _, g, _ = evaluate(y)
                    violation = max(0., -float(g.min()))
                    complementarity = float(np.max(abs(np.minimum(g, multiplier/rho))))
                    row = dict(exchange=exchange, outer=outer, rho=rho,
                               minimum_residual=float(g.min()), violation=violation,
                               complementarity_residual=complementarity,
                               inner_success=bool(result.success), inner_message=str(result.message),
                               inner_iterations=int(result.nit), inner_evaluations=int(result.nfev),
                               q=self.mapping.map_unconstrained(x[2:4]), variables=x.copy(),
                               elapsed_ms=1000*(time.perf_counter()-began))
                    rounds.append(row)
                    record(x, 'alm_outer_end', outer)
                    if violation <= 1e-5:
                        # Optimize the ORIGINAL constrained time objective before admission.
                        polished = minimize(lambda yy: evaluate(yy)[:2], y, method='SLSQP', jac=True,
                            bounds=bounds, constraints=dict(type='ineq', fun=lambda yy: evaluate(yy)[2],
                                                            jac=lambda yy: evaluate(yy)[3]),
                            callback=lambda yy: record(origin+scale*yy, 'hard_slsqp_polish', outer),
                            options=dict(maxiter=cfg.constrained_iterations, ftol=1e-7))
                        y = polished.x.copy()
                        x = origin+scale*y
                        raw = self.function(x, par)
                        g = np.asarray(raw[2]).ravel()
                        row['polish'] = dict(success=bool(polished.success), message=str(polished.message),
                                             minimum_residual=float(g.min()))
                        if polished.success and float(g.min()) >= -1e-5:
                            plan = self._plan(x, now, raw)
                            started = time.perf_counter()
                            radius = cfg.radius+cfg.frame_thickness+cfg.passage_margin
                            parts = [self.check_passage(plan, w, radius, k, deadline) for k in (0, 1)]
                            local_check_ms += 1000*(time.perf_counter()-started)
                            row['passage'] = dict(passed=all(p['passed'] for p in parts), parts=parts)
                            guard()
                            if row['passage']['passed']:
                                selected, status = plan, 'original_constraints_and_local_geometry_passed'
                                break
                            for k, part in enumerate(parts):
                                if part['passed'] or part['witness_u'] is None:
                                    continue
                                offset = (1-part['witness_u'] if k == 0 else part['witness_u'])*plan.durations[k]
                                needs_exchange = add_witness(offsets, used, k, offset) or needs_exchange
                            if not needs_exchange:
                                status = 'geometry_unresolved_no_new_witness'
                            break
                    multiplier = np.minimum(np.maximum(0., multiplier-rho*g), 1e12)
                    if complementarity > .5*previous_error:
                        rho = min(1e10, 10*rho)
                    previous_error = complementarity
                if selected is not None or not needs_exchange:
                    break
        except BudgetExpired:
            status = 'wall_budget'
            x = last.copy() if 'last' in locals() else x
        stats = dict(return_status=status, accepted=selected is not None,
                     initial_count=1, initial_variables=initial,
                     final_variables=x, final_q=self.mapping.map_unconstrained(x[2:4]),
                     optimization_ms=1000*(time.perf_counter()-began), local_check_ms=local_check_ms,
                     rounds=rounds, trace=trace, rho0=rho0, outer_iterations=outer_iterations,
                     inner_iterations=inner_iterations, explicit_nonlocal_search=False,
                     formal_certificate=False, flight_executed=False,
                     constraints='Original full dynamic NLP; nodal dynamics plus local geometric enclosure')
        self.solvers[0][0].value = stats
        if selected is None:
            raise RuntimeError(status)
        selected.optimizer_stats = stats
        selected.solve_ms = stats['optimization_ms']
        selected.event_valid, selected.event_reason = True, status
        selected.terminal_gate = None
        return selected
