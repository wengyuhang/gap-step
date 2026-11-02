"""Single-start local solve with an explicit, nonmonotone escape step.

The poll has no room label, target point, random seed or stored solution.
It changes only the SC disk point; all times and velocities are inherited.
No claim of global or finite-time feasibility completeness is made.
"""
import time
import numpy as np
from scipy.optimize import least_squares, minimize

from convex_timevarying_window.dynamic_passage_minco.planner import BudgetExpired
from convex_timevarying_window.single_gate_stop_racing.geometry import passage
from convex_timevarying_window.single_gate_stop_racing.solver import add_witness
from nonconvex_timevarying_window.sc_dynatogt.sc_mapping import B
from ..planner import Planner as BasePlanner


def disk_trials(d, radii=(.125, .25, .5, 1.), directions=8):
    """Symmetric trial displacements; discard endpoints outside the open disk."""
    origin = B(d)
    for radius in radii:
        for k in range(directions):
            angle = 2 * np.pi * k / directions
            point = origin + radius * np.array([np.cos(angle), np.sin(angle)])
            norm2 = float(point @ point)
            if norm2 < 1 - 1e-12:
                yield point / np.sqrt(1 - norm2), dict(radius=radius, angle_rad=angle)


def violation_loss(g):
    residual = np.minimum(np.asarray(g, float).ravel(), 0.)
    return float(residual @ residual)


class Planner(BasePlanner):
    def solve(self, state, now=0., first=0, *, escape=True, max_hops=3):
        if first != 0:
            raise ValueError('This experiment has one gate')
        cfg, w = self.cfg, self.windows[0]
        started = time.perf_counter()
        deadline = started + cfg.max_solve_ms / 1000.
        state = np.asarray(state, float)
        if state.size == 9:
            state = np.r_[state, np.zeros(3)]
        initial1 = max(.8, np.linalg.norm(w.center0-state[:3]) / cfg.arrival_seed_speed + .5)
        distance = np.linalg.norm(self.track.goal-w.center0)
        initial2 = max(.8, np.sqrt(6*distance/cfg.acceleration), (60*distance/cfg.jerk)**(1/3))
        omega = 2*np.pi/w.motion.translation_period
        nearest = round((omega*(now+initial1)+w.motion.phase-np.pi/2)/np.pi)
        duration = max(.3, (np.pi/2+nearest*np.pi-w.motion.phase)/omega-now)
        velocity = -.8*cfg.speed*self.signs[0]*self.rotation[:, 2]
        x = np.r_[np.log([duration, initial2*cfg.initial_tail_scale]), self.initial_d, velocity]
        x = np.clip(x, self.lower, self.upper)
        initial = x.copy()
        base = np.asarray(cfg.passage_seed_offsets)
        offsets = np.tile(np.r_[base, np.zeros(cfg.witness_slots-len(base))], (2, 1))
        used = [len(base), len(base)]
        trace, rounds, polls = [], [], []
        hops, exchanges, check_ms = 0, 0, 0.
        selected = None
        status = 'no_accepted_candidate'

        def record(z, phase):
            q = self.mapping.map_unconstrained(z[2:4])
            trace.append(dict(phase=phase, variables=z.tolist(), q=q.tolist()))

        def guard():
            if time.perf_counter() >= deadline-.003:
                raise BudgetExpired()

        record(x, 'single_initial')
        while exchanges < cfg.exchange_rounds:
            par = np.r_[now, state, offsets.ravel()]
            origin = x.copy()
            scale = np.r_[1/max(1., omega*np.exp(x[0])), 1., np.ones(2), np.full(3, cfg.speed)]
            cache_x, cache_value = None, None

            def evaluate(y):
                nonlocal cache_x, cache_value
                guard()
                z = origin+scale*y
                if cache_x is None or not np.array_equal(z, cache_x):
                    raw = self.function(z, par)
                    cache_x = z.copy()
                    cache_value = (float(raw[0]), np.asarray(raw[1]).ravel()*scale,
                                   np.asarray(raw[2]).ravel(), np.asarray(raw[3])*scale)
                return cache_value

            def objective(y):
                cost, grad, _, _ = evaluate(y)
                if cfg.log_cost_scaling:
                    return np.log1p(1000*cost), 1000*grad/(1+1000*cost)
                return cost, grad

            try:
                guard()
                y0 = np.zeros(7)
                before = evaluate(y0)[2]
                restore_message, restore_nfev = 'not_needed', 0
                if before.min() < -1e-5:
                    restoration = least_squares(
                        lambda y: np.minimum(evaluate(y)[2], 0.), y0,
                        jac=lambda y: evaluate(y)[3]*(evaluate(y)[2]<0)[:, None],
                        bounds=((self.lower-origin)/scale, (self.upper-origin)/scale),
                        max_nfev=40, ftol=1e-8, xtol=1e-8, gtol=1e-8)
                    y0 = restoration.x
                    restore_message, restore_nfev = restoration.message, restoration.nfev
                    record(origin+scale*y0, 'restoration_end')
                result = minimize(objective, y0, method='SLSQP', jac=True,
                    bounds=list(zip((self.lower-origin)/scale, (self.upper-origin)/scale)),
                    constraints={'type': 'ineq', 'fun': lambda y: evaluate(y)[2],
                                 'jac': lambda y: evaluate(y)[3]},
                    callback=lambda y: record(origin+scale*y, 'slsqp_iterate'),
                    options={'maxiter': cfg.constrained_iterations, 'ftol': 1e-7})
                x = origin+scale*result.x
                raw = self.function(x, par)
                g = np.asarray(raw[2]).ravel()
                record(x, 'local_end')
                row = dict(index=len(rounds), optimizer_success=bool(result.success),
                           status=str(result.message), minimum_residual=float(g.min()),
                           violation_loss=violation_loss(g), variables=x.tolist(),
                           restoration_message=str(restore_message), restoration_nfev=int(restore_nfev))
                rounds.append(row)
                if g.min() >= -1e-5:
                    # Feasibility is independently checked, not inferred from success.
                    plan = self._plan(x, now, raw)
                    began_check = time.perf_counter()
                    radius = cfg.radius+cfg.frame_thickness+cfg.passage_margin
                    parts = [passage(plan, w, radius, k, deadline) for k in (0, 1)]
                    check_ms += 1000*(time.perf_counter()-began_check)
                    row['passage'] = dict(passed=all(v['passed'] for v in parts), parts=parts)
                    guard()
                    if row['passage']['passed']:
                        selected, status = plan, 'finite_constraints_and_local_geometry_passed'
                        break
                    added = False
                    for k, part in enumerate(parts):
                        if part['passed'] or part['witness_u'] is None:
                            continue
                        offset = (1-part['witness_u'] if k == 0 else part['witness_u'])*plan.durations[k]
                        added = add_witness(offsets, used, k, offset) or added
                    exchanges += 1
                    if not added:
                        status = 'geometry_unresolved_no_new_witness'
                        break
                    continue
                if not escape or hops >= max_hops:
                    status = 'local_infeasible_stagnation'
                    break
                # Nonmonotone: the best *displaced* trial may be worse than x.
                # One continuation is selected; no local solves at other trials.
                trials, best = [], None
                for d, metadata in disk_trials(x[2:4]):
                    guard()
                    trial = x.copy()
                    trial[2:4] = d
                    value = self.function(trial, par)
                    loss = violation_loss(value[2])
                    item = dict(**metadata, variables=trial.tolist(),
                                q=self.mapping.map_unconstrained(d).tolist(), violation_loss=loss)
                    trials.append(item)
                    if np.isfinite(loss) and (best is None or loss < best['violation_loss']):
                        best = item
                poll = dict(origin=x.tolist(), origin_loss=violation_loss(g), trials=trials, selected=best)
                polls.append(poll)
                if best is None:
                    status = 'no_finite_poll_candidate'
                    break
                x = np.asarray(best['variables'])
                hops += 1
                record(x, 'nonmonotone_poll_selected')
            except BudgetExpired:
                status = 'wall_budget'
                break
        stats = dict(return_status=status, accepted=selected is not None,
                     optimization_ms=1000*(time.perf_counter()-started), local_check_ms=check_ms,
                     initial_count=1, initial_variables=initial.tolist(), escape_enabled=escape,
                     hop_count=hops, rounds=rounds, polls=polls, trace=trace,
                     sc_map_ms=self.map_ms, formal_certificate=False, flight_executed=False,
                     evidence='Finite dynamic nodes and floating-point local geometry enclosures only')
        self.solvers[0][0].value = stats
        if selected is None:
            raise RuntimeError(status)
        selected.optimizer_stats = stats
        selected.solve_ms = stats['optimization_ms']
        selected.event_valid, selected.event_reason = True, status
        selected.terminal_gate = None
        return selected
