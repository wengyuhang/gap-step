"""Joint seven-gate extension of the current single-stop planner, offline only."""
import time
from dataclasses import replace
import casadi as ca
import numpy as np
from scipy.optimize import least_squares, minimize
from convex_timevarying_window.single_gate_stop_racing import point_mapping
from convex_timevarying_window.single_gate_stop_racing.geometry import symbolic_constraints, passage
from convex_timevarying_window.single_gate_stop_racing.planner import Config, route_signs
from convex_timevarying_window.single_gate_stop_racing.scenario import workspace_bounds
from convex_timevarying_window.dynamic_passage_velocity_minco.hard_dynamics import constraints
from convex_timevarying_window.dynamic_passage_velocity_minco.trajectory import Plan
from convex_timevarying_window.togt_minco_mpc.trajectory import basis
from convex_timevarying_window.comparisons.original_fixed_plane_ours.adapter import symbolic_pose
from .trajectory import coefficients, full_state_coefficients


class Deadline(Exception):
    pass


class Planner:
    def __init__(self, track, *, position_only=False, full_state=False, seed_speed=6.5,
                 planning_thrust_fraction=1.):
        if position_only and full_state:
            raise ValueError('Choose position-only, PV, or full PVAJ')
        self.track = track
        self.position_only = position_only
        self.full_state = full_state
        self.cfg = replace(Config(), workspace_lower=workspace_bounds()[0], workspace_upper=workspace_bounds()[1],
                           physical_thrust_fraction=planning_thrust_fraction)
        # This only initializes the time allocation; it is not a speed bound.
        self.seed_speed = self.cfg.speed if seed_speed is None else float(seed_speed)
        if not np.isfinite(self.seed_speed) or self.seed_speed <= 0.:
            raise ValueError('seed_speed must be positive and finite')
        self.m = len(track.windows)
        self.n = self.m+1
        self.signs = route_signs(track)
        self.dims = [point_mapping.dimension(w.aperture) for w in track.windows]
        self.slices = []
        cursor = self.n
        for d in self.dims:
            self.slices.append(slice(cursor, cursor+d))
            cursor += d
        self.velocity_start = cursor
        self.nvar = cursor+(0 if position_only else (9 if full_state else 3)*self.m)
        x = ca.MX.sym('global_variables', self.nvar)
        T = ca.exp(x[:self.n])
        arrivals = ca.cumsum(T)[:self.m]
        points = []
        for k, w in enumerate(track.windows):
            center, R = symbolic_pose(w, arrivals[k])
            points.append(center+R[:, :2]@point_mapping.symbolic(w.aperture, x[self.slices[k]]))
        self.point_function = ca.Function('global_points', [x], [ca.horzcat(*points)])
        head = ca.horzcat(ca.DM(track.start), ca.DM.zeros(3, 3))
        tail = ca.horzcat(ca.DM(track.goal), ca.DM.zeros(3, 3))
        if position_only:
            from convex_timevarying_window.togt_minco_mpc.trajectory import minco_coefficients
            C = minco_coefficients(head, tail, ca.horzcat(*points), T)
        elif full_state:
            V, A, J = [ca.reshape(x[cursor+3*self.m*k:cursor+3*self.m*(k+1)], 3, self.m) for k in range(3)]
            C = full_state_coefficients(head, tail, ca.horzcat(*points), V, A, J, T)
            self.pv_seed_curve = ca.Function('pv_seed_curve', [x],
                [coefficients(head, tail, ca.horzcat(*points), V, T)])
        else:
            C = coefficients(head, tail, ca.horzcat(*points), ca.reshape(x[cursor:], 3, self.m), T)
        forward = ca.vertcat(*[
            ca.dot(ca.DM(-self.signs[k]*w.state_at(0.)[1][:, 2]),
                   C[8*k:8*k+8, :].T@basis(1., 1)/T[k])-.1
            for k, w in enumerate(self.track.windows)])
        self.forward = ca.Function('global_forward', [x], [forward, ca.jacobian(forward, x)])
        # Piece-major 24-coefficient blocks, also used in the analytic chain rule.
        vector = ca.vertcat(*[ca.vec(C[8*k:8*k+8, :]) for k in range(self.n)])
        self.curve = ca.Function('global_curve', [x], [vector, ca.jacobian(vector, x)], {'cse': True})
        # Geometric homotopy is optional and used only to obtain a shorter
        # initialization; weight zero recovers the unchanged time objective.
        abscissae, weights = np.polynomial.legendre.leggauss(32)
        length = 0.
        for piece in range(self.n):
            for u, weight in zip((abscissae+1)/2, weights/2):
                tangent = C[8*piece:8*piece+8, :].T@basis(float(u), 1)
                length += float(weight)*ca.sqrt(ca.sumsqr(tangent)+1e-12)
        self.arc = ca.Function('global_arc', [x], [length, ca.gradient(length, x)], {'cse': True})
        z = ca.MX.sym('piece_physics', 25)
        coeff = ca.reshape(z[:24], 8, 3)
        def state(piece, u, d=0):
            return coeff.T@basis(u, d)/z[24]**d
        G = ca.vertcat(*constraints(state, ca.vertcat(z[24]), self.cfg.nodes_per_piece,
                                    self.cfg.workspace_lower, self.cfg.workspace_upper,
                                    thrust_fraction=self.cfg.physical_thrust_fraction))
        self.physical = ca.Function('piece_physical', [z], [G, ca.jacobian(G, z)], {'cse': True})
        self.safety = []
        S = self.cfg.witness_slots
        for k, w in enumerate(track.windows):
            z = ca.MX.sym('piece_safety', 26)
            us = ca.MX.sym('witnesses', S)
            coeff = ca.reshape(z[:24], 8, 3)
            rows = []
            radius = self.cfg.radius+self.cfg.frame_thickness+self.cfg.passage_margin+self.cfg.witness_slack
            for u in ca.vertsplit(us):
                center, R = symbolic_pose(w, z[25]+u*z[24])
                q = R.T@(coeff.T@basis(u)-center)
                rows.extend(symbolic_constraints(w.aperture, q, radius))
            G = ca.vertcat(*rows)
            self.safety.append(ca.Function('piece_safety_'+str(k), [z, us],
                [G, ca.jacobian(G, z), ca.jacobian(G, us)], {'cse': True}))
        self.lower = np.r_[np.log(np.full(self.n, .3)), np.full(self.nvar-self.n, -np.inf)]
        self.upper = np.r_[np.log(np.full(self.n, 15.)), np.full(self.nvar-self.n, np.inf)]
        from convex_timevarying_window.single_gate_stop_racing.local_bounds import warmup
        warmup()

    def seeds(self):
        anchors = np.array([w.center0 for w in self.track.windows])
        nodes = np.vstack([self.track.start, anchors, self.track.goal])
        durations = np.maximum(.8, np.linalg.norm(np.diff(nodes, axis=0), axis=1)/self.seed_speed)
        durations[0] += .5
        distance = np.linalg.norm(nodes[-1]-nodes[-2])
        durations[-1] = max(.8, np.sqrt(6*distance/self.cfg.acceleration),
                            (60*distance/self.cfg.jerk)**(1/3))
        estimates = np.cumsum(durations)[:self.m]
        result = []
        for shift in (0, -1, -2, 1):
            arrivals = []
            for estimate, w in zip(estimates, self.track.windows):
                omega = 2*np.pi/w.motion.translation_period
                k = round((omega*estimate+w.motion.phase-np.pi/2)/np.pi)+shift
                arrivals.append((np.pi/2+k*np.pi-w.motion.phase)/omega)
            T = np.r_[np.diff(np.r_[0., arrivals]), durations[-1]]
            if np.min(T) < .3:
                continue
            centers = np.array([w.state_at(t)[0] for w, t in zip(self.track.windows, arrivals)])
            # Select the normal forward velocity only as an initial guess,
            # matching the current local planner. The optimizer is free in 3D.
            velocity = np.array([-.8*self.cfg.speed*s*w.state_at(0.)[1][:, 2]
                                 for s, w in zip(self.signs, self.track.windows)])
            points = np.concatenate([point_mapping.initial(w.aperture) for w in self.track.windows])
            result.append(np.r_[np.log(T), points] if self.position_only else
                          np.r_[np.log(T), points, velocity.ravel()])
        return [self.lift_pv(x) for x in result] if self.full_state else result

    def lift_pv(self, x):
        """Embed the exact old PV curve, not zero acceleration/jerk guesses."""
        if not self.full_state or len(x) != self.velocity_start+3*self.m:
            raise ValueError('Expected PV variables and a full-state planner')
        extended = np.r_[x, np.zeros(6*self.m)]
        C = np.asarray(self.pv_seed_curve(extended)).reshape(self.n, 8, 3)
        T = np.exp(x[:self.n])
        aj = [np.concatenate([C[k].T@np.asarray(basis(1., d)).ravel()/T[k]**d
                              for k in range(self.m)]) for d in (2, 3)]
        return np.r_[x, *aj]

    def witnesses(self):
        base = np.asarray(self.cfg.passage_seed_offsets)
        return {(k, k+side): dict(kind='before' if side == 0 else 'after', values=list(base))
                for k in range(self.m) for side in (0, 1)}

    def evaluate(self, x, witnesses):
        T = np.exp(x[:self.n])
        starts = np.r_[0., np.cumsum(T)[:-1]]
        tjac = np.zeros((self.n, self.nvar))
        tjac[:, :self.n] = np.diag(T)
        sjac = np.zeros_like(tjac)
        for k in range(self.n):
            sjac[k, :k] = T[:k]
        raw = self.curve(x)
        C, cj = np.asarray(raw[0]).ravel(), np.asarray(raw[1])
        values, jacobians = [], []
        for piece in range(self.n):
            sl = slice(24*piece, 24*(piece+1))
            out = self.physical(np.r_[C[sl], T[piece]])
            a = np.asarray(out[1])
            values.append(np.asarray(out[0]).ravel())
            jacobians.append(a[:, :24]@cj[sl]+a[:, 24, None]*tjac[piece])
        for (gate, piece), item in sorted(witnesses.items()):
            # The compiled batch size is not a limit on exchange constraints.
            # Keep all witnesses, evaluating additional batches as needed.
            for begin in range(0, len(item['values']), self.cfg.witness_slots):
                offsets = np.resize(np.asarray(item['values'][begin:begin+self.cfg.witness_slots]), self.cfg.witness_slots)
                active = offsets < T[piece]
                if item['kind'] == 'before':
                    u = 1-np.minimum(offsets, T[piece])/T[piece]
                    du = np.where(active, offsets/T[piece]**2, 0.)
                elif item['kind'] == 'after':
                    u = np.minimum(offsets, T[piece])/T[piece]
                    du = np.where(active, -offsets/T[piece]**2, 0.)
                else:
                    u, du = offsets, np.zeros_like(offsets)
                sl = slice(24*piece, 24*(piece+1))
                out = self.safety[gate](np.r_[C[sl], T[piece], starts[piece]], u)
                a, b = np.asarray(out[1]), np.asarray(out[2])
                values.append(np.asarray(out[0]).ravel())
                jacobians.append(a[:, :24]@cj[sl]+(a[:, 24]+b@du)[:, None]*tjac[piece]
                                 +a[:, 25, None]*sjac[piece])
        if self.position_only:
            forward = self.forward(x)
            values.append(np.asarray(forward[0]).ravel())
            jacobians.append(np.asarray(forward[1]))
        for k, (w, sign) in enumerate(zip(self.track.windows, self.signs)):
            if self.position_only:
                break
            normal = -sign*w.state_at(0.)[1][:, 2]
            sl = slice(self.velocity_start+3*k, self.velocity_start+3*k+3)
            row = np.zeros(self.nvar)
            row[sl] = normal
            values.append(np.array([normal@x[sl]-.1]))
            jacobians.append(row[None, :])
        return np.concatenate(values), np.vstack(jacobians), C.reshape(self.n, 3, 8).transpose(0, 2, 1)

    def verify(self, plan, deadline=np.inf):
        radius = self.cfg.radius+self.cfg.frame_thickness+self.cfg.passage_margin
        results = []
        for gate, w in enumerate(self.track.windows):
            for piece in range(self.n):
                checked = passage(plan, w, radius, piece, deadline)
                results.append(dict(gate=gate, piece=piece, **checked))
                if time.perf_counter() >= deadline:
                    return results, False
        return results, all(p['passed'] for p in results)

    def solve(self, seconds=180., progress=lambda record: None, *, initial_seeds=None, distance_weight=0.):
        began = time.perf_counter()
        deadline = began+seconds
        records = []
        best = np.inf
        first_nodes = None
        first_locals = None
        seeds = self.seeds() if initial_seeds is None else [np.asarray(x).copy() for x in initial_seeds]
        base = self.witnesses()
        seeds.sort(key=lambda x: (max(0., -self.evaluate(x, base)[0].min()), np.exp(x[:self.n]).sum()))
        for seed_id, seed in enumerate(seeds):
            x = seed.copy()
            witnesses = self.witnesses()
            # Phase extrema initialize the solve, but do not bound arrival times.
            for outer in range(24):
                origin = x.copy()
                omega = 2*np.pi/self.track.windows[0].motion.translation_period
                state_scale = np.full(self.nvar-self.velocity_start, self.cfg.speed)
                if self.full_state:
                    state_scale[3*self.m:6*self.m] = self.cfg.acceleration
                    state_scale[6*self.m:] = self.cfg.jerk
                scale = np.r_[1/np.maximum(1., omega*np.exp(x[:self.m])), 1.,
                              np.ones(self.velocity_start-self.n),
                              state_scale]
                cache_x = None
                cache = None
                best_restore = (np.inf, np.zeros(self.nvar))
                last = np.zeros(self.nvar)
                evaluations = 0
                last_progress = time.perf_counter()
                def evaluate(y):
                    nonlocal cache_x, cache, best, evaluations, last_progress
                    if time.perf_counter() >= deadline:
                        raise Deadline()
                    if cache_x is None or not np.array_equal(y, cache_x):
                        cache = self.evaluate(origin+scale*y, witnesses)
                        cache_x = y.copy()
                        evaluations += 1
                        best = min(best, max(0., -cache[0].min()))
                        if time.perf_counter()-last_progress > 5.:
                            progress(dict(stage='optimizing', seed=seed_id, round=outer,
                                elapsed_s=time.perf_counter()-began, evaluations=evaluations,
                                best_node_violation=best, current_node_violation=max(0., -float(cache[0].min()))))
                            last_progress = time.perf_counter()
                    return cache
                def residual(y):
                    nonlocal best_restore
                    r = np.minimum(evaluate(y)[0], 0.)
                    value = float(r@r)
                    if value < best_restore[0]:
                        best_restore = (value, y.copy())
                    return r
                def residual_jac(y):
                    g, jac, _ = evaluate(y)
                    return jac*scale*(g < 0.)[:, None]
                def objective(y):
                    variables = origin+scale*y
                    T = np.exp(variables[:self.n])
                    total = T.sum()
                    gradient = np.r_[T, np.zeros(self.nvar-self.n)]
                    if distance_weight:
                        length, derivative = self.arc(variables)
                        total += distance_weight*float(length)/self.cfg.speed
                        gradient += distance_weight*np.asarray(derivative).ravel()/self.cfg.speed
                    return np.log1p(total), gradient*scale/(1+total)
                def callback(y):
                    nonlocal last
                    last = y.copy()
                    if time.perf_counter() >= deadline:
                        raise Deadline()
                try:
                    y = np.zeros(self.nvar)
                    restored = least_squares(residual, y, jac=residual_jac,
                        bounds=((self.lower-origin)/scale, (self.upper-origin)/scale),
                        method='trf', max_nfev=200, ftol=1e-10, xtol=1e-10, gtol=1e-10)
                    y = restored.x
                    if evaluate(y)[0].min() >= -1e-5 and first_nodes is None:
                        first_nodes = time.perf_counter()-began
                    result = minimize(objective, y, jac=True, method='SLSQP',
                        bounds=list(zip((self.lower-origin)/scale, (self.upper-origin)/scale)),
                        constraints=dict(type='ineq', fun=lambda y: evaluate(y)[0],
                                         jac=lambda y: evaluate(y)[1]*scale), callback=callback,
                        options=dict(maxiter=400, ftol=1e-7, disp=False))
                    x = origin+scale*result.x
                    g, _, C = self.evaluate(x, witnesses)
                    minimum = float(g.min())
                    plan = Plan(0., 0, np.exp(x[:self.n]), C, x.copy(), 0.,
                                max(0., -minimum), str(result.message), self.m)
                    record = dict(seed=seed_id, round=outer, elapsed_s=time.perf_counter()-began,
                        optimizer_converged=bool(result.success), status=str(result.message),
                        iterations=int(result.nit), evaluations=evaluations, minimum_constraint=minimum,
                        duration_s=float(plan.durations.sum()), first_nodes_s=first_nodes)
                    records.append(record)
                    progress(record)
                    if not result.success or minimum < -1e-5:
                        break
                    checks, passed = self.verify(plan, deadline)
                    local_ok = all(p['passed'] for p in checks if p['piece'] in (p['gate'], p['gate']+1))
                    if local_ok and first_locals is None:
                        first_locals = time.perf_counter()-began
                    record['checks'] = checks
                    record['global_passed'] = passed
                    if passed:
                        elapsed = time.perf_counter()-began
                        plan.solve_ms = elapsed*1000
                        return plan, dict(found=True, solve_s=elapsed, first_nodes_s=first_nodes,
                            first_local_checks_s=first_locals, records=records, best_node_violation=best,
                            evidence='nominal nodal actuator bounds + all finite frames locally enclosed, not formal certificate')
                    added = False
                    for item in checks:
                        if item['passed'] or item['witness_u'] is None:
                            continue
                        key = (item['gate'], item['piece'])
                        if key not in witnesses:
                            witnesses[key] = dict(kind='normalized', values=[])
                        target = witnesses[key]
                        u = item['witness_u']
                        value = (1-u)*plan.durations[key[1]] if target['kind'] == 'before' else (
                            u*plan.durations[key[1]] if target['kind'] == 'after' else u)
                        if all(abs(v-value) > 1e-5 for v in target['values']):
                            target['values'].append(float(value))
                            added = True
                    if not added:
                        break
                except Deadline:
                    records.append(dict(seed=seed_id, round=outer, status='wall_budget',
                        elapsed_s=time.perf_counter()-began, evaluations=evaluations,
                        best_restore_residual=best_restore[0]))
                    break
            if time.perf_counter() >= deadline:
                break
        return None, dict(found=False, solve_s=time.perf_counter()-began, first_nodes_s=first_nodes,
                          first_local_checks_s=first_locals, records=records, best_node_violation=best,
                          failure='current_solver_budget_did_not_find_qualified_curve_not_proven_infeasible')
