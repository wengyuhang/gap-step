"""Seven-variable variant: two durations, gate point, and gate velocity.

The old four-variable planner and shared controller are unchanged. Local moving
frame constraints, margins, dynamics cost, witness exchange, and budget are the
same. No no-reversal penalty, extra waypoint, or changed finish criterion.
"""
from dataclasses import dataclass
import math
import tempfile
import time
import casadi as ca
import numpy as np
from scipy.optimize import minimize
from convex_timevarying_window.comparisons.original_fixed_plane_ours.adapter import symbolic_pose
from convex_timevarying_window.dynamic_passage_minco.planner import Config as BaseConfig, BudgetExpired, Stats
from convex_timevarying_window.dynamic_passage_minco.safety import passage
from convex_timevarying_window.dynamic_passage_minco.safety import power_to_bernstein
from convex_timevarying_window.togt_minco_mpc.trajectory import basis
from .trajectory import Plan, minco_coefficients


@dataclass(frozen=True)
class Config(BaseConfig):
    optimizer: str = 'slsqp'
    initial_tail_scale: float = 3.2
    free_crossing_acceleration: bool = False
    kinematic_arrival_seed: bool = False
    forward_tail: bool = False
    log_cost_scaling: bool = False


class Planner:
    def __init__(self, track, cfg=Config(), *, build_indices=None):
        if len(track.windows) != 1:
            raise ValueError('Only the isolated single-window validation is implemented')
        self.track, self.cfg = track, cfg
        self.windows = [track.windows[0]]
        w = self.windows[0]
        if w.aperture.kind != 'polygon' or len(w.aperture.physical_vertices) != 4:
            raise ValueError('Only the fixed-plane rectangular validation is implemented')
        self.rotation = w.state_at(0.)[1]
        self.signs = [float(np.sign((track.start-w.center0)@self.rotation[:, 2]))]
        self.admitted = None
        self._jit_dir = tempfile.TemporaryDirectory(prefix='passage_velocity_minco_jit_')
        self._build()
        self.solvers = [(Stats(),)]

    def commit(self, plan): self.admitted = plan
    def prefix_clearance(self, plan, at): return None

    def extra_constraints(self, state, times, coefficients): return []
    def extra_dynamic_residuals(self, state, times): return []
    def objective_cost(self, times, dynamic):
        return ca.sum1(times)+self.cfg.constraint_penalty*ca.sum1(dynamic**3)
    def order_seeds(self, seeds, state, now): return seeds

    def symbolic_pose(self, window, t):
        return symbolic_pose(window, t)

    def arrival_initialization(self, window, now, initial):
        period = window.motion.translation_period; omega = 2*np.pi/period
        nearest = round((omega*(now+initial)+window.motion.phase-np.pi/2)/np.pi)
        arrivals = [(np.pi/2+(nearest+step)*np.pi-window.motion.phase)/omega-now
                    for step in (0, -1, -2, 1)]
        return arrivals, omega, period

    def arrival_bounds(self, window, now, seed_arrival, omega, period):
        turn = (np.pi/2+round((omega*seed_arrival+window.motion.phase-np.pi/2)/np.pi)*np.pi
                -window.motion.phase)/omega
        return max(.3, turn-period/4-now), min(15., max(.3, turn+period/4-now))

    def local_passage(self, plan, window, piece):
        return passage(plan, window,
                       self.cfg.radius+self.cfg.frame_thickness+self.cfg.passage_margin,
                       self.cfg.frame_offset, piece_indices=(piece,))

    def _build(self):
        cfg = self.cfg; w = self.windows[0]
        self.nvar = 10 if cfg.free_crossing_acceleration else 7
        x = ca.MX.sym('passage_velocity_x', self.nvar)
        par = ca.MX.sym('passage_velocity_par', 13+2*cfg.witness_slots)
        times = ca.exp(x[:2]); crossing = par[0]+times[0]
        center, rotation = self.symbolic_pose(w, crossing)
        knot = center+rotation[:, :2]@x[2:4]
        C = minco_coefficients(ca.reshape(par[1:13], 3, 4),
            ca.horzcat(ca.DM(self.track.goal), ca.DM.zeros(3, 3)), knot, x[4:7], times,
            acceleration=x[7:10] if cfg.free_crossing_acceleration else None)

        def state(piece, u, derivative=0):
            return C[8*piece:8*piece+8, :].T@basis(u, derivative)/times[piece]**derivative

        violations = []
        for piece in range(2):
            for u in np.linspace(0., 1., cfg.nodes_per_piece+1):
                pos, vel, acc, jerk = [state(piece, float(u), d) for d in range(4)]
                demand = acc+cfg.response_time*jerk
                thrust = demand+ca.DM([0., 0., 9.8066])
                for quantity, limit in ((vel, cfg.speed), (acc, cfg.acceleration),
                                        (jerk, cfg.jerk), (demand, cfg.acceleration)):
                    violations.append(ca.fmax(ca.sumsqr(quantity)/limit**2-1., 0.))
                maximum_thrust = 4*8.54858/2.0643076923*.9
                violations += [ca.fmax((4.-ca.sumsqr(thrust))/maximum_thrust**2, 0.),
                    ca.fmax(ca.sumsqr(thrust)/maximum_thrust**2-1., 0.),
                    ca.fmax((ca.sumsqr(thrust[:2])-
                        (math.tan(math.radians(42))*thrust[2])**2)/9.8066**2, 0.)]
                if cfg.workspace_lower is not None:
                    for axis in range(3):
                        violations += [ca.fmax((cfg.workspace_lower[axis]-pos[axis])/30., 0.),
                                       ca.fmax((pos[axis]-cfg.workspace_upper[axis])/30., 0.)]
        violations.extend(self.extra_dynamic_residuals(state, times))
        dynamic = ca.vertcat(*violations)
        cost = self.objective_cost(times, dynamic)
        constraints = []
        half_sizes = np.max(np.abs(w.aperture.physical_vertices), axis=0)+cfg.frame_offset
        safe_radius = cfg.radius+cfg.frame_thickness+cfg.passage_margin+cfg.witness_slack
        for piece in range(2):
            for j in range(cfg.witness_slots):
                offset = ca.fmin(par[13+piece*cfg.witness_slots+j], times[piece])
                u = 1.-offset/times[0] if piece == 0 else offset/times[1]
                t = crossing-offset if piece == 0 else crossing+offset
                cc, rr = self.symbolic_pose(w, t)
                q = rr.T@(state(piece, u)-cc)
                for axis in (0, 1):
                    for sign in (-1, 1):
                        inside = ca.fmax(half_sizes[axis]-sign*q[axis], 0.)
                        constraints.append(inside**2+q[2]**2-safe_radius**2)
        constraints.append(-self.signs[0]*ca.dot(rotation[:, 2], state(0, 1., 1))-.1)
        if cfg.forward_tail:
            # A nonnegative Bernstein hull of normal velocity prevents a
            # post-window reversal, without a time grid or an extra path knot.
            normal = (self.track.goal-w.center0)/np.linalg.norm(self.track.goal-w.center0)
            velocity_power = ca.vertcat(*[
                k*ca.dot(C[8+k, :].T, ca.DM(normal))/times[1] for k in range(1, 8)])
            velocity_control = ca.DM(power_to_bernstein(np.eye(7)))@velocity_power
            # Crossing velocity is already constrained. Goal V=A=J=0 makes
            # the last THREE velocity control values identically zero;
            # omit these redundant rows rather than constrain roundoff noise.
            constraints.extend(ca.vertsplit(velocity_control[1:4]/cfg.speed))
        constraints.extend(self.extra_constraints(state, times, C))
        g = ca.vertcat(*constraints)
        options = {'cse': True}
        if cfg.compile_objective:
            options.update(jit=True, compiler='shell', jit_options={
                'flags': [cfg.compiler_optimization], 'directory': self._jit_dir.name+'/'})
        self.function = ca.Function('passage_velocity_eval', [x, par],
            [cost/1000., ca.gradient(cost/1000., x), g, ca.jacobian(g, x), C, ca.mmax(dynamic)], options)
        half_opening = np.max(np.abs(w.aperture.physical_vertices), axis=0)
        self.lower = np.r_[np.log([.3, .3]), -half_opening if cfg.free_points else [0., 0.],
                           np.full(self.nvar-4, -np.inf)]
        self.upper = np.r_[np.log([15., 15.]), half_opening if cfg.free_points else [0., 0.],
                           np.full(self.nvar-4, np.inf)]

    def _plan(self, x, now, outputs):
        return Plan(now, 0, np.exp(x[:2]), np.asarray(outputs[4]).reshape(2, 8, 3),
                    x.copy(), 0., float(outputs[5]), 'local_passage_velocity_constraints', 1)

    def solve(self, state, now, first):
        if first != 0: raise ValueError('Single-window planner already completed')
        began = time.perf_counter()
        deadline = float('inf') if self.cfg.max_solve_ms is None else began+self.cfg.max_solve_ms/1000.
        cfg = self.cfg; state = np.asarray(state)
        if len(state) == 9: state = np.r_[state, np.zeros(3)]
        w = self.windows[0]
        distance1 = np.linalg.norm(w.center0-state[:3]); distance2 = np.linalg.norm(self.track.goal-w.center0)
        initial1 = max(.8, distance1/6.5+.5)
        if cfg.kinematic_arrival_seed:
            direction = (w.center0-state[:3])/max(distance1, 1e-9)
            speed0 = np.clip(np.dot(state[3:6], direction), 0., cfg.speed)
            accelerate = (cfg.speed-speed0)/cfg.acceleration
            accelerate_distance = (cfg.speed**2-speed0**2)/(2*cfg.acceleration)
            if distance1 <= accelerate_distance:
                initial1 = (np.sqrt(speed0**2+2*cfg.acceleration*distance1)-speed0)/cfg.acceleration
            else:
                initial1 = accelerate+(distance1-accelerate_distance)/cfg.speed
            initial1 = max(.8, initial1+cfg.acceleration/cfg.jerk)
        initial2 = max(.8, np.sqrt(6*distance2/cfg.acceleration), (60*distance2/cfg.jerk)**(1/3))
        arrivals, omega, period = self.arrival_initialization(w, now, initial1)
        initial_velocity = -.8*cfg.speed*self.signs[0]*self.rotation[:, 2]
        initial_acceleration = (self.signs[0]*self.rotation[:, 2]
            *min(cfg.acceleration, np.dot(initial_velocity, initial_velocity)/(2*max(.1, distance2))))
        initial_derivatives = np.r_[initial_velocity, initial_acceleration] if cfg.free_crossing_acceleration else initial_velocity
        seeds = [np.r_[np.log([max(.3, arrival), initial2*cfg.initial_tail_scale]), 0., 0., initial_derivatives]
                 for arrival in arrivals]
        if self.admitted is not None: seeds.insert(0, self.admitted.x.copy())
        seeds = self.order_seeds(seeds, state, now)
        attempts = []; best = None; total_iterations = 0
        base_offsets = np.array(getattr(cfg, 'passage_seed_offsets',
                                       (0., .02, .04, .06, .08, .10, .14, .20)))
        for seed in seeds:
            if time.perf_counter() >= deadline: break
            x = np.clip(seed, self.lower, self.upper)
            basin_lower, basin_upper = self.lower.copy(), self.upper.copy()
            seed_arrival = now+np.exp(seed[0])
            lower_t, upper_t = self.arrival_bounds(w, now, seed_arrival, omega, period)
            basin_lower[0], basin_upper[0] = np.log([lower_t, upper_t])
            x = np.clip(x, basin_lower, basin_upper)
            offsets = np.tile(np.r_[base_offsets, np.zeros(cfg.witness_slots-len(base_offsets))], (2, 1))
            next_slots = [len(base_offsets)]*2
            round_records = []
            for outer in range(cfg.exchange_rounds):
                if time.perf_counter() >= deadline: break
                par = np.r_[now, state, offsets.ravel()]
                cached_x = None; cached = None; last = x.copy(); origin = x.copy()
                scaling = np.r_[1/max(1., omega*np.exp(x[0])), 1., 1., 1., np.full(3, cfg.speed),
                    np.full(3, cfg.acceleration) if cfg.free_crossing_acceleration else np.empty(0)]

                def evaluate(z):
                    nonlocal cached_x, cached
                    if getattr(cfg, 'strict_evaluation_deadline', False) and time.perf_counter() >= deadline:
                        raise BudgetExpired()
                    if cached_x is None or not np.array_equal(z, cached_x):
                        cached_x = z.copy(); cached = self.function(z, par)
                    return cached

                def objective(y):
                    out = evaluate(origin+scaling*y)
                    if cfg.log_cost_scaling:
                        value = 1000*float(out[0])
                        return np.log1p(value), np.asarray(out[1]).ravel()*scaling*(1000/(1+value))
                    return float(out[0]), np.asarray(out[1]).ravel()*scaling

                def constraint(y): return np.asarray(evaluate(origin+scaling*y)[2]).ravel()
                def jacobian(y): return np.asarray(evaluate(origin+scaling*y)[3])*scaling

                def callback(y):
                    nonlocal last
                    last = origin+scaling*y
                    if time.perf_counter() >= deadline: raise BudgetExpired()

                try:
                    result = minimize(objective, np.zeros(self.nvar), method='SLSQP', jac=True,
                        bounds=list(zip((basin_lower-origin)/scaling, (basin_upper-origin)/scaling)),
                        constraints={'type': 'ineq', 'fun': constraint, 'jac': jacobian},
                        callback=callback, options={'maxiter': cfg.constrained_iterations,
                                                   'ftol': 1e-7, 'disp': False})
                    x = origin+scaling*result.x; total_iterations += int(result.nit)
                    status = str(result.message); converged = bool(result.success)
                except BudgetExpired:
                    x = last; status = 'wall_budget'; converged = False
                    if getattr(cfg, 'strict_evaluation_deadline', False): break
                out = self.function(x, par); plan = self._plan(x, now, out)
                parts = [self.local_passage(plan, w, piece) for piece in range(2)]
                verified = dict(passed=all(p['passed'] for p in parts), parts=parts,
                    outcome='numerically_bounded' if all(p['passed'] for p in parts) else 'violated_or_unresolved',
                    minimum_sample_distance_m=min(p['minimum_sample_distance_m'] for p in parts),
                    boxes=sum(p['boxes'] for p in parts),
                    overlap_intervals=[interval for p in parts for interval in p['overlap_intervals']],
                    evidence='floating-point Bernstein/sine enclosures, not a formal certificate')
                record = dict(round=outer, status=status, variables=x.tolist(), objective=1000*float(out[0]),
                              minimum_witness_residual=float(np.min(np.asarray(out[2]))), passage=verified)
                round_records.append(record)
                if converged and verified['passed'] and record['minimum_witness_residual'] >= -1e-5:
                    best = 1000*float(out[0]), plan, verified
                    break
                added = False
                for piece, part in enumerate(parts):
                    if part['passed'] or part['witness_u'] is None: continue
                    u = part['witness_u']
                    offset = ((1-u) if piece == 0 else u)*plan.durations[piece]
                    slot = next_slots[piece]
                    if slot >= cfg.witness_slots: continue
                    offsets[piece, slot] = offset; next_slots[piece] += 1; added = True
                if not added: break
            attempts.append(dict(seed=seed.tolist(), rounds=round_records))
            if best is not None: break
        elapsed = 1000*(time.perf_counter()-began)
        failure_status = 'no_converged_safe_passage' if cfg.max_solve_ms is None else 'no_safe_passage_under_budget'
        stats = dict(iter_count=total_iterations, optimization_ms=elapsed, attempts=attempts,
                     wall_time_limit_ms=cfg.max_solve_ms,
                     return_status='safe_passage_found' if best else failure_status,
                     converged=best is not None, passage_model='sphere versus moving aperture outside half-planes',
                     formal_certificate=False, candidate_policy='first_converged_locally_safe',
                     interior_velocity_free=True)
        stats['interior_acceleration_free'] = cfg.free_crossing_acceleration
        self.solvers[0][0].value = stats
        if best is None: raise RuntimeError(failure_status+': '+str(elapsed))
        value, plan, verified = best
        stats.update(final_objective=value, passage=verified)
        plan.optimizer_stats = stats; plan.solve_ms = elapsed
        plan.event_valid = True; plan.event_reason = 'local_passage_numerically_bounded'
        plan.terminal_gate = None
        return plan
