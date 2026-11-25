"""Paper-specified MINCO core with explicit x500/task adaptations.

Published: s=3, M=4, time-uniform pieces, exact jerk energy, Eq.21
quadrature and Eq.23 actuator penalties. Independent implementation, not
an official reproduction. See ALIGNMENT.md for non-paper components.
"""
from dataclasses import dataclass
import math
import time
import casadi as ca
import numpy as np

from convex_timevarying_window.wang_free_state_mpc.planner import Config as ParentConfig
from convex_timevarying_window.togt_minco_mpc.planner import Planner as Geometry, ScalarSolver
from convex_timevarying_window.checked_soft_racing.planner import clearance_penalty
from convex_timevarying_window.receding_gate_racing.planner import symbolic_pose, symbolic_clearance_sq
from convex_timevarying_window.continuation_racing.guard import frame_clearances
from convex_timevarying_window.conditional_dual_constraint_cem.safety_penalty import polygon_halfspaces
from .trajectory import Plan, coefficients
from .events import plan_event_check
from .initialization import GlobalSeed


@dataclass(frozen=True)
class Config(ParentConfig):
    horizon: int = 1  # N=2 segments: next gate, then fixed forward PVA.
    pieces_per_leg: int = 4
    state_warm_start: bool = True
    time_weight: float = 1./.0006  # Shared x500 choice; not an unpublished paper default.
    lambda_thrust: float = 100.
    lambda_body_rate: float = 100.
    body_rate: float = 3.
    soft_clearance_weight: float = 0.


class Planner(Geometry):
    def __init__(self, track, cfg=Config(), *, build_indices=None):
        self.global_seed = GlobalSeed(track, [track.windows[i] for i in track.order], cfg)
        super().__init__(track, cfg, build_indices=range(len(track.order)) if build_indices is None else build_indices)

    def goal_in_horizon(self, first):
        ids = self.ids(first)
        return not ids or ids[-1] == len(self.windows)-1

    def layout(self, first):
        gates = len(self.ids(first)); legs = 2
        if gates != 1:
            raise ValueError('N=2 online gate planner requires one remaining gate; final return keeps its existing plan')
        return gates, legs, legs, legs+3*gates, legs+6*gates, legs+6*gates+3*(self.cfg.pieces_per_leg-1)*legs

    def variable_count(self, first):
        gates, _, _, _, _, qi = self.layout(first)
        return qi+2*gates

    def terminal_position(self, first):
        return self.terminal_state(first)[:, 0]

    def terminal_state(self, first):
        if self.goal_in_horizon(first):
            return np.column_stack([self.track.goal, np.zeros((3, 2))])
        at = (first+1.5)*self.global_seed.step
        return np.column_stack([self.global_seed.reference.sample(at, d) for d in range(3)])

    def _build(self, first):
        cfg = self.cfg; ids = self.ids(first)
        gates, legs, vi, ai, mi, qi = self.layout(first); M = cfg.pieces_per_leg; n = M*legs
        x = ca.SX.sym('x', self.variable_count(first)); par = ca.SX.sym('p', 13+4*n)
        Ts = ca.exp(x[:legs]); head = ca.reshape(par[1:10], 3, 3)
        blocks = []; durations = []; starts = []; stamp = par[0]
        for k in range(legs):
            T = Ts[k]; end = stamp+T
            if k < gates:
                center, R = symbolic_pose(self.windows[ids[k]], end)
                position = center+R[:, :2]@x[qi+2*k:qi+2*k+2]
                velocity = cfg.speed*x[vi+3*k:vi+3*k+3]
                acceleration = cfg.acceleration*x[ai+3*k:ai+3*k+3]
            else:
                final = ca.DM(self.terminal_state(first))
                position, velocity, acceleration = final[:, 0], final[:, 1], final[:, 2]
            tail = ca.horzcat(position, velocity, acceleration)
            middle = ca.reshape(10*x[mi+3*(M-1)*k:mi+3*(M-1)*(k+1)], 3, M-1)
            C = coefficients(head, tail, middle, T, M)
            blocks.extend([C[6*j:6*j+6, :] for j in range(M)])
            durations.extend([T/M]*M); starts.extend([stamp+j*T/M for j in range(M)])
            head = tail; stamp = end
        constraints = []; lower = []; upper = []; weights = []; quadrature_weight = 1.

        def con(value, low, high):
            scale = max(1., abs(low) if np.isfinite(low) else 0., abs(high) if np.isfinite(high) else 0.)
            constraints.append(ca.vec(value)/scale)
            weights.extend([quadrature_weight]*value.numel())
            lower.extend([low/scale]*value.numel()); upper.extend([high/scale]*value.numel())

        def state(piece, u, order=0):
            B = ca.vertcat(*[math.factorial(k)/math.factorial(k-order)*u**(k-order)
                if k>=order else 0. for k in range(6)])
            return blocks[piece].T@B/durations[piece]**order

        def frame_ids(piece):
            k = piece//M
            if k < gates:
                return range(max(0, ids[k]-1), min(len(self.frames), ids[k]+1))
            return range(max(0, first+gates-1), min(len(self.frames), first+gates+1))

        hard = cfg.radius+cfg.frame_thickness+cfg.tracking_margin
        objective = ca.sum1(Ts)
        for k, wi in enumerate(ids):
            w = self.windows[wi]; q = x[qi+2*k:qi+2*k+2]; inset = hard-cfg.frame_offset
            if w.aperture.kind == 'circle':
                con(ca.sumsqr(q), 0., (w.aperture.radius-inset)**2)
            else:
                hs = polygon_halfspaces(w.aperture.physical_vertices)
                for normal, offset in zip(hs.normals, hs.offsets):
                    con(ca.dot(normal, q), -np.inf, offset-inset)
            tv = ca.SX.sym('tv'); center, R = symbolic_pose(w, tv)
            rates = ca.Function(f'center_rates_{first}_{k}', [tv],
                [ca.jacobian(center, tv), ca.reshape(ca.jacobian(ca.vec(R), tv), 3, 3)])
            at = starts[M*k+M-1]+durations[M*k+M-1]; center, R = symbolic_pose(w, at); dc, dR = rates(at)
            pos, vel = state(M*k+M-1, 1.), state(M*k+M-1, 1., 1)
            con(-self.signs[wi]*(ca.dot(R[:, 2], vel-dc)+ca.dot(dR[:, 2], pos-center)), .1, np.inf)
            for offset in (.04, .08, .16, .28, .42):
                sides = [(M*k+M-1, 1., 1.)]
                if M*(k+1) < n: sides.append((M*(k+1), 0., -1.))
                for piece, endpoint, sign in sides:
                    fraction = ca.fmin(.45, offset/durations[piece]); u = endpoint-sign*fraction
                    t = starts[piece]+u*durations[piece]; cc, rr = symbolic_pose(w, t)
                    distance = hard*ca.fmin(1., fraction*durations[piece]/.28)
                    con(sign*self.signs[wi]*ca.dot(rr[:, 2], state(piece, u)-cc)-distance, 0., np.inf)
        for piece in range(n):
            cc = blocks[piece]
            for i in range(3, 6):
                for j in range(3, 6):
                    weight = math.factorial(i)/math.factorial(i-3)*math.factorial(j)/math.factorial(j-3)/(i+j-5)
                    objective += (1./cfg.time_weight)*weight*ca.dot(cc[i, :], cc[j, :])/durations[piece]**5
            nodes = list(np.linspace(0., 1., cfg.nodes_per_piece+1))
            # Uniform quadrature only: no hidden changing witness weights.
            for index, u in enumerate(nodes):
                quadrature_weight = durations[piece]/cfg.nodes_per_piece*(.5 if index in (0, cfg.nodes_per_piece) else 1.)
                pos, vel, acc, jerk = [state(piece, u, d) for d in range(4)]
                nominal_force = acc+ca.DM([0., 0., 9.8066])
                fn = ca.sqrt(ca.sumsqr(nominal_force)+1e-12)
                specific_thrust = fn
                body_rates = ca.cross(nominal_force/fn, jerk)/fn
                con(ca.sumsqr(body_rates), 0., cfg.body_rate**2)
                # Eq.23 with x500 force limits (N); zero-drag flatness adaptation.
                mass = 2.0643076923
                fmin, fmax = 2.*mass, 4*8.54858*.9
                gt = (mass*specific_thrust-(fmax+fmin)/2)**2-((fmax-fmin)/2)**2
                gb = ca.sumsqr(body_rates)-cfg.body_rate**2
                objective += quadrature_weight/cfg.time_weight*(cfg.lambda_thrust*ca.fmax(gt, 0.)**3+cfg.lambda_body_rate*ca.fmax(gb, 0.)**3)
                demand = acc+cfg.response_time*jerk; thrust = demand+ca.DM([0., 0., 9.8066])
                for value, limit in ((vel, cfg.speed), (acc, cfg.acceleration), (jerk, cfg.jerk), (demand, cfg.acceleration)):
                    con(ca.sumsqr(value), 0., limit**2)
                con(ca.sumsqr(thrust), 4., (4*8.54858/2.0643076923*.9)**2)
                con((ca.sumsqr(thrust[:2])-(math.tan(math.radians(42))*thrust[2])**2)/9.8066**2, -np.inf, 0.)
                if cfg.workspace_lower is not None:
                    for axis in range(3):
                        con(pos[axis], cfg.workspace_lower[axis]+cfg.tracking_margin, cfg.workspace_upper[axis]-cfg.tracking_margin)
                at = starts[piece]+u*durations[piece]
                for gj in frame_ids(piece):
                    con(symbolic_clearance_sq(self.frames[gj], pos, at), hard**2, np.inf)
                if index <= cfg.nodes_per_piece:
                    weight = durations[piece]/cfg.nodes_per_piece*(.5 if index in (0, cfg.nodes_per_piece) else 1.)
                    objective += weight*sum(clearance_penalty(symbolic_clearance_sq(self.frames[gj], pos, at), cfg)
                        for gj in frame_ids(piece))
        gg = ca.vertcat(*constraints); violations = []; penalty_terms = []
        for i, (low, high) in enumerate(zip(lower, upper)):
            if np.isfinite(low):
                v = ca.fmax(low-gg[i], 0.); violations.append(v); penalty_terms.append(weights[i]*v**3)
            if np.isfinite(high):
                v = ca.fmax(gg[i]-high, 0.); violations.append(v); penalty_terms.append(weights[i]*v**3)
        vv = ca.vertcat(*violations); cost = objective+cfg.constraint_penalty*sum(penalty_terms)
        fg = ca.Function(f'center_fg_{first}', [x, par], [cost, ca.gradient(cost, x), ca.mmax(vv)])
        evaluate = ca.Function(f'center_out_{first}', [x, par], [gg, ca.vertcat(*blocks), objective])
        bound = 2. if cfg.free_points else 0.
        lo = [np.log(cfg.min_leg_seconds)]*legs+[-5.]*(6*gates)+[-10.]*(3*(M-1)*legs)+[-bound]*(2*gates)
        hi = [np.log(15.)]*legs+[5.]*(6*gates)+[10.]*(3*(M-1)*legs)+[bound]*(2*gates)
        return ScalarSolver(fg, None, cfg), evaluate, np.array(lo), np.array(hi), np.array(lower), np.array(upper)

    def initial(self, state, now, first):
        """Center position seed, warm PVA derivatives and interior trajectory.

        Only executed/admitted trajectories supply absolute arrival times.
        Never reinterpret a rejected candidate's timestamps as executed motion.
        Reused segments are resampled after cropping to the NEW head timestamp.
        """
        ids = self.ids(first); gates, legs, vi, ai, mi, qi = self.layout(first)
        M = self.cfg.pieces_per_leg
        x = np.zeros(self.variable_count(first))
        source = np.asarray(state[:3]); stamp = now
        old = self.admitted
        old_times = {} if old is None or not self.cfg.state_warm_start else dict(zip(
            range(old.first_gate, old.first_gate+old.gate_count), old.crossing_times))
        provenance = []
        for k in range(legs):
            terminal = k == gates
            wi = ids[0]+1 if terminal else ids[k]
            target = self.terminal_position(first) if terminal else self.windows[wi].state_at(stamp)[0]
            end = old.end_time if terminal and self.cfg.state_warm_start and old is not None and old.first_gate == first else old_times.get(wi)
            if k == 0 and end is not None and end-stamp <= self.cfg.min_leg_seconds:
                raise ValueError('passage_already_committed: keep flying until body exit')
            if end is not None and end > stamp:
                T = float(end-stamp)
            else:
                distance = np.linalg.norm(target-source)
                T = max(.8, distance/6., np.sqrt(3*distance/self.cfg.acceleration))
            arrival = stamp+T
            if not terminal:
                target = self.windows[wi].state_at(arrival)[0]
            warm = (self.cfg.state_warm_start and old is not None and end is not None
                    and stamp >= old.made_at-1e-8 and arrival <= old.end_time+1e-8)
            if warm:
                times = stamp+T*np.arange(1, M)/M
                middle = old.sample_many(times)
                velocity, acceleration = old.sample(arrival, 1), old.sample(arrival, 2)
                origin = 'admitted'
            else:
                middle, velocity, acceleration = self.global_seed.leg(wi, source, target, T, M,
                    half=terminal and not self.goal_in_horizon(first))
                origin = 'global'
            if not terminal:
                x[vi+3*k:vi+3*k+3] = velocity/self.cfg.speed
                x[ai+3*k:ai+3*k+3] = acceleration/self.cfg.acceleration
            x[k] = np.log(T)
            x[mi+3*(M-1)*k:mi+3*(M-1)*(k+1)] = middle.ravel()/10
            provenance.append(dict(gate=None if terminal else wi, state_source=origin,
                                   old_time=None if end is None else float(end)))
            stamp = arrival; source = target
        self.seed_provenance = provenance
        # q remains zero; the optimizer can move it only in free-point mode.
        return x

    def validate(self, plan):
        # Same dynamics/geometry checks, with no intermediate swept-ball stop test.
        cfg = self.cfg
        tt = np.linspace(plan.made_at, plan.end_time, max(2, int(np.ceil((plan.end_time-plan.made_at)/.01))+1))
        p, v, a, j = [plan.sample_many(tt, d) for d in range(4)]
        demand = a+cfg.response_time*j; f = demand+np.array([0., 0., 9.8066]); tol = 1.006
        for values, limit in ((v, cfg.speed), (a, cfg.acceleration), (j, cfg.jerk), (demand, cfg.acceleration)):
            if np.linalg.norm(values, axis=1).max() > limit*tol: return -np.inf
        if np.linalg.norm(f, axis=1).max()>4*8.54858/2.0643076923*.9*tol or f[:, 2].min()<=0: return -np.inf
        if (np.linalg.norm(f[:, :2], axis=1)/f[:, 2]).max()>math.tan(math.radians(42))*tol: return -np.inf
        nominal = a+np.array([0., 0., 9.8066]); norm = np.linalg.norm(nominal, axis=1)
        omega = np.cross(nominal/norm[:, None], j)/norm[:, None]
        if np.linalg.norm(omega, axis=1).max() > cfg.body_rate*tol: return -np.inf
        clear = float(frame_clearances(self.frames, p, tt, cfg.radius, cfg.frame_thickness).min())
        if cfg.workspace_lower is not None:
            clear = min(clear, float((p-np.array(cfg.workspace_lower)).min()), float((np.array(cfg.workspace_upper)-p).min()))
        return clear

    def solve(self, state, now, first):
        began = time.perf_counter(); state = np.asarray(state)
        if len(state)==9: state = np.r_[state, np.zeros(3)]
        solver, evaluate, lbx, ubx, lbg, ubg = self.solvers[first]
        gates, legs, *_ = self.layout(first); M = self.cfg.pieces_per_leg; n = M*legs
        seed = np.clip(self.initial(state, now, first), lbx, ubx)
        old = self.search.get(first)
        nodes = np.tile([.15, .35, .65, .85], n)
        par = np.r_[now, state, nodes]
        result = solver(x0=seed, p=par, lbx=lbx, ubx=ubx, lbg=lbg, ubg=ubg)
        x = np.asarray(result['x']).ravel(); g, C, objective = evaluate(x, par)
        violation = float(np.maximum(np.maximum(lbg-np.asarray(g).ravel(), np.asarray(g).ravel()-ubg), 0.).max())
        plan = Plan(now, first, np.repeat(np.exp(x[:legs])/M, M), np.asarray(C).reshape(n, 6, 3), x,
            0., violation, solver.stats()['return_status'], gates, terminal_goal=self.goal_in_horizon(first), pieces_per_leg=M)
        plan.initial_variables = seed; plan.objective = float(objective)
        plan.seed_provenance = self.seed_provenance
        self.search[first] = plan
        plan.event_valid, plan.event_reason = False, 'optimizer_residual'
        if violation < .003:
            plan.event_valid, plan.event_reason = plan_event_check(plan, self.windows, self.signs, self.cfg)
            if plan.event_valid: plan.clearance = self.validate(plan)
        plan.solve_ms = 1000*(time.perf_counter()-began)
        return plan
