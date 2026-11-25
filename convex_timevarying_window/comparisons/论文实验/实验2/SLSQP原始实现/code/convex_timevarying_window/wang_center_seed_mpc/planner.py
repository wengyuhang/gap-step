"""Wang MINCO, three free-state gate endpoints, center seeds, no local stop."""
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


@dataclass(frozen=True)
class Config(ParentConfig):
    horizon: int = 3


class Planner(Geometry):
    def __init__(self, track, cfg=Config(), *, build_indices=None):
        super().__init__(track, cfg, build_indices=build_indices)

    def goal_in_horizon(self, first):
        ids = self.ids(first)
        return not ids or ids[-1] == len(self.windows)-1

    def layout(self, first):
        gates = len(self.ids(first)); legs = gates+int(self.goal_in_horizon(first))
        return gates, legs, legs, legs+3*gates, legs+6*gates, 4*legs+6*gates

    def terminal_position(self, first):
        if not self.goal_in_horizon(first):
            raise ValueError('No intermediate stationary terminal in this method')
        return np.asarray(self.track.goal)

    def _build(self, first):
        cfg = self.cfg; ids = self.ids(first)
        gates, legs, vi, ai, mi, qi = self.layout(first); n = 2*legs
        x = ca.SX.sym('x', 4*legs+8*gates); par = ca.SX.sym('p', 13+4*n)
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
                position = ca.DM(self.track.goal); velocity = acceleration = ca.DM.zeros(3)
            tail = ca.horzcat(position, velocity, acceleration)
            C = coefficients(head, tail, 10*x[mi+3*k:mi+3*k+3], T)
            blocks.extend([C[:6, :], C[6:, :]])
            durations.extend([T/2, T/2]); starts.extend([stamp, stamp+T/2])
            head = tail; stamp = end
        constraints = []; lower = []; upper = []

        def con(value, low, high):
            scale = max(1., abs(low) if np.isfinite(low) else 0., abs(high) if np.isfinite(high) else 0.)
            constraints.append(ca.vec(value)/scale)
            lower.extend([low/scale]*value.numel()); upper.extend([high/scale]*value.numel())

        def state(piece, u, order=0):
            B = ca.vertcat(*[math.factorial(k)/math.factorial(k-order)*u**(k-order)
                if k>=order else 0. for k in range(6)])
            return blocks[piece].T@B/durations[piece]**order

        def frame_ids(piece):
            k = piece//2
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
            at = starts[2*k+1]+durations[2*k+1]; center, R = symbolic_pose(w, at); dc, dR = rates(at)
            pos, vel = state(2*k+1, 1.), state(2*k+1, 1., 1)
            con(-self.signs[wi]*(ca.dot(R[:, 2], vel-dc)+ca.dot(dR[:, 2], pos-center)), .1, np.inf)
            for offset in (.04, .08, .16, .28, .42):
                sides = [(2*k+1, 1., 1.)]
                if 2*k+2 < n: sides.append((2*k+2, 0., -1.))
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
                    objective += cfg.jerk_weight*weight*ca.dot(cc[i, :], cc[j, :])/durations[piece]**5
            nodes = list(np.linspace(0., 1., cfg.nodes_per_piece+1))
            nodes += list(ca.vertsplit(par[13+4*piece:13+4*piece+4]))
            for index, u in enumerate(nodes):
                pos, vel, acc, jerk = [state(piece, u, d) for d in range(4)]
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
        gg = ca.vertcat(*constraints); violations = []
        for i, (low, high) in enumerate(zip(lower, upper)):
            if np.isfinite(low): violations.append(ca.fmax(low-gg[i], 0.))
            if np.isfinite(high): violations.append(ca.fmax(gg[i]-high, 0.))
        vv = ca.vertcat(*violations); cost = objective+cfg.constraint_penalty*ca.sum1(vv**3)
        fg = ca.Function(f'center_fg_{first}', [x, par], [cost, ca.gradient(cost, x), ca.mmax(vv)])
        evaluate = ca.Function(f'center_out_{first}', [x, par], [gg, ca.vertcat(*blocks), objective])
        bound = 2. if cfg.free_points else 0.
        lo = [np.log(cfg.min_leg_seconds)]*legs+[-5.]*(6*gates)+[-10.]*(3*legs)+[-bound]*(2*gates)
        hi = [np.log(15.)]*legs+[5.]*(6*gates)+[10.]*(3*legs)+[bound]*(2*gates)
        return ScalarSolver(fg, None, cfg), evaluate, np.array(lo), np.array(hi), np.array(lower), np.array(upper)

    def initial(self, state, now, first):
        ids = self.ids(first); gates, legs, vi, ai, mi, qi = self.layout(first)
        x = np.zeros(4*legs+8*gates); source = np.asarray(state[:3]); stamp = now
        old = self.admitted
        old_times = {} if old is None else dict(zip(range(old.first_gate, old.first_gate+old.gate_count), old.crossing_times))
        for k in range(legs):
            terminal = k == gates
            target = np.asarray(self.track.goal) if terminal else self.windows[ids[k]].state_at(stamp)[0]
            end = old_times.get(ids[k]) if not terminal else None
            if k == 0 and end is not None and end-stamp <= self.cfg.min_leg_seconds:
                raise ValueError('passage_already_committed: keep flying until body exit')
            if end is not None and end > stamp:
                T = float(end-stamp)
            else:
                distance = np.linalg.norm(target-source)
                T = max(.8, distance/6., np.sqrt(3*distance/self.cfg.acceleration))
            stamp += T
            if not terminal:
                target = self.windows[ids[k]].state_at(stamp)[0]
                nxt = self.windows[ids[k]+1].center0 if ids[k]+1<len(self.windows) else self.track.goal
                direction = np.asarray(nxt)-source
                x[vi+3*k:vi+3*k+3] = direction/max(np.linalg.norm(direction), 1e-6)*.45
            # q=0 and a=0 for every window on every call, even with caches.
            x[k] = np.log(T); x[mi+3*k:mi+3*k+3] = .5*(source+target)/10
            source = target
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
        clear = float(frame_clearances(self.frames, p, tt, cfg.radius, cfg.frame_thickness).min())
        if cfg.workspace_lower is not None:
            clear = min(clear, float((p-np.array(cfg.workspace_lower)).min()), float((np.array(cfg.workspace_upper)-p).min()))
        return clear

    def solve(self, state, now, first):
        began = time.perf_counter(); state = np.asarray(state)
        if len(state)==9: state = np.r_[state, np.zeros(3)]
        solver, evaluate, lbx, ubx, lbg, ubg = self.solvers[first]
        gates, legs, *_ = self.layout(first); n = 2*legs
        seed = np.clip(self.initial(state, now, first), lbx, ubx)
        old = self.search.get(first)
        nodes = self.refine(old) if old is not None else np.tile([.15, .35, .65, .85], n)
        par = np.r_[now, state, nodes]
        result = solver(x0=seed, p=par, lbx=lbx, ubx=ubx, lbg=lbg, ubg=ubg)
        x = np.asarray(result['x']).ravel(); g, C, objective = evaluate(x, par)
        violation = float(np.maximum(np.maximum(lbg-np.asarray(g).ravel(), np.asarray(g).ravel()-ubg), 0.).max())
        plan = Plan(now, first, np.repeat(np.exp(x[:legs])/2, 2), np.asarray(C).reshape(n, 6, 3), x,
            0., violation, solver.stats()['return_status'], gates, terminal_goal=self.goal_in_horizon(first))
        plan.initial_variables = seed; plan.objective = float(objective)
        self.search[first] = plan
        plan.event_valid, plan.event_reason = False, 'optimizer_residual'
        if violation < .003:
            plan.event_valid, plan.event_reason = plan_event_check(plan, self.windows, self.signs, self.cfg)
            if plan.event_valid: plan.clearance = self.validate(plan)
        plan.solve_ms = 1000*(time.perf_counter()-began)
        return plan
