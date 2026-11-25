"""Wang MINCO-s3 family, free gate PVA/incoming-J, checked safety and MPC.

The trajectory map is exactly the reproduction's constant normalized inverse.
Reuse only the existing geometry/admission/refinement and bounded scalar solver;
do not use its MINCO-s4 trajectory construction or C6/terminal-jerk conditions.
"""
from dataclasses import dataclass
import math
import time
import casadi as ca
import numpy as np
from convex_timevarying_window.togt_minco_mpc.planner import Planner as GeometryAndChecks
from convex_timevarying_window.togt_minco_mpc.planner import Config as ParentConfig, ScalarSolver
from convex_timevarying_window.checked_soft_racing.planner import clearance_penalty
from convex_timevarying_window.receding_gate_racing.planner import symbolic_pose, symbolic_clearance_sq
from convex_timevarying_window.conditional_dual_constraint_cem.safety_penalty import polygon_halfspaces
from .trajectory import Plan, from_arrival_jerk, arrival_jerk_for_middle, basis
from .events import plan_event_check


@dataclass(frozen=True)
class Config(ParentConfig):
    nodes_per_piece: int = 8
    free_points: bool = True
    min_leg_seconds: float = .12


class Planner(GeometryAndChecks):
    def __init__(self, track, cfg=Config(), *, build_indices=None):
        super().__init__(track, cfg, build_indices=build_indices)

    @staticmethod
    def offsets(gates):
        legs = gates+1
        return legs, legs+3*gates, legs+6*gates, 4*legs+6*gates

    def _build(self, first):
        cfg = self.cfg; ids = self.ids(first); gates = len(ids); legs = gates+1; n = 2*legs
        vi, ai, ji, qi = self.offsets(gates)
        x = ca.SX.sym('x', 4*legs+8*gates)
        par = ca.SX.sym('p', 13+4*n)
        Ts = ca.exp(x[:legs]); head = ca.reshape(par[1:10], 3, 3)
        coefficients = []; durations = []; starts = []; stamp = par[0]
        for k in range(legs):
            T = Ts[k]; end_time = stamp+T
            if k < gates:
                center, R = symbolic_pose(self.windows[ids[k]], end_time)
                position = center+R[:, :2] @ x[qi+2*k:qi+2*k+2]
                velocity = cfg.speed*x[vi+3*k:vi+3*k+3]
                acceleration = cfg.acceleration*x[ai+3*k:ai+3*k+3]
            else:
                position = ca.DM(self.terminal_position(first)); velocity = acceleration = ca.DM.zeros(3)
            tail = ca.horzcat(position, velocity, acceleration)
            C = from_arrival_jerk(head, tail, cfg.jerk*x[ji+3*k:ji+3*k+3], T)
            coefficients.extend([C[:6, :], C[6:, :]])
            durations.extend([T/2, T/2]); starts.extend([stamp, stamp+T/2])
            head = tail; stamp = end_time
        C = ca.vertcat(*coefficients)
        lower = []; upper = []; constraints = []
        def con(value, low, high):
            scale = max(1., abs(low) if np.isfinite(low) else 0., abs(high) if np.isfinite(high) else 0.)
            constraints.append(ca.vec(value)/scale)
            lower.extend([low/scale]*value.numel()); upper.extend([high/scale]*value.numel())
        def state(piece, u, order=0):
            # Polynomial basis supports symbolic witness positions.
            B = ca.vertcat(*[math.factorial(k)/math.factorial(k-order)*u**(k-order)
                            if k >= order else 0. for k in range(6)])
            return coefficients[piece].T @ B / durations[piece]**order
        def frame_ids(piece):
            k = piece//2
            if k < gates: return range(max(0, ids[k]-1), min(len(self.frames), ids[k]+1))
            return range(max(0, first+gates-1), min(len(self.frames), first+gates+1))
        hard = cfg.radius+cfg.frame_thickness+cfg.tracking_margin
        objective = ca.sum1(Ts)
        for k, wi in enumerate(ids):
            w = self.windows[wi]; q = x[qi+2*k:qi+2*k+2]; inset = hard-cfg.frame_offset
            if w.aperture.kind == 'circle': con(ca.sumsqr(q), 0., (w.aperture.radius-inset)**2)
            else:
                hs = polygon_halfspaces(w.aperture.physical_vertices)
                for normal, offset in zip(hs.normals, hs.offsets): con(ca.dot(normal, q), -np.inf, offset-inset)
            tv = ca.SX.sym('tv'); center, R = symbolic_pose(w, tv)
            rates = ca.Function(f'wang_rates_{first}_{k}', [tv],
                [ca.jacobian(center, tv), ca.reshape(ca.jacobian(ca.vec(R), tv), 3, 3)])
            at = starts[2*k+1]+durations[2*k+1]; center, R = symbolic_pose(w, at); dc, dR = rates(at)
            pos, vel = state(2*k+1, 1.), state(2*k+1, 1., 1)
            con(-self.signs[wi]*(ca.dot(R[:, 2], vel-dc)+ca.dot(dR[:, 2], pos-center)), .1, np.inf)
            for offset in (.04, .08, .16, .28, .42):
                for piece, endpoint, sign in ((2*k+1, 1., 1.), (2*k+2, 0., -1.)):
                    fraction = ca.fmin(.45, offset/durations[piece]); u = endpoint-sign*fraction
                    t = starts[piece]+u*durations[piece]; cc, rr = symbolic_pose(w, t)
                    distance = hard*ca.fmin(1., fraction*durations[piece]/.28)
                    con(sign*self.signs[wi]*ca.dot(rr[:, 2], state(piece, u)-cc)-distance, 0., np.inf)
        for piece in range(n):
            # Exact minimum-jerk integral, as in the Wang reproduction.
            cc = coefficients[piece]
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
                    for axis in range(3): con(pos[axis], cfg.workspace_lower[axis]+cfg.tracking_margin, cfg.workspace_upper[axis]-cfg.tracking_margin)
                at = starts[piece]+u*durations[piece]
                for gj in frame_ids(piece): con(symbolic_clearance_sq(self.frames[gj], pos, at), hard**2, np.inf)
                if index <= cfg.nodes_per_piece:
                    weight = durations[piece]/cfg.nodes_per_piece*(.5 if index in (0, cfg.nodes_per_piece) else 1.)
                    objective += weight*sum(clearance_penalty(symbolic_clearance_sq(self.frames[gj], pos, at), cfg) for gj in frame_ids(piece))
        gg = ca.vertcat(*constraints); violations = []
        for i, (low, high) in enumerate(zip(lower, upper)):
            if np.isfinite(low): violations.append(ca.fmax(low-gg[i], 0.))
            if np.isfinite(high): violations.append(ca.fmax(gg[i]-high, 0.))
        vv = ca.vertcat(*violations); cost = objective+cfg.constraint_penalty*ca.sum1(vv**3)
        fg = ca.Function(f'wang_free_fg_{first}', [x, par], [cost, ca.gradient(cost, x), ca.mmax(vv)])
        evaluate = ca.Function(f'wang_free_out_{first}', [x, par], [gg, C, objective])
        bound = 2. if cfg.free_points else 0.
        lo = [np.log(cfg.min_leg_seconds)]*legs+[-5.]*(6*gates+3*legs)+[-bound]*(2*gates)
        hi = [np.log(15.)]*legs+[5.]*(6*gates+3*legs)+[bound]*(2*gates)
        return ScalarSolver(fg, None, cfg), evaluate, np.array(lo), np.array(hi), np.array(lower), np.array(upper)

    def initial(self, state, now, first):
        searched = self.search.get(first)
        if searched is not None and (self.admitted is None or self.admitted.first_gate != first): return searched.x.copy()
        ids = self.ids(first); gates = len(ids); legs = gates+1; vi, ai, ji, qi = self.offsets(gates)
        x = np.zeros(4*legs+8*gates); stamp = now; head = np.asarray(state[:9]).reshape(3, 3)
        old = self.admitted; old_ids = self.ids(old.first_gate) if old is not None else []
        for k in range(legs):
            terminal = k == gates
            old_k = old_ids.index(ids[k]) if not terminal and ids[k] in old_ids else None
            same_terminal = terminal and old is not None and old.first_gate == first
            end = old.end_time if same_terminal else old.crossing_times[old_k] if old_k is not None else None
            target = self.terminal_position(first) if terminal else self.windows[ids[k]].state_at(stamp)[0]
            q = np.zeros(2)
            if old_k is not None and self.cfg.free_points:
                old_qi = self.offsets(len(old_ids))[3]; q = old.x[old_qi+2*old_k:old_qi+2*old_k+2]
            if k == 0 and not terminal and end is not None and end-stamp <= self.cfg.min_leg_seconds:
                raise ValueError('passage_already_committed: keep its checked continuation until body exit')
            reuse = end is not None and end > stamp
            if reuse: T = float(end-stamp)
            else:
                distance = np.linalg.norm(target-head[0])
                T = max(.8, distance/6., np.sqrt(3*distance/self.cfg.acceleration))
            if not terminal:
                center, R = self.windows[ids[k]].state_at(stamp+T)[:2]
                target = center+R[:, :2]@q
            if reuse:
                velocity = old.sample(end, 1); acceleration = old.sample(end, 2)
                middle = old.sample(stamp+.5*T)
            else:
                nxt = self.windows[ids[k]+1].center0 if not terminal and ids[k]+1 < len(self.windows) else self.track.goal
                direction = np.asarray(nxt)-head[0]
                velocity = direction/max(np.linalg.norm(direction), 1e-6)*(.45*self.cfg.speed)
                acceleration = np.zeros(3); middle = .5*(head[0]+target)
            if terminal: velocity = np.zeros(3); acceleration = np.zeros(3)
            tail = np.array([target, velocity, acceleration])
            x[k] = np.log(T)
            if not terminal:
                x[vi+3*k:vi+3*k+3] = velocity/self.cfg.speed
                x[ai+3*k:ai+3*k+3] = acceleration/self.cfg.acceleration
                x[qi+2*k:qi+2*k+2] = q
            x[ji+3*k:ji+3*k+3] = arrival_jerk_for_middle(head, tail, middle, T)/self.cfg.jerk
            head = tail; stamp += T
        return x

    def solve(self, state, now, first):
        began = time.perf_counter(); state = np.asarray(state)
        if len(state) == 9: state = np.r_[state, np.zeros(3)]
        solver, evaluate, lbx, ubx, lbg, ubg = self.solvers[first]
        seed = self.initial(state, now, first); gates = len(self.ids(first)); legs = gates+1; n = 2*legs
        old = self.search.get(first)
        nodes = self.refine(old) if old is not None else np.tile([.15, .35, .65, .85], n)
        par = np.r_[now, state, nodes]
        result = solver(x0=np.clip(seed, lbx, ubx), p=par, lbx=lbx, ubx=ubx, lbg=lbg, ubg=ubg)
        x = np.asarray(result['x']).ravel(); g, C, _ = evaluate(x, par)
        violation = float(np.maximum(np.maximum(lbg-np.asarray(g).ravel(), np.asarray(g).ravel()-ubg), 0.).max())
        durations = np.repeat(np.exp(x[:legs])/2, 2)
        plan = Plan(now, first, durations, np.asarray(C).reshape(n, 6, 3), x, 0., violation,
                    solver.stats()['return_status'], gates)
        self.search[first] = plan
        plan.event_valid, plan.event_reason = False, 'optimizer_residual'
        if violation < .003:
            plan.event_valid, plan.event_reason = plan_event_check(plan, self.windows, self.signs, self.cfg)
            if plan.event_valid: plan.clearance = self.validate(plan)
        plan.solve_ms = 1000*(time.perf_counter()-began)
        return plan
