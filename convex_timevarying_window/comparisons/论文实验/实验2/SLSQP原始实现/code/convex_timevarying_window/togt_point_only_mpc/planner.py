"""Global MINCO-s4 through start, moving-gate points, and terminal point.

Only leg times and two in-plane coordinates per gate are optimized. Gate
PVAJ and all polynomial coefficients follow from the coupled MINCO solve.
The racing, dynamic, and soft frame costs match the prior soft-only method;
as in that method, a residual is recorded but is not an admission veto.
"""
from dataclasses import dataclass
import math
import time

import casadi as ca
import numpy as np

from convex_timevarying_window.checked_soft_racing.planner import clearance_penalty
from convex_timevarying_window.comparisons.original_fixed_plane_ours.adapter import (
    symbolic_clearance_sq, symbolic_pose,
)
from convex_timevarying_window.conditional_dual_constraint_cem.safety_penalty import polygon_halfspaces
from convex_timevarying_window.simple_two_gate_mpc.planner import Config as ParentConfig
from convex_timevarying_window.simple_two_gate_soft_only_mpc.planner import Planner as Parent
from convex_timevarying_window.simple_two_gate_soft_only_mpc.solver import Solver
from convex_timevarying_window.togt_minco_mpc.trajectory import basis, minco_coefficients
from .trajectory import Plan


@dataclass(frozen=True)
class Config(ParentConfig):
    # The former eight nodes on EACH half-leg become sixteen on one full leg.
    nodes_per_piece: int = 16


class Planner(Parent):
    def _build(self, first):
        cfg = self.cfg
        ids = self.ids(first)
        gates = len(ids)
        legs = gates + 1
        x = ca.MX.sym(f'point_only_x_{first}', legs + 2 * gates)
        par = ca.MX.sym(f'point_only_p_{first}', 13 + 4 * legs)
        durations = [ca.exp(x[k]) for k in range(legs)]
        starts = [par[0]]
        for duration in durations:
            starts.append(starts[-1] + duration)

        head = ca.reshape(par[1:13], 3, 4)
        positions = []
        for k, wi in enumerate(ids):
            center, R = symbolic_pose(self.windows[wi], starts[k + 1])
            positions.append(center + R[:, :2] @ x[legs + 2*k:legs + 2*k + 2])
        if self.free_terminal(first):
            endpoint, _ = symbolic_pose(self.windows[first + gates], starts[-1])
        else:
            endpoint = ca.DM(self.track.goal)
        tail = ca.horzcat(endpoint, ca.DM.zeros(3, 3))
        knots = ca.horzcat(*positions) if positions else ca.DM.zeros(3, 0)
        C = minco_coefficients(head, tail, knots, ca.vertcat(*durations))

        constraints, lower, upper = [], [], []
        def con(value, low, high):
            scale = max(1., abs(low) if np.isfinite(low) else 0.,
                        abs(high) if np.isfinite(high) else 0.)
            constraints.append(ca.vec(value)/scale)
            lower.extend([low/scale]*value.numel())
            upper.extend([high/scale]*value.numel())

        def state(piece, u, order=0):
            return C[8*piece:8*piece+8, :].T @ basis(u, order) / durations[piece]**order

        def frame_ids(piece):
            wi = ids[piece] if piece < gates else first + gates - 1
            return range(max(0, wi - 1), min(len(self.frames), wi + 1))

        hard = cfg.radius + cfg.frame_thickness + cfg.tracking_margin
        objective = ca.sum1(ca.vertcat(*durations))
        for k, wi in enumerate(ids):
            w = self.windows[wi]
            q = x[legs + 2*k:legs + 2*k + 2]
            # Keep the crossing point inside the physical aperture only.
            # Body/frame clearance is already represented by the time-varying
            # frame penalties below, including the event-centred samples.
            inset = 0.
            if w.aperture.kind == 'circle':
                con(ca.sumsqr(q), 0., (w.aperture.radius - inset)**2)
            else:
                hs = polygon_halfspaces(w.aperture.physical_vertices)
                for normal, offset in zip(hs.normals, hs.offsets):
                    con(ca.dot(normal, q), -np.inf, offset - inset)

            tv = ca.MX.sym(f'point_only_time_{first}_{k}')
            center, R = symbolic_pose(w, tv)
            rates = ca.Function(f'point_only_rates_{first}_{k}', [tv],
                [ca.jacobian(center, tv), ca.reshape(ca.jacobian(ca.vec(R), tv), 3, 3)])
            at = starts[k + 1]
            center, R = symbolic_pose(w, at)
            center_rate, R_rate = rates(at)
            pos, vel = state(k, 1.), state(k, 1., 1)
            con(-self.signs[wi] * (ca.dot(R[:, 2], vel - center_rate)
                + ca.dot(R_rate[:, 2], pos - center)), .1, np.inf)

            # Identical signed approach/departure times, on the two adjacent
            # pieces rather than the old four half-leg pieces.
            for offset in (.04, .08, .16, .28, .42):
                for piece, endpoint_u, direction in ((k, 1., 1.), (k+1, 0., -1.)):
                    fraction = ca.fmin(.45, offset/durations[piece])
                    u = endpoint_u - direction*fraction
                    t = starts[piece] + u*durations[piece]
                    cc, rr = symbolic_pose(w, t)
                    distance = hard*ca.fmin(1., fraction*durations[piece]/.28)
                    con(direction*self.signs[wi]*ca.dot(rr[:, 2], state(piece, u)-cc)-distance,
                        0., np.inf)

        for piece in range(legs):
            nodes = list(np.linspace(0., 1., cfg.nodes_per_piece+1))
            nodes += list(ca.vertsplit(par[13+4*piece:13+4*piece+4]))
            for index, u in enumerate(nodes):
                pos, vel, acc, jerk = [state(piece, u, d) for d in range(4)]
                demand = acc + cfg.response_time*jerk
                thrust = demand + ca.DM([0., 0., 9.8066])
                for value, limit in ((vel, cfg.speed), (acc, cfg.acceleration),
                                     (jerk, cfg.jerk), (demand, cfg.acceleration)):
                    con(ca.sumsqr(value), 0., limit**2)
                con(ca.sumsqr(thrust), 4., (4*8.54858/2.0643076923*.9)**2)
                con((ca.sumsqr(thrust[:2])
                     -(math.tan(math.radians(42))*thrust[2])**2)/9.8066**2,
                    -np.inf, 0.)
                if cfg.workspace_lower is not None:
                    for axis in range(3):
                        con(pos[axis], cfg.workspace_lower[axis]+cfg.tracking_margin,
                            cfg.workspace_upper[axis]-cfg.tracking_margin)
                t = starts[piece] + u*durations[piece]
                for fj in frame_ids(piece):
                    con(symbolic_clearance_sq(self.frames[fj], pos, t), hard**2, np.inf)
                if index <= cfg.nodes_per_piece:
                    weight = durations[piece]/cfg.nodes_per_piece * (
                        .5 if index in (0, cfg.nodes_per_piece) else 1.)
                    objective += weight * sum(clearance_penalty(
                        symbolic_clearance_sq(self.frames[fj], pos, t), cfg)
                        for fj in frame_ids(piece))

        # Same event-centred 20 ms soft safety quadrature as the previous
        # representation, including the interval after the centre crosses.
        count = math.ceil(cfg.crossing_half_span_s/cfg.crossing_max_step_s)
        for k, wi in enumerate(ids):
            for piece, ending in ((k, True), (k+1, False)):
                T = durations[piece]
                span = ca.fmin(cfg.crossing_half_span_s, T)
                for j in range(count+1):
                    offset = span*j/count
                    u = 1.-offset/T if ending else offset/T
                    t = starts[piece]+u*T
                    sq = symbolic_clearance_sq(self.frames[wi], state(piece, u), t)
                    con(sq, hard**2, np.inf)
                    weight = span/count*(.5 if j in (0, count) else 1.)
                    objective += weight*clearance_penalty(sq, cfg)

        g = ca.vertcat(*constraints)
        violations = []
        for i, (low, high) in enumerate(zip(lower, upper)):
            if np.isfinite(low): violations.append(ca.fmax(low-g[i], 0.))
            if np.isfinite(high): violations.append(ca.fmax(g[i]-high, 0.))
        v = ca.vertcat(*violations)
        cost = objective + cfg.constraint_penalty*ca.sum1(v**3)
        options = {'cse': True}
        if cfg.compile_objective:
            options.update(jit=True, compiler='shell', jit_options={
                'flags': [cfg.compiler_optimization], 'directory': self._jit_dir.name+'/'})
        fg = ca.Function(f'point_only_fg_{first}', [x, par],
                         [cost, ca.gradient(cost, x), ca.mmax(v)], options)
        evaluate = ca.Function(f'point_only_out_{first}', [x, par], [g, C, objective])
        bound = 2. if cfg.free_points else 0.
        lo = np.r_[np.log([cfg.min_leg_seconds]*legs), [-bound]*(2*gates)]
        hi = np.r_[np.log([15.]*legs), [bound]*(2*gates)]
        return Solver(fg, cfg), evaluate, lo, hi, np.asarray(lower), np.asarray(upper)

    def initial(self, state, now, first):
        old = self.admitted
        cfg = self.cfg
        ids = self.ids(first)
        legs = len(ids)+1
        durations, qs = [], []
        source = np.asarray(state[:3])
        stamp = now
        for k in range(legs):
            target = (self.windows[ids[k]].state_at(stamp)[0] if k < len(ids)
                      else self.track.goal if not self.free_terminal(first)
                      else self.windows[first+len(ids)].state_at(stamp)[0])
            old_k = (self.ids(old.first_gate).index(ids[k]) if
                     old is not None and k < len(ids)
                     and ids[k] in self.ids(old.first_gate) else None)
            crossing = old.crossing_times[old_k] if old_k is not None else None
            if crossing is not None and crossing > stamp+.15:
                T = float(crossing-stamp)
                old_legs = old.gate_count+1
                qs.extend(old.x[old_legs+2*old_k:old_legs+2*old_k+2])
            else:
                distance = float(np.linalg.norm(target-source))
                T = max(.8, distance/6.5 + (.5 if k == 0 else 0.))
                if k == len(ids):
                    T = max(T, np.sqrt(6*distance/cfg.acceleration),
                            (60*distance/cfg.jerk)**(1/3))
                if k < len(ids): qs.extend([0., 0.])
            durations.append(T)
            stamp += T
            source = target
        return np.r_[np.log(durations), qs]

    def solve(self, state, now, first):
        began = time.perf_counter()
        state = np.asarray(state)
        if len(state) == 9: state = np.r_[state, np.zeros(3)]
        solver, evaluate, lbx, ubx, lbg, ubg = self.solvers[first]
        seed = np.clip(self.initial(state, now, first), lbx, ubx)
        legs = len(self.ids(first))+1
        par = np.r_[now, state, np.tile([.15, .35, .65, .85], legs)]
        # The periodic window creates separate arrival-time basins. A fixed,
        # geometry-scaled seed set searches the SAME objective with no
        # scene-specific phase or gate-side choice. With four variables in the
        # single-gate case, all six starts fit the online solve budget.
        seeds = [seed]
        if self.ids(first):
            for first_scale, last_scale in ((1., 1.6), (1., 3.2),
                                            (.8, 1.6), (.8, 3.2),
                                            (1.2, 1.6), (1.2, 3.2)):
                candidate = seed.copy()
                candidate[0] += np.log(first_scale)
                candidate[legs-1] += np.log(last_scale)
                seeds.append(np.clip(candidate, lbx, ubx))
        best = None
        attempts = []
        for candidate in seeds:
            result = solver(x0=candidate, p=par, lbx=lbx, ubx=ubx,
                            lbg=lbg, ubg=ubg)
            stats = solver.stats().copy()
            attempts.append(dict(objective=stats.get('final_objective'),
                                 iterations=stats.get('iter_count'),
                                 optimization_ms=stats.get('optimization_ms')))
            if best is None or stats['final_objective'] < best[0]:
                best = (stats['final_objective'], np.asarray(result['x']).ravel(), stats)
        _, x, selected_stats = best
        solver._stats = selected_stats
        g, C, _ = evaluate(x, par)
        violation = float(np.maximum(np.maximum(lbg-np.asarray(g).ravel(),
                                                np.asarray(g).ravel()-ubg), 0.).max())
        plan = Plan(now, first, np.exp(x[:legs]),
                    np.asarray(C).reshape(legs, 8, 3), x, 0., violation,
                    solver.stats()['return_status'], len(self.ids(first)))
        self.search[first] = plan
        plan.event_valid = None
        plan.event_reason = 'not_checked_soft_only'
        plan.optimizer_stats = dict(selected_stats, multistart=attempts)
        plan.terminal_gate = first+plan.gate_count if self.free_terminal(first) else None
        plan.solve_ms = 1000*(time.perf_counter()-began)
        return plan

    def prefix_clearance(self, plan, at):
        return None
