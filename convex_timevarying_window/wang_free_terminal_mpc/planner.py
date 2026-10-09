"""Free local terminal PVAJ extension; preserve the old fixed-tail implementation.

N=2/M=4, degree-five MINCO, same objective/admission/MPC. Only the local
terminal state and its initialization differ. No claim of official Wang code.
"""
import math
import casadi as ca
import numpy as np
from convex_timevarying_window.wang_paper_aligned_mpc.planner import Planner as BasePlanner, Config
from convex_timevarying_window.wang_paper_aligned_mpc.trajectory import coefficients
from convex_timevarying_window.togt_minco_mpc.planner import ScalarSolver
from convex_timevarying_window.checked_soft_racing.planner import clearance_penalty
from convex_timevarying_window.receding_gate_racing.planner import symbolic_pose, symbolic_clearance_sq
from convex_timevarying_window.conditional_dual_constraint_cem.safety_penalty import polygon_halfspaces
from 复现.polynomial_dynamic_racing_2023.minco import basis
from .trajectory import from_terminal_jerk


class Planner(BasePlanner):
    # Original internal-position slot 23:26 becomes the last incoming jerk.
    jerk_index = 23

    def __init__(self, track, cfg=Config(), *, build_indices=None):
        if cfg.pieces_per_leg != 4 or cfg.horizon != 1:
            raise ValueError('This extension requires N=2/M=4')
        super().__init__(track, cfg, build_indices=build_indices)

    def variable_count(self, first):
        return 28 if self.goal_in_horizon(first) else 37

    def terminal_state(self, first):
        """Initialization only for local horizons; physical mission goal stays fixed."""
        seed = super().terminal_state(first).copy()
        if not self.goal_in_horizon(first) and self.cfg.workspace_lower is not None:
            seed[:, 0] = np.clip(seed[:, 0],
                np.array(self.cfg.workspace_lower)+self.cfg.tracking_margin,
                np.array(self.cfg.workspace_upper)-self.cfg.tracking_margin)
        return seed

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
                final = (ca.DM(self.terminal_state(first)) if self.goal_in_horizon(first)
                         else ca.horzcat(10*x[28:31], cfg.speed*x[31:34], cfg.acceleration*x[34:37]))
                position, velocity, acceleration = final[:, 0], final[:, 1], final[:, 2]
            tail = ca.horzcat(position, velocity, acceleration)
            middle = ca.reshape(10*x[mi+3*(M-1)*k:mi+3*(M-1)*(k+1)], 3, M-1)
            C = (coefficients(head, tail, middle, T, M) if k < gates else
                 from_terminal_jerk(head, tail, middle[:, :M-2], cfg.jerk*x[self.jerk_index:self.jerk_index+3], T, M))
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
        lo[self.jerk_index:self.jerk_index+3] = [-5.]*3
        hi[self.jerk_index:self.jerk_index+3] = [5.]*3
        if not self.goal_in_horizon(first):
            lo += [-10.]*3+[-5.]*6
            hi += [10.]*3+[5.]*6
        return ScalarSolver(fg, None, cfg), evaluate, np.array(lo), np.array(hi), np.array(lower), np.array(upper)

    def initial(self, state, now, first):
        x = super().initial(state, now, first)
        old = self.admitted
        warm = (self.cfg.state_warm_start and old is not None and
                old.first_gate == first and self.seed_provenance[-1]['state_source'] == 'admitted')
        tail = (np.column_stack([old.sample(old.end_time, d) for d in range(3)])
                if warm else self.terminal_state(first))
        if self.goal_in_horizon(first):
            tail = self.terminal_state(first)
        else:
            x[28:31], x[31:34], x[34:37] = tail[:, 0]/10, tail[:, 1]/self.cfg.speed, tail[:, 2]/self.cfg.acceleration
        at = now+np.exp(x[0])
        center, R = self.windows[first].state_at(at)[:2]
        head = np.column_stack([center+R[:, :2]@x[26:28], self.cfg.speed*x[2:5], self.cfg.acceleration*x[5:8]])
        T = np.exp(x[1])
        # The old initializer supplies three internal points. Compute the jerk
        # of THAT curve before replacing its last internal-point coordinate.
        C = np.asarray(coefficients(head, tail, (10*x[17:26]).reshape(3, 3).T, T, 4)).reshape(4, 6, 3)
        jerk = old.sample(old.end_time, 3) if warm else basis(1., 3)@C[-1]/(T/4)**3
        x[self.jerk_index:self.jerk_index+3] = jerk/self.cfg.jerk
        self.seed_provenance[-1]['terminal_state_source'] = 'admitted' if warm else 'global_seed_only'
        return x
