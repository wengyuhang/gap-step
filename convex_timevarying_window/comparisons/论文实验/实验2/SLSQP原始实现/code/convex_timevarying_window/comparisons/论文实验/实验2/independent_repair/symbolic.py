"""Exact sparse expression of the original finite exchange constraints."""
import time
import casadi as ca
import numpy as np
from convex_timevarying_window.dynamic_passage_velocity_minco.hard_dynamics import KF
from convex_timevarying_window.dynamic_passage_velocity_minco.hard_dynamics import constraints as physical_constraints
from convex_timevarying_window.global_gate_minco_probe.trajectory import full_state_coefficients
from convex_timevarying_window.togt_minco_mpc.trajectory import basis
from ..safety import symbolic_constraints


def physical_values(p):
    z = ca.MX.sym('piece_values_only', 25)
    coefficients = ca.reshape(z[:24], 8, 3)
    def state(piece, u, derivative=0):
        return coefficients.T@basis(u, derivative)/z[24]**derivative
    values = physical_constraints(state, ca.vertcat(z[24]), p.cfg.nodes_per_piece,
        p.cfg.workspace_lower, p.cfg.workspace_upper, thrust_fraction=p.cfg.physical_thrust_fraction)
    return ca.Function('physical_values_only', [z], [ca.vertcat(*values)], {'cse': True})


def constraints(p, x, witnesses):
    times = ca.exp(x[:p.n])
    starts = ca.vertcat(0., ca.cumsum(times)[:-1])
    assert p.full_state
    head = ca.horzcat(ca.DM(p.track.start), ca.DM.zeros(3, 3))
    tail = ca.horzcat(ca.DM(p.track.goal), ca.DM.zeros(3, 3))
    state = [ca.reshape(x[p.velocity_start+3*p.m*i:p.velocity_start+3*p.m*(i+1)], 3, p.m) for i in range(3)]
    coefficients = full_state_coefficients(head, tail, p.point_function(x), *state, times)
    physical_function = physical_values(p)
    rows = []
    upper = []
    high = KF*1000**2
    rotor_span = p.cfg.physical_thrust_fraction-(KF*150**2)/high
    workspace_span = (np.array(p.cfg.workspace_upper)-p.cfg.workspace_lower)/30.
    for piece in range(p.n):
        physical = physical_function(ca.vertcat(ca.vec(coefficients[8*piece:8*(piece+1), :]), times[piece]))
        for node in range(p.cfg.nodes_per_piece+1):
            offset = node*17
            # Each original lower/upper pair becomes one two-sided bound.
            indices = (offset+np.r_[0:7, 11:14]).tolist()
            rows.append(physical[indices])
            upper.extend([np.inf]*3+[rotor_span]*4+workspace_span.tolist())
    for (gate, piece), item in sorted(witnesses.items()):
        for begin in range(0, len(item['values']), p.cfg.witness_slots):
            offsets = np.resize(item['values'][begin:begin+p.cfg.witness_slots], p.cfg.witness_slots)
            clipped = ca.fmin(ca.DM(offsets), times[piece])/times[piece]
            u = 1.-clipped if item['kind'] == 'before' else clipped if item['kind'] == 'after' else ca.DM(offsets)
            unique = np.unique(offsets, return_index=True)[1]
            radius = p.cfg.radius+p.cfg.frame_thickness+p.cfg.passage_margin+p.cfg.witness_slack
            for index in sorted(unique):
                at = u[index]
                center, rotation = p.pose_function(p.track.windows[gate], starts[piece]+at*times[piece])
                q = rotation.T@(coefficients[8*piece:8*(piece+1), :].T@basis(at)-center)
                values = symbolic_constraints(p.track.windows[gate].aperture, q, radius)
                rows.append(ca.vertcat(*values))
                upper.extend([np.inf]*len(values))
    for gate, (window, sign) in enumerate(zip(p.track.windows, p.signs)):
        normal = ca.DM(-sign*window.state_at(0.)[1][:, 2])
        offset = p.velocity_start+3*gate
        rows.append(ca.dot(normal, x[offset:offset+3])-.1)
        upper.append(np.inf)
    return ca.vertcat(*rows), np.asarray(upper)


class Progress(ca.Callback):
    def __init__(self, variables, lower, upper, deadline, progress):
        self.variables = variables
        self.lower = lower
        self.upper = upper
        self.deadline = deadline
        self.progress = progress
        self.began = self.last = time.perf_counter()
        self.count = 0
        ca.Callback.__init__(self)
        self.construct('independent_ipopt_progress', {})

    def get_n_in(self):
        return ca.nlpsol_n_out()

    def get_n_out(self):
        return 1

    def get_name_in(self, index):
        return ca.nlpsol_out(index)

    def get_sparsity_in(self, index):
        name = ca.nlpsol_out(index)
        size = self.variables if name in ('x', 'lam_x') else len(self.lower) if name in ('g', 'lam_g') else 1 if name == 'f' else 0
        return ca.Sparsity.dense(size, 1)

    def get_sparsity_out(self, index):
        return ca.Sparsity.scalar()

    def eval(self, args):
        self.count += 1
        now = time.perf_counter()
        if now-self.last > 5.:
            values = np.asarray(args[2]).ravel()
            minimum = float(np.minimum(values-self.lower, self.upper-values).min())
            self.progress(dict(stage='exact_interior_point', iteration=self.count,
                elapsed_s=now-self.began, objective=float(args[1]), minimum_constraint=minimum))
            self.last = now
        return [int(now >= self.deadline)]
