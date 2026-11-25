"""Small segment functions for the original fixed-quadrature soft problem."""
from functools import lru_cache
from pathlib import Path

import casadi as ca
import numpy as np

from convex_timevarying_window.togt_minco_mpc.trajectory import basis
from convex_timevarying_window.global_gate_minco_probe.trajectory import INVERSE, SNAP
from convex_timevarying_window.single_gate_stop_racing import point_mapping
from ..safety import symbolic_constraints
from .symbolic_penalty import instantaneous, positive_penalty


RESIDUAL_SCALE = np.array([14.74348422, 12.18469679, *([4.27429**2]*4)])


def residual_scale(model, limit_fraction=1.):
    if model == 'x500':
        from .x500_dynamics import RATE_LIMITS, FMIN, FMAX
        return np.r_[RATE_LIMITS*limit_fraction**2,
            np.full(4, ((FMAX*limit_fraction-FMIN)/2.)**2)]
    return RESIDUAL_SCALE*limit_fraction**2


def local_options(name):
    directory = Path('/tmp/convex_ipopt_local_jit')
    directory.mkdir(exist_ok=True)
    optimization = '-O0' if name == 'minco_states' else '-O2'
    compiled = dict(jit=True, jit_name=name,
        jit_options=dict(flags=[optimization], directory=str(directory)+'/'))
    derivatives = dict(compiled, der_options=compiled)
    return dict(compiled, cse=True, never_inline=True, der_options=derivatives)


@lru_cache(maxsize=1)
def segment_coefficients():
    state = ca.SX.sym('segment_end_states', 25)
    duration = state[0]
    head, tail = ca.reshape(state[1:13], 3, 4), ca.reshape(state[13:], 3, 4)
    conditions = ca.vertcat(*[endpoint[:, order].T*duration**order
        for endpoint in (head, tail) for order in range(4)])
    return ca.Function('segment_pvaj_coefficients', [state], [ca.vec(ca.DM(INVERSE)@conditions)],
        local_options('pvaj_coefficients'))


def curve_coefficients(planner, x):
    times = ca.exp(x[:planner.n])
    points = planner.point_function(x)
    states = [ca.reshape(x[planner.velocity_start+3*planner.m*order:
        planner.velocity_start+3*planner.m*(order+1)], 3, planner.m) for order in range(3)]
    nodes = [ca.horzcat(ca.DM(planner.track.start), ca.DM.zeros(3, 3))]
    nodes.extend(ca.horzcat(points[:, gate], *[state[:, gate] for state in states])
        for gate in range(planner.m))
    nodes.append(ca.horzcat(ca.DM(planner.track.goal), ca.DM.zeros(3, 3)))
    values = ca.horzcat(*[ca.vertcat(times[piece], ca.vec(nodes[piece]), ca.vec(nodes[piece+1]))
        for piece in range(planner.n)])
    return ca.vec(segment_coefficients().map(planner.n)(values))


@lru_cache(maxsize=4)
def minco_states(pieces):
    """Eliminate the internal derivatives using a 3(N-1) SPD snap system."""
    points = ca.MX.sym('minco_points', 3, pieces-1)
    times = ca.MX.sym('minco_times', pieces)
    endpoints = ca.MX.sym('minco_endpoints', 3, 2)
    dimension = 3*(pieces-1)
    matrix = ca.MX(ca.Sparsity(dimension, dimension))
    rhs = ca.MX(ca.Sparsity(dimension, 3))
    for piece in range(pieces):
        duration = times[piece]
        known = ca.MX(ca.Sparsity(8, 3))
        free = ca.MX(ca.Sparsity(8, dimension))
        for side, node in enumerate((piece, piece+1)):
            row = 4*side
            known[row, :] = (endpoints[:, 0] if node == 0 else
                endpoints[:, 1] if node == pieces else points[:, node-1]).T
            if 0 < node < pieces:
                for order in (1, 2, 3):
                    free[row+order, (order-1)*(pieces-1)+node-1] = duration**order
        constant, mapping = ca.DM(INVERSE)@known, ca.DM(INVERSE)@free
        quadratic = ca.DM(SNAP)/duration**7
        matrix += mapping.T@quadratic@mapping
        rhs -= mapping.T@quadratic@constant
    matrix = (matrix+matrix.T)/2.
    diagonal = ca.diag(1./ca.sqrt(ca.diag(matrix)))
    states = diagonal@ca.solve(diagonal@matrix@diagonal, diagonal@rhs, 'ldl')
    return ca.Function('reduced_minco_states', [points, times, endpoints], [ca.vec(states.T)],
        local_options('minco_states'))


@lru_cache(maxsize=8)
def node_residuals(model='togt', limit_fraction=1.):
    if model == 'x500':
        from .x500_dynamics import instantaneous as x500_instantaneous
        native = x500_instantaneous(limit_fraction)
        state = ca.SX.sym('node_dynamics', 12)
    else:
        native = instantaneous(inline=True, limit_fraction=limit_fraction)
        state = ca.SX.sym('node_dynamics', 9)
    residual = native.call([state[index:index+3] for index in range(0, int(state.numel()), 3)], True, False)[3]
    return ca.Function('node_dynamic_residuals', [state], [residual/ca.DM(residual_scale(model, limit_fraction))],
        local_options('node_dynamics'))


@lru_cache(maxsize=8)
def dynamic_residuals(model='togt', limit_fraction=1.):
    state = ca.MX.sym('segment_dynamics', 25)
    coefficients = ca.reshape(state[:24], 8, 3)
    duration = state[24]
    states = []
    for order in ((1, 2, 3, 4) if model == 'x500' else (2, 3, 4)):
        nodes = np.column_stack([np.asarray(basis(node/32., order)).ravel() for node in range(33)])
        states.append(coefficients.T@ca.DM(nodes)/duration**order)
    residuals = node_residuals(model, limit_fraction).map(33)(ca.vertcat(*states))
    return ca.Function('segment_dynamic_residuals', [state], [ca.vec(residuals)],
        {'cse': True, 'never_inline': True})


@lru_cache(maxsize=8)
def node_penalty(model='togt', limit_fraction=1.):
    violations = ca.SX.sym('node_violations', 6)
    physical = violations*ca.DM(residual_scale(model, limit_fraction))
    return ca.Function('node_slack_penalty', [violations],
        [1000.*sum(positive_penalty(value) for value in ca.vertsplit(physical))],
        local_options('node_penalty'))


@lru_cache(maxsize=8)
def slack_integral(model='togt', limit_fraction=1.):
    violations = ca.MX.sym('segment_violations', 33*6)
    duration = ca.MX.sym('segment_duration')
    penalties = node_penalty(model, limit_fraction).map(33)(ca.reshape(violations, 6, 33))
    weights = ca.DM([.5]+[1.]*31+[.5])
    total = penalties@weights
    return ca.Function('segment_slack_integral', [violations, duration],
        [duration/32.*total], {'cse': True, 'never_inline': True})


class PolynomialPenalty:
    def __init__(self, model, limit_fraction=1.):
        values = ca.MX.sym('segment_coefficients', 25)
        residuals = dynamic_residuals(model, limit_fraction)(values)
        cost = slack_integral(model, limit_fraction)(ca.fmax(residuals, 0.), values[24])
        self.segment = ca.Function('segment_penalty', [values],
            [cost, ca.gradient(cost, values)], {'cse': True})

    def evaluate(self, coefficients, durations):
        coefficients = np.asarray(coefficients, dtype=float)
        durations = np.asarray(durations, dtype=float)
        if coefficients.shape != (len(durations), 8, 3):
            raise ValueError('Expected normalized degree-seven coefficients')
        values = np.vstack([coefficients.transpose(0, 2, 1).reshape(len(durations), 24).T, durations])
        costs, derivative = self.segment.map(len(durations))(values)
        derivative = np.asarray(derivative).T
        gradient = derivative[:, :24].reshape(len(durations), 3, 8).transpose(0, 2, 1)
        return float(np.asarray(costs).sum()), gradient, derivative[:, 24]


def frame_constraints(window, pose_function, kind, radius):
    state = ca.MX.sym('segment_frame_state', 26)
    offsets = ca.MX.sym('segment_frame_offsets')
    coefficients = ca.reshape(state[:24], 8, 3)
    duration, start = state[24], state[25]
    clipped = ca.fmin(offsets, duration)/duration
    u = 1.-clipped if kind == 'before' else clipped if kind == 'after' else offsets
    point = ca.SX.sym('local_frame_position', 3)
    distances = ca.Function('local_frame_distances', [point],
        [ca.vertcat(*symbolic_constraints(window.aperture, point, radius))],
        local_options('frame_distances'))
    rows = []
    for fraction in ca.vertsplit(u):
        center, rotation = pose_function(window, start+fraction*duration)
        local = rotation.T@(coefficients.T@basis(fraction)-center)
        rows.append(distances(local))
    return ca.Function('segment_frame_constraints', [state, offsets], [ca.vertcat(*rows)],
        dict(cse=True, never_inline=True))


def geometric_constraints(planner, x, witnesses, coefficients=None):
    times = ca.exp(x[:planner.n])
    starts = ca.vertcat(0., ca.cumsum(times)[:-1])
    if coefficients is None:
        coefficients = planner.curve_values(x)
    rows, upper = [], []
    span = (np.asarray(planner.cfg.workspace_upper)-planner.cfg.workspace_lower)/30.
    nodes = np.column_stack([np.asarray(basis(float(u))).ravel()
        for u in np.linspace(0., 1., planner.cfg.nodes_per_piece+1)])
    for piece in range(planner.n):
        curve = ca.reshape(coefficients[24*piece:24*(piece+1)], 8, 3)
        positions = curve.T@ca.DM(nodes)
        rows.append(ca.vec((positions-ca.repmat(ca.DM(planner.cfg.workspace_lower), 1, positions.size2()))/30.))
        upper.extend(np.tile(span, planner.cfg.nodes_per_piece+1).tolist())
    if not hasattr(planner, '_frame_kernels'):
        planner._frame_kernels = {}
    radius = planner.cfg.radius+planner.cfg.frame_thickness+planner.cfg.passage_margin+planner.cfg.witness_slack
    for (gate, piece), item in sorted(witnesses.items()):
        for begin in range(0, len(item['values']), planner.cfg.witness_slots):
            offsets = np.resize(item['values'][begin:begin+planner.cfg.witness_slots], planner.cfg.witness_slots)
            offsets = offsets[sorted(np.unique(offsets, return_index=True)[1])]
            key = gate, item['kind']
            if key not in planner._frame_kernels:
                planner._frame_kernels[key] = frame_constraints(planner.track.windows[gate],
                    planner.pose_function, item['kind'], radius)
            state = ca.vertcat(coefficients[24*piece:24*(piece+1)], times[piece], starts[piece])
            values = planner._frame_kernels[key].map(len(offsets))(
                ca.repmat(state, 1, len(offsets)), ca.DM(offsets).T)
            at_knot = ((item['kind'] == 'before' and piece == gate) or
                (item['kind'] == 'after' and piece == gate+1))
            if at_knot and 0. in offsets:
                window = planner.track.windows[gate]
                point_apertures = getattr(planner, 'point_apertures', None)
                aperture = window.aperture if point_apertures is None else point_apertures[gate]
                point = (point_mapping.symbolic(aperture, x[planner.slices[gate]])
                    if planner.free_points else ca.DM.zeros(2))
                knot_values = ca.vertcat(*symbolic_constraints(window.aperture, ca.vertcat(point, 0.), radius))
                values[:, int(np.flatnonzero(offsets == 0.)[0])] = knot_values
            rows.append(ca.vec(values))
            upper.extend([np.inf]*int(values.numel()))
    for gate, (window, sign) in enumerate(zip(planner.track.windows, planner.signs)):
        normal = ca.DM(-sign*window.state_at(0.)[1][:, 2])
        offset = planner.velocity_start+3*gate
        rows.append(ca.dot(normal, x[offset:offset+3])-.1)
        upper.append(np.inf)
    return ca.vertcat(*rows), np.asarray(upper)
