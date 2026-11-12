"""Export whole-trajectory, uniform-in-time dynamics for the C++ SIP solver.

This builds a fixed 257-node graph. The node times are fractions of the current
total duration, so their segment assignment changes with the optimization
variables. The graph applies the same six normalized TOGT dynamics residuals
and 95% limit fraction as the existing soft formulation.
"""
import argparse
import json
from pathlib import Path

import casadi as ca

from convex_timevarying_window.togt_minco_mpc.trajectory import basis
from convex_timevarying_window.comparisons.论文实验.实验2.soft_dynamics import kernels


def export(graph: Path, count: int = 257) -> None:
    manifest = json.loads((graph / 'manifest.json').read_text())
    pieces = manifest['n']
    coefficients = ca.MX.sym('coefficients', 24 * pieces)
    durations = ca.MX.sym('durations', pieces)
    ends = ca.cumsum(durations)
    starts = ca.vertcat(0., ends[:-1])
    total = ca.sum1(durations)
    states = []
    for node in range(count):
        time = (node / (count - 1)) * total
        selected = None
        for piece in range(pieces - 1, -1, -1):
            duration = durations[piece]
            u = ca.fmin(1., ca.fmax(0., (time - starts[piece]) / duration))
            curve = ca.reshape(coefficients[24*piece:24*(piece+1)], 8, 3)
            local = ca.vertcat(*[
                curve.T @ basis(u, derivative) / duration**derivative
                for derivative in (2, 3, 4)
            ])
            selected = local if selected is None else ca.if_else(time <= ends[piece], local, selected)
        states.append(selected)
    nodal = kernels.node_residuals('togt', .95).map(len(states))(ca.horzcat(*states))
    function = ca.Function('global_uniform_dynamics', [coefficients, durations],
                           [ca.vec(nodal)], {'never_inline': True})
    function.save(str(graph / 'global_uniform_dynamics.casadi'))
    (graph / 'global_uniform_dynamics.json').write_text(json.dumps({
        'sample_count': count,
        'sample_fractions': [node / (count - 1) for node in range(count)],
        'model': 'togt',
        'limit_fraction': .95,
        'constraints_per_sample': 6,
    }, indent=2) + '\n')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('graph', type=Path)
    parser.add_argument('--nodes', type=int, default=257)
    args = parser.parse_args()
    if args.nodes < 3:
        parser.error('at least three global nodes are required')
    export(args.graph, args.nodes)
