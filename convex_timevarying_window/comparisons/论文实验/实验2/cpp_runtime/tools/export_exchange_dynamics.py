"""Export one variable-time dynamics node and the identical physical constants."""
import argparse
import json
from pathlib import Path

import casadi as ca
import numpy as np

from convex_timevarying_window.comparisons.论文实验.实验2.soft_dynamics import kernels
from convex_timevarying_window.togt_minco_mpc.trajectory import basis
from convex_dynamic_seven_window_gazebo.x500_togt.experiment import X500_MASS, X500_INERTIA
from convex_timevarying_window.dynamic_passage_velocity_minco.hard_dynamics import MIXER


def export(directory):
    directory.mkdir(parents=True, exist_ok=True)
    state = ca.MX.sym('segment_state', 25)
    u = ca.MX.sym('segment_fraction')
    curve = ca.reshape(state[:24], 8, 3)
    derivatives = ca.vertcat(*[curve.T @ basis(u, order) / state[24]**order
                              for order in (2, 3, 4)])
    residual = kernels.node_residuals('togt', .95)(derivatives)
    ca.Function('exchange_dynamic_node', [state, u], [residual],
                {'never_inline': True}).save(str(directory/'exchange_dynamic_node.casadi'))
    metadata = dict(mass=X500_MASS, inertia=np.asarray(X500_INERTIA).tolist(),
        inverse_mixer=np.linalg.inv(MIXER).tolist(), gravity=9.8066,
        rate_squared_limits=[14.74348422*.95**2, 12.18469679*.95**2],
        rotor_midpoint=4.27429*.95,
        residual_scale=kernels.residual_scale('togt', .95).tolist(),
        initialization='baseline_peaks', togt_time_coordinates=True,
        limit_fraction=.95, finite_residual_slack=1e-5,
        verification_residual_tolerance=1e-6)
    (directory/'exchange_dynamics.json').write_text(json.dumps(metadata, indent=2)+'\n')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('directory', type=Path)
    export(parser.parse_args().directory)
