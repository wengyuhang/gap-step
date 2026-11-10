"""Same-target restoration with the previously audited smooth heading model."""
import time
import numpy as np
from convex_timevarying_window.global_gate_minco_probe import planner as core
from convex_timevarying_window.tilt_heading_minco import dynamics
from convex_timevarying_window.togt_minco_mpc.trajectory import basis
from .augmented import Planner as AugmentedPlanner


class Planner(AugmentedPlanner):
    def __init__(self, *args, **kwargs):
        previous = core.constraints
        core.constraints = dynamics.constraints
        try:
            super().__init__(*args, **kwargs)
        finally:
            core.constraints = previous

    def verify(self, plan, deadline=np.inf):
        for piece in range(self.n):
            for u in np.linspace(0., 1., self.cfg.nodes_per_piece+1):
                velocity, acceleration = [plan.coefficients[piece].T@np.asarray(basis(float(u), d)).ravel()/plan.durations[piece]**d for d in (1, 2)]
                health = dynamics.domain(velocity, acceleration)
                if not health['passed']:
                    return [dict(gate=-1, piece=piece, passed=False, outcome='attitude_chart_unresolved',
                                 witness_u=None, **{k: v for k, v in health.items() if k != 'passed'})], False
                if time.perf_counter() >= deadline:
                    return [], False
        return super().verify(plan, deadline)

    def solve_all(self, *args, **kwargs):
        plan, result = super().solve_all(*args, **kwargs)
        result['attitude_model'] = 'north_tilt_heading_pi_over_2'
        return plan, result
