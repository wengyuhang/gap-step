"""Identical hard constraints; only replace sinusoidal motion and phase seeds."""
import casadi as ca
import numpy as np
from convex_timevarying_window.dynamic_passage_velocity_minco.hard_planner import Planner as Parent, Config
from convex_timevarying_window.dynamic_passage_minco.safety import passage


class Planner(Parent):
    def symbolic_pose(self, window, t):
        m = window.motion
        return ca.DM(window.center0)+ca.DM(m.velocity)*(t-m.anchor_time), ca.DM(self.rotation)

    def arrival_initialization(self, window, now, initial):
        # No periodic phase basins or artificial crossing-time limits.
        return [initial*k for k in (1., .8, 1.2, 1.5)], 0., np.inf

    def arrival_bounds(self, window, now, seed_arrival, omega, period):
        return .3, 15.

    def local_passage(self, plan, window, piece):
        local_v = self.rotation.T@window.motion.velocity
        def bounds(a,b):
            return local_v*(a-window.motion.anchor_time), local_v*(b-window.motion.anchor_time)
        result = passage(plan, window,
            self.cfg.radius+self.cfg.frame_thickness+self.cfg.passage_margin,
            self.cfg.frame_offset, piece_indices=(piece,), translation_bounds=bounds)
        result['evidence'] = 'floating-point Bernstein/affine translation bounds, not interval certificate'
        return result
