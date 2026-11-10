"""Independent free-point solver with an explicit planning-sphere radius."""
from dataclasses import replace

from convex_timevarying_window.global_gate_minco_probe import planner as core
from ..independent_free_solve.planner import Planner as OriginalPlanner

ORIGINAL_RADIUS_M = .38594472
SMALL_RADIUS_M = .9*ORIGINAL_RADIUS_M


class Planner(OriginalPlanner):
    def __init__(self, track, free_points=True, *, pose_function, radius_m=SMALL_RADIUS_M):
        if not 0. < radius_m <= ORIGINAL_RADIUS_M:
            raise ValueError('Planning radius must be positive and at most the original radius')
        config_factory = core.Config
        core.Config = lambda: replace(config_factory(), radius=float(radius_m))
        try:
            super().__init__(track, free_points, pose_function=pose_function)
        finally:
            core.Config = config_factory
        assert self.cfg.radius == radius_m
