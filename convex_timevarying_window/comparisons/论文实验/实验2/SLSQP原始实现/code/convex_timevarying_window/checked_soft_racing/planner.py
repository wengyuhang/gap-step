"""Objective-only variant: prefer clearance without weakening admission."""
from dataclasses import dataclass
import math

import casadi as ca

from convex_timevarying_window.receding_gate_racing.planner import (
    Config as CheckedConfig, symbolic_clearance_sq,
)
from convex_timevarying_window.continuation_racing.guard import CheckedPlanner


@dataclass(frozen=True)
class Config(CheckedConfig):
    soft_clearance_buffer: float = .15
    soft_clearance_weight: float = 1.

    def __post_init__(self):
        if not math.isfinite(self.soft_clearance_buffer) or self.soft_clearance_buffer <= 0:
            raise ValueError('soft clearance buffer must be finite and positive')
        if not math.isfinite(self.soft_clearance_weight) or self.soft_clearance_weight < 0:
            raise ValueError('soft clearance weight must be finite and nonnegative')


def clearance_penalty(distance_squared, cfg):
    """C2 cubic hinge in squared distance; one at the original hard margin.

    Distance is to the actual frame centreline, not to the gate centre or
    infinite gate plane. Radii therefore include both drone and sleeve.
    The hinge itself is C2; nearest-edge geometry remains piecewise smooth.
    """
    hard = cfg.radius + cfg.frame_thickness + cfg.tracking_margin
    preferred = hard + cfg.soft_clearance_buffer
    deficit = ca.fmax(0., (preferred**2 - distance_squared)/(preferred**2 - hard**2))
    return cfg.soft_clearance_weight * deficit**3


class Planner(CheckedPlanner):
    def __init__(self, track, cfg=Config()):
        super().__init__(track, cfg)

    def extra_stage_cost(self, position, absolute_time, first, wi, k):
        if self.cfg.soft_clearance_weight == 0:
            return 0.
        return sum(clearance_penalty(
            symbolic_clearance_sq(self.frames[gj], position, absolute_time), self.cfg)
            for gj in self.segment_frames(first, wi, k))
