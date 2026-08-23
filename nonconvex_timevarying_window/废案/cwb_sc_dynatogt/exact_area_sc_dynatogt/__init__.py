"""Exact-area whole-body safety for SC-DynaTOGT.

This sibling method keeps the base planner untouched.  The current milestone
implements the exact polygonal geometry, safety functional, independent
validator, and the Experiment-B counterexample.  The large A/C/D/E/F studies
are intentionally left as protocol entries rather than fabricated results.
"""

from .geometry import (
    Cuboid,
    GateFrame,
    IntersectionMetrics,
    PlaneSection,
    exact_intersection_metrics,
    plane_section,
)
from .penalty import (
    integrated_penalty,
    integrated_penalty_gradient,
    instantaneous_penalty,
    instantaneous_penalty_gradient,
)

__all__ = [
    "Cuboid",
    "GateFrame",
    "IntersectionMetrics",
    "PlaneSection",
    "exact_intersection_metrics",
    "integrated_penalty",
    "integrated_penalty_gradient",
    "instantaneous_penalty",
    "instantaneous_penalty_gradient",
    "plane_section",
]
