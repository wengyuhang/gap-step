"""Fast certified SIP for fixed-plane rotating/scaling windows."""

from .certificate import certify
from .model import (
    PlanarRSConfig,
    PlanarRSDynamicWindow,
    PlanarRSMotion,
    PlanarRSSIPWindow,
    make_planar_problem,
)
from .scenario import benchmark_boundaries, build_benchmark, build_ordinary
from .solver import solve

__all__ = [
    "PlanarRSConfig",
    "PlanarRSDynamicWindow",
    "PlanarRSMotion",
    "PlanarRSSIPWindow",
    "benchmark_boundaries",
    "build_benchmark",
    "build_ordinary",
    "certify",
    "make_planar_problem",
    "solve",
]
