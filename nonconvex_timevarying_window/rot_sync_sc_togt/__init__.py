"""SC/MINCO planner with analytic rotation-synchronised gate crossings."""

from .geometry import RotatingWindow
from nonconvex_timevarying_window.sc_dynatogt.collision import CuboidBody
from .optimizer import (
    RotSyncOptimizationConfig,
    RotSyncOptimizationResult,
    optimize_track,
)
from .scenarios import (
    REALISTIC_BODY,
    REALISTIC_RHO,
    RotSyncScenario,
    build_formal_scenarios,
    build_multi_scenarios,
    build_realistic_extreme_scenario,
    build_smoke_scenario,
    scenario_difficulty_metrics,
)
from .trajectory import CompositeTrajectory, RotationSyncSegment

__all__ = [
    "CompositeTrajectory",
    "CuboidBody",
    "RotationSyncSegment",
    "RotSyncOptimizationConfig",
    "RotSyncOptimizationResult",
    "RotSyncScenario",
    "RotatingWindow",
    "REALISTIC_BODY",
    "REALISTIC_RHO",
    "build_formal_scenarios",
    "build_multi_scenarios",
    "build_realistic_extreme_scenario",
    "build_smoke_scenario",
    "optimize_track",
    "scenario_difficulty_metrics",
]
