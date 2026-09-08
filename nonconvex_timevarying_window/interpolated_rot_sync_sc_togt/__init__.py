"""SC-input-interpolated rotation-synchronised TOGT method."""

from nonconvex_timevarying_window.rot_sync_sc_togt.optimizer import (
    RotSyncOptimizationConfig,
    RotSyncOptimizationResult,
)

from .optimizer import (
    InterpolatedRotSyncForwardPass,
    InterpolatedRotSyncObjective,
    optimize_interpolated_track,
)
from .scenarios import build_oblique_smoke_scenario
from .trajectory import CompositeTrajectory, SCInputInterpolatedSyncSegment

__all__ = [
    "CompositeTrajectory",
    "InterpolatedRotSyncForwardPass",
    "InterpolatedRotSyncObjective",
    "RotSyncOptimizationConfig",
    "RotSyncOptimizationResult",
    "SCInputInterpolatedSyncSegment",
    "build_oblique_smoke_scenario",
    "optimize_interpolated_track",
]
