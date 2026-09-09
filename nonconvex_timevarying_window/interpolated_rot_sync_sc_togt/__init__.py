"""SC-input-interpolated rotation-synchronised TOGT method."""

from nonconvex_timevarying_window.rot_sync_sc_togt.optimizer import (
    RotSyncOptimizationConfig,
    RotSyncOptimizationResult,
)

from .optimizer import (
    InterpolatedRotSyncForwardPass,
    InterpolatedRotSyncObjective,
    SplineRotSyncForwardPass,
    SplineRotSyncObjective,
    optimize_interpolated_track,
    optimize_spline_track,
)
from .scenarios import build_oblique_smoke_scenario
from .trajectory import (
    CompositeTrajectory,
    SCInputInterpolatedSyncSegment,
    SCInputSplineSyncSegment,
)

__all__ = [
    "CompositeTrajectory",
    "InterpolatedRotSyncForwardPass",
    "InterpolatedRotSyncObjective",
    "RotSyncOptimizationConfig",
    "RotSyncOptimizationResult",
    "SCInputInterpolatedSyncSegment",
    "SCInputSplineSyncSegment",
    "SplineRotSyncForwardPass",
    "SplineRotSyncObjective",
    "build_oblique_smoke_scenario",
    "optimize_interpolated_track",
    "optimize_spline_track",
]
