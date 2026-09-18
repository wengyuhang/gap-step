import numpy as np
import pytest

from convex_timevarying_window.joint_feasible_time_refinement.refinement import (
    JointTimeRefinementConfig,
    _soft_feasible,
    canonicalize,
)
from convex_timevarying_window.togt.experiment import build_track
from convex_timevarying_window.togt.native_objective import NativeJointTOGTObjective


def test_refinement_config_rejects_invalid_resolution():
    with pytest.raises(ValueError):
        JointTimeRefinementConfig(scan_points=2)


def test_soft_screen_tolerates_only_roundoff_scale_residuals():
    assert _soft_feasible(
        {"dynamic_integral": 4e-15, "safety_integral": 0.0}, 1e-12
    )
    assert not _soft_feasible(
        {"dynamic_integral": 2e-6, "safety_integral": 0.0}, 1e-12
    )


def test_canonicalize_normalizes_polygon_blocks_and_clips_circle_blocks():
    track, config = build_track()
    objective = NativeJointTOGTObjective(track, config)
    x = objective.initial_guess()
    offset = objective.temporal_dimension
    for window, size in zip(track.windows, objective.spatial_dimensions):
        if window.aperture.kind == "polygon":
            x[offset:offset + size] *= 3.0
        else:
            x[offset:offset + size] = 100.0
        offset += size
    normalized = canonicalize(x, objective)
    offset = objective.temporal_dimension
    for window, size in zip(track.windows, objective.spatial_dimensions):
        block = normalized[offset:offset + size]
        if window.aperture.kind == "polygon":
            assert np.linalg.norm(block) == pytest.approx(1.0)
        else:
            assert np.max(block) == pytest.approx(12.0)
        offset += size
