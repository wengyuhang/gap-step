from __future__ import annotations

import math

import numpy as np
import pytest

from nonconvex_timevarying_window.sc_dynatogt.boundary import CircularArc, DenseBoundary, Line, chang_uniform_resample, densify_boundary
from nonconvex_timevarying_window.sc_dynatogt.offset import (
    DEFAULT_INWARD_OFFSET,
    DEFAULT_MITER_LIMIT,
    OffsetValidationError,
    inward_offset,
)


def test_default_315mm_inward_offset_is_valid_and_uses_clipper2() -> None:
    points = np.asarray([[0.0, 0.0], [2.0, 0.0], [2.0, 2.0], [0.0, 2.0]])
    dense = densify_boundary([Line(points[i], points[(i + 1) % 4]) for i in range(4)])
    sampled = chang_uniform_resample(dense, 256)
    result = inward_offset(sampled)
    assert result.distance == pytest.approx(0.315)
    assert DEFAULT_INWARD_OFFSET == pytest.approx(0.315)
    assert np.allclose(result.vertices.min(axis=0), [0.315, 0.315], atol=2e-6)
    assert np.allclose(result.vertices.max(axis=0), [1.685, 1.685], atol=2e-6)
    assert result.area == pytest.approx(1.37**2, abs=1e-5)
    assert result.diagnostics.valid
    assert result.metadata.backend == "pyclipr (Clipper2)"
    assert result.metadata.miter_limit == DEFAULT_MITER_LIMIT
    # All true square corners are convex for a negative offset; Round and Miter
    # yield the same inward line intersection, allowing smooth samples to stay Round.
    assert result.metadata.applied_path_join == "Round"
    assert result.metadata.per_vertex_request_exact


def test_all_smooth_boundary_requests_round_join() -> None:
    dense = DenseBoundary.from_segments([CircularArc([0.0, 0.0], 1.2, 0.0, 2.0 * math.pi)])
    sampled = chang_uniform_resample(dense, 256)
    result = inward_offset(sampled)
    assert result.metadata.corner_count == 0
    assert result.metadata.smooth_vertex_count == 256
    assert result.metadata.applied_path_join == "Round"
    assert result.metadata.join_strategy == "global_round_all_vertices_are_smooth"
    radii = np.linalg.norm(result.vertices, axis=1)
    assert np.max(np.abs(radii - (1.2 - 0.315))) < 1e-3
    # A 1 mm physical arc tolerance should not become a 1e-9 m tolerance
    # after pyclipr's default 1e6 coordinate scaling.
    assert len(result.vertices) < 512


def test_reflex_true_corner_is_miter_and_limitation_is_reported() -> None:
    # CCW L shape: (1,1) is a forced reflex corner; line densification also
    # creates smooth/collinear samples, exposing pyclipr's per-path join limit.
    points = np.asarray([[0.0, 0.0], [3.0, 0.0], [3.0, 1.0], [1.0, 1.0], [1.0, 3.0], [0.0, 3.0]])
    dense = densify_boundary([Line(points[i], points[(i + 1) % len(points)]) for i in range(len(points))])
    sampled = chang_uniform_resample(dense, 256)
    result = inward_offset(sampled)
    assert result.metadata.reflex_corner_count == 1
    assert result.metadata.applied_path_join == "Miter"
    assert not result.metadata.per_vertex_join_supported
    assert not result.metadata.per_vertex_request_exact
    assert "dense_miter_approximation" in result.metadata.join_strategy


def test_offset_rejects_split_components() -> None:
    # Two 2x2 lobes joined by a 0.2 m neck; the fixed 0.315 m deflation splits it.
    dumbbell = np.asarray(
        [
            [-2.0, -1.0],
            [-0.1, -1.0],
            [-0.1, -0.1],
            [0.1, -0.1],
            [0.1, -1.0],
            [2.0, -1.0],
            [2.0, 1.0],
            [0.1, 1.0],
            [0.1, 0.1],
            [-0.1, 0.1],
            [-0.1, 1.0],
            [-2.0, 1.0],
        ]
    )
    with pytest.raises(OffsetValidationError) as caught:
        inward_offset(dumbbell, corner_mask=np.ones(len(dumbbell), dtype=bool))
    assert caught.value.diagnostics.component_count == 2
    assert not caught.value.diagnostics.single_component


def test_offset_rejects_empty_and_area_below_point_one() -> None:
    too_small = np.asarray([[0.0, 0.0], [0.5, 0.0], [0.5, 0.5], [0.0, 0.5]])
    with pytest.raises(OffsetValidationError) as empty:
        inward_offset(too_small, corner_mask=np.ones(4, dtype=bool))
    assert not empty.value.diagnostics.nonempty

    low_area = np.asarray([[0.0, 0.0], [0.8, 0.0], [0.8, 0.8], [0.0, 0.8]])
    with pytest.raises(OffsetValidationError) as caught:
        inward_offset(low_area, corner_mask=np.ones(4, dtype=bool))
    assert caught.value.diagnostics.nonempty
    assert caught.value.diagnostics.area <= 0.1
    assert not caught.value.diagnostics.area_above_threshold
