from __future__ import annotations

import math

import numpy as np
import pytest

from nonconvex_timevarying_window.sc_dynatogt.boundary import (
    ADAPTIVE_VERTEX_COUNTS,
    BSpline,
    Bezier,
    BoundaryPreprocessError,
    BoundaryValidationError,
    CircularArc,
    DenseBoundary,
    Line,
    adaptive_chang_resample,
    chang_uniform_resample,
    densify_boundary,
    validate_polygon,
)


def _square_dense(side: float = 2.0) -> DenseBoundary:
    points = np.asarray([[0.0, 0.0], [side, 0.0], [side, side], [0.0, side]])
    return densify_boundary([Line(points[i], points[(i + 1) % 4]) for i in range(4)])


def test_common_recursive_densification_and_exact_line_corners() -> None:
    dense = _square_dense()
    edge_lengths = np.linalg.norm(np.roll(dense.vertices, -1, axis=0) - dense.vertices, axis=1)
    assert np.max(edge_lengths) <= 0.01 + 1e-12
    assert len(dense.corners) == 4
    assert np.array_equal(dense.corners, np.asarray([[0.0, 0.0], [2.0, 0.0], [2.0, 2.0], [0.0, 2.0]]))
    assert validate_polygon(dense.vertices).valid


def test_arc_bezier_bspline_and_mixed_boundary_are_supported() -> None:
    arc = CircularArc([0.0, 0.0], 1.0, 0.0, math.pi / 2.0)
    assert np.allclose(arc.evaluate(0.0), [1.0, 0.0])
    assert np.allclose(arc.evaluate(1.0), [0.0, 1.0], atol=1e-12)

    bezier = Bezier([[2.0, 1.0], [1.5, 1.5], [0.5, 1.5], [0.0, 1.0]])
    assert np.allclose(bezier.evaluate(0.0), [2.0, 1.0])
    assert np.allclose(bezier.evaluate(1.0), [0.0, 1.0])

    spline = BSpline([[0.0, 0.0], [0.5, 0.4], [1.5, 0.4], [2.0, 0.0]], degree=3)
    assert np.allclose(spline.evaluate(0.0), [0.0, 0.0])
    assert np.allclose(spline.evaluate(1.0), [2.0, 0.0])

    mixed = densify_boundary(
        [
            Line([0.0, 0.0], [2.0, 0.0]),
            Line([2.0, 0.0], [2.0, 1.0]),
            bezier,
            Line([0.0, 1.0], [0.0, 0.0]),
        ]
    )
    assert validate_polygon(mixed.vertices).valid
    assert np.max(np.linalg.norm(np.roll(mixed.vertices, -1, axis=0) - mixed.vertices, axis=1)) <= 0.01 + 1e-12


def test_csv_boundary_with_header_and_forced_corner_indices(tmp_path) -> None:
    path = tmp_path / "boundary.csv"
    path.write_text("x,y\n0,0\n2,0\n2,1\n0,1\n0,0\n", encoding="utf-8")
    dense = DenseBoundary.from_csv(path, corner_indices=[0, 1, 2, 3])
    assert dense.vertices.shape == (4, 2)
    assert dense.corner_indices == (0, 1, 2, 3)
    assert validate_polygon(dense.vertices).valid

    corners_path = tmp_path / "corners.csv"
    corners_path.write_text("x,y\n0,0\n2,1\n", encoding="utf-8")
    selected = DenseBoundary.from_csv(path, corners_path=corners_path)
    assert selected.corner_indices == (0, 2)


def test_validation_rejects_clockwise_zero_area_and_self_intersection() -> None:
    clockwise = np.asarray([[0.0, 0.0], [0.0, 1.0], [1.0, 1.0], [1.0, 0.0]])
    bow_tie = np.asarray([[0.0, 0.0], [1.0, 1.0], [0.0, 1.0], [1.0, 0.0]])
    zero_area = np.asarray([[0.0, 0.0], [1.0, 0.0], [2.0, 0.0]])
    assert not validate_polygon(clockwise).valid
    assert not validate_polygon(bow_tie).valid
    assert not validate_polygon(zero_area).valid
    with pytest.raises(BoundaryValidationError):
        DenseBoundary(clockwise)


def test_chang_resampling_preserves_corners_and_uses_largest_remainders() -> None:
    dense = _square_dense(side=2.0)
    sampled = chang_uniform_resample(dense, 11)
    assert sampled.vertices.shape == (11, 2)
    assert sampled.corner_mask.sum() == 4
    assert all(any(np.array_equal(vertex, corner) for vertex in sampled.vertices) for corner in dense.corners)
    # Seven interior positions distributed over four equal intervals gives 2,2,2,1.
    counts_between_corners = []
    ids = np.flatnonzero(sampled.corner_mask)
    for i, index in enumerate(ids):
        counts_between_corners.append((ids[(i + 1) % len(ids)] - index - 1) % len(sampled.vertices))
    assert counts_between_corners == [2, 2, 2, 1]
    assert validate_polygon(sampled.vertices).valid


def test_adaptive_resampling_reports_5mm_and_3mm_acceptance() -> None:
    circle = DenseBoundary.from_segments([CircularArc([0.0, 0.0], 1.0, 0.0, 2.0 * math.pi)])
    sampled = adaptive_chang_resample(circle)
    assert sampled.m in ADAPTIVE_VERTEX_COUNTS
    assert sampled.report is not None and sampled.report.accepted
    assert sampled.report.max_boundary_error <= 0.005
    assert sampled.report.max_concavity_error <= 0.003
    assert sampled.report.is_simple and sampled.report.is_ccw and sampled.report.has_nonzero_area


def test_adaptive_resampling_marks_failure_after_last_candidate() -> None:
    circle = DenseBoundary.from_segments([CircularArc([0.0, 0.0], 1.0, 0.0, 2.0 * math.pi)])
    with pytest.raises(BoundaryPreprocessError) as caught:
        adaptive_chang_resample(circle, vertex_counts=(3, 4), boundary_tolerance=1e-8, concavity_tolerance=1e-8)
    assert [report.target_count for report in caught.value.reports] == [3, 4]
    assert not caught.value.reports[-1].accepted
