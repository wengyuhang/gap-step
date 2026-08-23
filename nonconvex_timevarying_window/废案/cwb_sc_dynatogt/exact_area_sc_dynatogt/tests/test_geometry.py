import numpy as np

from nonconvex_timevarying_window.exact_area_sc_dynatogt.geometry import (
    Cuboid,
    GateFrame,
    PlaneSection,
    exact_intersection_metrics,
    plane_section,
)


def test_axis_aligned_midplane_section_has_true_metric_area():
    cuboid = Cuboid(np.array([1.0, 0.5, 0.25]))
    frame = GateFrame(np.zeros(3), np.eye(3)[:, :2], np.array([0.0, 0.0, 1.0]), 2.0)
    section = plane_section(cuboid, np.zeros(3), np.eye(3), frame)
    assert len(section.vertices_2d) == 4
    assert np.isclose(section.area, 2.0)
    assert not section.degenerate_contact


def test_u_gate_intersection_preserves_two_connected_components():
    # U = bottom bar plus two arms; a horizontal section across the arms gives two pieces.
    gate = np.array([(0, 0), (1, 0), (1, 3), (0.7, 3), (0.7, 1), (0.3, 1), (0.3, 3), (0, 3)])
    section_vertices = np.array([(0, 2), (1, 2), (1, 3), (0, 3)], dtype=float)
    section = PlaneSection(np.column_stack((section_vertices, np.zeros(4))), section_vertices, 1.0, False)
    metrics = exact_intersection_metrics(section, gate)
    assert len(metrics.intersection_components) == 2
    assert np.isclose(metrics.intersection_area, 0.6)
    assert np.isclose(metrics.outside_area, 0.4)
    assert metrics.whole_body_collision


def test_full_containment_and_no_intersection_have_zero_penalty():
    gate = np.array([(-2, -2), (2, -2), (2, 2), (-2, 2)], dtype=float)
    inside_vertices = np.array([(-1, -1), (1, -1), (1, 1), (-1, 1)], dtype=float)
    inside = PlaneSection(np.column_stack((inside_vertices, np.zeros(4))), inside_vertices, 4.0, False)
    metrics = exact_intersection_metrics(inside, gate)
    assert np.isclose(metrics.intersection_ratio, 1.0)
    assert metrics.penalty == 0.0
    assert not metrics.whole_body_collision

    away_vertices = inside_vertices + np.array([5.0, 0.0])
    away = PlaneSection(np.column_stack((away_vertices, np.zeros(4))), away_vertices, 4.0, False)
    metrics = exact_intersection_metrics(away, gate)
    assert metrics.intersection_area == 0.0
    assert metrics.penalty == 0.0
    assert not metrics.whole_body_collision
