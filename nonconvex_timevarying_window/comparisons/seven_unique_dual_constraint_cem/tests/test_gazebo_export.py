import numpy as np
from pathlib import Path
from shapely.geometry import Polygon

from nonconvex_timevarying_window.comparisons.seven_unique_dual_constraint_cem.gazebo.course_spec import (
    GATES,
    MESH_CHORD_TOLERANCE_M,
)
from nonconvex_timevarying_window.comparisons.seven_unique_dual_constraint_cem.gazebo.export_world import (
    outward_offset_polygon,
    simplified_visual_polygon,
    tube_mesh,
)
from nonconvex_timevarying_window.comparisons.seven_unique_dual_constraint_cem.gazebo.validate_course import (
    validate as validate_course,
)


def test_tube_mesh_is_watertight_and_centered_on_source_edges():
    points = np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]])
    mesh = tube_mesh(points, 0.01, 12, closed=True)
    assert mesh.is_watertight
    assert len(mesh.faces) == 4 * 12 * 4
    assert np.max(np.abs(mesh.bounds[:, 2])) <= 0.010000001
    assert mesh.bounds[0, 0] <= -0.009
    assert mesh.bounds[1, 0] >= 1.009


def test_visual_gate_offset_expands_a_ccw_aperture():
    square = np.array([[-1.0, -1.0], [1.0, -1.0], [1.0, 1.0], [-1.0, 1.0]])
    expanded = outward_offset_polygon(square, 0.075)
    np.testing.assert_allclose(expanded,
                               np.array([[-1.075, -1.075], [1.075, -1.075],
                                         [1.075, 1.075], [-1.075, 1.075]]),
                               atol=1e-12)


def test_visual_polygon_simplification_has_bounded_error():
    angles = np.linspace(0.0, 2.0 * np.pi, 2048, endpoint=False)
    circle = np.column_stack((2.0 * np.cos(angles), 2.0 * np.sin(angles)))
    simplified, error = simplified_visual_polygon(circle, 0.006)
    assert len(simplified) < 100
    assert error <= 0.006


def test_course_definition_and_runtime_assets_are_algorithm_independent():
    source = (
        Path(__file__).parents[1] / "gazebo" / "course_spec.py"
    ).read_text(encoding="utf-8")
    assert "preprocess" not in source.lower()
    assert "sc_dynatogt" not in source
    assert len(GATES) == 7
    assert MESH_CHORD_TOLERANCE_M == 0.002
    assert all(Polygon(gate.boundary).is_valid for gate in GATES)
    validation = validate_course()
    assert validation["passed"]
    assert validation["algorithm_inputs"] == []
