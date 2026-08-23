from __future__ import annotations

import numpy as np

from nonconvex_timevarying_window.cwb_sc_dynatogt.body_model import CuboidBody
from nonconvex_timevarying_window.cwb_sc_dynatogt.config import WholeBodySafetyConfig
from nonconvex_timevarying_window.cwb_sc_dynatogt.gate_frame import frame_at
from nonconvex_timevarying_window.cwb_sc_dynatogt.plane_section import (
    find_planned_crossing_interval,
    gate_local_vertex_coordinates,
    has_plane_section,
    plane_section_from_vertices,
)

from .conftest import LinearTrajectory, StaticXPlaneWindow


def test_static_x_plane_section_and_contacts(square_map) -> None:
    window = StaticXPlaneWindow(square_map)
    frame = frame_at(window, 2.0)
    body = CuboidBody(np.array([0.3, 0.2, 0.1]))
    for center_x, expected in [(1.6, False), (1.7, True), (2.0, True), (2.3, True), (2.4, False)]:
        vertices = body.vertices_body + np.array([center_x, 0.0, 0.0])
        xi = gate_local_vertex_coordinates(vertices, frame)[:, 2]
        assert has_plane_section(xi, 1e-10) is expected
    vertices = body.vertices_body + np.array([2.0, 0.0, 0.0])
    section = plane_section_from_vertices(vertices, frame, body, time=2.0)
    assert not section.degenerate
    assert len(section.vertices) == 4
    assert np.allclose(np.ptp(section.local_polygon, axis=0), [0.4, 0.2])


def test_planned_component_matches_analytic_interval(square_map) -> None:
    config = WholeBodySafetyConfig(
        half_extents=(0.3, 0.2, 0.1), time_tolerance=1e-6,
        interval_scan_steps=128,
    )
    interval = find_planned_crossing_interval(
        window_index=0, traversal_time=2.0, trajectory=LinearTrajectory(),
        window=StaticXPlaneWindow(square_map),
        body=CuboidBody(np.array(config.half_extents)), config=config,
    )
    assert np.isclose(interval.start, 1.7, atol=2e-6)
    assert np.isclose(interval.end, 2.3, atol=2e-6)
    assert interval.direction == 1
