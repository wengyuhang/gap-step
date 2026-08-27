from __future__ import annotations

import numpy as np

from nonconvex_timevarying_window.comparisons.sc_sip_fast_closed_loop.cuboid_replay import (
    cuboid_plane_section,
    cuboid_vertices,
)


def test_axis_aligned_cuboid_plane_section() -> None:
    vertices = cuboid_vertices(np.zeros(3), np.eye(3), (0.3, 0.2, 0.1))
    section = cuboid_plane_section(vertices, np.zeros(3), np.array([1.0, 0.0, 0.0]))
    assert section.shape == (4, 3)
    assert np.allclose(section[:, 0], 0.0)
    assert np.isclose(np.ptp(section[:, 1]), 0.4)
    assert np.isclose(np.ptp(section[:, 2]), 0.2)
