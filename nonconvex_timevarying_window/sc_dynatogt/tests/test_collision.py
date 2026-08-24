import numpy as np
import pytest

from nonconvex_timevarying_window.sc_dynatogt.collision import (
    CuboidBody,
    point_to_oriented_cuboid_distance_squared,
    whole_body_clearance_residual,
)


def test_exact_oriented_cuboid_distance_and_vectorization():
    body = CuboidBody((1.0, 1.0, 0.5))
    rotation = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
    points = np.array([[0.0, 1.25, 0.0], [0.0, 0.0, 0.0], [0.0, 1.0, 1.0]])
    distance2 = point_to_oriented_cuboid_distance_squared(
        points, np.zeros(3), rotation, body
    )
    np.testing.assert_allclose(distance2, [0.25**2, 0.0, 0.5**2])
    assert whole_body_clearance_residual(
        points[0], np.zeros(3), rotation, body, 0.30
    ) == pytest.approx(0.30**2 - 0.25**2)


def test_sc_conservative_inset_contains_same_cuboid_for_every_attitude():
    body = CuboidBody()
    clearance = 0.015
    required = body.conservative_center_clearance(clearance)
    assert required == pytest.approx(np.linalg.norm(body.half_extents) + clearance)
    # A frame point at this centre distance is at least ``clearance`` from the
    # cuboid for every attitude by the enclosing-sphere argument.
    assert required > 0.315
