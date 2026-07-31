import numpy as np

from mdg.backend_adapter import free_point_map


def test_10000_free_points_stay_in_unit_disc_and_jacobian_matches_difference():
    rng = np.random.default_rng(0)
    values = rng.normal(0.0, 3.0, (10000, 2))
    mapped = np.asarray([free_point_map(value)[0] for value in values])
    assert np.max(np.linalg.norm(mapped, axis=1)) < 1.0
    step = 1.0e-6
    for value in values[:50]:
        _, jacobian = free_point_map(value)
        numeric = np.column_stack(
            (
                (free_point_map(value + np.array((step, 0.0)))[0]
                 - free_point_map(value - np.array((step, 0.0)))[0])
                / (2 * step),
                (free_point_map(value + np.array((0.0, step)))[0]
                 - free_point_map(value - np.array((0.0, step)))[0])
                / (2 * step),
            )
        )
        np.testing.assert_allclose(jacobian, numeric, rtol=1.0e-5, atol=1.0e-7)
    _, zero_jacobian = free_point_map(np.zeros(2))
    np.testing.assert_allclose(zero_jacobian, np.eye(2))

