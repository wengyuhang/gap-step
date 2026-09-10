import numpy as np

from nonconvex_timevarying_window.comparisons.seven_unique_dual_constraint_cem_no_inset.experiment import (
    remove_exact_collinear,
)


def test_exact_straight_run_cleanup_preserves_corners_and_area():
    dense_square = np.array([
        [0., 0.], [.5, 0.], [1., 0.], [1., .5],
        [1., 1.], [.5, 1.], [0., 1.], [0., .5],
    ])
    cleaned = remove_exact_collinear(dense_square)
    expected = np.array([[0., 0.], [1., 0.], [1., 1.], [0., 1.]])
    np.testing.assert_allclose(cleaned, expected)


def test_noncollinear_curve_samples_are_retained():
    polygon = np.array([[0., 0.], [1., 0.], [1.2, .4], [1., 1.], [0., 1.]])
    np.testing.assert_allclose(remove_exact_collinear(polygon), polygon)
