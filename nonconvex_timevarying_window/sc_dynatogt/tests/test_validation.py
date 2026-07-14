import numpy as np

from nonconvex_timevarying_window.sc_dynatogt.dynamics import ObjectiveWeights, PenaltyWeights
from nonconvex_timevarying_window.sc_dynatogt.environment import MotionProfile, SCDynamicWindow
from nonconvex_timevarying_window.sc_dynatogt.optimizer import OptimizationConfig
from nonconvex_timevarying_window.sc_dynatogt.sc_mapping import SCDiskMap
from nonconvex_timevarying_window.sc_dynatogt.validation import (
    check_window_gradients,
    validate_sc_mapping,
)


def _window():
    vertices = np.array([[-1.4, -1.0], [1.4, -1.0], [1.4, 1.0], [0.1, 1.0], [0.1, 0.2], [-1.4, 0.2]])
    mapping = SCDiskMap.fit(vertices, quadrature_order=32)
    motion = MotionProfile(
        np.array([0.1, 0.2, 0.1]), np.array([0.08, 0.05, 0.03]), 0.1,
        translation_period=5.0, rotation_period=7.0, scale_period=9.0,
    )
    return SCDynamicWindow("test", mapping, vertices, np.zeros(3), np.zeros(3), motion)


def test_window_gradient_protocol_thresholds():
    report = check_window_gradients(_window(), sample_count=12, seed=8)
    assert report.passed
    assert report.median_relative_error < 1.0e-5
    assert report.p99_relative_error < 1.0e-3


def test_mapping_legality_streaming_protocol():
    report = validate_sc_mapping(_window().sc_map, sample_count=200, seed=9, batch_size=37)
    assert report.passed
    assert report.inside_count == 200
    assert report.nan_count == report.inf_count == report.degenerate_jacobian_count == 0


def test_mapping_validation_uses_complete_psi_after_b_jacobian():
    class IdentityDiskMap:
        vertices = np.array([[-1.1, -1.1], [1.1, -1.1], [1.1, 1.1], [-1.1, 1.1]])

        @staticmethod
        def evaluate_many(points):
            return np.asarray(points, dtype=float)

        @staticmethod
        def jacobian_many(points):
            return np.repeat(np.eye(2)[None, :, :], len(points), axis=0)

    sample_count, seed = 29, 17
    report = validate_sc_mapping(IdentityDiskMap(), sample_count=sample_count, seed=seed, batch_size=7)
    d = np.random.default_rng(seed).normal(0.0, 2.0, size=(sample_count, 2))
    expected = float(np.min((1.0 + np.einsum("ij,ij->i", d, d)) ** -2))
    assert report.passed
    assert np.isclose(report.minimum_abs_determinant, expected, rtol=1e-14, atol=0.0)
