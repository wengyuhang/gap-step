from __future__ import annotations

import numpy as np

from nonconvex_timevarying_window.sc_dynatogt.baselines import (
    BaselineTOGTObjective,
    BaselineTrack,
    StaticBaselineWindow,
    convex_hull_polygon,
    optimize_baseline_track,
    original_togt_convex_jacobian,
    original_togt_convex_map,
)
from nonconvex_timevarying_window.sc_dynatogt.dynamics import ObjectiveWeights, PenaltyWeights
from nonconvex_timevarying_window.sc_dynatogt.optimizer import OptimizationConfig


def test_original_togt_convex_map_and_gradient() -> None:
    rectangle = np.array([[-2.0, -1.0], [2.0, -1.0], [2.0, 1.0], [-2.0, 1.0]])
    d = np.array([0.7, -1.1, 0.4, 0.9])
    point = original_togt_convex_map(d, rectangle)
    assert np.all(point >= rectangle.min(axis=0)) and np.all(point <= rectangle.max(axis=0))
    analytic = original_togt_convex_jacobian(d, rectangle)
    h = 1.0e-6
    numeric = np.column_stack(
        [
            (original_togt_convex_map(d + np.eye(4)[i] * h, rectangle) - original_togt_convex_map(d - np.eye(4)[i] * h, rectangle))
            / (2.0 * h)
            for i in range(4)
        ]
    )
    assert np.allclose(analytic, numeric, rtol=1.0e-7, atol=1.0e-9)


def test_convex_hull_fills_nonconvex_notch() -> None:
    u_shape = np.array([[-2, -2], [2, -2], [2, 2], [1, 2], [1, -0.5], [-1, -0.5], [-1, 2], [-2, 2]], dtype=float)
    hull = convex_hull_polygon(u_shape)
    assert len(hull) == 4


def test_original_and_fixed_baseline_objective_and_optimizer() -> None:
    rectangle = np.array([[-1.0, -0.8], [1.0, -0.8], [1.0, 0.8], [-1.0, 0.8]])
    config = OptimizationConfig(
        max_iterations=2,
        samples_per_segment=4,
        objective_weights=ObjectiveWeights(time=1.0, snap_energy=1e-4),
        penalty_weights=PenaltyWeights(0.0, 0.0, 0.0, 0.0),
    )
    for kind in ("fixed", "original_convex"):
        window = StaticBaselineWindow(
            kind, rectangle, np.array([0.0, 0.0, 1.0]),
            np.array([0.0, np.pi / 2, 0.0]), kind=kind,
        )
        track = BaselineTrack(
            kind, np.array([-2.0, 0.0, 1.0]), np.array([2.0, 0.0, 1.0]), (window,)
        )
        objective = BaselineTOGTObjective(track, config)
        x = objective.initial_guess()
        value, gradient = objective.value_and_gradient(x)
        assert np.isfinite(value)
        assert gradient.shape == x.shape
        result = optimize_baseline_track(track, config=config, initial_x=x)
        assert np.all(result.durations > 0.0)
        assert window.contains(result.waypoints[0])
