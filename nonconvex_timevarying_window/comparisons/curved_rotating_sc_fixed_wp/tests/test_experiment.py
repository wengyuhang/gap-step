import numpy as np

from nonconvex_timevarying_window.comparisons.curved_rotating_sc_fixed_wp.experiment import (
    FixedMultiWindowObjective,
    SHAPES,
    build_curved_track,
    embedding_errors,
    fixed_waypoints,
)
from nonconvex_timevarying_window.random_dk_sc_dynatogt.multi_window import MultiWindowObjective


def test_curved_track_and_fixed_embedding_are_well_defined():
    scenario, config = build_curved_track(vertex_count=256, quadrature_order=64)
    assert tuple(window.name for window in scenario.windows) == SHAPES
    assert all(window.thickness == 0.0 for window in scenario.windows)
    assert all(np.allclose(window.normal, (1.0, 0.0, 0.0)) for window in scenario.windows)
    local, fixed_d = fixed_waypoints(scenario)
    assert local.shape == fixed_d.shape == (3, 2)
    free = MultiWindowObjective(scenario, config)
    fixed = FixedMultiWindowObjective(free, fixed_d)
    k = fixed.initial_guess()
    fixed_forward = fixed.forward(k)
    free_forward = free.forward(fixed.full_x(k))
    errors = embedding_errors(fixed_forward, free_forward)
    assert max(errors["derivative_max_abs_errors"].values()) < 1e-12
