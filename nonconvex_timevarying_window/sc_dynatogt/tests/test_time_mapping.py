from __future__ import annotations

import numpy as np

from nonconvex_timevarying_window.sc_dynatogt.time_mapping import (
    add_traversal_time_gradients,
    backpropagate_to_k,
    duration_jacobian_diagonal,
    durations_from_k,
    k_from_durations,
    traversal_times,
)


def test_togt_time_mapping_round_trip_and_gradient() -> None:
    k = np.array([-2.1, -0.4, 0.0, 0.3, 2.0])
    durations = durations_from_k(k)
    assert np.all(durations > 0.0)
    assert np.allclose(k_from_durations(durations), k, atol=1.0e-12)

    upstream = np.array([0.7, -0.1, 0.3, 1.2, -0.4])
    analytic = backpropagate_to_k(k, upstream)
    h = 1.0e-6
    numeric = np.empty_like(k)
    for i in range(len(k)):
        plus, minus = k.copy(), k.copy()
        plus[i] += h
        minus[i] -= h
        numeric[i] = (upstream @ durations_from_k(plus) - upstream @ durations_from_k(minus)) / (2.0 * h)
    assert np.allclose(analytic, numeric, rtol=1.0e-7, atol=1.0e-9)


def test_prefix_time_gradient_accumulation() -> None:
    durations = np.array([0.5, 0.7, 0.9, 1.1])
    assert np.allclose(traversal_times(durations), [0.5, 1.2, 2.1])
    direct = np.array([1.0, 2.0, 3.0, 4.0])
    crossing = np.array([0.2, 0.3, 0.5])
    assert np.allclose(add_traversal_time_gradients(direct, crossing), [2.0, 2.8, 3.5, 4.0])


def test_extreme_finite_variables_preserve_positive_duration_contract() -> None:
    k = np.array([-1.0e308, -1.0e155, 1.0e155, 1.0e308])
    durations = durations_from_k(k)
    assert np.all(np.isfinite(durations))
    assert np.all(durations > 0.0)
    assert np.all(np.isfinite(duration_jacobian_diagonal(k)))
