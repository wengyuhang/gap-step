import numpy as np
import pytest

from nonconvex_timevarying_window.exact_area_sc_dynatogt.penalty import (
    instantaneous_penalty,
    instantaneous_penalty_gradient,
    integrated_penalty_gradient,
)


def test_stable_topology_chain_rule_matches_centered_difference():
    def areas(x):
        return 2.0 + 0.3 * x, 0.8 - 0.1 * x

    x = 0.2
    area_a, area_c = areas(x)
    analytic = instantaneous_penalty_gradient(area_a, area_c, np.array([0.3]), np.array([-0.1]))[0]
    step = 1.0e-6
    plus = instantaneous_penalty(*areas(x + step))
    minus = instantaneous_penalty(*areas(x - step))
    assert np.isclose(analytic, (plus - minus) / (2.0 * step), rtol=1e-7, atol=1e-10)


def test_a_zero_is_value_extension_not_gradient_claim():
    assert instantaneous_penalty(0.0, 0.0) == 0.0
    with pytest.raises(ValueError):
        instantaneous_penalty_gradient(0.0, 0.0, np.zeros(1), np.zeros(1))


def test_integrated_gradient_keeps_total_time_terminal_term():
    times = np.array([0.0, 1.0])
    gradients = np.array([[1.0, 2.0], [3.0, 4.0]])
    result = integrated_penalty_gradient(
        times, gradients, terminal_penalty=0.5, total_time_gradient=np.array([2.0, -2.0])
    )
    assert np.allclose(result, np.array([3.0, 2.0]))
