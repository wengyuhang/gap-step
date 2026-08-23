from __future__ import annotations

import numpy as np
import pytest

from nonconvex_timevarying_window.cwb_sc_dynatogt.sc_inverse import inverse_sc_map, sc_margin


def test_newton_inverse_round_trip_and_margin(square_map) -> None:
    for point in (np.array([0.0, 0.0]), np.array([0.2, -0.3]), np.array([-0.7, 0.1])):
        mapped = square_map.evaluate(point)
        result = inverse_sc_map(square_map, mapped, tolerance=1e-9)
        assert result.converged
        assert np.allclose(result.z, point, atol=2e-7)
    margin, result = sc_margin(square_map, square_map.evaluate([0.5, 0.0]), 0.9)
    assert result.converged
    assert np.isclose(margin, 0.9**2 - 0.5**2, atol=2e-7)


def test_inverse_rejects_outside_domain(square_map) -> None:
    with pytest.raises(ValueError, match="outside"):
        inverse_sc_map(square_map, [2.0, 0.0])
