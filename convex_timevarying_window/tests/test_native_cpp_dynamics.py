import numpy as np
import pytest

from convex_timevarying_window.togt.experiment import TOGT_MAX_TILT, build_track
from convex_timevarying_window.togt.native_backend import AnalyticTOGTCore


def test_standard_constraint_configuration_matches_released_cpp():
    _, config = build_track()
    limits = config.dynamic_limits
    assert limits.max_velocity == 60.0
    assert limits.max_body_rate_xy == 10.0
    assert limits.max_body_rate_z == 10.0
    assert TOGT_MAX_TILT == 6.28
    assert limits.min_rotor_thrust == 0.25
    assert limits.max_rotor_thrust == 5.0
    assert limits.min_collective_thrust == 1.0
    assert limits.max_collective_thrust == 20.0
    assert config.penalty_weights.velocity == 0.0
    assert config.penalty_weights.body_rate == 1.0
    assert config.penalty_weights.rotor_thrust == 1.0


def test_native_cpp_dynamics_returns_released_quada_hover_state():
    pvajs = np.zeros((1, 5, 3))
    sample = AnalyticTOGTCore().sample_dynamics(pvajs)
    assert sample["regular_branch"].tolist() == [True]
    assert sample["speed"][0] == 0.0
    assert sample["tilt"][0] == 0.0
    assert sample["body_rate"][0] == pytest.approx(np.zeros(3), abs=1e-14)
    assert sample["collective_thrust"][0] == pytest.approx(9.8066, abs=1e-13)
    assert sample["rotor_thrusts"][0] == pytest.approx(
        np.full(4, 9.8066 / 4.0), abs=1e-13
    )
    assert sample["instantaneous_penalty"][0] == 0.0
