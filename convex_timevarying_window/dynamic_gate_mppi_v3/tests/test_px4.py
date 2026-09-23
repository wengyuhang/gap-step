import numpy as np
from convex_timevarying_window.dynamic_gate_mppi_v3.px4 import motor_normalized
from convex_timevarying_window.dynamic_gate_mppi_v3.rollout import KF


def test_installed_px4_output_inverse():
    desired = np.array([KF*150**2, 3., 5., KF*1000**2])
    normalized = motor_normalized(desired)
    speed = 150+850*normalized
    np.testing.assert_allclose(KF*speed**2, desired, atol=1e-12)
