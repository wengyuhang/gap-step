from __future__ import annotations

import numpy as np

from nonconvex_timevarying_window.sc_dynatogt.minco import BoundaryState, MincoSnap
from nonconvex_timevarying_window.timewarp_sc_tracking import (
    LocalTimeWarpTrajectory,
    TimeWarpPatch,
)


def _nominal():
    return MincoSnap(
        BoundaryState(np.asarray((-1.0, 0.0, 0.5))),
        BoundaryState(np.asarray((1.0, 0.2, 0.5))),
        np.empty((0, 3)),
        np.asarray((2.0,)),
    )


def test_patch_recovers_the_complete_downstream_schedule() -> None:
    nominal = _nominal()
    warped = LocalTimeWarpTrajectory(
        nominal, (TimeWarpPatch(0.4, 1.2, -0.04),)
    )
    assert warped.total_time == nominal.total_time
    assert warped.warp_time(0.4) == 0.4
    assert warped.warp_time(1.2) == 1.2
    for instant in (0.1, 1.2, 1.5, 2.0):
        for derivative in range(6):
            assert np.allclose(
                warped.evaluate(instant, derivative),
                nominal.evaluate(instant, derivative),
                atol=1.0e-10,
            )
    assert np.min(warped.time_derivative(np.linspace(0.0, 2.0, 1001))) > 0.0
    assert np.max(warped.interface_residuals()) < 1.0e-10


def test_multiple_disjoint_gate_patches_leave_later_times_unchanged() -> None:
    nominal = _nominal()
    warped = LocalTimeWarpTrajectory(
        nominal,
        (
            TimeWarpPatch(0.2, 0.7, 0.01),
            TimeWarpPatch(1.0, 1.5, -0.01),
        ),
    )
    assert np.allclose(warped.evaluate(0.8), nominal.evaluate(0.8))
    assert np.allclose(warped.evaluate(1.8), nominal.evaluate(1.8))
