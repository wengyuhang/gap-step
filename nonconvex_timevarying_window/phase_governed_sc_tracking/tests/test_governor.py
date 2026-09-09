from __future__ import annotations

import numpy as np

from nonconvex_timevarying_window.phase_governed_sc_tracking.governor import (
    DelaySearchConfig,
    WaitThenTrackTrajectory,
)
from nonconvex_timevarying_window.sc_dynatogt.minco import BoundaryState, MincoSnap


def test_wait_then_track_preserves_the_nominal_path_and_derivatives() -> None:
    nominal = MincoSnap(
        BoundaryState(np.asarray((-1.0, 0.0, 0.5))),
        BoundaryState(np.asarray((1.0, 0.2, 0.5))),
        np.empty((0, 3)),
        np.asarray((1.4,)),
    )
    delayed = WaitThenTrackTrajectory(nominal, 0.25)
    assert np.allclose(delayed.evaluate(0.1), nominal.evaluate(0.0))
    for derivative in range(6):
        assert np.allclose(
            delayed.evaluate(0.25 + 0.7, derivative),
            nominal.evaluate(0.7, derivative),
            atol=1.0e-11,
        )
    assert np.isclose(delayed.total_time, nominal.total_time + 0.25)
    assert np.max(delayed.interface_residuals()) < 1.0e-10


def test_delay_search_config_rejects_invalid_steps() -> None:
    try:
        DelaySearchConfig(max_delay=1.0, delay_step=0.0)
    except ValueError:
        pass
    else:
        raise AssertionError("zero delay step must be rejected")
