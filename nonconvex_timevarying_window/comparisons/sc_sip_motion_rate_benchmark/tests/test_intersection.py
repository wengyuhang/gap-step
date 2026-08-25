from __future__ import annotations

import numpy as np
import pytest

pytest.importorskip("flint")

from nonconvex_timevarying_window.comparisons.sc_sip_motion_rate_benchmark.intersection import (
    IntersectionStatus, certify_physical_intersection,
)
from nonconvex_timevarying_window.sc_dynatogt.boundary import Line
from nonconvex_timevarying_window.sc_dynatogt.environment import MotionProfile
from nonconvex_timevarying_window.sip_dynatogt.model import PolynomialTrajectory, SIPConfig, SIPProblem, SIPWindow


def test_original_boundary_strictly_inside_body_is_confirmed() -> None:
    boundary = (
        Line((-0.10, 0.0), (0.10, 0.0)), Line((0.10, 0.0), (0.10, 0.20)),
        Line((0.10, 0.20), (-0.10, 0.20)), Line((-0.10, 0.20), (-0.10, 0.0)),
    )
    window = SIPWindow("inside", np.zeros(3), np.zeros(3), MotionProfile(np.zeros(3), np.zeros(3), 0.0), boundary)
    problem = SIPProblem("inside", (window,), (0,))
    trajectory = PolynomialTrajectory(np.ones(2), np.zeros((2, 8, 3)))
    report = certify_physical_intersection(problem, trajectory, SIPConfig(max_cells=1000, max_depth=10, precision_bits=(128,)))
    assert report.status is IntersectionStatus.PHYSICAL_INTERSECTION_CONFIRMED
    assert report.witness is not None
    assert min(report.witness.axis_interior_margins) > 0.0
