from __future__ import annotations

import numpy as np
import pytest

from nonconvex_timevarying_window.sc_dynatogt.environment import (
    MotionProfile,
    SCDynamicWindow,
    SCWindowTrack,
)
from nonconvex_timevarying_window.sc_dynatogt.sc_mapping import SCDiskMap


@pytest.fixture(scope="session")
def square_polygon() -> np.ndarray:
    return np.array([[-1.0, -1.0], [1.0, -1.0], [1.0, 1.0], [-1.0, 1.0]])


@pytest.fixture(scope="session")
def static_track(square_polygon: np.ndarray) -> SCWindowTrack:
    window = SCDynamicWindow(
        "square",
        SCDiskMap.fit(square_polygon, quadrature_order=24),
        square_polygon,
        np.array([0.0, 0.0, 1.0]),
        np.array([0.0, np.pi / 2.0, 0.0]),
        MotionProfile.static(),
        physical_boundary=square_polygon,
    )
    return SCWindowTrack(
        "square_track",
        np.array([-2.0, 0.0, 1.0]),
        np.array([2.0, 0.0, 1.0]),
        (window,),
        (0,),
    )


@pytest.fixture(scope="session")
def dynamic_track(square_polygon: np.ndarray) -> SCWindowTrack:
    motion = MotionProfile(
        np.array([0.05, 0.08, 0.04]),
        np.array([0.03, 0.02, 0.04]),
        0.03,
        phase=0.2,
    )
    window = SCDynamicWindow(
        "moving_square",
        SCDiskMap.fit(square_polygon, quadrature_order=24),
        square_polygon,
        np.array([0.0, 0.0, 1.0]),
        np.array([0.0, np.pi / 2.0, 0.0]),
        motion,
        physical_boundary=square_polygon,
    )
    return SCWindowTrack(
        "moving_square_track",
        np.array([-2.0, 0.0, 1.0]),
        np.array([2.0, 0.0, 1.0]),
        (window,),
        (0,),
    )
