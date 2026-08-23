from __future__ import annotations

import numpy as np
import pytest

from nonconvex_timevarying_window.sc_dynatogt.sc_mapping import SCDiskMap


@pytest.fixture(scope="session")
def square_map() -> SCDiskMap:
    polygon = np.array([[-1.0, -1.0], [1.0, -1.0], [1.0, 1.0], [-1.0, 1.0]])
    return SCDiskMap.fit(polygon, quadrature_order=48)


class LinearTrajectory:
    """Minimal trajectory protocol: p(t)=(t,0,0), constant-yaw flatness."""

    durations = np.array([4.0])
    total_time = 4.0

    def evaluate(self, time, derivative: int = 0):
        values = np.asarray(time)
        if derivative == 0:
            return np.stack((values, np.zeros_like(values), np.zeros_like(values)), axis=-1)
        if derivative == 1:
            return np.broadcast_to(np.array([1.0, 0.0, 0.0]), values.shape + (3,)).copy()
        return np.zeros(values.shape + (3,))


class StaticXPlaneWindow:
    def __init__(self, mapping, half_size: float = 1.0):
        self.name = "square"
        self.sc_map = mapping
        self.safe_polygon = half_size * np.array(
            [[-1.0, -1.0], [1.0, -1.0], [1.0, 1.0], [-1.0, 1.0]]
        )
        self.center0 = np.array([2.0, 0.0, 0.0])

    def state_at(self, time: float):
        del time
        basis = np.array([[0.0, 0.0], [1.0, 0.0], [0.0, 1.0]])
        return self.center0.copy(), basis, 1.0, np.zeros(3), np.zeros((3, 2)), 0.0
