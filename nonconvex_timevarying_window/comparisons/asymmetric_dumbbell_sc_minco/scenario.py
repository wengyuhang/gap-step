"""A physical simple polygon, not its convex hull or an eroded SC domain."""
from dataclasses import dataclass
import numpy as np
from shapely.geometry import Point, Polygon

from convex_timevarying_window.geometry import ConvexAperture, PeriodicConvexWindow
from nonconvex_timevarying_window.sc_dynatogt.environment import SCWindowTrack
from 复现.polynomial_dynamic_racing_2023.px4_fixed_plane.scenario import PlanarMotion


VERTICES = np.array([
    [-1.6, -.6], [-.4, -.6], [-.4, -.2], [0., -.2],
    [0., -1.6], [3.2, -1.6], [3.2, 1.6], [0., 1.6],
    [0., .2], [-.4, .2], [-.4, .6], [-1.6, .6],
])
LEFT_INITIAL_POINT = np.array([-1., 0.])


class NonconvexAperture(ConvexAperture):
    def __post_init__(self):
        super().__post_init__()
        polygon = Polygon(self.physical_vertices)
        if not polygon.is_valid or not polygon.exterior.is_ccw or polygon.interiors:
            raise ValueError('Expected a CCW simple polygon without holes')

    def contains(self, q, tolerance=1e-9, safe=False):
        # The parent uses convex half-planes, which are NOT valid here.
        polygon = Polygon(self.physical_vertices)
        return bool(polygon.buffer(tolerance).covers(Point(np.asarray(q, float))))


@dataclass(frozen=True)
class Case:
    frequency: float = 3.
    amplitude: float = 2.
    phase: float = 0.

    @property
    def name(self):
        return f'f{self.frequency:g}_a{self.amplitude:g}'


def room(q):
    if q[0] < -.4:
        return 'left'
    if q[0] > 0.:
        return 'right'
    return 'corridor'


def build_case(case=Case()):
    if case.frequency <= 0 or case.amplitude < 0:
        raise ValueError('Positive frequency and nonnegative amplitude required')
    aperture = NonconvexAperture('polygon', physical_vertices=VERTICES.copy(),
                               safe_vertices=VERTICES.copy())
    motion = PlanarMotion(np.array([case.amplitude, 0., 0.]),
                          1 / case.frequency, case.phase)
    window = PeriodicConvexWindow('asymmetric_dumbbell', aperture,
                                 np.array([0., 0., 3.2]),
                                 np.array([np.pi / 2, 0., 0.]), motion)
    return SCWindowTrack('left_seeded_dumbbell', np.array([0., -6., 3.2]),
                         np.array([0., 6., 3.2]), (window,), (0,))
