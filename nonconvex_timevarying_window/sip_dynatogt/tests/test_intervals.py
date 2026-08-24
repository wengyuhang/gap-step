from __future__ import annotations

import math
import numpy as np

from nonconvex_timevarying_window.sc_dynatogt.boundary import BSpline, Bezier, CircularArc, Line
from nonconvex_timevarying_window.sc_dynatogt.environment import MotionProfile
from nonconvex_timevarying_window.sip_dynatogt.intervals import (
    boundary_interval,
    boundary_parameter_spans,
    flatness_interval,
    interval_ball,
    window_state_interval,
)
from nonconvex_timevarying_window.sip_dynatogt.model import PolynomialTrajectory, SIPConfig, SIPWindow


def _contains_vector(intervals, point):
    assert all(value.contains(float(sample)) for value, sample in zip(intervals, point))


def test_supported_boundary_interval_extensions_contain_dense_values():
    segments = (
        Line((-1.0, 0.2), (1.5, -0.4)),
        CircularArc((0.2, -0.1), 1.3, -0.4, 1.7),
        Bezier(((-1.0, 0.0), (-0.2, 1.4), (0.8, -0.7), (1.2, 0.3))),
        BSpline(
            ((-1.0, 0.0), (-0.4, 1.1), (0.4, -0.5), (1.0, 0.2), (1.4, 0.0)),
            degree=3,
        ),
    )
    for segment in segments:
        for lo, hi in boundary_parameter_spans(segment):
            enclosure = boundary_interval(segment, interval_ball(lo, hi))
            for parameter in np.linspace(lo, hi, 17):
                _contains_vector(enclosure, segment.evaluate(float(parameter)))


def test_bspline_whole_interval_contains_a_non_dyadic_normalized_knot():
    segment = BSpline(
        ((-1.0, 0.0), (-0.4, 1.1), (0.4, -0.5), (1.0, 0.2), (1.4, 0.0)),
        degree=3,
        knots=(2.0, 2.0, 2.0, 2.0, 3.0, 5.0, 5.0, 5.0, 5.0),
    )
    # The interior knot is u=(3-2)/(5-2)=1/3, which binary64 cannot encode.
    # The interval evaluator must union both adjacent de Boor spans.
    enclosure = boundary_interval(segment, interval_ball(0.0, 1.0))
    for parameter in np.linspace(0.0, 1.0, 101):
        _contains_vector(enclosure, segment.evaluate(float(parameter)))


def test_window_motion_interval_contains_translation_rotation_and_scale():
    points = [(-1, -1), (1, -1), (1, 1), (-1, 1)]
    boundary = tuple(Line(points[i], points[(i + 1) % 4]) for i in range(4))
    motion = MotionProfile(
        np.array([0.3, 0.2, 0.1]),
        np.array([0.15, 0.1, 0.08]),
        0.12,
        phase=0.3,
    )
    window = SIPWindow("moving", np.array([1.0, -2.0, 0.5]), np.array([0.1, 0.2, -0.3]), motion, boundary)
    center, rotation, scale = window_state_interval(window, interval_ball(0.7, 1.1))
    for time in np.linspace(0.7, 1.1, 9):
        c, r, s = window.state_at(float(time))
        _contains_vector(center, c)
        assert scale.contains(float(s))
        for row in range(3):
            _contains_vector(rotation[row], r[row])


def test_hover_flatness_interval_contains_exact_hover_state():
    trajectory = PolynomialTrajectory(np.array([1.0]), np.zeros((1, 8, 3)))
    flat = flatness_interval(trajectory, 0, interval_ball(0.0, 1.0), SIPConfig())
    _contains_vector(flat.position, np.zeros(3))
    _contains_vector(flat.velocity, np.zeros(3))
    _contains_vector(flat.body_rate, np.zeros(3))
    assert flat.collective_thrust.contains(9.8066)
    for thrust in flat.rotor_thrusts:
        assert thrust.contains(9.8066 / 4.0)
    for row in range(3):
        _contains_vector(flat.rotation[row], np.eye(3)[row])
