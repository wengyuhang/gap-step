from __future__ import annotations

import math
import numpy as np

import nonconvex_timevarying_window.sip_dynatogt.constraints as constraint_module

from nonconvex_timevarying_window.sc_dynatogt.boundary import Line
from nonconvex_timevarying_window.sc_dynatogt.dynamics import DynamicLimits
from nonconvex_timevarying_window.sc_dynatogt.environment import MotionProfile
from nonconvex_timevarying_window.sip_dynatogt.certificate import certify
from nonconvex_timevarying_window.sip_dynatogt.constraints import (
    initial_witnesses,
    witness_constraint_values,
)
from nonconvex_timevarying_window.sip_dynatogt.model import (
    CertificateStatus,
    PolynomialTrajectory,
    SIPConfig,
    SIPProblem,
    SIPWindow,
)


def _square(radius: float):
    p = [(-radius, -radius), (radius, -radius), (radius, radius), (-radius, radius)]
    return tuple(Line(p[i], p[(i + 1) % 4]) for i in range(4))


def _hover_problem(*, center=(10.0, 0.0, 0.0), radius=1.0, motion=None):
    window = SIPWindow(
        "window",
        np.asarray(center, dtype=float),
        np.zeros(3),
        MotionProfile.static() if motion is None else motion,
        _square(radius),
    )
    return SIPProblem("hover", (window,), (0,))


def _hover_trajectory():
    return PolynomialTrajectory(np.array([0.5, 0.5]), np.zeros((2, 8, 3)))


def _loose_limits(max_velocity=100.0):
    return DynamicLimits(
        max_velocity=max_velocity,
        max_body_rate_xy=1e6,
        max_body_rate_z=1e6,
        min_rotor_thrust=-1e6,
        max_rotor_thrust=1e6,
    )


def test_safe_hover_is_certified_and_small_window_collision_is_rejected():
    config = SIPConfig(dynamic_limits=_loose_limits(), precision_bits=(128,), max_cells=20_000)
    safe = certify(_hover_problem(), _hover_trajectory(), config)
    assert safe.status is CertificateStatus.CERTIFIED_FEASIBLE
    unsafe = certify(_hover_problem(center=(0, 0, 0), radius=0.1), _hover_trajectory(), config)
    assert unsafe.status is CertificateStatus.VIOLATED
    assert any(witness.kind == "safety" for witness in unsafe.witnesses)


def test_interval_separator_finds_collision_between_endpoint_midpoint_samples():
    # With period 0.5 and phase -pi/2 the small frame is at x=+-2 at every
    # initial node, but crosses the hovering body at t=0.125, 0.375, ...
    motion = MotionProfile(
        np.array([2.0, 0.0, 0.0]),
        np.zeros(3),
        0.0,
        translation_period=0.5,
        phase=-math.pi / 2,
        rotation_enabled=False,
        scale_enabled=False,
    )
    problem = _hover_problem(center=(0, 0, 0), radius=0.1, motion=motion)
    config = SIPConfig(dynamic_limits=_loose_limits(), precision_bits=(128,), max_cells=30_000)
    report = certify(problem, _hover_trajectory(), config)
    assert report.status is CertificateStatus.VIOLATED
    assert report.checked_cells > 0  # the coarse endpoint/midpoint pass did not find it
    assert report.witnesses[0].source.startswith("arb-")


def test_interval_separator_finds_narrow_velocity_peak_between_samples():
    coefficients = np.zeros((2, 8, 3))
    duration = 0.5
    amplitude = 100.0
    # p=A*t^2*(T-t)^2: velocity is zero at t=0,T/2,T but exceeds 1 between.
    coefficients[0, 2, 0] = amplitude * duration**2
    coefficients[0, 3, 0] = -2 * amplitude * duration
    coefficients[0, 4, 0] = amplitude
    trajectory = PolynomialTrajectory(np.array([duration, duration]), coefficients)
    config = SIPConfig(
        dynamic_limits=_loose_limits(max_velocity=1.0),
        precision_bits=(128,),
        max_cells=30_000,
    )
    report = certify(_hover_problem(), trajectory, config)
    assert report.status is CertificateStatus.VIOLATED
    assert report.checked_cells > 0
    assert report.witnesses[0].kind == "velocity"


def _cubic_acceleration_trajectory(amplitudes):
    """Return p with p''=A*t*(t-1/2)*(t-1) on the first piece."""

    coefficients = np.zeros((2, 8, 3))
    amplitudes = np.asarray(amplitudes, dtype=float)
    coefficients[0, 3] = amplitudes / 12.0
    coefficients[0, 4] = -amplitudes / 8.0
    coefficients[0, 5] = amplitudes / 20.0
    return PolynomialTrajectory(np.array([1.0, 1.0]), coefficients)


def test_interval_separator_finds_rotor_thrust_peak_between_samples():
    # Acceleration is zero at tau=0,1/2,1, so all initial rotor samples hover
    # at g/4.  Between them the vertical acceleration pushes one rotor bound
    # above 3 N.
    trajectory = _cubic_acceleration_trajectory([0.0, 0.0, 60.0])
    limits = DynamicLimits(
        max_velocity=100.0,
        max_body_rate_xy=100.0,
        max_body_rate_z=100.0,
        min_rotor_thrust=0.0,
        max_rotor_thrust=3.0,
    )
    config = SIPConfig(dynamic_limits=limits, precision_bits=(128,), max_cells=30_000)
    report = certify(_hover_problem(), trajectory, config)
    assert report.status is CertificateStatus.VIOLATED
    assert report.checked_cells > 0
    assert report.witnesses[0].kind.endswith("_upper")


def test_interval_separator_finds_heading_singularity_between_samples():
    # At t=1/4, choose a+g*e3=(0,1,0), parallel to the constant-yaw heading-y
    # vector.  The endpoint/midpoint samples all have the regular hover frame.
    q_quarter = 3.0 / 64.0
    trajectory = _cubic_acceleration_trajectory(
        [0.0, 1.0 / q_quarter, -9.8066 / q_quarter]
    )
    config = SIPConfig(
        dynamic_limits=_loose_limits(),
        precision_bits=(128,),
        max_cells=30_000,
        max_depth=24,
    )
    report = certify(_hover_problem(), trajectory, config)
    assert report.status is CertificateStatus.VIOLATED
    assert report.checked_cells > 0
    assert report.witnesses[0].kind == "heading_cross_singularity"


def test_budget_exhaustion_is_unresolved_never_safe():
    config = SIPConfig(dynamic_limits=_loose_limits(), precision_bits=(128,), max_cells=1)
    report = certify(_hover_problem(), _hover_trajectory(), config)
    assert report.status is CertificateStatus.UNRESOLVED
    assert not report.certified


def test_finite_activity_evaluation_reuses_flatness_at_shared_time_nodes(monkeypatch):
    problem = _hover_problem()
    trajectory = _hover_trajectory()
    config = SIPConfig(dynamic_limits=_loose_limits())
    witnesses = initial_witnesses(problem, trajectory.num_segments, config)
    original = constraint_module.point_flatness
    calls = []

    def counted(*args, **kwargs):
        calls.append((args[1], args[2]))
        return original(*args, **kwargs)

    monkeypatch.setattr(constraint_module, "point_flatness", counted)
    values = witness_constraint_values(problem, trajectory, witnesses, config)
    assert np.all(np.isfinite(values))
    assert len(calls) == trajectory.num_segments * len(config.initial_nodes)
    assert len(set(calls)) == len(calls)
