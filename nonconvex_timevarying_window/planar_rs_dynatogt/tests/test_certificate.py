import numpy as np

from nonconvex_timevarying_window.sc_dynatogt.boundary import Line
from nonconvex_timevarying_window.sc_dynatogt.dynamics import DynamicLimits
from nonconvex_timevarying_window.sip_dynatogt.certificate import certify as sip_certify
from nonconvex_timevarying_window.sip_dynatogt.model import (
    CertificateStatus,
    PolynomialTrajectory,
    SIPConfig,
    SIPProblem,
)

from nonconvex_timevarying_window.planar_rs_dynatogt import (
    PlanarRSConfig,
    PlanarRSMotion,
    PlanarRSSIPWindow,
    certify,
)


def _square(radius: float):
    points = [
        (-radius, -radius),
        (radius, -radius),
        (radius, radius),
        (-radius, radius),
    ]
    return tuple(Line(points[i], points[(i + 1) % 4]) for i in range(4))


def _limits():
    return DynamicLimits(
        max_velocity=100.0,
        max_body_rate_xy=1e6,
        max_body_rate_z=1e6,
        min_rotor_thrust=-1e6,
        max_rotor_thrust=1e6,
    )


def _trajectory():
    return PolynomialTrajectory(np.array([0.5, 0.5]), np.zeros((2, 8, 3)))


def _problem(center, radius=1.0):
    motion = PlanarRSMotion(
        angle_amplitude=1.2, angle_period=0.7, scale_amplitude=0.35, scale_period=0.9
    )
    window = PlanarRSSIPWindow(
        "window", np.asarray(center, dtype=float), np.eye(3), motion, _square(radius)
    )
    return SIPProblem("planar", (window,), (0,))


def _config():
    sip = SIPConfig(
        dynamic_limits=_limits(), precision_bits=(128,), max_cells=20_000, max_depth=20
    )
    return PlanarRSConfig(sip=sip, plane_prune_max_depth=10)


def test_fixed_plane_pruning_preserves_safe_status_and_reduces_cells():
    problem = _problem((0.0, 0.0, 10.0))
    trajectory = _trajectory()
    config = _config()
    baseline = sip_certify(problem, trajectory, config.sip)
    pruned = certify(problem, trajectory, config)
    assert baseline.status is pruned.status is CertificateStatus.CERTIFIED_FEASIBLE
    assert pruned.checked_cells < baseline.checked_cells
    assert pruned.minimum_safety_squared_margin is not None
    assert pruned.minimum_safety_squared_margin > 0.0


def test_physical_penetration_is_never_pruned():
    problem = _problem((0.0, 0.0, 0.0), radius=0.1)
    report = certify(problem, _trajectory(), _config())
    assert report.status is CertificateStatus.VIOLATED
    assert any(witness.kind == "safety" for witness in report.witnesses)
    assert all(witness.source.startswith("arb") for witness in report.witnesses)


def test_arbitrarily_oriented_plane_normal_stays_fixed():
    angle = 0.7
    c, s = np.cos(angle), np.sin(angle)
    fixed = np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])
    window = PlanarRSSIPWindow(
        "tilted",
        np.zeros(3),
        fixed,
        PlanarRSMotion(angle_amplitude=2.0, scale_amplitude=0.4),
        _square(1.0),
    )
    expected = fixed[:, 2]
    for time in np.linspace(0.0, 20.0, 41):
        np.testing.assert_allclose(
            window.state_at(float(time))[1][:, 2], expected, rtol=0.0, atol=1e-14
        )
