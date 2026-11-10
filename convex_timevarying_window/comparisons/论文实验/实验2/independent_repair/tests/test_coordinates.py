import numpy as np
from scipy.optimize._numdiff import approx_derivative
from ..coordinates import ArrivalCoordinates


def test_roundtrip_and_chain_derivative():
    origin = np.array([1., .5, -.3, 2., -1.])
    scale = np.array([.1, .2, .3, 12., 8.])
    lower = np.r_[(np.log(.3)-origin[:3])/scale[:3], -2., -3.]
    upper = np.r_[(np.log(15.)-origin[:3])/scale[:3], 2., 3.]
    c = ArrivalCoordinates(origin, scale, 3, list(zip(lower, upper)))
    y = np.array([.2, -.1, .3, .1, -.4])
    z = c.pack(y)
    rebuilt, derivative = c.unpack(z)
    np.testing.assert_allclose(rebuilt, y, atol=2e-15)
    np.testing.assert_allclose(approx_derivative(lambda x: c.unpack(x)[0], z), derivative, atol=2e-8)
    np.testing.assert_allclose(approx_derivative(c.margins, z), c.margin_jacobian(z), atol=2e-9)
    assert c.margins(z).min() > 0


def test_invalid_arrival_order_is_finite_and_rejected_by_temporal_margins():
    c = ArrivalCoordinates(np.zeros(3), np.ones(3), 3, [(-2., 2.)]*3)
    z = np.array([2., 1., 3.])
    y, derivative = c.unpack(z)
    assert np.isfinite(y).all() and np.isfinite(derivative).all()
    assert c.margins(z).min() < 0
