import numpy as np
from convex_timevarying_window.dynamic_gate_mppi_v3.state_consistency import PositionVelocityObserver


def test_detects_position_correction_without_ground_truth():
    o=PositionVelocityObserver();v=np.array([2.,0,0]);drift=np.array([.5,-.3,.1])
    for t in np.arange(0,1,.008):result=o.update(t,(v+drift)*t,v)
    np.testing.assert_allclose(result,v+drift,atol=1e-5)


def test_linear_acceleration_does_not_create_fake_bias():
    o=PositionVelocityObserver();a=np.array([1.,2.,-.5])
    for t in np.arange(0,1,.008):result=o.update(t,.5*a*t*t,a*t)
    np.testing.assert_allclose(result,a*t,atol=1e-12)
    previous=o.correction.copy();o.update(t,np.ones(3)*99,a*t)
    np.testing.assert_array_equal(previous,o.correction)
