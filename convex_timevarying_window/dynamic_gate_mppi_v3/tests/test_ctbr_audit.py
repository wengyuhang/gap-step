import numpy as np
from scipy.spatial.transform import Rotation
from convex_timevarying_window.dynamic_gate_mppi_v3.audit_ctbr import body_center_position


def test_rotating_model_origin_does_not_create_body_translation():
    r=Rotation.from_euler('xyz',np.column_stack([np.linspace(-.5,.5,21),np.zeros((21,2))]))
    center=np.tile([1.,2.,3.],(21,1))
    origin=center-r.apply(np.tile([0.,0.,-.24],(21,1)))
    np.testing.assert_allclose(body_center_position(origin,r.as_quat()),center,atol=1e-12)
