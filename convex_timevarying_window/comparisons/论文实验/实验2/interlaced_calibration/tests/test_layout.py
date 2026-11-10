import numpy as np
import pytest
from convex_timevarying_window.comparisons.论文实验.实验2.interlaced_calibration import scenario
from convex_timevarying_window.single_gate_stop_racing.scenario import build_case,Case


@pytest.mark.parametrize('layout',scenario.LAYOUTS)
def test_actual_historical_planes_shapes_and_nearby_anchors(layout):
    old,_=build_case(Case(1.,0.,0.));tr,_=scenario.build_case(scenario.Case(layout,'static'))
    assert len(tr.windows)==7 and np.array_equal(tr.start,old.start)
    for i,(w,v) in enumerate(zip(tr.windows,old.windows)):
        np.testing.assert_allclose(w.R.T@w.R,np.eye(3),atol=1e-14)
        np.testing.assert_allclose(w.R[:,0],[0.,0.,1.],atol=1e-14)
        assert w.aperture.kind==v.aperture.kind and w.aperture.radius==v.aperture.radius
        if w.aperture.kind=='polygon':np.testing.assert_array_equal(w.aperture.physical_vertices,v.aperture.physical_vertices)
        anchor=old.windows[scenario.ANCHOR_ORDER[i]].center0
        if layout=='original':np.testing.assert_array_equal(w.center0,anchor)
        else:assert np.linalg.norm(w.center0-anchor)<3.1
        assert w.center0[2]>1.6
    # Historical progression contains a genuine horizontal reversal after W3.
    assert tr.windows[2].center0[0]-tr.windows[4].center0[0]>55.
    assert tr.windows[5].center0[1]-tr.windows[4].center0[1]>50.
