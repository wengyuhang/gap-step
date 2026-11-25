import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from convex_timevarying_window.comparisons.论文实验.实验2.compact_calibration import scenario

@pytest.mark.parametrize('layout',scenario.LAYOUTS)
def test_compact_geometry_and_uniform_x500_inset(layout):
    case=scenario.Case(layout,'static');old,_=scenario.parent.build_case(case);new,_=scenario.build_case(case)
    np.testing.assert_array_equal(new.start,old.start);np.testing.assert_array_equal(new.goal,old.goal)
    rng=np.random.default_rng(31)
    for i,(a,b) in enumerate(zip(old.windows,new.windows)):
        expected=new.start[:2]+.25*(a.center0[:2]-old.start[:2])
        if i==0:expected+=scenario.FIRST_GATE_SIDE_SHIFT_M*a.R[:2,1]
        np.testing.assert_allclose(b.center0[:2],expected)
        assert a.center0[2]==b.center0[2]
        if layout=='height':np.testing.assert_allclose(b.R[:,2],scenario.HEIGHT_NORMALS[i],atol=1e-14)
        else:np.testing.assert_array_equal(a.R,b.R)
        np.testing.assert_allclose(b.R.T@b.R,np.eye(3),atol=1e-14)
        np.testing.assert_allclose(Rotation.from_euler('xyz',b.angles0_rpy).as_matrix(),b.R,atol=1e-14)
        assert a.aperture.radius==b.aperture.radius and b.aperture.margin==scenario.WINDOW_MARGIN
        if a.aperture.kind=='polygon':
            np.testing.assert_array_equal(a.aperture.physical_vertices,b.aperture.physical_vertices)
            assert np.max(np.linalg.norm(b.aperture.safe_vertices,axis=1))<np.max(np.linalg.norm(a.aperture.safe_vertices,axis=1))
        for _ in range(10):
            q,_=b.aperture.point_and_jacobian(rng.standard_normal(b.aperture.dimension))
            assert b.aperture.contains(q,safe=True) and a.aperture.contains(q,safe=True)
    metadata=scenario.manifest(case)
    assert not metadata['physical_apertures_scaled'] and not metadata['togt_original_course_reproduction']
    assert max(metadata['inter_gate_distances_m'])<17. and min(metadata['inter_gate_distances_m'])>5.
