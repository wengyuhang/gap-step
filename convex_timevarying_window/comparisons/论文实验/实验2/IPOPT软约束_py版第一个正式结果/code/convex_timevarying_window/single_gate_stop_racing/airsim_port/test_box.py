import numpy as np
import casadi as ca
from .box_geometry import SIGNS,symbolic,numeric_clearance
from ..scenario import build_case,Case


def test_box_slice_symbolic_equals_vectorized_geometry():
    from scipy.spatial.transform import Rotation
    track,_=build_case(Case(4,4,0));rng=np.random.default_rng(123)
    half=np.array([.492,.492,.148]);offset=np.array([0,0,-.033])
    for aperture in (track.windows[0].aperture,track.windows[1].aperture):
        q=ca.MX.sym('q',3);r=ca.MX.sym('r',3,3)
        f=ca.Function('box',[q,r],[ca.vertcat(*symbolic(aperture,q,r,half,offset,.085,.09))])
        for _ in range(50):
            p=rng.uniform(-1,1,3);rot=Rotation.random(random_state=rng).as_matrix()
            corners=(p+(SIGNS*half+offset)@rot.T)[None]
            n=numeric_clearance(aperture,corners,.085,.09)[0]
            s=np.asarray(f(p,rot)).min()
            np.testing.assert_allclose(s,n,atol=1e-10)


def test_full_body_outside_slab_does_not_require_aperture_containment():
    aperture=build_case(Case(4,4,0))[0].windows[0].aperture
    corners=(SIGNS*np.array([.5,.5,.15])+[20,20,1])[None]
    assert numeric_clearance(aperture,corners,.085,.09)[0]>0


def test_body_part_intersecting_plane_cannot_be_outside_opening():
    aperture=build_case(Case(4,4,0))[0].windows[0].aperture
    corners=(SIGNS*np.array([.5,.5,.15])+[0,1,0])[None]
    assert numeric_clearance(aperture,corners,.085,.09)[0]<0


def test_native_pva_knot_is_exact_and_jerk_continuous():
    from convex_timevarying_window.dynamic_passage_velocity_minco.trajectory import minco_coefficients
    from convex_timevarying_window.togt_minco_mpc.trajectory import basis
    times=ca.DM([3.,4.]);point=ca.DM([2.,1.,3.]);v=ca.DM([4.,0.,0.]);a=ca.DM([0.,1.,-2.])
    c=minco_coefficients(ca.DM.zeros(3,4),ca.horzcat(ca.DM([8.,2.,1.]),ca.DM.zeros(3,3)),
        point,v,times,a)
    c=np.asarray(ca.Function('coeff',[],[c])()['o0'])
    for d,expected in enumerate((point,v,a)):
        left=c[:8].T@np.asarray(basis(1.,d)).ravel()/float(times[0])**d
        right=c[8:].T@np.asarray(basis(0.,d)).ravel()/float(times[1])**d
        np.testing.assert_allclose(left,np.asarray(expected).ravel(),atol=1e-8)
        np.testing.assert_allclose(right,left,atol=1e-8)
    np.testing.assert_allclose(c[:8].T@np.asarray(basis(1.,3)).ravel()/27,
        c[8:].T@np.asarray(basis(0.,3)).ravel()/64,atol=1e-8)


def test_native_crossing_seed_uses_same_togt_shape_map():
    from .box_solver import shape_inverse
    from ..point_mapping import numeric
    track,_=build_case(Case(4,4,0))
    for window in track.windows:
        for p in ([0.,0.],[.1,.2],[-.3,.4]):
            np.testing.assert_allclose(numeric(window.aperture,shape_inverse(window.aperture,p)),p,atol=1e-9)


def test_motion_seed_queries_real_window_state_interface():
    from types import SimpleNamespace
    from .box_solver import crossing_point_seed
    from .box_planner import Config
    from ..point_mapping import symbolic
    track,_=build_case(Case(4,4,0))
    for w in track.windows:
        cfg=Config(passage_margin=.04)
        seed=crossing_point_seed(SimpleNamespace(windows=[w],cfg=cfg),.0625,np.array([21.,0,0]))
        point=np.asarray(symbolic(w.aperture,ca.DM(seed))).ravel()
        assert w.aperture.contains(point)
        assert np.all(np.isfinite(seed))


def test_intrinsic_shape_map_is_inside_and_invertible():
    from .shape import symbolic,inverse
    rng=np.random.default_rng(85)
    track,_=build_case(Case(4,4,0))
    for w in track.windows:
        for z in rng.normal(size=(100,2))*5:
            p=np.asarray(symbolic(w.aperture,ca.DM(z))).ravel()
            assert w.aperture.contains(p)
            np.testing.assert_allclose(np.asarray(symbolic(w.aperture,ca.DM(inverse(w.aperture,p)))).ravel(),p,atol=1e-9)
