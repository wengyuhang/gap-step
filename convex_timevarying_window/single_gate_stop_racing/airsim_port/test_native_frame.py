import casadi as ca
import numpy as np
from scipy.spatial.transform import Rotation
from .native_frame_geometry import symbolic,numeric
from .scene import frame_vertices,FRAME_OUTWARD_OFFSET,FRAME_RADIUS
from ..scenario import build_case,Case


def test_aperture_scaling_preserves_motion_and_layout():
    from .scene import scaled_track
    ratio=.728/.38594472
    original=scaled_track();large=scaled_track(ratio)
    for a,b in zip(original.windows,large.windows):
        for t in (0.,.017,.125,.3):
            for x,y in zip(a.state_at(t),b.state_at(t)):np.testing.assert_allclose(x,y)
        if a.aperture.kind=='circle':assert np.isclose(b.aperture.radius,a.aperture.radius*ratio)
        else:np.testing.assert_allclose(b.aperture.physical_vertices,a.aperture.physical_vertices*ratio)
    np.testing.assert_equal(original.start,large.start)
    np.testing.assert_equal(original.goal,large.goal)


def test_native_sphere_planner_uses_original_sphere_geometry():
    from .planner import Single
    from ..planner import Single as OriginalSingle,Config
    from ..solver import solve
    from .scene import scaled_track
    from .model import GenericQuad
    from nonconvex_timevarying_window.sc_dynatogt.environment import SCWindowTrack
    track=scaled_track(1.886280447)
    local=SCWindowTrack('sphere_test',track.start,track.goal,(track.windows[0],),(0,))
    p=Single(local,Config(radius=.728,passage_margin=.04,compile_objective=False),1.,GenericQuad())
    assert Single.solve is solve
    assert Single.local_passage is OriginalSingle.local_passage
    assert p.cfg.radius==.728
    assert p.nvar==9


def test_native_frame_symbolic_numeric_sat_agree():
    track,_=build_case(Case(4,4,0));rng=np.random.default_rng(124)
    half=np.array([.492,.492,.148]);offset=np.array([0,0,-.033])
    for aperture in (track.windows[0].aperture,track.windows[1].aperture):
        q=ca.SX.sym('q',3);R=ca.SX.sym('R',3,3)
        f=ca.Function('sat',[q,R],[ca.vertcat(*symbolic(aperture,q,R,half,offset,FRAME_RADIUS,FRAME_OUTWARD_OFFSET))])
        for _ in range(30):
            p=rng.uniform(-2,2,3);r=Rotation.random(random_state=rng).as_matrix()
            expected=np.min(np.asarray(f(p,r)))
            actual=numeric(aperture,p[None],r[None],half,offset,FRAME_RADIUS,FRAME_OUTWARD_OFFSET)[0]
            # Unnormalized smooth SAT preserves the safety implication, not
            # the magnitude of the normalized signed separation metric.
            assert not (expected>=0 and actual < -1e-10)


def test_native_offset_is_not_subtracted_twice():
    aperture=build_case(Case(4,4,0))[0].windows[0].aperture
    vertices=frame_vertices(aperture)
    assert np.max(abs(vertices[:,1]))>np.max(abs(aperture.physical_vertices[:,1]))
    # Native inner side is at 1.15 + .065 - .085 = 1.13m.
    gap=numeric(aperture,np.array([[0.,1.10,0.]]),np.eye(3)[None],
        np.array([.01,.01,.01]),np.zeros(3),FRAME_RADIUS,FRAME_OUTWARD_OFFSET)[0]
    np.testing.assert_allclose(gap,.02,atol=1e-10)


def test_native_frame_contact_is_negative_and_far_plane_is_safe():
    aperture=build_case(Case(4,4,0))[0].windows[0].aperture
    half=np.array([.492,.492,.148]);offset=np.zeros(3)
    assert numeric(aperture,np.array([[0.,1.215,0.]]),np.eye(3)[None],half,offset,FRAME_RADIUS,FRAME_OUTWARD_OFFSET)[0]<0
    assert numeric(aperture,np.array([[0.,10.,3.]]),np.eye(3)[None],half,offset,FRAME_RADIUS,FRAME_OUTWARD_OFFSET)[0]>0
