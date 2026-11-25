import numpy as np
import trimesh
from convex_timevarying_window.comparisons.论文实验.实验2.basic_group import scenario
from convex_timevarying_window.comparisons.论文实验.实验2.basic_group.trajectory_only.collision import MeshDistance, _piece, CheckConfig


def setup():
    track,_=scenario.build_case(scenario.Case('original','static'));w=track.windows[0]
    raw=trimesh.load_mesh(scenario.ASSETS/'meshes'/f'gate_{w.name}_sleeve.stl')
    return w,raw,MeshDistance(raw)


def test_mesh_distance_matches_all_triangles():
    _,raw,mesh=setup();rng=np.random.default_rng(3);points=rng.uniform(-3,3,(60,3))
    expected=trimesh.proximity.closest_point_naive(raw,points)[1]
    np.testing.assert_allclose([mesh.distance(p) for p in points],expected,atol=1e-12)


def test_center_crossing_is_clear_and_side_crossing_collides():
    w,_,mesh=setup()
    for offset,expected in [(0.,'clear'),(1.1,'collision')]:
        C=np.zeros((8,3));C[0]=w.center0+w.R@np.array([0.,offset,-2.]);C[1]=4*w.R[:,2]
        assert _piece(C,1.,0.,w,mesh,CheckConfig(),np.inf)['outcome']==expected


def test_moving_frame_collision_between_safe_endpoints_and_midpoint():
    w,_,mesh=setup();w.motion=scenario.Motion('translation',w.R,2.,2.)
    C=np.zeros((8,3));C[0]=w.center0
    # At 0,T/2,T the frame is centered; an edge hits the body between them.
    r=_piece(C,.5,0.,w,mesh,CheckConfig(),np.inf)
    assert r['outcome']=='collision' and 0<r['witness']['time_s']<.5


def test_budget_exhaustion_is_never_safe():
    w,_,mesh=setup();C=np.zeros((8,3));C[0]=w.center0
    assert _piece(C,1.,0.,w,mesh,CheckConfig(max_boxes_per_piece=0),np.inf)['outcome']=='unresolved'


def test_remote_plane_crossing_is_clear_not_infinite_wall():
    w,_,mesh=setup();C=np.zeros((8,3));C[0]=w.center0+w.R@np.array([0.,10.,-2.]);C[1]=4*w.R[:,2]
    assert _piece(C,1.,0.,w,mesh,CheckConfig(),np.inf)['outcome']=='clear'
