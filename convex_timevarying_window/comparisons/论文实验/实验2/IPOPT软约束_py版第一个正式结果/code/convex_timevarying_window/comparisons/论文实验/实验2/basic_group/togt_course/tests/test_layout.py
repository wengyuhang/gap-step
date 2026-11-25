import numpy as np
from scipy.optimize import minimize_scalar
from scipy.spatial.transform import Rotation
from convex_timevarying_window.comparisons.论文实验.实验2.basic_group.togt_course import scenario,layout


def test_layout_preserves_official_positions_order_and_first_six_poses():
    tr,_=scenario.build_case(scenario.Case())
    np.testing.assert_allclose((scenario.ANCHORS-scenario.OFFSET)/scenario.SCALE,scenario.UPSTREAM['positions'])
    for w,r in zip(tr.windows,scenario.UPSTREAM['rpy_degrees_first_six']):
        np.testing.assert_allclose(w.R,Rotation.from_euler('xyz',r,degrees=True).as_matrix(),atol=1e-14)
    np.testing.assert_allclose(scenario.ANCHORS[3,:2],scenario.ANCHORS[4,:2]);assert scenario.ANCHORS[3,2]>scenario.ANCHORS[4,2]


def test_maximum_sweeps_do_not_block_nonincident_tubes():
    r=layout.audit();assert r['passed'] and r['min_nonincident_gap_m']>2.8 and r['min_pair_sweep_gap_m']>2.


def test_segment_box_distance_matches_independent_scalar_minimum():
    rng=np.random.default_rng(5)
    for _ in range(30):
        a,b=rng.normal(size=(2,3))*3;lo=rng.normal(size=3);hi=lo+rng.uniform(.1,3,3)
        f=lambda u:np.linalg.norm(np.maximum(np.maximum(lo-a-u*(b-a),a+u*(b-a)-hi),0.))
        m=minimize_scalar(f,bounds=(0,1),method='bounded',options={'xatol':1e-12})
        np.testing.assert_allclose(layout.segment_box_distance(a,b,lo,hi),min(f(0),f(1),m.fun),atol=1e-7)


def test_only_one_challenge_changes_and_all_motion_interfaces_match():
    for a,b in zip(scenario.LEVELS[1:-1],scenario.LEVELS[2:]):
        changed=[i for i,(x,y) in enumerate(zip(scenario.profile(a),scenario.profile(b))) if x!=y]
        assert len(changed)==1 and changed[0] in (2,3)
    assert layout.motion_audit()['passed']
