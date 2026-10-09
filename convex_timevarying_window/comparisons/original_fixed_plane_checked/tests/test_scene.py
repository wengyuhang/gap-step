import casadi as ca
import numpy as np
import pytest
from convex_timevarying_window.comparisons.original_fixed_plane_checked import adapter
from convex_timevarying_window.receding_gate_racing import planner
from convex_timevarying_window.continuation_racing import guard
from 复现.polynomial_dynamic_racing_2023.px4_fixed_plane.scenario import Case,build_case


@pytest.mark.parametrize('frequency,amplitude',[(.6,1.2),(1.,2.)])
def test_all_gate_queries_match_scene(frequency,amplitude):
    track,_=build_case(Case(frequency,amplitude,0.)); t=ca.SX.sym('t')
    for i,w in enumerate(track.windows):
        c,R=adapter.symbolic_pose(w,t)
        f=ca.Function('state',[t],[c,R,ca.jacobian(c,t)])
        times=np.linspace(0.,10.,23)
        positions=np.array([w.state_at(float(s))[0]+[.1,.2,.3] for s in times])
        for at in times:
            p,r,dp=f(at);expected=w.state_at(float(at))
            np.testing.assert_allclose(np.array(p).ravel(),expected[0],atol=1e-10)
            np.testing.assert_allclose(np.array(r),expected[1],atol=1e-10)
            numeric=(w.state_at(float(at+1e-5))[0]-w.state_at(float(at-1e-5))[0])/2e-5
            np.testing.assert_allclose(np.array(dp).ravel(),numeric,atol=1e-7)
        actual=adapter.frame_clearances([w],positions,times,.38594472,.085)
        expected=[]
        for p,at in zip(positions,times):
            q,z=w.world_to_local(p,float(at))
            expected.append(np.hypot(w.aperture.boundary_distance(q),z)-.38594472-.085)
        np.testing.assert_allclose(actual,expected,atol=1e-10)


def test_install_changes_only_scene_functions(monkeypatch):
    old_class=guard.CheckedPlanner;old_solve=old_class.solve;old_validate=old_class.validate
    with monkeypatch.context() as patch:
        # Register old attributes so direct explicit install is restored afterwards.
        patch.setattr(planner,'symbolic_pose',planner.symbolic_pose)
        patch.setattr(planner,'symbolic_clearance_sq',planner.symbolic_clearance_sq)
        patch.setattr(guard,'frame_clearances',guard.frame_clearances)
        adapter.install()
        assert guard.CheckedPlanner is old_class
        assert guard.CheckedPlanner.solve is old_solve
        assert guard.CheckedPlanner.validate is old_validate
        assert planner.symbolic_pose is adapter.symbolic_pose
        assert guard.frame_clearances is adapter.frame_clearances
