from dataclasses import replace
import casadi as ca
import numpy as np
import pytest
from convex_timevarying_window.comparisons.original_fixed_plane_ours import adapter
from 复现.polynomial_dynamic_racing_2023.px4_fixed_plane.scenario import CASES, build_case


@pytest.mark.parametrize('case',CASES,ids=lambda c:c.name)
def test_symbolic_numeric_geometry_and_derivative(case):
    track,_ = build_case(case); t = ca.SX.sym('t'); p = ca.SX.sym('p',3)
    for w in track.windows:
        c,R = adapter.symbolic_pose(w,t)
        f = ca.Function('pose',[t],[c,R,ca.jacobian(c,t)])
        distance = ca.Function('d',[p,t],[adapter.symbolic_clearance_sq(w,p,t)])
        for stamp in (.1,.31,1.5,5.1):
            actual,rotation,velocity,_ = w.state_at(stamp)
            cc,rr,vv = f(stamp)
            assert np.allclose(np.asarray(cc).ravel(),actual)
            assert np.allclose(rr,rotation) and np.allclose(np.asarray(vv).ravel(),velocity)
            point = actual+rotation@np.array([.31,-.51,.27])
            local = adapter.local_coordinates(w,point[None],np.array([stamp]))[0]
            assert np.allclose(local,[.31,-.51,.27])
            dsq = w.aperture.boundary_distance(local[:2])**2+local[2]**2
            assert float(distance(point,stamp))==pytest.approx(dsq,abs=2e-11)
            clearance = adapter.frame_clearances([w],point[None],np.array([stamp]),.385,.085)
            assert clearance[0]==pytest.approx(np.sqrt(dsq)-.47,abs=2e-11)


def test_existing_planner_mechanisms_remain_and_seed_has_free_point():
    adapter.install_scene_queries()
    from convex_timevarying_window.wang_paper_aligned_mpc.planner import Planner,Config
    track,_ = build_case(CASES[0]); cfg=replace(Config(),workspace_lower=(-28.,-30.,.65),workspace_upper=(28.,22.,12.))
    planner=Planner(track,cfg,build_indices=[0])
    assert cfg.free_points and cfg.state_warm_start and cfg.horizon==1 and cfg.pieces_per_leg==4
    assert planner.variable_count(0)==28
    state=np.r_[track.start,np.zeros(9)]; x=planner.initial(state,0.,0)
    assert np.array_equal(x[26:],np.zeros(2))
    assert np.linalg.norm(planner.terminal_state(0)[:,1])>.1
    assert np.allclose(planner.terminal_state(6)[:,1:],0)
    plan=planner.solve(state,0.,0)
    at=plan.crossing_times[0]; c,R,*_=track.windows[0].state_at(at)
    assert np.allclose(plan.sample(at),c+R[:,:2]@plan.x[26:28],atol=1e-8)
    planner.commit(plan)
    x=planner.initial(np.r_[plan.sample(.2),plan.sample(.2,1),plan.sample(.2,2),np.zeros(3)],.2,0)
    assert np.array_equal(x[26:],np.zeros(2))
    assert planner.seed_provenance[0]['state_source']=='admitted'
    assert np.allclose(x[2:5]*cfg.speed,plan.sample(at,1))
