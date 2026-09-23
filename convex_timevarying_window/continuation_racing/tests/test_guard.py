import numpy as np
from convex_dynamic_seven_window_gazebo.x500_togt.experiment import build_track
from convex_timevarying_window.receding_gate_racing.scenario import vary_track
from convex_timevarying_window.receding_gate_racing.planner import Plan,coefficients
from convex_timevarying_window.continuation_racing.guard import sample_many,frame_clearances,GuardedPlan


def test_batch_geometry_agrees_with_independent_scalar_geometry():
    track,_=build_track();vary_track(track,.4,0,10,4,4)
    rng=np.random.default_rng(21);times=np.linspace(0,11,53);points=rng.normal(size=(53,3))*10
    actual=frame_clearances(track.windows,points,times,.38594472,.085)
    expected=[]
    for p,t in zip(points,times):
        distances=[]
        for w in track.windows:
            q,z=w.world_to_local(p,t);distances.append(np.hypot(w.aperture.boundary_distance(q),z)-.38594472-.085)
        expected.append(min(distances))
    np.testing.assert_allclose(actual,expected,atol=1e-12)


def test_batch_plan_and_guard_are_c2_at_braking_switch():
    z=np.zeros(3);C=coefficients(z,z,z,np.array([10.,0.,2.]),np.array([3.,0.,0.]),z,3.)
    nominal=Plan(1.,0,np.array([3.]),C[None],np.zeros(9),0,0,'test')
    switch=2.;T=2.;p,v,a=[nominal.sample(switch,d) for d in range(3)]
    end=p+.5*T*v+T*T*a/12;B=coefficients(p,v,a,end,z,z,T)
    brake=Plan(switch,0,np.array([T]),B[None],np.zeros(9),0,0,'test');guard=GuardedPlan(nominal,brake,1.)
    for d in range(3):np.testing.assert_allclose(nominal.sample(switch,d),brake.sample(switch,d),atol=1e-12)
    for d in range(4):
        times=np.linspace(0,6,47)
        np.testing.assert_allclose(sample_many(guard,times,d),[guard.sample(t,d) for t in times],atol=1e-11)
    assert guard.end_time==brake.end_time
    np.testing.assert_allclose(guard.sample(8.,1),z,atol=1e-11)
    np.testing.assert_allclose(guard.sample(8.,2),z,atol=1e-11)
