import numpy as np
from 复现.polynomial_dynamic_racing_2023.px4_fixed_plane.scenario import Case,build_case
from convex_timevarying_window.single_gate_stop_racing.planner import Config,Single,parking_points,route_signs
from convex_timevarying_window.single_gate_stop_racing.geometry import halfplanes
from convex_timevarying_window.single_gate_stop_racing import point_mapping
from nonconvex_timevarying_window.sc_dynatogt.environment import SCWindowTrack


def test_parking_and_terminal():
    track,_=build_case(Case(1.,2.,0.));cfg=Config(compile_objective=False)
    points=parking_points(track,cfg);signs=route_signs(track)
    np.testing.assert_array_equal(points[-1],track.goal)
    assert not cfg.full_task_error_recovery
    for i in range(6):
        w=track.windows[i];out=-signs[i]*w.state_at(0.)[1][:,2]
        assert (points[i]-w.center0)@out>cfg.radius+cfg.frame_thickness


def test_all_original_shapes_gradient_and_time_objective():
    track,_=build_case(Case(1.,2.,0.));cfg=Config(compile_objective=False)
    points=parking_points(track,cfg);signs=route_signs(track)
    for i in (0,1,2,4):
        w=track.windows[i];local=SCWindowTrack('test',track.start,points[i],(w,),(0,))
        p=Single(local,cfg,signs[i]);d=point_mapping.initial(w.aperture)+.11
        x=np.r_[np.log([3.,3.]),d,5.,1.,.2]
        par=np.r_[.162,track.start,np.zeros(9),np.tile(np.linspace(0.,.2,cfg.witness_slots),2)]
        a=p.function(x,par);np.testing.assert_allclose(float(a[0])*1000,6.)
        for k in range(p.nvar):
            delta=np.eye(p.nvar)[k]*1e-6;lo,hi=p.function(x-delta,par),p.function(x+delta,par)
            np.testing.assert_allclose(np.array(a[3])[:,k],np.array(hi[2]-lo[2]).ravel()/2e-6,rtol=2e-3,atol=1e-3)


def test_polygon_planes():
    t,_=build_case(Case(1.,1.,0.))
    for w in t.windows:
        if w.aperture.kind=='polygon':
            n,b=halfplanes(w.aperture)
            assert np.all(b>0)
            assert np.all(w.aperture.physical_vertices@n.T<=b+1e-8)


def test_event_queries_actual_sine_time():
    from convex_timevarying_window.single_gate_stop_racing.events import Progress
    track,_=build_case(Case(4.,4.,0.));w=track.windows[0];R=w.state_at(0.)[1]
    at=.0625;center=w.state_at(at)[0];point=center+R[:,:2]@np.array([0.,1.14])
    progress=Progress([w],[1.],.38,.085)
    progress.observe(point+.2*R[:,2],at-.01)
    progress.observe(point-.2*R[:,2],at+.01)
    assert len(progress.crossings)==1
    np.testing.assert_allclose(progress.crossings[0]['q'],[0.,1.14],atol=1e-9)


def test_togt_maps_every_iterate_inside_true_aperture():
    import casadi as ca
    track,_=build_case(Case(3.5,4.,0.));rng=np.random.default_rng(7)
    for w in track.windows:
        ap=w.aperture;n=point_mapping.dimension(ap);d=ca.MX.sym('d',n)
        q=point_mapping.symbolic(ap,d)
        fun=ca.Function('map_test',[d],[q,ca.jacobian(q,d)])
        np.testing.assert_allclose(point_mapping.numeric(ap,point_mapping.initial(ap)),0.,atol=1e-14)
        for v in [np.zeros(n)]+[rng.normal(size=n)*scale for scale in (1e-9,.1,1.,10.,1e9) for _ in range(30)]:
            q=np.asarray(fun(v)[0]).ravel()
            if ap.kind=='circle':assert np.linalg.norm(q)<=ap.radius+1e-12
            else:
                normal,b=halfplanes(ap)
                assert np.all(normal@q<=b+1e-12)
        v=point_mapping.initial(ap)+rng.normal(size=n)*.1
        analytic=np.asarray(fun(v)[1])
        numeric=np.column_stack([(np.asarray(fun(v+np.eye(n)[i]*1e-6)[0])-
                                 np.asarray(fun(v-np.eye(n)[i]*1e-6)[0])).ravel()/2e-6 for i in range(n)])
        np.testing.assert_allclose(analytic,numeric,rtol=1e-6,atol=1e-8)


def test_duplicate_witness_and_rebased_failed_seed():
    from convex_timevarying_window.single_gate_stop_racing.solver import add_witness,rebase_seed
    offsets=np.zeros((2,4));used=[1,1]
    assert not add_witness(offsets,used,0,0.)
    assert add_witness(offsets,used,0,.052)
    assert not add_witness(offsets,used,0,.052001)
    assert used==[2,1]
    x=np.r_[np.log([3.,2.]),1.,2.,3.]
    rebased=rebase_seed(dict(x=x,now=10.),11.)
    np.testing.assert_allclose(np.exp(rebased[:2]),[2.,2.])
    np.testing.assert_array_equal(rebased[2:],x[2:])
    assert rebase_seed(dict(x=x,now=10.),13.) is None


def test_local_bounds_keep_finite_frames_and_honor_deadline():
    from types import SimpleNamespace
    from convex_timevarying_window.single_gate_stop_racing.geometry import passage
    track,_=build_case(Case(1.,0.,0.))
    for w in track.windows:
        R=w.state_at(0.)[1];C=np.zeros((2,8,3))
        C[0,0]=w.center0+2*R[:,2];C[0,1]=-2*R[:,2]
        C[1,0]=w.center0;C[1,1]=-2*R[:,2]
        plan=SimpleNamespace(coefficients=C,made_at=0.,durations=np.ones(2))
        assert passage(plan,w,.57,0)['passed']
        assert passage(plan,w,.57,1)['passed']
        assert passage(plan,w,.57,0,deadline=0.)['outcome']=='check_budget'
        C[0,0]+=5*R[:,0];C[0,1]=-4*R[:,2]
        assert passage(plan,w,.57,0)['passed']
        q=np.array([w.aperture.radius,0.]) if w.aperture.kind=='circle' else w.aperture.physical_vertices[0]
        C[0,0]=w.center0+R[:,:2]@q+2*R[:,2];C[0,1]=-2*R[:,2]
        assert not passage(plan,w,.57,0)['passed']


def test_native_local_bound_against_python_and_dense_evidence():
    from types import SimpleNamespace
    from convex_timevarying_window.single_gate_stop_racing.geometry import passage,passage_python
    track,_=build_case(Case(3.,.3,0.));rng=np.random.default_rng(19)
    for w in track.windows:
        R=w.state_at(0.)[1]
        for _ in range(4):
            now=rng.uniform(0.,1.);center=w.state_at(now+.5)[0]
            C=np.zeros((2,8,3));C[0,0]=center+2*R[:,2]+R[:,:2]@rng.uniform(-.4,.4,2)
            C[0,1]=-2*R[:,2]+R[:,:2]@rng.uniform(-.4,.4,2)
            plan=SimpleNamespace(coefficients=C,made_at=now,durations=np.full(2,.5))
            native=passage(plan,w,.57,0);reference=passage_python(plan,w,.57,0)
            if native['passed']:
                assert reference['passed']
                u=np.linspace(0.,1.,1001);points=C[0,0]+u[:,None]*C[0,1]
                q=np.array([w.world_to_local(p,float(now+.5*s))[0] for s,p in zip(u,points)])
                z=2*(1-u)
                distances=np.hypot(w.aperture.boundary_distance(q),z)
                assert np.min(distances)>=.57-1e-8
