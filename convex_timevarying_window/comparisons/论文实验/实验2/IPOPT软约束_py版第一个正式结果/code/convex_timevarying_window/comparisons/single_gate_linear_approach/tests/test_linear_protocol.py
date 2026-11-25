from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from convex_timevarying_window.comparisons.single_gate_linear_approach.scenario import Case,build_case,write_world
from convex_timevarying_window.comparisons.single_gate_linear_approach.planner import Planner,Config
from convex_timevarying_window.dynamic_passage_minco.safety import rectangle_distance_squared


def test_motion_same_geometry_and_never_through_start():
    zero,_=build_case(Case(0.))
    for speed in (-20.,-5.,0.,5.,20.):
        track,_=build_case(Case(speed));w=track.windows[0];R=w.state_at(0.)[1]
        np.testing.assert_allclose(track.start,zero.start)
        np.testing.assert_allclose(track.goal,[3.58,4.,6.48])
        np.testing.assert_allclose(w.center0,[-2.42,-26.,6.48])
        for t in (0.,.35,3.,20.):
            np.testing.assert_allclose(w.state_at(t)[0],w.center0+[0,speed*t,0])
            assert abs((track.start-w.state_at(t)[0])@R[:,2])>13.5
        if speed>0:
            assert (w.state_at(.5)[0]-track.start)@w.motion.velocity<0
            assert (w.state_at(31./speed)[0]-track.start)@w.motion.velocity>0


def test_world_matches_query_and_contact_clock(tmp_path):
    track,_=build_case(Case(10.,80.));w=track.windows[0]
    np.testing.assert_allclose(w.center0,[-2.42,-76.,6.48])
    path=write_world(track,Path('convex_dynamic_seven_window_gazebo'),tmp_path)
    world=ET.parse(path).find('world')
    model=next(m for m in world.findall('model') if m.get('name','').startswith('gate_'))
    plugin=model.find("plugin[@name='convex_dynamic_course::PeriodicGateMotion']")
    np.testing.assert_allclose(np.fromstring(plugin.find('center0').text,sep=' '),w.center0)
    np.testing.assert_allclose(np.fromstring(plugin.find('linear_velocity').text,sep=' '),[0,10,0])
    assert float(plugin.find('motion_start_time').text)==30
    assert float(plugin.find('linear_anchor_time').text)==0
    assert float(world.find('physics/max_step_size').text)==.001
    assert all(float(s.find('update_rate').text)==1000 for s in world.findall("model/link/sensor[@type='contact']"))


def test_frozen_reference_only_activates_at_common_race_epoch():
    from convex_timevarying_window.global_gate_minco_probe.flight import Reference
    from types import SimpleNamespace
    start=np.array([-16.,4.,3.2]);plan=SimpleNamespace(made_at=0.)
    ref=Reference(start,plan)
    assert ref.advance(-.01,0,'approach') is None
    assert ref.advance(0.,0,'approach') is plan
    assert ref.advance(5.,1,'approach') is plan


def test_affine_safe_path_matches_dense_independent_check():
    from convex_timevarying_window.dynamic_passage_velocity_minco.trajectory import Plan
    track,_=build_case(Case(10.))
    p=Planner(track,Config(compile_objective=False,full_task_error_recovery=False))
    assert p.cfg.max_solve_ms is None
    w=track.windows[0];R=w.state_at(0.)[1]
    # Analytic geometric fixture, not a claim that the optimizer found a plan.
    # Follow the affine centre tangentially while crossing its fixed plane.
    C=np.zeros((2,8,3))
    for piece in range(2):
        C[piece,0]=w.state_at(.15+piece)[0]+R[:,2]*(1-piece)
        C[piece,1]=w.motion.velocity-R[:,2]
    plan=Plan(.15,0,np.ones(2),C,np.zeros(7),0.,0.,'geometric_test',1)
    tt=np.linspace(plan.made_at,plan.made_at+sum(plan.durations),10001)
    q=np.array([R.T@(plan.sample(t)-w.state_at(t)[0]) for t in tt])
    half=np.max(abs(w.aperture.physical_vertices),axis=0)+p.cfg.frame_offset
    minimum=np.sqrt(rectangle_distance_squared(q,half)).min()
    assert minimum>=p.cfg.radius+p.cfg.frame_thickness+p.cfg.passage_margin-1e-4
    assert all(p.local_passage(plan,w,piece)['passed'] for piece in range(2))
    C[:,0] += 2*R[:,1]
    assert not all(p.local_passage(plan,w,piece)['passed'] for piece in range(2))
