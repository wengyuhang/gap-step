from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from convex_timevarying_window.comparisons.single_gate_swing import scenario
from convex_timevarying_window.comparisons.single_gate_swing.planner import Planner, Config
from convex_timevarying_window.comparisons.single_gate_swing.contacts import parse_contacts


def test_motion_plane_and_unrestricted_arrival():
    track,_ = scenario.build_case(scenario.Case(4.,4.,0.))
    window = track.windows[0]
    rotation = window.state_at(0.)[1]
    for t in (.013,.127,.219):
        center = window.state_at(t)[0]
        np.testing.assert_allclose(center-window.center0,4*np.sin(8*np.pi*t)*rotation[:,1],atol=1e-12)
    planner = object.__new__(Planner)
    for now,seed in ((.15,.5),(3.21,8.)):
        assert planner.arrival_bounds(window,now,seed,8*np.pi,.25)==(.3,15.)


def test_world_has_one_gate_and_ground_sensor(tmp_path):
    track,_ = scenario.build_case(scenario.Case(2.,2.,0.))
    path = scenario.write_world(track,Path('convex_dynamic_seven_window_gazebo'),tmp_path)
    world = ET.parse(path).find('world')
    assert float(world.find('physics/max_step_size').text)==.001
    assert len([m for m in world.findall('model') if m.get('name','').startswith('gate_')])==1
    assert world.find("model[@name='ground']/link/sensor/topic").text==scenario.GROUND_TOPIC
    assert all(float(s.find('update_rate').text)==1000 for s in world.findall("model/link/sensor[@type='contact']"))


def test_contacts_exclude_preparation_and_cleanup(tmp_path):
    path = tmp_path/'contacts.pbtxt'
    message = ('header {\n stamp { sec: %d nsec: 0 }\n}\n'
        'contact { collision1 { name: "ground::ground_collision" } '
        'collision2 { name: "x500_0::base_link::base_link_collision" } }\n')
    path.write_text(''.join(message%t for t in (29,31,35)))
    result = parse_contacts(path,32.)
    assert result['contact_pair_count']==1
    assert result['first_contact_sim_time_s']==31.


def test_unlimited_solve_ignores_wall_clock_but_finite_budget_still_expires(monkeypatch):
    import itertools
    from dataclasses import replace
    from types import SimpleNamespace
    import pytest
    from convex_timevarying_window.dynamic_passage_velocity_minco import planner as module
    track,_=scenario.build_case(scenario.Case(1.,0.,0.))
    p=Planner(track,Config(compile_objective=False))
    clock=itertools.count(0.,.25)
    monkeypatch.setattr(module,'time',SimpleNamespace(perf_counter=lambda:next(clock)))
    head=np.r_[track.start,np.zeros(9)]
    p.cfg=replace(p.cfg,max_solve_ms=90.)
    with pytest.raises(RuntimeError,match='no_safe_passage_under_budget'):
        p.solve(head,.15,0)
    p.cfg=replace(p.cfg,max_solve_ms=None)
    plan=p.solve(head,.15,0)
    assert plan.solve_ms>90.
    assert plan.optimizer_stats['converged']
    assert plan.optimizer_stats['wall_time_limit_ms'] is None
    assert plan.optimizer_stats['passage']['passed']
