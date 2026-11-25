import json
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
import casadi as ca
import numpy as np
import pytest
from convex_timevarying_window.crossing_resolved_mpc.planner import Planner,Config
from convex_timevarying_window.wang_free_state_mpc import planner as base
from convex_timevarying_window.wang_progress_mpc.planner import DeadlineSolver
from convex_timevarying_window.comparisons.original_fixed_plane_deadline.adapter import install
from 复现.polynomial_dynamic_racing_2023.px4_fixed_plane.scenario import Case,build_case


@pytest.fixture(scope='module')
def problem():
    data=json.loads(Path(__file__).with_name('collision_candidate.json').read_text())
    install();tr,_=build_case(Case(**data['case']));cfg=Config(**data['config'])
    p=Planner(tr,cfg,build_indices=[data['first'],7])
    x=np.array(data['x']);par=np.r_[data['made_at'],data['head'],np.tile([.15,.35,.65,.85],6)]
    return p,data,x,par


def test_saved_collision_is_detected_inside_optimizer(problem):
    p,data,x,par=problem;solver,ev,_,_,lo,hi=p.solvers[data['first']]
    times,sq,v=p.crossing_geometry[data['first']](x,par)
    margin=np.sqrt(np.array(sq).ravel())-p.cfg.radius-p.cfg.frame_thickness
    assert margin.min()<0
    assert float(ca.mmax(v))>.003
    g,_,_=ev(x,par);g=np.array(g).ravel()
    assert np.maximum(lo-g,0).max()>.003
    value=solver.solver.fg(x,np.r_[par,[par[0]+1000]*2])
    assert float(value[2])>.003


def test_time_grid_resolves_each_side_and_crossing(problem):
    p,data,x,par=problem;times=np.array(p.crossing_geometry[data['first']](x,par)[0]).ravel()
    count=26
    assert len(times)==4*count
    for chunk in times.reshape(4,count):
        assert np.max(abs(np.diff(chunk)))<=.020000001
        assert abs(chunk[-1]-chunk[0])<=.500000001
    for j in (0,2):
        assert abs(times.reshape(4,count)[j,0]-times.reshape(4,count)[j+1,0])<1e-10


def test_minco_coefficients_and_unmodified_base_constraints(problem):
    p,data,x,par=problem
    _,old,_,_,lbg,ubg=base.Planner._build(p,data['first'])
    _,ev,_,_,lo,hi=p.solvers[data['first']]
    go,co,_=old(x,par);gn,cn,_=ev(x,par)
    np.testing.assert_allclose(cn,co,atol=1e-12)
    np.testing.assert_allclose(np.array(gn).ravel()[:len(lbg)],np.array(go).ravel(),atol=1e-12)
    np.testing.assert_array_equal(lo[:len(lbg)],lbg)
    np.testing.assert_array_equal(hi[:len(ubg)],ubg)


def test_dynamic_grid_gradient(problem):
    p,data,x,par=problem;fn=p.solvers[data['first']][0].solver.fg
    pp=np.r_[par,[par[0]+1000]*2];_,gradient,_=fn(x,pp)
    analytic=np.array(gradient).ravel()
    for i in (0,1,2,9,24,26):
        delta=np.zeros(len(x));delta[i]=1e-6
        numeric=(float(fn(x+delta,pp)[0])-float(fn(x-delta,pp)[0]))/2e-6
        assert abs(numeric-analytic[i])<=.003*max(1.,abs(numeric)),(i,numeric,analytic[i])


def test_no_dense_validation_and_same_deadline(problem):
    p,data,x,par=problem
    plan=SimpleNamespace();assert p.validate(plan) is None
    assert plan.post_trajectory_validation_skipped
    assert isinstance(p.solvers[data['first']][0],DeadlineSolver)
    assert p.cfg.solver_seconds==.055
    assert p.cfg.soft_clearance_weight==10
    assert p.cfg.tracking_margin==.35
    assert p.crossing_geometry[7].size1_out(0)==0


def test_invalid_sampling_parameters():
    with pytest.raises(ValueError):replace(Config(),crossing_max_step_s=0)
