import numpy as np
from types import SimpleNamespace
from convex_timevarying_window.race_to_go_mpc.planner import select_candidate,finish_time
from convex_timevarying_window.race_to_go_mpc.reference import Reference


def test_never_trade_later_finish_for_earlier_first_gate():
    assert select_candidate((True,20.),(True,21.))=='center'
    assert select_candidate((True,20.),(True,19.))=='free'


def test_ties_and_invalid_candidates():
    assert select_candidate((True,20.),(True,20.))=='center'
    assert select_candidate((True,20.),(False,10.))=='center'
    assert select_candidate((True,20.),(True,np.nan))=='center'
    assert select_candidate((False,np.inf),(True,21.))=='free'


def test_departure_is_body_exit_not_center_time():
    c=np.zeros((4,6,3));c[2,1,2]=-2.;c[3,0,2]=-2.;c[3,1,2]=-2.
    p=SimpleNamespace(gate_count=1,crossing_times=[3.],end_time=5.,durations=np.ones(4),coefficients=c)
    w=SimpleNamespace(center0=np.zeros(3),state_at=lambda t:(np.zeros(3),np.eye(3)))
    assert abs(finish_time(p,w,1.,.5)-3.25)<1e-10


def test_global_finish_contract_not_first_gate_contract():
    r=Reference(np.zeros(3))
    def plan(finish,first_crossing):
        return SimpleNamespace(made_at=1.,first_gate=0,gate_count=7,end_time=30.,
            nominal_finish_s=finish,crossing_times=[first_crossing],sample=lambda t,d:np.zeros(3))
    r.active=plan(25.,3.)
    assert r.candidate_reason(plan(24.,4.),0,'approach') is None
    assert r.candidate_reason(plan(26.,2.),0,'approach')=='delayed_race_finish'
