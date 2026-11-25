from types import SimpleNamespace
import numpy as np
from convex_timevarying_window.togt_minco_mpc.planner import Planner,Config


def test_unexecuted_search_does_not_lose_flight_time():
    planner=Planner.__new__(Planner);planner.cfg=Config();planner.admitted=None
    values=np.r_[[1.,1.2,1.4,1.6,1.8,2.],np.arange(9.),np.zeros(4)]
    planner.search={0:SimpleNamespace(first_gate=0,made_at=.15,x=values)}
    # A rejected candidate is a relative search seed, not an executing plan.
    np.testing.assert_equal(planner.initial(np.zeros(12),.95,0),values)


def test_new_horizon_search_is_not_shrunk_using_previous_gate_plan():
    planner=Planner.__new__(Planner);planner.cfg=Config()
    planner.admitted=SimpleNamespace(first_gate=0)
    values=np.arange(19,dtype=float)+1
    planner.search={1:SimpleNamespace(first_gate=1,made_at=2.,x=values)}
    np.testing.assert_equal(planner.initial(np.zeros(12),2.4,1),values)
