import numpy as np
from convex_timevarying_window.dynamic_gate_mppi_v3.terminal_cost import velocity_correction_distance


def test_equal_position_prefers_velocity_toward_gate_not_flyby():
    pose=np.zeros((1,1,12));pose[0,0,3:]=np.array([[0,0,1],[0,1,0],[-1,0,0]]).ravel()
    x=np.zeros((2,17));x[:,0]=-3.;x[0,3]=6.;x[1,3]=-6.
    cost=velocity_correction_distance(x,np.zeros(2,dtype=int),pose,np.ones(1),[4,0,0],7.,12.,np,1.)
    assert cost[0]<cost[1]


def test_at_goal_stopping_wins():
    pose=np.zeros((1,1,12));x=np.zeros((2,17));x[1,3]=5.
    cost=velocity_correction_distance(x,np.ones(2,dtype=int),pose,np.ones(1),[0,0,0],7.,12.,np,1.)
    assert cost[0]==0 and cost[1]>0
