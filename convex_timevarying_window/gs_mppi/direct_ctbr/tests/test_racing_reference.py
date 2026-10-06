import numpy as np
from convex_timevarying_window.gs_mppi.direct_ctbr.racing_reference import Core

def test_private_racing_objective_point_time_and_tail_gradients():
    core=Core();head=np.zeros((3,4));tail=np.zeros((3,4));tail[:,0]=[13.,3.,1.]
    tail[:,1]=[2.,-.5,.2];points=np.array([[4.,1.,.7],[9.,3.,1.]])
    times=np.array([1.21,1.13,1.17]);value,gp,gt,gh=core.value_and_gradient_with_tail(head,tail,points,times)
    assert np.isfinite(value)
    for array,gradient in [(points,gp),(times,gt),(tail,gh)]:
        for index in np.ndindex(array.shape):
            old=array[index];eps=1e-6;array[index]=old+eps
            plus=core.value_and_gradient(head,tail,points,times)[0]
            array[index]=old-eps;minus=core.value_and_gradient(head,tail,points,times)[0];array[index]=old
            assert np.isclose((plus-minus)/(2*eps),gradient[index],rtol=2e-3,atol=2e-3)

def test_private_reference_penalizes_unrealistically_short_time():
    core=Core();head=np.zeros((3,4));tail=head.copy();tail[0,0]=15.
    points=np.array([[5.,0.,0.],[10.,0.,0.]])
    fast=core.value_and_gradient(head,tail,points,np.full(3,.65))[0]
    slow=core.value_and_gradient(head,tail,points,np.full(3,1.4))[0]
    assert fast>slow+100.

def test_moving_plane_direction_time_chain_and_tail_gradients():
    from convex_dynamic_seven_window_gazebo.x500_togt.experiment import build_track
    core=Core();track,_=build_track();core.context=(track.windows[:2],.4,[-1.,1.])
    head=np.zeros((3,4));tail=head.copy();tail[:,0]=[4.,1.,2.]
    points=np.array([[2.,0.,1.]]);times=np.array([2.31,2.17])
    cost,gp,gt,gh=core.value_and_gradient_with_tail(head,tail,points,times)
    assert cost>Core().value_and_gradient(head,tail,points,times)[0]+1.
    for array,gradient in [(points,gp),(times,gt),(tail,gh)]:
        for index in np.ndindex(array.shape):
            old=array[index];eps=1e-5;array[index]=old+eps
            plus=core.value_and_gradient(head,tail,points,times)[0]
            array[index]=old-eps;minus=core.value_and_gradient(head,tail,points,times)[0];array[index]=old
            assert np.isclose((plus-minus)/(2*eps),gradient[index],rtol=2e-3,atol=.01)

def test_short_first_piece_keeps_pvaj_and_time_gradient():
    core=Core(bridge=True);head=np.zeros((3,4));head[:,1]=[2.,.5,0.];head[:,3]=[40.,-10.,5.]
    tail=np.zeros((3,4));tail[:,0]=[10.,3.,1.];points=np.array([[3.,1.,.5]])
    for first_time in [.47,1.6]:
        times=np.array([first_time,2.1]);cost,gp,gt,gh=core.value_and_gradient_with_tail(head,tail,points,times)
        for array,gradient in [(points,gp),(times,gt),(tail,gh)]:
            for index in np.ndindex(array.shape):
                old=array[index];eps=1e-6;array[index]=old+eps;plus=core.value_and_gradient(head,tail,points,times)[0]
                array[index]=old-eps;minus=core.value_and_gradient(head,tail,points,times)[0];array[index]=old
                assert np.isclose((plus-minus)/(2*eps),gradient[index],rtol=.004,atol=.05)
    from nonconvex_timevarying_window.sc_dynatogt.minco import BoundaryState,MincoSnap
    from convex_timevarying_window.dynamic_gate_mppi_v3.reference import PolynomialReference
    pp,tt,_,_=core.augmented(head,points,times)
    curve=PolynomialReference(MincoSnap(BoundaryState(*head.T),BoundaryState(*tail.T),pp,tt))
    for derivative in range(4):assert np.allclose(curve.evaluate(0.,derivative),head[:,derivative])
