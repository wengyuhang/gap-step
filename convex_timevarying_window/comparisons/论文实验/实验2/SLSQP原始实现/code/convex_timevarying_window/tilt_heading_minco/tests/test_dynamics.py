import casadi as ca
import numpy as np
from convex_timevarying_window.tilt_heading_minco import dynamics
from convex_timevarying_window.dynamic_passage_velocity_minco import hard_dynamics as old

REGRESSION=[np.array(x) for x in (
    [7.371884496455808,-2.628819127782468,1.0284480988490878],
    [-1.067141029061423,14.45915671883871,-9.896949685268229],
    [-.7128701337749405,4.495760435721538,-3.09021260831014],
    [11.668077695185737,-33.04316216442244,62.66931018982543])]


def test_exact_horizontal_directions_are_regular_rotations():
    z=ca.MX.sym('z',3);R=dynamics.rotation_from_direction(z)
    fn=ca.Function('rotation',[z],[R,ca.jacobian(ca.vec(R),z)])
    for angle in np.linspace(0,2*np.pi,101):
        n=np.array([np.cos(angle),np.sin(angle),0.]);r,J=fn(n);r=np.array(r)
        np.testing.assert_allclose(r.T@r,np.eye(3),atol=1e-12)
        np.testing.assert_allclose(r[:,2],n,atol=1e-12)
        assert np.linalg.det(r)>1.-1e-12 and np.max(abs(np.array(J)))<5.


def test_rates_and_angular_acceleration_match_independent_time_differences():
    v,a,j,s=REGRESSION
    def jet(h):return v+a*h+j*h*h/2+s*h**3/6,a+j*h+s*h*h/2,j+s*h
    R=np.array(dynamics.orientation(v,a)[0]);omega,alpha=[np.array(x).ravel() for x in dynamics.rates(v,a,j,s)]
    for h in (1e-4,1e-5):
        vp,ap,jp=jet(h);vm,am,jm=jet(-h)
        Rdot=(np.array(dynamics.orientation(vp,ap)[0])-np.array(dynamics.orientation(vm,am)[0]))/(2*h)
        W=R.T@Rdot;numeric=np.array([W[2,1]-W[1,2],W[0,2]-W[2,0],W[1,0]-W[0,1]])/2
        np.testing.assert_allclose(omega,numeric,atol=2e-6,rtol=2e-5)
        numeric=(np.array(dynamics.rates(vp,ap,jp,s)[0]).ravel()-np.array(dynamics.rates(vm,am,jm,s)[0]).ravel())/(2*h)
        np.testing.assert_allclose(alpha,numeric,atol=2e-6,rtol=2e-5)


def test_rotor_wrench_is_consistent_and_hover_equal():
    F,omega,rotor=dynamics.flat_actuators(*REGRESSION)
    _,alpha=dynamics.rates(*REGRESSION)
    w=np.array(omega).ravel();a=np.array(alpha).ravel();I=old.X500_INERTIA
    np.testing.assert_allclose(old.MIXER@np.array(rotor).ravel(),np.r_[float(F),I*a+np.cross(w,I*w)],atol=1e-11)
    F,w,r=dynamics.flat_actuators(*[np.zeros(3)]*4)
    np.testing.assert_allclose(np.array(w),0.,atol=1e-12)
    np.testing.assert_allclose(np.array(r).ravel(),float(F)/4,atol=1e-12)


def test_regression_removes_gradient_blowup_not_by_clipping_rotors():
    x=ca.MX.sym('jet',12);args=[x[3*k:3*k+3] for k in range(4)]
    old_rotor=old.flat_actuators(*args)[2];new_rotor=dynamics.flat_actuators(*args)[2]
    f=ca.Function('jacobians',[x],[ca.jacobian(old_rotor,x),ca.jacobian(new_rotor,x)])
    A,B=[np.array(q) for q in f(np.concatenate(REGRESSION))]
    assert np.linalg.norm(A)>1000*np.linalg.norm(B)
    assert dynamics.domain(*REGRESSION[:2])['passed']


def test_invalid_north_chart_is_not_accepted_as_physical_feasibility():
    assert dynamics.domain(np.zeros(3),np.zeros(3))['passed']
    assert not dynamics.domain(np.zeros(3),np.array([0.,0.,-30.]))['passed']
