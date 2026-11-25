"""Offline interface checks, run only after the translation timing batch."""
import numpy as np
import casadi as ca
from .motion import Motion,Case,build,symbolic_pose,R0,DIAMETER,SCALE

def test_ode_spline_and_symbolic_query():
    motion=Motion(120)
    t=np.random.default_rng(4).uniform(0,40,121)
    symbolic=np.array([float(motion.symbolic_theta(v)) for v in t])
    np.testing.assert_allclose(symbolic,motion.theta(t),atol=2e-10,rtol=2e-10)
    assert abs(DIAMETER-.9*SCALE)<1e-12
    assert abs(motion.theta(0)-np.radians(120))<1e-12
    x=ca.MX.sym('t');d=ca.Function('spline_derivative',[x],[ca.jacobian(motion.symbolic_theta(x),x)])
    np.testing.assert_allclose([float(d(v)) for v in t],motion.theta(t,1),rtol=1e-8,atol=1e-8)

def test_motion_bounds_and_rotation():
    w=build(Case(120)).windows[0]
    for a,b in ((0,.12),(.63,1.3),(2.1,3.4),(11.1,11.8)):
        lo,hi=w.motion.translation_bounds(a,b)
        for t in np.linspace(a,b,101):
            c,R,v,Rd=w.state_at(t);delta=c-w.center0
            assert np.all(delta>=lo-1e-10) and np.all(delta<=hi+1e-10)
            np.testing.assert_allclose(R.T@R,np.eye(3),atol=1e-13)
            np.testing.assert_allclose(R[:,2],[1,0,0],atol=1e-14)
            dt=1e-6
            if t>0:
                np.testing.assert_allclose((w.state_at(t+dt)[0]-w.state_at(t-dt)[0])/(2*dt),v,atol=1e-7)
    t=ca.MX.sym('t');c,R=symbolic_pose(w,t);f=ca.Function('pose',[t],[c,R])
    for at in (.1,.9,1.37,4.):
        cs,rs=f(at);cn,rn,*_=w.state_at(at);np.testing.assert_allclose(np.asarray(cs).ravel(),cn,atol=1e-10);np.testing.assert_allclose(np.asarray(rs),rn,atol=1e-10)

def test_circle_safety_rejects_near_frame():
    from .safety import passage
    from convex_timevarying_window.dynamic_passage_velocity_minco.trajectory import Plan
    w=build(Case(0)).windows[0];c=w.state_at(0)[0]
    def trajectory(offset):
        coefficients=np.zeros((2,8,3));coefficients[0,0]=c+np.array([-2,offset,0]);coefficients[0,1]=[2,0,0]
        coefficients[1,0]=c+np.array([0,offset,0]);coefficients[1,1]=[2,0,0]
        return Plan(0,0,np.array([1.,1.]),coefficients,np.zeros(7),0.,0.,'test',1)
    assert passage(trajectory(0),w,.571,.085,0)['passed']
    assert passage(trajectory(0),w,.571,.085,1)['passed']
    assert not passage(trajectory(w.aperture.radius-.01),w,.571,.085,0)['passed']

    assert not passage(trajectory(w.aperture.radius+2),w,.571,.085,0)['passed']


def test_independent_pose_parser(tmp_path):
    from .pose_audit import audit
    from scipy.spatial.transform import Rotation
    w=build(Case(60)).windows[0];messages=[]
    for t in (1.,1.1,1.2):
        c,R,*_=w.state_at(t);q=Rotation.from_matrix(R).as_quat();sec=int(30+t);nsec=round((30+t-sec)*1e9)
        text=f'header {{ stamp {{ sec: {sec} nsec: {nsec} }} }}\npose {{ name: "gate_01_pendulum" position {{ x: {c[0]} y: {c[1]} z: {c[2]} }} orientation {{ x: {q[0]} y: {q[1]} z: {q[2]} w: {q[3]} }} }}\n'
        messages.append(text)
    (tmp_path/'gate_pose.pbtxt').write_text(''.join(messages))
    assert audit(tmp_path,w,2.)['passed']
