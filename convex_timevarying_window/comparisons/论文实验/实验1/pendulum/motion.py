"""Shared damped-pendulum ODE table, queried by optimizer and simulator."""
from dataclasses import dataclass
from functools import cached_property
import numpy as np
from scipy.integrate import solve_ivp
from scipy.interpolate import CubicSpline
from scipy.spatial.transform import Rotation
import casadi as ca
from convex_timevarying_window.dynamic_passage_minco.safety import sine_range

R0=np.array([[0.,0.,1.],[1.,0.,0.],[0.,1.,0.]])
AUTHOR_PROP=.127
X500_PROP=.2792307692307692
SCALE=X500_PROP/AUTHOR_PROP
DIAMETER=.9*SCALE
PIVOT=np.array([-13.,4.,4.2])
START=np.array([-16.,4.,3.2])
GOAL=np.array([-11.,4.,3.2])

@dataclass(frozen=True)
class Case:
    angle_deg: float

class Motion:
    length=2.; damping=.1; mass=2.; gravity=9.81
    translation_period=2*np.pi*np.sqrt(2/9.81)
    phase=0.; translation_amplitude=np.array([0.,2.,2.]); rotation_enabled=True
    translation_enabled=True; rotation_amplitude=np.zeros(3); scale_enabled=False
    def __init__(self, angle_deg, horizon=80., dt=.01):
        self.angle_deg=float(angle_deg);self.times=np.linspace(0,horizon,round(horizon/dt)+1)
        f=lambda t,x:[x[1],-self.gravity/self.length*np.sin(x[0])-self.damping/self.mass*x[1]]
        solution=solve_ivp(f,[0,horizon],[np.radians(angle_deg),0.],method='DOP853',rtol=2e-13,atol=2e-14,t_eval=self.times)
        if not solution.success:raise RuntimeError(solution.message)
        self.spline=CubicSpline(self.times,solution.y[0]);self.ode_states=solution.y
    def theta(self,t,derivative=0):
        if np.any(np.asarray(t)>self.times[-1]) or np.any(np.asarray(t)<-1e-10):raise ValueError('motion query outside frozen table')
        return self.spline(np.maximum(t,0.),derivative)
    @cached_property
    def symbolic_theta(self):
        return ca.interpolant('pendulum_theta','bspline',[self.times.tolist()],self.ode_states[0].tolist())
    def translation(self,t):
        theta,w=self.theta(t),self.theta(t,1)
        return np.array([0.,self.length*np.sin(theta),-self.length*np.cos(theta)]),np.array([0.,self.length*w*np.cos(theta),self.length*w*np.sin(theta)])
    def theta_bounds(self,a,b):
        if a<0 or b>self.times[-1]:raise ValueError('bound outside motion table')
        values=[]
        i0=max(0,np.searchsorted(self.times,a,side='right')-1);i1=min(len(self.times)-2,np.searchsorted(self.times,b,side='left'))
        for i in range(i0,i1+1):
            lo=max(a,self.times[i])-self.times[i];hi=min(b,self.times[i+1])-self.times[i]
            if hi<lo:continue
            coeff=self.spline.c[:,i];values.extend(np.polyval(coeff,[lo,hi]))
            for root in np.roots(np.polyder(coeff)):
                if abs(root.imag)<1e-10 and lo<root.real<hi:values.append(np.polyval(coeff,root.real))
        return min(values)-1e-11,max(values)+1e-11
    def translation_bounds(self,a,b):
        lo,hi=self.theta_bounds(a,b);sy=sine_range(lo,hi);cz=sine_range(lo+np.pi/2,hi+np.pi/2)
        return np.array([0.,self.length*sy[0],-self.length*cz[1]]),np.array([0.,self.length*sy[1],-self.length*cz[0]])
    def write_table(self,path):
        np.savetxt(path,np.c_[self.times[:-1],self.spline.c.T],fmt='%.17g')

class Aperture:
    kind='circle';radius=DIAMETER/2
    def contains(self,q):return bool(np.linalg.norm(q)<=self.radius+1e-10)
    def boundary_distance(self,q):return self.radius-np.linalg.norm(q)
    def boundary_points(self,count=128):
        a=np.arange(count)*2*np.pi/count;return self.radius*np.c_[np.cos(a),np.sin(a)]

class Window:
    name='pendulum_circle';center0=PIVOT.copy();angles0_rpy=Rotation.from_matrix(R0).as_euler('xyz')
    aperture=Aperture()
    def __init__(self,case):self.motion=Motion(case.angle_deg)
    def state_at(self,t):
        theta=float(self.motion.theta(t));veltheta=float(self.motion.theta(t,1));d,v=self.motion.translation(t)
        R=Rotation.from_rotvec([theta,0,0]).as_matrix()@R0
        Rdot=np.array([[0.,0.,0.],[0.,0.,-veltheta],[0.,veltheta,0.]])@R
        return self.center0+d,R,v,Rdot
    def world_to_local(self,p,t):
        c,R,*_=self.state_at(t);q=R.T@(np.asarray(p)-c);return q[:2],float(q[2])
    def boundary_at(self,t,circle_samples=128):
        c,R,*_=self.state_at(t);return c+self.aperture.boundary_points(circle_samples)@R[:,:2].T

@dataclass
class Track:
    start: np.ndarray
    goal: np.ndarray
    windows: tuple
    order: tuple=(0,)
    name: str='scaled_author_pendulum_x500'

def build(case):return Track(START.copy(),GOAL.copy(),(Window(case),))

def symbolic_pose(window,t):
    theta=window.motion.symbolic_theta(t);ct,st=ca.cos(theta),ca.sin(theta)
    c=ca.DM(PIVOT)+ca.vertcat(0,2*st,-2*ct)
    R=ca.horzcat(ca.vertcat(0,ct,st),ca.vertcat(0,-st,ct),ca.DM([1,0,0]))
    return c,R
