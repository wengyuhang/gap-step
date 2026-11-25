"""17-state CTBR prediction with shared rotor force/torque allocation.

This is a physical approximation, not an exact PX4 PID emulator. Cost, horizon,
rate/thrust input box and RTI solver stay identical to the old tracker.
"""
import casadi as ca
import numpy as np
from convex_timevarying_window.checked_soft_mpc_racing.mpc import (
    MASS, G, KF, RATE_TAU, NX, NU, LOWER, UPPER)
from convex_timevarying_window.dynamic_gate_mppi_v3.rollout import MIXER
from convex_dynamic_seven_window_gazebo.x500_togt.experiment import X500_INERTIA
from convex_timevarying_window.simple_two_gate_soft_only_mpc.tracking_mpc import MPC as Parent, Config


def rotor_targets(collective, torque):
    """Preserve achievable differential force, then desaturate collective."""
    inverse=ca.DM(np.linalg.inv(MIXER))
    delta=inverse[:,1:]@torque
    low,high=KF*150**2,KF*1000**2
    scale=ca.fmin(1.,(high-low)/(ca.mmax(delta)-ca.mmin(delta)+1e-9))
    delta=delta*scale
    common=ca.fmax(low-ca.mmin(delta),ca.fmin(collective/4.,high-ca.mmax(delta)))
    return ca.sqrt(ca.fmax(low,common+delta)/KF)


def dynamics():
    x,u=ca.SX.sym('allocated_x',NX),ca.SX.sym('allocated_u',NU)
    qw,qa,qb,qc=ca.vertsplit(x[6:10]); omega=x[10:13]
    axis=ca.vertcat(2*(qa*qc+qw*qb),2*(qb*qc-qw*qa),1-2*(qa*qa+qb*qb))
    inertia=ca.DM(X500_INERTIA)
    gyroscopic=ca.cross(omega,inertia*omega)
    requested=inertia*(u[1:4]-omega)/ca.DM(RATE_TAU)+gyroscopic
    target=rotor_targets(u[0],requested)
    motors=ca.fmax(0.,ca.fmin(1000.,x[13:17]))
    rotor_force=KF*motors**2
    wrench=ca.DM(MIXER)@rotor_force
    acceleration=axis*wrench[0]/MASS-ca.vertcat(0.,0.,G)
    drag=8.06428e-5*ca.sum1(motors)/MASS
    acceleration-=drag*(x[3:6]-ca.dot(x[3:6],axis)*axis)
    p,q,r=ca.vertsplit(omega)
    quaternion_dot=.5*ca.vertcat(-qa*p-qb*q-qc*r,qw*p+qb*r-qc*q,
                                qw*q+qc*p-qa*r,qw*r+qa*q-qb*p)
    derivative=ca.Function('allocated_derivative',[x,u],[ca.vertcat(x[3:6],acceleration,
        quaternion_dot,(wrench[1:4]-gyroscopic)/inertia,
        (target-motors)/ca.if_else(target>motors,.0125,.025))])
    h=ca.SX.sym('allocated_h');y=x
    for _ in range(4):
        hh=h/4;k1=derivative(y,u);k2=derivative(y+hh*k1/2,u)
        k3=derivative(y+hh*k2/2,u);k4=derivative(y+hh*k3,u)
        y+=hh*(k1+2*k2+2*k3+k4)/6
        y=ca.vertcat(y[:6],y[6:10]/ca.norm_2(y[6:10]),y[10:])
    step=ca.Function('allocated_step',[x,u,h],[y])
    linear=ca.Function('allocated_linear',[x,u,h],[y,ca.jacobian(y,x),ca.jacobian(y,u)])
    return derivative,step,linear


class MPC(Parent):
    def __init__(self,cfg=Config()):
        super().__init__(cfg)
        self.derivative,self.step,linear=dynamics()
        self.linear=linear.map(cfg.steps)
        self._accumulate=self.step.mapaccum(cfg.steps)

    def command(self,state,now,reference):
        result=super().command(state,now,reference)
        self.logs[-1]['controller']='allocation_coupled_tracking_input_box_rti'
        return result
