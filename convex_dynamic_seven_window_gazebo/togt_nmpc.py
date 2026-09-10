#!/usr/bin/env python3
"""TOGT-paper tracking NMPC adapted to the PX4 Gazebo x500.

The formulation follows Sun et al. TRO 2022, which is the controller cited by
the TOGT flight experiments: full p/v/q/omega rigid-body prediction, four
rotor thrust inputs, N=20, dt=50 ms, and the published diagonal weights.
"""
from __future__ import annotations

from pathlib import Path
import sys
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE / ".runtime/togt_mpc_python"))
import casadi as ca

MASS = 2.0643076923076924
INERTIA = np.array([0.02383948, 0.02394241, 0.04399995])
MAX_ROTOR_THRUST = 8.54858
MAX_BODY_RATE = np.deg2rad([220.0, 220.0, 200.0])
DT = 0.05
N = 20
G = 9.8066
Q_POSITION = np.array([200.0, 200.0, 500.0])
Q_VELOCITY = np.ones(3)
Q_QUATERNION = np.array([5.0, 5.0, 200.0])
Q_BODY_RATE = np.ones(3)
Q_INPUT = np.full(4, 6.0)
TORQUE_FROM_ROTORS_FRD = np.array([
    [-.174, .174, .174, -.174],
    [ .174,-.174, .174, -.174],
    [ .016, .016,-.016, -.016],
])


def quaternion_product(a, b):
    return ca.vertcat(
        a[0]*b[0]-ca.dot(a[1:4], b[1:4]),
        a[0]*b[1]+b[0]*a[1]+a[2]*b[3]-a[3]*b[2],
        a[0]*b[2]+b[0]*a[2]+a[3]*b[1]-a[1]*b[3],
        a[0]*b[3]+b[0]*a[3]+a[1]*b[2]-a[2]*b[1],
    )


def rotation_from_quaternion(q):
    w,x,y,z=q[0],q[1],q[2],q[3]
    return ca.vertcat(
        ca.horzcat(1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y)),
        ca.horzcat(2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x)),
        ca.horzcat(2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)),
    )


class TOGTTrackingNMPC:
    def __init__(self):
        x0=ca.SX.sym("x0",13); xr=ca.SX.sym("xr",13,N+1); ur=ca.SX.sym("ur",4,N)
        u=ca.SX.sym("u",4,N); state=x0; cost=0
        states=[]
        inertia=ca.DM(INERTIA); torque_map=ca.DM(TORQUE_FROM_ROTORS_FRD)
        def dynamics(xv,uv):
            p=xv[0:3]; v=xv[3:6]; q=xv[6:10]; omg=xv[10:13]
            rot=rotation_from_quaternion(q)
            pd=v
            vd=ca.vertcat(0,0,G)-rot[:,2]*ca.sum1(uv)/MASS
            qd=.5*quaternion_product(q,ca.vertcat(0,omg))
            tau=torque_map@uv
            od=(tau-ca.cross(omg,inertia*omg))/inertia
            return ca.vertcat(pd,vd,qd,od)
        def state_cost(xv,ref):
            ep=xv[0:3]-ref[0:3]; ev=xv[3:6]-ref[3:6]
            qr=ref[6:10]; q= xv[6:10]
            qe=quaternion_product(ca.vertcat(qr[0],-qr[1:4]),q)[1:4]
            eo=xv[10:13]-ref[10:13]
            return ca.dot(ca.DM(Q_POSITION)*ep,ep)+ca.dot(ca.DM(Q_VELOCITY)*ev,ev)+ca.dot(ca.DM(Q_QUATERNION)*qe,qe)+ca.dot(ca.DM(Q_BODY_RATE)*eo,eo)
        for k in range(N):
            states.append(state); uk=u[:,k]
            cost += state_cost(state,xr[:,k])+ca.dot(ca.DM(Q_INPUT)*(uk-ur[:,k]),uk-ur[:,k])
            k1=dynamics(state,uk); k2=dynamics(state+DT*k1/2,uk); k3=dynamics(state+DT*k2/2,uk); k4=dynamics(state+DT*k3,uk)
            state=state+DT*(k1+2*k2+2*k3+k4)/6
            state=ca.vertcat(state[0:6],state[6:10]/ca.norm_2(state[6:10]),state[10:13])
        states.append(state); cost += state_cost(state,xr[:,N])
        parameter=ca.vertcat(x0,ca.reshape(xr,-1,1),ca.reshape(ur,-1,1))
        problem={"x":ca.reshape(u,-1,1),"p":parameter,"f":cost}
        options={"print_time":False,"ipopt.print_level":0,"ipopt.sb":"yes","ipopt.max_iter":25,
                 "ipopt.tol":1e-3,"ipopt.acceptable_tol":3e-3,"ipopt.warm_start_init_point":"yes"}
        self.solver=ca.nlpsol("togt_tracking","ipopt",problem,options)
        self.rollout=ca.Function("rollout",[x0,ca.reshape(u,-1,1)],[ca.horzcat(*states)])
        self.lower=np.zeros(4*N); self.upper=np.full(4*N,MAX_ROTOR_THRUST)
        self.guess=np.full(4*N,MASS*G/4)

    def solve(self,x0,x_reference,u_reference):
        xr=np.asarray(x_reference,float).T; ur=np.asarray(u_reference,float).T
        parameter=np.r_[np.asarray(x0,float),xr.reshape(-1,order="F"),ur.reshape(-1,order="F")]
        answer=self.solver(x0=self.guess,p=parameter,lbx=self.lower,ubx=self.upper)
        controls=np.asarray(answer["x"]).reshape((4,N),order="F")
        self.guess=np.c_[controls[:,1:],controls[:,-1]].reshape(-1,order="F")
        states=np.asarray(self.rollout(np.asarray(x0,float),controls.reshape(-1,order="F")))
        return controls[:,0],states[10:13,1],{"iterations":int(self.solver.stats().get("iter_count",-1)),"success":bool(self.solver.stats()["success"])}


def rotation_to_quaternion(rotation):
    from scipy.spatial.transform import Rotation
    xyzw=Rotation.from_matrix(rotation).as_quat()
    return np.array([xyzw[3],xyzw[0],xyzw[1],xyzw[2]])


def flat_reference_ned(reference):
    """Add q, omega and rotor thrust references in PX4 NED/FRD frames."""
    p=np.asarray(reference["position_enu"]); v=np.asarray(reference["velocity_enu"])
    a=np.asarray(reference["acceleration_enu"]); j=np.asarray(reference["jerk_enu"]); s=np.asarray(reference["snap_enu"])
    from convex_dynamic_seven_window_gazebo.x500_togt.native_backend import AnalyticTOGTCore
    pvajs=np.stack((p,v,a,j,s),axis=1); dyn=AnalyticTOGTCore().sample_dynamics(pvajs)
    c_ne=np.array([[0,1,0],[1,0,0],[0,0,-1.]])
    c_flu_frd=np.diag([1.,-1.,-1.]); q=[]
    for ai in a:
        zb=(ai+np.array([0,0,G])); zb/=np.linalg.norm(zb)
        xc=np.array([1.,0,0]); yb=np.cross(zb,xc); yb/=np.linalg.norm(yb); xb=np.cross(yb,zb)
        q.append(rotation_to_quaternion(c_ne@np.column_stack((xb,yb,zb))@c_flu_frd))
    x=np.column_stack((p[:,1],p[:,0],-p[:,2],v[:,1],v[:,0],-v[:,2],np.asarray(q),dyn["body_rate"]@c_flu_frd))
    return x,np.asarray(dyn["rotor_thrusts"])

__all__=["TOGTTrackingNMPC","flat_reference_ned","DT","N","MASS","G"]
