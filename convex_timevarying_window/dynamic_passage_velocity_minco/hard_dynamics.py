"""Nominal drag-flatness actuator constraints; not a closed-loop certificate."""
import casadi as ca
import numpy as np
from .repaired_planner import flat_force
from .allocation_mpc import MIXER, X500_INERTIA, KF


def build_map():
    v,a,j,s=[ca.MX.sym(n,3) for n in ('velocity','acceleration','jerk','snap')]
    force=flat_force(v,a);magnitude=ca.sqrt(ca.sumsqr(force)+1e-12)
    z=force/magnitude;side=ca.cross(z,ca.DM([0.,1.,0.]))
    y=side/ca.sqrt(ca.sumsqr(side)+1e-12)
    R=ca.horzcat(ca.cross(y,z),y,z)
    orientation=ca.Function('hard_orientation',[v,a],[R,magnitude])
    R,F=orientation(v,a)
    Rdot=ca.reshape(ca.jtimes(ca.vec(R),ca.vertcat(v,a),ca.vertcat(a,j)),3,3)
    W=R.T@Rdot
    omega=ca.vertcat(W[2,1]-W[1,2],W[0,2]-W[2,0],W[1,0]-W[0,1])/2
    rates=ca.Function('hard_rates',[v,a,j],[omega])
    omega=rates(v,a,j)
    alpha=ca.jtimes(omega,ca.vertcat(v,a,j),ca.vertcat(a,j,s))
    inertia=ca.DM(X500_INERTIA)
    torque=inertia*alpha+ca.cross(omega,inertia*omega)
    rotor=ca.DM(np.linalg.inv(MIXER))@ca.vertcat(F,torque)
    return ca.Function('hard_flat_actuators',[v,a,j,s],[F,omega,rotor])


flat_actuators=build_map()


def constraints(state,times,nodes=16,workspace_lower=None,workspace_upper=None,*,thrust_fraction=1.):
    if not np.isfinite(thrust_fraction) or not (150./1000.)**2 < thrust_fraction <= 1.:
        raise ValueError('Planning thrust fraction must exceed the rotor minimum and be <= 1')
    result=[];low=KF*150**2;high=KF*1000**2
    planning_high=high*thrust_fraction
    limits=ca.DM([3.8,3.8,3.49])
    for piece in range(int(times.numel())):
        for u in np.linspace(0.,1.,nodes+1):
            force,rates,rotor=flat_actuators(*[state(piece,float(u),d) for d in (1,2,3,4)])
            result.extend(ca.vertsplit(1.-(rates/limits)**2))
            result.extend(ca.vertsplit((rotor-low)/high))
            # Keep the original residual scaling and the physical lower bound.
            # This reserves feedback authority only in the planning upper bound.
            result.extend(ca.vertsplit((planning_high-rotor)/high))
            if workspace_lower is not None:
                pos=state(piece,float(u),0)
                result.extend(ca.vertsplit((pos-ca.DM(workspace_lower))/30.))
                result.extend(ca.vertsplit((ca.DM(workspace_upper)-pos)/30.))
    return result
