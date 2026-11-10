"""Tilt/heading flatness chart: no horizontal-thrust cross-product singularity.

This changes the heading convention, not just the quaternion representation.
The north chart covers b3 != -world_z; its pole is explicitly audited.
No claim of globally nonsingular attitude recovery is made.
"""
import casadi as ca
import numpy as np
from convex_timevarying_window.dynamic_passage_velocity_minco import hard_dynamics as old

HOVER_HEADING=np.pi/2
CHART_MIN=1e-6
FORCE_MIN=1e-4


def rotation_from_direction(z,heading=HOVER_HEADING):
    # Rodrigues' shortest tilt from world +Z to body +Z, followed by a
    # constant heading about body +Z. Equivalent to the north Hopf chart.
    # The floor only keeps invalid trial evaluations finite; qualification
    # explicitly rejects its neighbourhood instead of calling it feasible.
    axis=ca.vertcat(-z[1],z[0],0.)
    K=ca.skew(axis)
    tilt=ca.DM.eye(3)+K+K@K/ca.fmax(1.+z[2],1e-12)
    c,s=np.cos(heading),np.sin(heading)
    yaw=ca.DM([[c,-s,0.],[s,c,0.],[0.,0.,1.]])
    return tilt@yaw


def build_maps():
    v,a,j,s=[ca.MX.sym(n,3) for n in ('velocity','acceleration','jerk','snap')]
    force=old.flat_force(v,a);magnitude=ca.sqrt(ca.sumsqr(force)+1e-12)
    z=force/magnitude;R=rotation_from_direction(z)
    orientation=ca.Function('tilt_heading_orientation',[v,a],[R,magnitude,1.+z[2]])
    Rdot=ca.reshape(ca.jtimes(ca.vec(R),ca.vertcat(v,a),ca.vertcat(a,j)),3,3)
    W=R.T@Rdot
    omega=ca.vertcat(W[2,1]-W[1,2],W[0,2]-W[2,0],W[1,0]-W[0,1])/2
    alpha=ca.jtimes(omega,ca.vertcat(v,a,j),ca.vertcat(a,j,s))
    inertia=ca.DM(old.X500_INERTIA)
    torque=inertia*alpha+ca.cross(omega,inertia*omega)
    rotor=ca.DM(np.linalg.inv(old.MIXER))@ca.vertcat(magnitude,torque)
    actuators=ca.Function('tilt_heading_actuators',[v,a,j,s],[magnitude,omega,rotor])
    rates=ca.Function('tilt_heading_rates',[v,a,j,s],[omega,alpha])
    return orientation,actuators,rates


orientation,flat_actuators,rates=build_maps()


def constraints(*args,**kwargs):
    # Same residuals, physical bounds, normalization and collocation nodes.
    # Construction is serial in an isolated planner process; restore always.
    previous=old.flat_actuators
    old.flat_actuators=flat_actuators
    try:return old.constraints(*args,**kwargs)
    finally:old.flat_actuators=previous


def domain(v,a):
    R,F,margin=orientation(v,a)
    R=np.array(R);F=float(F);margin=float(margin)
    finite=bool(np.all(np.isfinite(R)) and np.isfinite(F) and np.isfinite(margin))
    orth=float(np.max(abs(R.T@R-np.eye(3)))) if finite else float('inf')
    return dict(passed=bool(finite and F>FORCE_MIN and margin>CHART_MIN and orth<1e-7),
                force_n=F,north_chart_margin=margin,orthogonality_error=orth)
