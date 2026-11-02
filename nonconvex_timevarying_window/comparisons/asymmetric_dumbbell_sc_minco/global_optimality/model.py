"""Explicit seven-variable PV-MINCO, equivalent to the original QR solve.

Physical coordinates remove the numerical SC integral only from the offline
reference model. Interior A/J still minimize snap; they are not extra freedoms.
"""
import math
import casadi as ca
import numpy as np

from convex_timevarying_window.dynamic_passage_velocity_minco.hard_dynamics import flat_actuators
from convex_timevarying_window.dynamic_passage_velocity_minco.allocation_mpc import KF
from convex_timevarying_window.single_gate_stop_racing.geometry import symbolic_constraints
from convex_timevarying_window.togt_minco_mpc.trajectory import basis
from ..spin.scenario import symbolic_pose


def hermite_matrix():
    a = np.vstack([np.asarray(basis(s, d)).ravel() for s in (0., 1.) for d in range(4)])
    # These endpoint Hermite coefficients are exact multiples of 1/6.
    return np.rint(np.linalg.inv(a)*6)/6


HERMITE = hermite_matrix()


def coefficients(times, point, velocity, start, goal):
    """Eliminate common knot A/J through snap and fifth-derivative continuity."""
    t, r = times[0], times[1]
    h = ca.DM(HERMITE)
    # Each row is a 3-vector; endpoints have zero V/A/J in this frozen task.
    zero = 0*point.T
    c0 = h @ ca.vertcat(start.T, zero, zero, zero, point.T, t*velocity.T, zero, zero)
    c1 = h @ ca.vertcat(point.T, r*velocity.T, zero, zero, goal.T, zero, zero, zero)
    rows=[]; right=[]
    for d in (4, 5):
        b0, b1 = basis(1., d).T/t**d, basis(0., d).T/r**d
        aa = (b0@h[:, 6])*t**2-(b1@h[:, 2])*r**2
        jj = (b0@h[:, 7])*t**3-(b1@h[:, 3])*r**3
        rows.append((aa,jj)); right.append(-(b0@c0-b1@c1))
    a,b=rows[0]; c,d=rows[1]
    # Positive factored determinant avoids catastrophic interval cancellation.
    det=4800*(t+r)**4/(t**4*r**4)
    acceleration=(d*right[0]-b*right[1])/det
    jerk=(-c*right[0]+a*right[1])/det
    c0 += h[:, 6]@(t**2*acceleration)+h[:, 7]@(t**3*jerk)
    c1 += h[:, 2]@(r**2*acceleration)+h[:, 3]@(r**3*jerk)
    return ca.vertcat(c0,c1)


class PhysicalModel:
    def __init__(self, track, cfg, dynamics_slots=33, geometry_slots=32):
        self.track,self.cfg=track,cfg
        self.dynamics_slots,self.geometry_slots=dynamics_slots,geometry_slots
        self.window=track.windows[0]
        x=ca.MX.sym('physical',7)
        par=ca.MX.sym('witness',2*dynamics_slots+2*geometry_slots)
        times=ca.exp(x[:2])
        center,rot=symbolic_pose(self.window,times[0])
        point=center+rot[:,:2]@x[2:4]
        cc=coefficients(times,point,x[4:],ca.DM(track.start),ca.DM(track.goal))
        g=[]; high=KF*1000**2;low=KF*150**2
        def state(piece,u,d=0):
            return cc[piece*8:piece*8+8,:].T@basis(u,d)/times[piece]**d
        for piece in range(2):
            for k in range(dynamics_slots):
                u=par[piece*dynamics_slots+k]
                force,rates,motors=flat_actuators(*[state(piece,u,d) for d in (1,2,3,4)])
                g.extend(ca.vertsplit(1-(rates/ca.DM([3.8,3.8,3.49]))**2))
                g.extend(ca.vertsplit((motors-low)/high))
                g.extend(ca.vertsplit((high-motors)/high))
                pos=state(piece,u)
                g.extend(ca.vertsplit((pos-ca.DM(cfg.workspace_lower))/30))
                g.extend(ca.vertsplit((ca.DM(cfg.workspace_upper)-pos)/30))
        self.dynamic_count=len(g)
        rho=cfg.radius+cfg.frame_thickness+cfg.passage_margin+cfg.witness_slack
        for piece in range(2):
            for k in range(geometry_slots):
                delta=ca.fmin(par[2*dynamics_slots+piece*geometry_slots+k],times[piece])
                u=1-delta/times[0] if piece==0 else delta/times[1]
                at=times[0]-delta if piece==0 else times[0]+delta
                ctr,rr=symbolic_pose(self.window,at)
                q=rr.T@(state(piece,u)-ctr)
                g.extend(symbolic_constraints(self.window.aperture,q,rho))
        # Original desired direction: world +y.
        g.append(x[5]-.1)
        gg=ca.vertcat(*g)
        f=ca.log1p(ca.sum1(times))
        self.function=ca.Function('physical_global_eval',[x,par],
            [f,ca.gradient(f,x),gg,ca.jacobian(gg,x),cc],{'cse':True})
        self.coefficient_function=ca.Function('explicit_coefficients',[x],[cc])
        self.initial_dynamics=np.tile(np.r_[np.linspace(0,1,17),np.ones(dynamics_slots-17)],(2,1))
        base=np.asarray(cfg.passage_seed_offsets)
        self.initial_geometry=np.tile(np.r_[base,np.zeros(geometry_slots-len(base))],(2,1))

    def parameters(self,dynamics=None,geometry=None):
        return np.r_[(self.initial_dynamics if dynamics is None else dynamics).ravel(),
                     (self.initial_geometry if geometry is None else geometry).ravel()]

    def plan(self,x):
        from convex_timevarying_window.dynamic_passage_velocity_minco.trajectory import Plan
        cc=np.asarray(self.coefficient_function(x)).reshape(2,8,3)
        return Plan(0.,0,np.exp(x[:2]),cc,np.asarray(x),0.,0.,'offline_global_attempt',1)


def bound_knot_velocity(cfg):
    """A valid bound implied by original 17 workspace nodes on piece zero.

    Reconstruct a degree-seven polynomial derivative at s=1 from those nodes.
    No additional physical speed limit or arbitrary search box is introduced.
    """
    nodes=np.linspace(0,1,17)
    v=np.vander(nodes,8,increasing=True)
    weights=np.asarray(basis(1.,1)).ravel()@np.linalg.pinv(v)
    lo=np.asarray(cfg.workspace_lower);hi=np.asarray(cfg.workspace_upper)
    l=(np.maximum(weights,0).sum()*lo+np.minimum(weights,0).sum()*hi)/.3
    u=(np.maximum(weights,0).sum()*hi+np.minimum(weights,0).sum()*lo)/.3
    # Widen for the numerical pseudoinverse used to compute these constants.
    return l-1e-5,u+1e-5
