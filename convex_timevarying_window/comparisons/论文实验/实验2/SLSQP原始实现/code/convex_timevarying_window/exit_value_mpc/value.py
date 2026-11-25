"""Differentiable center-policy proxy; not a safety or cost-to-go certificate."""
import casadi as ca
import numpy as np
from convex_timevarying_window.event_free_terminal_mpc.core import symbolic_pose


def norm(v): return ca.sqrt(ca.sumsqr(v)+1e-8)


def travel_time(p,v,target,cfg):
    dvec=target-p;d=norm(dvec);direction=dvec/d
    longitudinal=ca.fmin(cfg.speed,ca.fmax(-cfg.speed,ca.dot(v,direction)))
    lateral=norm(v-longitudinal*direction)
    # Acceleration-limited straight-line minimum time, with a lateral turning
    # allowance. This approximation does not test collision along the route.
    accel_distance=(cfg.speed**2-longitudinal**2)/(2*cfg.acceleration)
    accelerate=(ca.sqrt(longitudinal**2+2*cfg.acceleration*d)-longitudinal)/cfg.acceleration
    cruise=d/cfg.speed+(cfg.speed-longitudinal)**2/(2*cfg.acceleration*cfg.speed)
    return ca.if_else(d<accel_distance,accelerate,cruise)+.5*lateral/cfg.acceleration


def continuation(p,v,a,t,first,windows,signs,cfg,goal):
    """Predicted time to finish remaining center crossings from an exit state.

    Two fixed-point updates query each moving center at its predicted arrival.
    End velocities are an explicit geometric center-policy approximation, not
    the MINCO/MPC center baseline. PVA and absolute phase all affect the value.
    """
    began=t;v=v+.5*cfg.response_time*a;slab=cfg.radius+cfg.frame_thickness
    for wi in range(first,len(windows)):
        w=windows[wi];target,_=symbolic_pose(w,t)
        T=travel_time(p,v,target,cfg)
        for _ in range(2):
            target,_=symbolic_pose(w,t+T)
            T=travel_time(p,v,target,cfg)
        center,_=symbolic_pose(w,t+T)
        n=ca.DM(-signs[wi]*w.state_at(0.)[1][:,2])
        nxt=ca.DM(windows[wi+1].center0 if wi+1<len(windows) else goal)
        incoming=(center-p)/norm(center-p);outgoing=(nxt-center)/norm(nxt-center)
        direction=incoming+outgoing
        direction+=n*ca.fmax(.5-ca.dot(n,direction),0.)
        direction/=norm(direction)
        turn=ca.fmax(.05,1.-ca.dot(incoming,outgoing))
        speed=ca.fmin(cfg.speed,ca.sqrt(cfg.acceleration*norm(nxt-center)/(2*turn)))
        v=speed*direction
        t=t+T+slab/ca.fmax(.1,ca.dot(n,v))
        p=center+slab*n
    return t-began


def departure(plan,gate,window,sign,slab):
    """Numerical polynomial root for actual body exit from a fixed plane."""
    k=gate-plan.first_gate;at=float(plan.crossing_times[k]);normal=sign*window.state_at(0.)[1][:,2]
    for i in range(2*(k+1),len(plan.durations)):
        c=plan.coefficients[i]@normal;c=c.copy();c[0]+=slab-normal@window.center0
        if c[0]<=0:return at
        roots=np.polynomial.polynomial.polyroots(c)
        roots=sorted(float(r.real) for r in roots if abs(r.imag)<1e-7 and 0<=r.real<=1
            and np.polynomial.polynomial.polyval(r.real,np.polynomial.polynomial.polyder(c))<0.)
        if roots:return at+roots[0]*plan.durations[i]
        at+=plan.durations[i]
    return float('inf')
