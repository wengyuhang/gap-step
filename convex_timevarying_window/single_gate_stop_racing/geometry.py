"""Finite convex frame distances and local floating-point motion enclosures."""
import heapq
import math
import time
import numpy as np
import casadi as ca
from convex_timevarying_window.dynamic_passage_minco.safety import (
    power_to_bernstein, restrict, split, sine_range, overlap_intervals)


def halfplanes(aperture):
    v=aperture.physical_vertices
    e=np.roll(v,-1,axis=0)-v
    n=np.column_stack((e[:,1],-e[:,0]));n/=np.linalg.norm(n,axis=1)[:,None]
    b=np.sum(n*v,axis=1)
    if np.mean(b)<0:n=-n;b=-b
    return n,b


def symbolic_constraints(aperture,q,radius):
    if aperture.kind=='circle':
        rho=ca.sqrt(ca.sumsqr(q[:2])+1e-14)
        return [(rho-aperture.radius)**2+q[2]**2-radius**2]
    v=aperture.physical_vertices;result=[]
    for a,b in zip(v,np.roll(v,-1,axis=0)):
        e=ca.DM(b-a);delta=q[:2]-ca.DM(a)
        u=ca.fmin(1.,ca.fmax(0.,ca.dot(delta,e)/ca.dot(e,e)))
        result.append(ca.sumsqr(delta-u*e)+q[2]**2-radius**2)
    return result


def passage_python(plan,window,radius,piece,deadline=np.inf):
    R=window.state_at(0.)[1];amp=R.T@window.motion.translation_amplitude
    omega=2*np.pi/window.motion.translation_period;phase=window.motion.phase
    coeff=plan.coefficients[piece]@R;coeff=coeff.copy();coeff[0]-=window.center0@R
    starts=np.r_[plan.made_at,plan.made_at+np.cumsum(plan.durations)[:-1]]
    start=starts[piece];T=plan.durations[piece]
    base=power_to_bernstein(coeff);queue=[];serial=0;best=np.inf;witness=None;intervals=[]
    aperture=window.aperture
    if aperture.kind!='circle':
        vertices=aperture.physical_vertices
        edges=np.roll(vertices,-1,axis=0)-vertices
        lengths=np.linalg.norm(edges,axis=1)
        tangents=edges/lengths[:,None]
        normals=np.column_stack((tangents[:,1],-tangents[:,0]))
        along0=np.sum(vertices*tangents,axis=1)
        across0=np.sum(vertices*normals,axis=1)
    def distance2(q):
        if aperture.kind=='circle':return (np.linalg.norm(q[:2])-aperture.radius)**2
        delta=q[:2]-vertices
        u=np.clip(np.sum(delta*tangents,axis=1),0.,lengths)
        return float(np.min(np.sum((delta-u[:,None]*tangents)**2,axis=1)))
    def position(u):
        q=np.polynomial.polynomial.polyval(u,coeff)
        return q-amp*np.sin(omega*(start+u*T)+phase)
    def value(u):
        q=position(u)
        return distance2(q)+q[2]*q[2]
    def enqueue(a,b,control):
        nonlocal serial,best,witness
        sl,su=sine_range(omega*(start+a*T)+phase,omega*(start+b*T)+phase)
        lower=control.min(axis=0)-np.maximum(amp*sl,amp*su)-1e-10
        upper=control.max(axis=0)-np.minimum(amp*sl,amp*su)+1e-10
        lo,hi=lower[:2],upper[:2]
        if aperture.kind=='circle':
            nearest=np.maximum(np.maximum(lo,-hi),0.)
            farthest=np.maximum(abs(lo),abs(hi))
            d=max(np.linalg.norm(nearest)-aperture.radius,
                  aperture.radius-np.linalg.norm(farthest),0.)
            planar=d*d
        else:
            # Project the box into each finite segment's orthonormal frame.
            # Independent projection gaps give a conservative squared bound,
            # including beyond segment endpoints; no boundary-distance calls.
            center=(lo+hi)/2;half=(hi-lo)/2
            ac=tangents@center-along0;ar=abs(tangents)@half
            nc=normals@center-across0;nr=abs(normals)@half
            ag=np.maximum(np.maximum(-ac-ar,ac-ar-lengths),0.)
            ng=np.maximum(abs(nc)-nr,0.)
            planar=float(np.min(ag*ag+ng*ng))
        normal=max(lower[2],-upper[2],0.)
        bound=planar+normal*normal
        for u in (a,(a+b)/2,b):
            val=value(u)
            if val<best:best=val;witness=u
        serial+=1;heapq.heappush(queue,(bound,serial,a,b,control))
    # Inside-aperture crossings other than the designated knot are forbidden.
    roots=np.polynomial.polynomial.polyroots(coeff[:,2])
    for z in roots:
        if abs(z.imag)<1e-7 and 1e-6<z.real<1-1e-6:
            if window.aperture.contains(position(float(z.real))[:2]):
                return dict(passed=False,outcome='repeat_aperture_crossing',minimum_sample_distance_m=0.,
                            witness_u=None,boxes=0,overlap_intervals=[])
    slabs=overlap_intervals(coeff[:,2],radius)
    primary=next(((a,b) for a,b in slabs if (b>=1.-1e-7 if piece==0 else a<=1e-7)),None)
    for a,b in slabs:
        intervals.append(dict(piece=piece,start=float(start+a*T),end=float(start+b*T)))
        enqueue(a,b,restrict(base,a,b))
    count=0
    while queue and count<512:
        if time.perf_counter()>=deadline:
            return dict(passed=False,outcome='check_budget',minimum_sample_distance_m=math.sqrt(best),
                        witness_u=witness,boxes=count,overlap_intervals=intervals,primary_interval=primary)
        bound,_,a,b,control=heapq.heappop(queue)
        if bound>=radius**2:continue
        if best<(radius-2e-4)**2 or (b-a)*T<1e-6:break
        left,right=split(control,.5);mid=(a+b)/2
        enqueue(a,mid,left);enqueue(mid,b,right);count+=1
    else:
        if not queue:
            return dict(passed=True,outcome='floating_local_bound',minimum_sample_distance_m=math.sqrt(best),
                        witness_u=witness,boxes=count,overlap_intervals=intervals)
    return dict(passed=False,outcome='violated_or_unresolved',minimum_sample_distance_m=math.sqrt(best),
                witness_u=witness,boxes=count,overlap_intervals=intervals,primary_interval=primary)


def passage(plan,window,radius,piece,deadline=np.inf):
    from .local_bounds import check
    if time.perf_counter()>=deadline:
        return dict(passed=False,outcome='check_budget',minimum_sample_distance_m=np.inf,
                    witness_u=None,boxes=0,overlap_intervals=[])
    R=window.state_at(0.)[1];amp=R.T@window.motion.translation_amplitude
    if abs(amp[2])>1e-9 or window.motion.rotation_enabled:
        raise ValueError('Local bounds require fixed planes and in-plane translation')
    coeff=plan.coefficients[piece]@R;coeff=coeff.copy();coeff[0]-=window.center0@R
    start=float(plan.made_at+sum(plan.durations[:piece]));T=float(plan.durations[piece])
    omega=2*np.pi/window.motion.translation_period;phase=window.motion.phase
    for z in np.polynomial.polynomial.polyroots(coeff[:,2]):
        if abs(z.imag)<1e-7 and 1e-6<z.real<1-1e-6:
            q=np.polynomial.polynomial.polyval(z.real,coeff)-amp*np.sin(omega*(start+z.real*T)+phase)
            if window.aperture.contains(q[:2]):
                return dict(passed=False,outcome='repeat_aperture_crossing',minimum_sample_distance_m=0.,
                            witness_u=None,boxes=0,overlap_intervals=[])
    cuts=np.asarray(overlap_intervals(coeff[:,2],radius)).reshape(-1,2)
    base=power_to_bernstein(coeff)
    controls=np.asarray([restrict(base,a,b) for a,b in cuts]).reshape(-1,8,3)
    intervals=[dict(piece=piece,start=float(start+a*T),end=float(start+b*T)) for a,b in cuts]
    ap=window.aperture;circle=ap.kind=='circle'
    vertices=np.zeros((0,2)) if circle else np.asarray(ap.physical_vertices)
    outcome,best,witness,boxes=check(controls,cuts,coeff,start,T,amp,float(omega),float(phase),
                                   vertices,circle,float(ap.radius) if circle else 0.,float(radius))
    expired=time.perf_counter()>=deadline
    return dict(passed=bool(outcome==0 and not expired),
                outcome='check_budget' if expired else ('floating_local_bound','violated','unresolved')[outcome],
                minimum_sample_distance_m=math.sqrt(best),witness_u=float(witness) if len(cuts) else None,
                boxes=int(boxes),overlap_intervals=intervals,
                primary_interval=next(((float(a),float(b)) for a,b in cuts
                    if (b>=1.-1e-7 if piece==0 else a<=1e-7)),None))
