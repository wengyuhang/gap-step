"""Existing finite-frame Bernstein check extended only to analytic arc motion."""
import copy,heapq,math,time
import casadi as ca
import numpy as np
from convex_timevarying_window.dynamic_passage_minco.safety import power_to_bernstein,restrict,split,overlap_intervals
from convex_timevarying_window.single_gate_stop_racing.geometry import symbolic_constraints as original_constraints
from .scenario import FRAME_OFFSET

def frame_aperture(ap):
    ap=copy.copy(ap)
    if ap.kind=='circle':ap.radius+=FRAME_OFFSET
    else:
        v=ap.physical_vertices.copy();v+=np.sign(v)*FRAME_OFFSET;ap.physical_vertices=v
    return ap

def symbolic_constraints(ap,q,radius):return original_constraints(frame_aperture(ap),q,radius)

def passage(plan,w,radius,piece,deadline=np.inf):
    R=w.R;coeff=(plan.coefficients[piece]@R).copy();coeff[0]-=w.center0@R
    start=float(plan.made_at+sum(plan.durations[:piece]));T=float(plan.durations[piece]);ap=frame_aperture(w.aperture)
    def position(u):return np.polynomial.polynomial.polyval(u,coeff)-w.motion.local(start+u*T)[0]
    def value(u):
        q=position(u)
        if ap.kind=='circle':d2=(np.linalg.norm(q[:2])-ap.radius)**2
        else:
            v=ap.physical_vertices;e=np.roll(v,-1,axis=0)-v;delta=q[:2]-v
            alpha=np.clip(np.sum(delta*e,axis=1)/np.sum(e*e,axis=1),0,1)
            d2=np.min(np.sum((delta-alpha[:,None]*e)**2,axis=1))
        return float(d2+q[2]**2)
    for z in np.polynomial.polynomial.polyroots(coeff[:,2]):
        if abs(z.imag)<1e-7 and 1e-6<z.real<1-1e-6 and w.aperture.contains(position(z.real)[:2]):
            return dict(passed=False,outcome='repeat_aperture_crossing',minimum_sample_distance_m=0.,witness_u=None,boxes=0,overlap_intervals=[])
    queue=[];serial=0;best=np.inf;witness=None;safe=np.inf
    if ap.kind!='circle':
        vertices=ap.physical_vertices;edges=np.roll(vertices,-1,axis=0)-vertices;lengths=np.linalg.norm(edges,axis=1)
        tangents=edges/lengths[:,None];normals=np.column_stack((tangents[:,1],-tangents[:,0]));along0=np.sum(vertices*tangents,axis=1);across0=np.sum(vertices*normals,axis=1)
    def enqueue(a,b,control):
        nonlocal best,witness,serial
        lo,hi=w.motion.local_bounds(start+a*T,start+b*T);lower=control.min(axis=0)-hi-1e-10;upper=control.max(axis=0)-lo+1e-10
        lo,hi=lower[:2],upper[:2]
        if ap.kind=='circle':
            near=np.maximum(np.maximum(lo,-hi),0.);far=np.maximum(abs(lo),abs(hi));d=max(np.linalg.norm(near)-ap.radius,ap.radius-np.linalg.norm(far),0.);planar=d*d
        else:
            center=(lo+hi)/2;half=(hi-lo)/2;ac=tangents@center-along0;ar=abs(tangents)@half;nc=normals@center-across0;nr=abs(normals)@half
            ag=np.maximum(np.maximum(-ac-ar,ac-ar-lengths),0.);ng=np.maximum(abs(nc)-nr,0.);planar=float(np.min(ag*ag+ng*ng))
        bound=planar+max(lower[2],-upper[2],0.)**2
        for u in (a,(a+b)/2,b):
            d=value(u)
            if d<best:best,witness=d,u
        serial+=1;heapq.heappush(queue,(bound,serial,a,b,control))
    intervals=overlap_intervals(coeff[:,2],radius)
    for a,b in intervals:enqueue(a,b,restrict(power_to_bernstein(coeff),a,b))
    boxes=0;outcome='floating_local_bound'
    while queue and boxes<512:
        if time.perf_counter()>=deadline:outcome='check_budget';break
        bound,_,a,b,control=heapq.heappop(queue)
        if bound>=radius**2:safe=min(safe,bound);continue
        if best<(radius-2e-4)**2:outcome='violated';break
        if (b-a)*T<1e-6:outcome='unresolved';break
        left,right=split(control,.5);mid=(a+b)/2;enqueue(a,mid,left);enqueue(mid,b,right);boxes+=1
    if queue and outcome=='floating_local_bound':outcome='budget_unresolved'
    return dict(passed=outcome=='floating_local_bound',outcome=outcome,minimum_sample_distance_m=math.sqrt(best),witness_u=witness,boxes=boxes,
        distance_lower_bound_m=math.sqrt(safe) if outcome=='floating_local_bound' else None,
        overlap_intervals=[dict(piece=piece,start=start+a*T,end=start+b*T) for a,b in intervals],
        evidence='floating-point Bernstein and analytic arc ranges; no formal interval certificate')
