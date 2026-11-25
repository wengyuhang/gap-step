"""Local Bernstein enclosure; exact circle-outside distance, shared ODE spline."""
import heapq
import math
import numpy as np
from scipy.optimize import minimize_scalar
from convex_timevarying_window.dynamic_passage_minco.safety import power_to_bernstein,restrict,split,overlap_intervals
from .motion import R0

def passage(plan,window,radius,frame_offset,piece):
    coeff=(plan.coefficients[piece]@R0).copy();coeff[0]-=window.center0@R0
    start=float(plan.made_at+sum(plan.durations[:piece]));T=float(plan.durations[piece])
    frame_radius=window.aperture.radius+frame_offset
    def value(u):
        t=start+u*T;q=np.polynomial.polynomial.polyval(u,coeff)-R0.T@window.motion.translation(t)[0]
        return max(frame_radius-np.linalg.norm(q[:2]),0.)**2+q[2]**2
    intervals=overlap_intervals(coeff[:,2],radius);queue=[];serial=0;best=np.inf;witness=None;safe=np.inf
    def enqueue(a,b,control):
        nonlocal serial,best,witness
        lo,hi=window.motion.translation_bounds(start+a*T,start+b*T);lo,hi=lo@R0,hi@R0
        lower=control.min(axis=0)-hi-1e-10;upper=control.max(axis=0)-lo+1e-10
        rho=np.linalg.norm(np.maximum(abs(lower[:2]),abs(upper[:2])))
        bound=max(frame_radius-rho,0.)**2+max(lower[2],-upper[2],0.)**2
        for u in (a,(a+b)/2,b):
            q=value(u)
            if q<best:best,witness=q,u
        serial+=1;heapq.heappush(queue,(bound,serial,a,b,control))
    for a,b in intervals:enqueue(a,b,restrict(power_to_bernstein(coeff),a,b))
    boxes=0;outcome='numerically_bounded'
    while queue and boxes<512:
        bound,_,a,b,control=heapq.heappop(queue)
        if bound>=radius**2:safe=min(safe,bound);continue
        if best<(radius-2e-4)**2:
            q=minimize_scalar(value,bounds=(a,b),method='bounded',options={'xatol':1e-11})
            if q.fun<best:best,witness=float(q.fun),float(q.x)
            outcome='violated';break
        if (b-a)*T<1e-6:outcome='unresolved';break
        left,right=split(control,.5);mid=(a+b)/2;enqueue(a,mid,left);enqueue(mid,b,right);boxes+=1
    if queue and outcome=='numerically_bounded':outcome='budget_unresolved'
    return dict(passed=outcome=='numerically_bounded',outcome=outcome,minimum_sample_distance_m=math.sqrt(best),
        distance_lower_bound_m=math.sqrt(safe) if outcome=='numerically_bounded' else None,witness_piece=piece,witness_u=witness,boxes=boxes,
        overlap_intervals=[dict(piece=piece,start=start+a*T,end=start+b*T) for a,b in intervals],
        evidence='floating-point local Bernstein and shared cubic pendulum table bounds; not a formal certificate')
