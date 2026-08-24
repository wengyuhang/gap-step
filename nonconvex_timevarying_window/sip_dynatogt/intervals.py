"""Outward-rounded Arb interval extensions for all certified constraints."""

from __future__ import annotations
from dataclasses import dataclass
from math import factorial
from typing import Any, Iterable
import numpy as np

try:
    from flint import arb, ctx, fmpq
except ImportError:  # fail closed in certify()
    arb = ctx = fmpq = None  # type: ignore[assignment]

from nonconvex_timevarying_window.sc_dynatogt.boundary import BSpline, Bezier, BoundarySegment, CircularArc, Line
from .model import PolynomialTrajectory, SIPConfig, SIPWindow


class IntervalDependencyError(RuntimeError): pass
class FlatnessIndeterminate(RuntimeError):
    def __init__(self, kind: str, residual: Any): super().__init__(kind); self.kind, self.residual = kind, residual


def require_flint() -> None:
    if arb is None: raise IntervalDependencyError("python-flint>=0.9,<0.10 is required for certification")


def exact_rational(value: float):
    require_flint(); value=float(value)
    if not np.isfinite(value): raise ValueError("interval input must be finite")
    n,d=value.as_integer_ratio(); return fmpq(n,d)


def exact_ball(value: float): require_flint(); return arb(exact_rational(value))
def interval_ball(lower: float, upper: float):
    lo,hi=exact_rational(lower),exact_rational(upper)
    if lo>hi: raise ValueError("reversed interval")
    return arb((lo+hi)/2,(hi-lo)/2)
def _zero(): return exact_ball(0.0)


def iv_square(x):
    if x>=0 or x<=0: return x*x
    magnitude=(-x.lower()).max(x.upper()); return _zero().union(magnitude*magnitude)
def iv_dot(a: Iterable[Any],b: Iterable[Any]):
    out=_zero()
    for x,y in zip(a,b): out += x*y
    return out
def iv_norm2(v: Iterable[Any]):
    out=_zero()
    for x in v: out += iv_square(x)
    return out
def iv_cross(a:list[Any],b:list[Any])->list[Any]: return [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]]
def iv_add(a:list[Any],b:list[Any])->list[Any]: return [x+y for x,y in zip(a,b)]
def iv_scale(s:Any,v:list[Any])->list[Any]: return [s*x for x in v]
def iv_transpose(m:list[list[Any]])->list[list[Any]]: return [list(c) for c in zip(*m)]
def iv_matvec(m:list[list[Any]],v:list[Any])->list[Any]: return [iv_dot(row,v) for row in m]
def iv_matmul(a:list[list[Any]],b:list[list[Any]])->list[list[Any]]:
    cols=list(zip(*b)); return [[iv_dot(row,col) for col in cols] for row in a]


def _rotation(roll:Any,pitch:Any,yaw:Any)->list[list[Any]]:
    cr,sr,cp,sp,cy,sy=roll.cos(),roll.sin(),pitch.cos(),pitch.sin(),yaw.cos(),yaw.sin(); z,o=_zero(),exact_ball(1)
    rz=[[cy,-sy,z],[sy,cy,z],[z,z,o]]; ry=[[cp,z,sp],[z,o,z],[-sp,z,cp]]; rx=[[o,z,z],[z,cr,-sr],[z,sr,cr]]
    return iv_matmul(iv_matmul(rz,ry),rx)


def window_state_interval(window:SIPWindow,time:Any)->tuple[list[Any],list[list[Any]],Any]:
    m=window.motion; two_pi=exact_ball(2)*arb.pi(); translation=[]; delta=[]
    for i,phase in enumerate((0.0,0.7,1.4)):
        angle=two_pi/exact_ball(m.translation_period)*time+exact_ball(m.phase)+exact_ball(phase)
        translation.append(exact_ball(float(m.translation_amplitude[i]))*angle.sin() if m.translation_enabled else _zero())
    for i,phase in enumerate((0.0,0.9,1.8)):
        angle=two_pi/exact_ball(m.rotation_period)*time+exact_ball(m.phase)+exact_ball(phase)
        delta.append(exact_ball(float(m.rotation_amplitude[i]))*angle.sin() if m.rotation_enabled else _zero())
    scale=exact_ball(1)+exact_ball(m.scale_amplitude)*(two_pi/exact_ball(m.scale_period)*time+exact_ball(m.phase)).sin() if m.scale_enabled else exact_ball(1)
    center=[exact_ball(float(window.center0[i]))+translation[i] for i in range(3)]
    angles=[exact_ball(float(window.angles0[i]))+delta[i] for i in range(3)]
    return center,_rotation(*angles),scale


def boundary_parameter_spans(segment:BoundarySegment)->tuple[tuple[float,float],...]:
    if not isinstance(segment,BSpline): return ((0.0,1.0),)
    knots=np.asarray(segment.knots); p=segment.degree; lo,hi=float(knots[p]),float(knots[-p-1]); values=[0.0]
    values += [float((k-lo)/(hi-lo)) for k in np.unique(knots[p+1:-p-1]) if lo<k<hi]; values.append(1.0)
    return tuple(zip(values[:-1],values[1:]))


def _bspline(segment:BSpline,u:Any)->list[Any]:
    points=np.asarray(segment.control_points); knots=np.asarray(segment.knots); p=segment.degree
    lo,hi=float(knots[p]),float(knots[-p-1])
    t=exact_ball(lo)+u*(exact_ball(hi)-exact_ball(lo))

    # A normalized binary64 knot split need not coincide exactly with the
    # corresponding real knot.  Evaluate every knot span touched by ``t`` and
    # union the results, so no tiny cross-knot sliver can escape the enclosure.
    pieces:list[list[Any]]=[]
    for span in range(p,len(points)):
        left,right=float(knots[span]),float(knots[span+1])
        if left==right: continue
        try: local_t=t.intersection(interval_ball(left,right))
        except ValueError: continue
        work=[[exact_ball(float(x)) for x in points[span-p+j]] for j in range(p+1)]
        for r in range(1,p+1):
            for j in range(p,r-1,-1):
                i=span-p+j; ka,kb=float(knots[i]),float(knots[i+p-r+1])
                den=exact_ball(kb)-exact_ball(ka)
                alpha=_zero() if ka==kb else (local_t-exact_ball(ka))/den
                work[j]=[(exact_ball(1)-alpha)*work[j-1][axis]+alpha*work[j][axis] for axis in range(2)]
        pieces.append(work[p])
    if not pieces: raise ValueError("B-spline interval misses its parameter domain")
    result=[]
    for axis in range(2):
        hull=pieces[0][axis]
        for piece in pieces[1:]: hull=hull.union(piece[axis])
        result.append(hull)
    return result


def boundary_interval(segment:BoundarySegment,u:Any)->list[Any]:
    if isinstance(segment,Line): return [(exact_ball(1)-u)*exact_ball(float(segment.start[a]))+u*exact_ball(float(segment.end[a])) for a in range(2)]
    if isinstance(segment,CircularArc):
        angle=exact_ball(float(segment.start_angle))+u*exact_ball(float(segment._sweep)); return [exact_ball(float(segment.center[0]))+exact_ball(segment.radius)*angle.cos(),exact_ball(float(segment.center[1]))+exact_ball(segment.radius)*angle.sin()]
    if isinstance(segment,Bezier):
        work=[[exact_ball(float(x)) for x in p] for p in np.asarray(segment.control_points)]
        for count in range(len(work)-1,0,-1):
            for i in range(count): work[i]=[(exact_ball(1)-u)*work[i][a]+u*work[i+1][a] for a in range(2)]
        return work[0]
    if isinstance(segment,BSpline): return _bspline(segment,u)
    raise TypeError(type(segment).__name__)


def polynomial_derivative_interval(traj:PolynomialTrajectory,segment:int,tau:Any,derivative:int)->list[Any]:
    t=exact_ball(float(traj.durations[segment]))*tau; c=traj.coefficients[segment]; out=[]
    for axis in range(3):
        value=_zero()
        for power in range(7,derivative-1,-1):
            # Keep the integer differentiation multiplier inside Arb.  A
            # binary64 multiplication here could round inward before the
            # coefficient reaches the interval calculation.
            multiplier=factorial(power)//factorial(power-derivative)
            value=value*t+exact_ball(float(multiplier))*exact_ball(float(c[power,axis]))
        out.append(value)
    return out


def _normalize(vector:list[Any],first:list[Any],second:list[Any],floor:float,kind:str):
    norm2=iv_norm2(vector); floor2=iv_square(exact_ball(floor))
    if not norm2>floor2: raise FlatnessIndeterminate(kind,floor2-norm2)
    norm=norm2.sqrt(); dot=iv_dot(vector,first); ndot=dot/norm; nddot=(iv_dot(first,first)+iv_dot(vector,second))/norm-dot*dot/(norm*norm*norm)
    unit=[x/norm for x in vector]; udot=[first[i]/norm-vector[i]*ndot/(norm*norm) for i in range(3)]
    uddot=[second[i]/norm-exact_ball(2)*first[i]*ndot/(norm*norm)-vector[i]*nddot/(norm*norm)+exact_ball(2)*vector[i]*ndot*ndot/(norm*norm*norm) for i in range(3)]
    return unit,udot,uddot,norm2


@dataclass(frozen=True)
class IntervalFlatness:
    position:list[Any]; velocity:list[Any]; rotation:list[list[Any]]; body_rate:list[Any]; collective_thrust:Any; rotor_thrusts:list[Any]; specific_force_norm2:Any; heading_cross_norm2:Any


def flatness_interval(traj:PolynomialTrajectory,segment:int,tau:Any,config:SIPConfig)->IntervalFlatness:
    p=polynomial_derivative_interval(traj,segment,tau,0); v=polynomial_derivative_interval(traj,segment,tau,1); a=polynomial_derivative_interval(traj,segment,tau,2); j=polynomial_derivative_interval(traj,segment,tau,3); s=polynomial_derivative_interval(traj,segment,tau,4)
    force=list(a); force[2]+=exact_ball(config.quadrotor.gravity); bz,bzd,bzdd,force2=_normalize(force,j,s,config.flatness_floor,"specific_force_singularity")
    heading=[_zero(),exact_ball(1),_zero()]; raw=iv_cross(heading,bz); rawd=iv_cross(heading,bzd); rawdd=iv_cross(heading,bzdd); bx,bxd,bxdd,cross2=_normalize(raw,rawd,rawdd,config.flatness_floor,"heading_cross_singularity")
    by=iv_cross(bz,bx); byd=iv_add(iv_cross(bzd,bx),iv_cross(bz,bxd)); bydd=iv_add(iv_add(iv_cross(bzdd,bx),iv_scale(exact_ball(2),iv_cross(bzd,bxd))),iv_cross(bz,bxdd))
    R=[[bx[r],by[r],bz[r]] for r in range(3)]; Rd=[[bxd[r],byd[r],bzd[r]] for r in range(3)]; Rdd=[[bxdd[r],bydd[r],bzdd[r]] for r in range(3)]
    wh=iv_matmul(iv_transpose(R),Rd); whd=iv_matmul(iv_transpose(Rd),Rd); term=iv_matmul(iv_transpose(R),Rdd); whd=[[whd[r][c]+term[r][c] for c in range(3)] for r in range(3)]; half=exact_ball(.5)
    omega=[half*(wh[2][1]-wh[1][2]),half*(wh[0][2]-wh[2][0]),half*(wh[1][0]-wh[0][1])]; omegad=[half*(whd[2][1]-whd[1][2]),half*(whd[0][2]-whd[2][0]),half*(whd[1][0]-whd[0][1])]
    inertia=[[exact_ball(float(x)) for x in row] for row in np.asarray(config.quadrotor.inertia)]; momentum=iv_matvec(inertia,omega); torque=iv_add(iv_matvec(inertia,omegad),iv_cross(omega,momentum)); collective=exact_ball(config.quadrotor.mass)*force2.sqrt(); wrench=[collective,*torque]
    allocation=[[exact_ball(float(x)) for x in row] for row in np.asarray(config.quadrotor.allocation_matrix)]; rotors=iv_matvec(allocation,wrench)
    return IntervalFlatness(p,v,R,omega,collective,rotors,force2,cross2)


def global_time_interval(traj:PolynomialTrajectory,segment:int,tau:Any):
    prefix=_zero()
    for d in traj.durations[:segment]: prefix += exact_ball(float(d))
    return prefix+exact_ball(float(traj.durations[segment]))*tau


def safety_residual_interval(window:SIPWindow,boundary:BoundarySegment,traj:PolynomialTrajectory,segment:int,tau:Any,u:Any,config:SIPConfig):
    flat=flatness_interval(traj,segment,tau,config); state=window_state_interval(window,global_time_interval(traj,segment,tau)); q=boundary_interval(boundary,u)
    return safety_residual_from_interval_components(flat,state,q,config)


def safety_residual_from_interval_components(flat:IntervalFlatness,window_state:tuple[list[Any],list[list[Any]],Any],q:list[Any],config:SIPConfig):
    center,Rw,scale=window_state; y=iv_add(center,iv_matvec(Rw,[scale*q[0],scale*q[1],_zero()])); rel=[y[i]-flat.position[i] for i in range(3)]; z=iv_matvec(iv_transpose(flat.rotation),rel); rho2=_zero()
    for value,extent in zip(z,config.body.half_extents):
        excess=(abs(value)-exact_ball(extent)).nonnegative_part(); rho2 += iv_square(excess)
    return iv_square(exact_ball(config.clearance))-rho2


def dynamic_residual_intervals(flat:IntervalFlatness,config:SIPConfig)->list[tuple[str,Any]]:
    l=config.dynamic_limits
    velocity_limit2=iv_square(exact_ball(l.max_velocity))
    floor2=iv_square(exact_ball(config.flatness_floor))
    body_rate_xy_limit2=iv_square(exact_ball(l.max_body_rate_xy))
    body_rate_z_limit2=iv_square(exact_ball(l.max_body_rate_z))
    result=[("velocity",iv_norm2(flat.velocity)-velocity_limit2),("specific_force_singularity",floor2-flat.specific_force_norm2),("heading_cross_singularity",floor2-flat.heading_cross_norm2),("collective_lower",exact_ball(l.min_collective_thrust)-flat.collective_thrust),("body_rate_xy",iv_square(flat.body_rate[0])+iv_square(flat.body_rate[1])-body_rate_xy_limit2),("body_rate_z",iv_square(flat.body_rate[2])-body_rate_z_limit2)]
    if np.isfinite(l.max_collective_thrust): result.append(("collective_upper",flat.collective_thrust-exact_ball(l.max_collective_thrust)))
    for i,t in enumerate(flat.rotor_thrusts): result.extend(((f"rotor_{i}_lower",exact_ball(l.min_rotor_thrust)-t),(f"rotor_{i}_upper",t-exact_ball(l.max_rotor_thrust))))
    return result


__all__=["FlatnessIndeterminate","IntervalDependencyError","boundary_interval","boundary_parameter_spans","ctx","dynamic_residual_intervals","flatness_interval","global_time_interval","interval_ball","require_flint","safety_residual_from_interval_components","safety_residual_interval","window_state_interval"]
