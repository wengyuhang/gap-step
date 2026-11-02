"""Local oriented-box/slab clipping. Numerical sampling, NOT certification."""
from functools import lru_cache
import time
import casadi as ca
import numpy as np
from .model import GenericQuad,flat_map
from ..geometry import halfplanes
from convex_timevarying_window.dynamic_passage_minco.safety import overlap_intervals

SIGNS=np.array([[x,y,z] for x in (-1,1) for y in (-1,1) for z in (-1,1)])
EDGES=[(i,j) for i in range(8) for j in range(i+1,8) if np.count_nonzero(SIGNS[i]!=SIGNS[j])==1]


def symbolic_projected(aperture,q,rotation,half,offset,slab,shrink):
    """Conservative optimization surrogate; exact clipping still verifies.

    While the box overlaps the slab, its entire projection must fit inside
    the aperture. Outside the slab the positive normal separation relaxes
    this requirement. No feasible overlapping box can violate containment.
    Unlike max(opening,gap), a negative gap cannot hide the lateral gradient.
    """
    corners=q+rotation@ca.DM((SIGNS*half+offset).T)
    gap=ca.fmax(ca.mmin(corners[2,:])-slab,-ca.mmax(corners[2,:])-slab)
    relief=100*ca.fmax(gap,0.)
    if aperture.kind=='circle':
        return [aperture.radius-shrink-ca.sqrt(ca.sumsqr(corners[:2,i])+1e-14)+relief for i in range(8)]
    n,b=halfplanes(aperture)
    return ca.vertsplit(ca.vec(ca.repmat(ca.DM(b-shrink),1,8)-ca.DM(n)@corners[:2,:]+relief))


@lru_cache(maxsize=8)
def orientation_map(density):
    v,a=ca.SX.sym('v',3),ca.SX.sym('a',3)
    R=flat_map(GenericQuad(air_density=density))(v,a,ca.SX.zeros(3),ca.SX.zeros(3))[0]
    return ca.Function('box_orientation',[v,a],[R])


def symbolic(aperture,q,rotation,half,offset,slab,shrink):
    corners=q+rotation@ca.DM((SIGNS*half+offset).T)
    def clearance(uv):
        if aperture.kind=='circle':return aperture.radius-shrink-ca.sqrt(ca.sumsqr(uv)+1e-14)
        n,b=halfplanes(aperture)
        return ca.mmin(ca.DM(b-shrink)-ca.DM(n)@uv)
    result=[];activity=[]
    for i in range(8):
        z=corners[2,i]
        active=ca.fabs(z)<=slab;activity.append(active)
        result.append(ca.if_else(active,clearance(corners[:2,i]),1e3))
    for i,j in EDGES:
        a,b=corners[:,i],corners[:,j];den=b[2]-a[2]
        denominator=ca.if_else(ca.fabs(den)>1e-9,den,1.)
        for plane in (-slab,slab):
            u=(plane-a[2])/denominator
            active=ca.logic_and(ca.fabs(den)>1e-9,ca.logic_and(u>=0,u<=1))
            activity.append(active)
            point=a+ca.fmin(1.,ca.fmax(0.,u))*(b-a)
            result.append(ca.if_else(active,clearance(point[:2]),1e3))
    any_active=ca.mmax(ca.vertcat(*activity))
    opening=ca.if_else(any_active,ca.mmin(ca.vertcat(*result)),-1e3)
    normal_gap=ca.fmax(ca.mmin(corners[2,:])-slab,-ca.mmax(corners[2,:])-slab)
    # During overlap optimize the actual opening clearance. max(opening,gap)
    # has the same feasible set but can hide its lateral gradient behind a
    # negative normal gap, wrongly pushing only for a faster normal crossing.
    # This activation is piecewise/discontinuous at first slab contact; it is
    # an optimization residual, not a smooth signed-distance certificate.
    return [ca.if_else(any_active,opening,normal_gap)]


def numeric_clearance(aperture,corners,slab,shrink):
    """Minimum slice-vertex opening clearance per time, vectorized."""
    ntime=len(corners);points=[];active=[]
    for i in range(8):
        points.append(corners[:,i,:2]);active.append(abs(corners[:,i,2])<=slab)
    for i,j in EDGES:
        a,b=corners[:,i],corners[:,j];den=b[:,2]-a[:,2]
        for plane in (-slab,slab):
            u=(plane-a[:,2])/np.where(abs(den)>1e-9,den,1.)
            active.append((abs(den)>1e-9)&(u>=0)&(u<=1))
            points.append((a+np.clip(u,0,1)[:,None]*(b-a))[:,:2])
    points=np.stack(points,axis=1);active=np.stack(active,axis=1)
    if aperture.kind=='circle':g=aperture.radius-shrink-np.linalg.norm(points,axis=2)
    else:
        normals,bounds=halfplanes(aperture)
        g=np.min(bounds-shrink-points@normals.T,axis=2)
    opening=np.min(np.where(active,g,np.inf),axis=1)
    opening=np.where(np.any(active,axis=1),opening,-1e3)
    normal_gap=np.maximum(np.min(corners[:,:,2],axis=1)-slab,-np.max(corners[:,:,2],axis=1)-slab)
    return np.where(np.any(active,axis=1),opening,normal_gap)


def passage(plan,window,radius,piece,deadline=np.inf):
    Rw=window.state_at(0.)[1];T=plan.durations[piece]
    begin=plan.made_at+sum(plan.durations[:piece])
    coeff=plan.coefficients[piece]@Rw;coeff=coeff.copy();coeff[0]-=window.center0@Rw
    half=np.array(plan.native_box_half)+plan.native_box_margin
    offset=np.array(plan.native_box_offset);slab=plan.native_frame_radius
    sphere=np.linalg.norm(half+abs(offset))+slab
    cuts=overlap_intervals(coeff[:,2],sphere)
    root=np.polynomial.polynomial.polyroots(coeff[:,2])
    for r in root:
        if abs(r.imag)<1e-7 and 1e-6<r.real<1-1e-6:
            t=begin+r.real*T;q,_=window.world_to_local(plan.sample(t),t)
            if window.aperture.contains(q):
                return dict(passed=False,outcome='repeat_aperture_crossing',minimum_sample_distance_m=0.,witness_u=None,boxes=0,overlap_intervals=[])
    samples=[]
    for a,b in cuts:samples.extend(np.linspace(a,b,max(2,int(np.ceil((b-a)*T/.0005))+1)))
    if not samples:return dict(passed=True,outcome='no_box_slab_overlap',minimum_sample_distance_m=np.inf,witness_u=None,boxes=0,overlap_intervals=[])
    u=np.unique(samples);tt=begin+u*T
    derivatives=[plan.sample_many(tt,d) for d in (0,1,2)]
    ro=orientation_map(plan.native_air_density)
    rotations=np.asarray(ro.map(len(tt))(derivatives[1].T,derivatives[2].T)).reshape(3,len(tt),3).transpose(1,0,2)
    vertices=SIGNS*half+offset
    world=derivatives[0][:,None,:]+np.einsum('tij,kj->tki',rotations,vertices)
    motion=window.motion;centers=window.center0+np.sin(2*np.pi*tt/motion.translation_period+motion.phase)[:,None]*motion.translation_amplitude
    local=(world-centers[:,None,:])@Rw
    from .native_frame_geometry import numeric
    q=(derivatives[0]-centers)@Rw
    local_R=np.einsum('ij,tjk->tik',Rw.T,rotations)
    clearance=numeric(window.aperture,q,local_R,half,offset,plan.native_frame_radius,
        plan.native_frame_radius-plan.native_aperture_shrink)
    index=int(np.argmin(clearance));minimum=float(clearance[index]);expired=time.perf_counter()>=deadline
    return dict(passed=bool(minimum>=-1e-5 and not expired),outcome='local_box_sample_pass' if minimum>=-1e-5 else 'box_slab_violation',
        minimum_sample_distance_m=minimum,witness_u=float(u[index]),boxes=len(tt),
        overlap_intervals=[dict(piece=piece,start=float(begin+a*T),end=float(begin+b*T)) for a,b in cuts],
        sample_step_s=.0005,formal_certificate=False)
