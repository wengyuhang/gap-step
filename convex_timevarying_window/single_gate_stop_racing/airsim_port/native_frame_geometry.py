"""Native moving Cylinder frames enclosed by OBBs, with continuous SAT gaps.

No active-slab or aperture-clipping branch in the optimization residual.
Nonnegative SAT separation suffices to avoid each conservative rod enclosure.
Dense local-time verification is numerical evidence, not certification.
"""
import casadi as ca
import numpy as np
from .scene import frame_vertices


def rods(aperture,offset,radius):
    p=frame_vertices(aperture,offset)
    for a,b in zip(p,np.roll(p,-1,axis=0)):
        z=np.r_[b-a,0.];length=np.linalg.norm(z);z/=length
        x=np.array([0.,0.,1.]);y=np.cross(z,x)
        yield np.r_[(a+b)/2,0.],np.column_stack((x,y,z)),np.array([radius,radius,length/2])


def symbolic(aperture,q,R,half,offset,radius,outward):
    values=[]
    for center,rodR,rodhalf in rods(aperture,outward,radius):
        delta=ca.DM(center)-q-R@ca.DM(offset)
        axes=[R[:,i] for i in range(3)]+[ca.DM(rodR[:,i]) for i in range(3)]
        axes += [ca.cross(R[:,i],ca.DM(rodR[:,j])) for i in range(3) for j in range(3)]
        gaps=[]
        for a in axes:
            eps=1e-5
            support=ca.dot(ca.DM(half),ca.sqrt((R.T@a)**2+eps**2))+ca.dot(ca.DM(rodhalf),ca.sqrt((ca.DM(rodR.T)@a)**2+eps**2))
            # Do not normalize cross axes: a zero axis then has a strictly
            # negative smoothed gap instead of producing a singularity or a
            # false zero-separation certificate. Positive gaps still imply
            # an actual, nonzero separating axis.
            # Parallel-edge cross products are not separating axes. Leaving
            # their near-zero gaps in the smooth maximum creates a flat
            # infeasible plateau and hides the useful face-axis gradient.
            # Subtracting a nonnegative penalty preserves conservatism.
            degeneracy_penalty=.1*ca.exp(-ca.dot(a,a)/.01)
            gaps.append(ca.sqrt(ca.dot(delta,a)**2+eps**2)-eps-support-degeneracy_penalty)
        g=ca.vertcat(*gaps);maximum=ca.mmax(g);tau=.002
        # Smooth maximum UNDERestimate, so a nonnegative residual implies
        # an actual separating axis. Do not use ordinary log-sum-exp, which
        # overestimates separation and could accept an intersection.
        values.append(maximum+tau*ca.log(ca.sum1(ca.exp((g-maximum)/tau)))-tau*np.log(len(gaps)))
    return values


def numeric(aperture,q,R,half,offset,radius,outward):
    result=np.full(len(q),np.inf)
    for center,rodR,rodhalf in rods(aperture,outward,radius):
        delta=center-q-np.einsum('tij,j->ti',R,offset)
        axes=np.concatenate((R.transpose(0,2,1),np.broadcast_to(rodR.T,(len(q),3,3)),
            np.cross(R.transpose(0,2,1)[:,:,None,:],rodR.T[None,None,:,:]).reshape(len(q),9,3)),axis=1)
        norms=np.linalg.norm(axes,axis=2)
        body_support=np.einsum('tai,i->ta',abs(np.einsum('taj,tji->tai',axes,R)),half)
        rod_support=abs(axes@rodR)@rodhalf
        gaps=(abs(np.einsum('taj,tj->ta',axes,delta))-body_support-rod_support)/np.maximum(norms,1e-20)
        gaps=np.where(norms>1e-7,gaps,-1e3)
        result=np.minimum(result,gaps.max(axis=1))
    return result
