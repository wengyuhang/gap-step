"""Compiled implementation of the same finite-frame local enclosure check.

No dense full-flight scan, no changed radii: each subdivision encloses a
polynomial by its Bernstein hull and the known sinusoid by its exact range.
"""
import math
import numpy as np
from numba import njit


@njit(cache=True)
def _distance(q,vertices,circle,R):
    if circle:return (math.sqrt(q[0]*q[0]+q[1]*q[1])-R)**2+q[2]*q[2]
    best=np.inf
    for i in range(len(vertices)):
        a=vertices[i];b=vertices[(i+1)%len(vertices)]
        ex=b[0]-a[0];ey=b[1]-a[1];dx=q[0]-a[0];dy=q[1]-a[1]
        u=min(1.,max(0.,(dx*ex+dy*ey)/(ex*ex+ey*ey)))
        best=min(best,(dx-u*ex)**2+(dy-u*ey)**2)
    return best+q[2]*q[2]


@njit(cache=True)
def _value(u,coeff,start,T,amp,omega,phase,vertices,circle,R):
    q=np.zeros(3)
    for k in range(7,-1,-1):q=q*u+coeff[k]
    q-=amp*math.sin(omega*(start+u*T)+phase)
    return _distance(q,vertices,circle,R)


@njit(cache=True)
def check(controls,cuts,coeff,start,T,amp,omega,phase,vertices,circle,R,radius):
    # At most seven normal polynomial roots; 512 subdivisions like the Python
    # reference. Preallocation and native loops remove Python per-box overhead.
    stack_c=np.empty((1030,8,3));stack_a=np.empty(1030);stack_b=np.empty(1030)
    size=len(cuts);best=np.inf;witness=0.;boxes=0
    for i in range(size):stack_c[i]=controls[i];stack_a[i]=cuts[i,0];stack_b[i]=cuts[i,1]
    while size:
        size-=1;a=stack_a[size];b=stack_b[size];control=stack_c[size].copy()
        lo=np.empty(3);hi=np.empty(3)
        sa=omega*(start+a*T)+phase;sb=omega*(start+b*T)+phase
        sl=min(math.sin(sa),math.sin(sb));su=max(math.sin(sa),math.sin(sb))
        for k in range(math.ceil((sa-math.pi/2)/math.pi),math.floor((sb-math.pi/2)/math.pi)+1):
            if k%2:sl=-1.
            else:su=1.
        for axis in range(3):
            low=control[0,axis];high=low
            for k in range(1,8):low=min(low,control[k,axis]);high=max(high,control[k,axis])
            lo[axis]=low-max(amp[axis]*sl,amp[axis]*su)-1e-10
            hi[axis]=high-min(amp[axis]*sl,amp[axis]*su)+1e-10
        normal=max(lo[2],-hi[2],0.)
        if circle:
            near=0.;far=0.
            for axis in range(2):
                near+=max(lo[axis],-hi[axis],0.)**2
                far+=max(abs(lo[axis]),abs(hi[axis]))**2
            planar=max(math.sqrt(near)-R,R-math.sqrt(far),0.)**2
        else:
            planar=np.inf
            for i in range(len(vertices)):
                v=vertices[i];e=vertices[(i+1)%len(vertices)]-v;length=math.sqrt(e@e)
                tx=e[0]/length;ty=e[1]/length;nx=ty;ny=-tx
                cx=(lo[0]+hi[0])/2-v[0];cy=(lo[1]+hi[1])/2-v[1]
                hx=(hi[0]-lo[0])/2;hy=(hi[1]-lo[1])/2
                ac=tx*cx+ty*cy;ar=abs(tx)*hx+abs(ty)*hy
                nc=nx*cx+ny*cy;nr=abs(nx)*hx+abs(ny)*hy
                ag=max(-ac-ar,ac-ar-length,0.);ng=max(abs(nc)-nr,0.)
                planar=min(planar,ag*ag+ng*ng)
        for u in (a,(a+b)/2,b):
            value=_value(u,coeff,start,T,amp,omega,phase,vertices,circle,R)
            if value<best:best=value;witness=u
        if best<(radius-2e-4)**2:
            # Refine the violating basin, as in the isolated single-window
            # checker. A coarse midpoint cut otherwise needs repeated almost
            # identical outer solves before reaching the actual minimum.
            left_u=a;right_u=b;ratio=(math.sqrt(5.)-1.)/2
            u1=right_u-ratio*(right_u-left_u);u2=left_u+ratio*(right_u-left_u)
            f1=_value(u1,coeff,start,T,amp,omega,phase,vertices,circle,R)
            f2=_value(u2,coeff,start,T,amp,omega,phase,vertices,circle,R)
            for iteration in range(38):
                if f1<f2:
                    right_u=u2;u2=u1;f2=f1;u1=right_u-ratio*(right_u-left_u)
                    f1=_value(u1,coeff,start,T,amp,omega,phase,vertices,circle,R)
                else:
                    left_u=u1;u1=u2;f1=f2;u2=left_u+ratio*(right_u-left_u)
                    f2=_value(u2,coeff,start,T,amp,omega,phase,vertices,circle,R)
            if f1<best:best=f1;witness=u1
            if f2<best:best=f2;witness=u2
            return 1,best,witness,boxes
        if planar+normal*normal>=radius*radius:continue
        if (b-a)*T<1e-6 or boxes>=512:return 2,best,witness,boxes
        work=control.copy();left=np.empty((8,3));right=np.empty((8,3))
        left[0]=work[0];right[7]=work[7]
        for level in range(1,8):
            for k in range(8-level):work[k]=.5*(work[k]+work[k+1])
            left[level]=work[0];right[7-level]=work[7-level]
        mid=(a+b)/2
        stack_c[size]=right;stack_a[size]=mid;stack_b[size]=b
        stack_c[size+1]=left;stack_a[size+1]=a;stack_b[size+1]=mid
        size+=2;boxes+=1
    return 0,best,witness,boxes


def warmup():
    check(np.zeros((0,8,3)),np.zeros((0,2)),np.zeros((8,3)),0.,1.,np.zeros(3),
          1.,0.,np.zeros((4,2)),False,1.,.57)
