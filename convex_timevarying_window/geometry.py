"""TOGT-native convex mappings in periodically moving 3-D planes."""
from dataclasses import dataclass
import numpy as np
from nonconvex_timevarying_window.sc_dynatogt.environment import MotionProfile, rotation_and_derivative

@dataclass(frozen=True)
class ConvexAperture:
    kind: str
    radius: float | None = None
    margin: float = 0.0
    physical_vertices: np.ndarray | None = None
    safe_vertices: np.ndarray | None = None
    def __post_init__(self):
        if self.kind=="circle":
            if self.radius is None or self.radius<=self.margin/2: raise ValueError("invalid circle")
            object.__setattr__(self,"physical_vertices",None); object.__setattr__(self,"safe_vertices",None)
        elif self.kind=="polygon":
            p=np.asarray(self.physical_vertices,float); s=np.asarray(self.safe_vertices,float)
            if p.ndim!=2 or p.shape[1]!=2 or s.shape!=p.shape or len(p)<3: raise ValueError("invalid polygon")
            object.__setattr__(self,"physical_vertices",p); object.__setattr__(self,"safe_vertices",s)
            object.__setattr__(self,"radius",None)
        else: raise ValueError("invalid kind")
    @property
    def dimension(self): return 2 if self.kind=="circle" else len(self.safe_vertices)
    def initial_d(self):
        return np.zeros(2) if self.kind=="circle" else np.full(self.dimension,np.sqrt(1/self.dimension))
    def point_and_jacobian(self,d):
        d=np.asarray(d,float)
        if d.shape!=(self.dimension,): raise ValueError("wrong D dimension")
        if self.kind=="circle":
            r=float(self.radius)-self.margin/2; den=1+d@d
            return 2*r*d/den,2*r*(np.eye(2)/den-2*np.outer(d,d)/den**2)
        n=np.linalg.norm(d)
        if n<=1e-12: raise ValueError("TOGT polygon map singular at zero")
        u=d/n; delta=(self.safe_vertices[1:]-self.safe_vertices[0]).T
        return self.safe_vertices[0]+delta@(u[:-1]**2), delta@np.diag(2*u[:-1])@((np.eye(len(u))-np.outer(u,u))/n)[:-1]
    def contains(self,q,tolerance=1e-9,safe=False):
        q=np.asarray(q,float)
        if self.kind=="circle": return bool(np.linalg.norm(q)<=float(self.radius)-(self.margin/2 if safe else 0)+tolerance)
        v=self.safe_vertices if safe else self.physical_vertices
        edge=np.roll(v,-1,axis=0)-v; rel=q-v
        z=edge[:,0]*rel[:,1]-edge[:,1]*rel[:,0]
        return bool(np.all(z>=-tolerance) or np.all(z<=tolerance))
    def boundary_distance(self,q):
        q=np.asarray(q,float)
        if self.kind=="circle": return np.abs(np.linalg.norm(q,axis=-1)-float(self.radius))
        v=self.physical_vertices; e=np.roll(v,-1,axis=0)-v; rel=q[...,None,:]-v
        a=np.sum(rel*e,axis=-1)/np.sum(e*e,axis=1); c=v+np.clip(a,0,1)[..., :,None]*e
        return np.min(np.linalg.norm(q[...,None,:]-c,axis=-1),axis=-1)
    def boundary_points(self,circle_samples=128,safe=False):
        if self.kind=="circle":
            a=np.linspace(0,2*np.pi,circle_samples,endpoint=False); r=float(self.radius)-(self.margin/2 if safe else 0)
            return r*np.column_stack((np.cos(a),np.sin(a)))
        return (self.safe_vertices if safe else self.physical_vertices).copy()

@dataclass
class PeriodicConvexWindow:
    name:str; aperture:ConvexAperture; center0:np.ndarray; angles0_rpy:np.ndarray; motion:MotionProfile
    def __post_init__(self): self.center0=np.asarray(self.center0,float); self.angles0_rpy=np.asarray(self.angles0_rpy,float)
    def state_at(self,t):
        x,dx=self.motion.translation(float(t)); a,da=self.motion.rotation(float(t)); R,dR=rotation_and_derivative(self.angles0_rpy+a,da)
        return self.center0+x,R,dx,dR
    def point_and_jacobians(self,d,t):
        q,J=self.aperture.point_and_jacobian(d); c,R,dc,dR=self.state_at(t)
        return c+R[:,:2]@q,q,R[:,:2]@J,dc+dR[:,:2]@q
    def to_point(self,d,t): return self.point_and_jacobians(d,t)[0]
    def world_to_local(self,p,t):
        c,R,*_=self.state_at(t); q=R.T@(np.asarray(p)-c); return q[:2],float(q[2])
    def boundary_at(self,t,circle_samples=128,safe=False):
        c,R,*_=self.state_at(t); q=self.aperture.boundary_points(circle_samples,safe)
        return c+(R[:,:2]@q.T).T
__all__=["ConvexAperture","PeriodicConvexWindow"]
