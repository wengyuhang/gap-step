"""A short racing prefix with an explicitly checked braking continuation.

The full racing candidate is only a search object. Only prefix+brake is sent.
Vectorised local checks are sampled numerical evidence, not a SIP certificate.
"""
import math
import time
import numpy as np
from convex_timevarying_window.receding_gate_racing.planner import Plan, coefficients, Planner as BasePlanner
from .planner import Planner as ContinuationPlanner


def sample_many(plan,times,order=0):
    times=np.asarray(times,dtype=float)
    if isinstance(plan,GuardedPlan):
        out=np.empty((len(times),3));before=times<=plan.switch_time
        out[before]=sample_many(plan.nominal,times[before],order)
        out[~before]=sample_many(plan.brake,times[~before],order)
        return out
    elapsed=np.maximum(0,times-plan.made_at);ends=np.cumsum(plan.durations)
    indices=np.minimum(np.searchsorted(ends,elapsed,side='right'),len(ends)-1)
    out=np.empty((len(times),3))
    for i,T in enumerate(plan.durations):
        mask=indices==i;s=np.clip((elapsed[mask]-(ends[i-1] if i else 0))/T,0,1)
        basis=np.column_stack([math.factorial(k)/math.factorial(k-order)*s**(k-order)/T**order
                               if k>=order else np.zeros_like(s) for k in range(6)])
        out[mask]=basis@plan.coefficients[i]
    return out


def frame_clearances(frames,positions,times,radius,thickness):
    """Batch inverse Euler transforms; same exact circle/polygon geometry."""
    times=np.asarray(times);clear=np.full(len(times),np.inf)
    for w in frames:
        m=w.motion
        tr=m.translation_amplitude if m.translation_enabled else np.zeros(3)
        ar=m.rotation_amplitude if m.rotation_enabled else np.zeros(3)
        center=w.center0+tr*np.sin(2*np.pi*times[:,None]/m.translation_period+m.phase+np.array([0,.7,1.4]))
        angles=w.angles0_rpy+ar*np.sin(2*np.pi*times[:,None]/m.rotation_period+m.phase+np.array([0,.9,1.8]))
        roll,pitch,yaw=angles.T;cr,sr=np.cos(roll),np.sin(roll);cp,sp=np.cos(pitch),np.sin(pitch);cy,sy=np.cos(yaw),np.sin(yaw)
        d=positions-center;x=cy*d[:,0]+sy*d[:,1];y=-sy*d[:,0]+cy*d[:,1]
        qx=cp*x-sp*d[:,2];z=sp*x+cp*d[:,2];qy=cr*y+sr*z;qz=-sr*y+cr*z
        distance=w.aperture.boundary_distance(np.column_stack([qx,qy]))
        clear=np.minimum(clear,np.hypot(distance,qz)-radius-thickness)
    return clear


class GuardedPlan:
    def __init__(self,nominal,brake,clearance):
        self.nominal=nominal;self.brake=brake;self.clearance=clearance
        self.switch_time=brake.made_at
    def __getattr__(self,name):return getattr(self.nominal,name)
    @property
    def end_time(self):return self.brake.end_time
    def sample(self,t,order=0):
        return self.nominal.sample(t,order) if t<=self.switch_time else self.brake.sample(t,order)


class Planner(ContinuationPlanner):
    # Braking is generated analytically, not another NLP terminal segment.
    horizon_ids=BasePlanner.horizon_ids
    terminal_position=BasePlanner.terminal_position
    segment_frames=BasePlanner.segment_frames
    extra_collocation=4

    def collocation_parameters(self,first,ids):
        cache=getattr(self,'collocation_cache',{})
        return cache.get(first,np.tile(np.linspace(.15,.85,self.extra_collocation),len(ids)))

    def refine_collocation(self,plan):
        # A fixed-size constraint exchange, not ever-growing dense NLP grids.
        # Locate the lowest-clearance parts using cheap vectorised queries and
        # put only four witness locations per segment into the next solve.
        if not hasattr(self,'collocation_cache'):self.collocation_cache={}
        slots=[];start=plan.made_at
        for k,(wi,T) in enumerate(zip(self.horizon_ids(plan.first_gate),plan.durations)):
            offsets=np.linspace(0,min(1.2,T),61)
            local=np.unique(np.r_[np.linspace(0,T,25),offsets,T-offsets])
            times=start+local;p=sample_many(plan,times)
            frames=[self.frames[i] for i in self.segment_frames(plan.first_gate,wi,k)]
            clearance=frame_clearances(frames,p,times,self.cfg.radius,self.cfg.frame_thickness)
            v,a,j=[sample_many(plan,times,d) for d in (1,2,3)];u=a+self.cfg.response_time*j
            score=clearance/self.cfg.tracking_margin-1
            for values,limit in ((v,self.cfg.speed),(a,self.cfg.acceleration),(j,self.cfg.jerk),(u,self.cfg.acceleration)):
                score=np.minimum(score,1-np.sum(values*values,axis=1)/limit**2)
            selected=[]
            for index in np.argsort(score):
                u=float(local[index]/T)
                if all(abs(u-v)>.025 for v in selected):selected.append(u)
                if len(selected)==self.extra_collocation:break
            while len(selected)<self.extra_collocation:selected.append(.5)
            slots.extend(selected);start+=T
        self.collocation_cache[plan.first_gate]=np.array(slots)

    def seed_incumbent(self):
        plan=getattr(self,'admitted',None)
        if isinstance(plan,GuardedPlan) and self._solve_time>=plan.switch_time:return None
        return plan

    def holdable(self,point):
        if self.cfg.workspace_lower is not None:
            if np.any(point<np.asarray(self.cfg.workspace_lower)+self.cfg.tracking_margin):return False
            if np.any(point>np.asarray(self.cfg.workspace_upper)-self.cfg.tracking_margin):return False
        return all(np.linalg.norm(point-center)>=radius for center,radius in self.swept_balls())

    def validate(self,plan):
        cfg=self.cfg
        endpoint=plan.sample(plan.end_time)
        if not self.holdable(endpoint):return -np.inf
        count=max(2,int(np.ceil((plan.end_time-plan.made_at)/.01))+1)
        times=np.linspace(plan.made_at,plan.end_time,count)
        p,v,a,j=(sample_many(plan,times,k) for k in range(4))
        demanded=a+cfg.response_time*j;thrust=demanded+np.array([0,0,9.8066])
        tolerance=1.006
        if any(np.max(np.linalg.norm(values,axis=1))>limit*tolerance for values,limit in
               ((v,cfg.speed),(a,cfg.acceleration),(j,cfg.jerk),(demanded,cfg.acceleration))):return -np.inf
        if np.max(np.linalg.norm(thrust,axis=1))>(4*8.54858/2.0643076923*.90)*tolerance:return -np.inf
        if np.min(thrust[:,2])<=0:return -np.inf
        if np.max(np.linalg.norm(thrust[:,:2],axis=1)/thrust[:,2])>math.tan(math.radians(42))*tolerance:return -np.inf
        clearance=float(np.min(frame_clearances(self.frames,p,times,cfg.radius,cfg.frame_thickness)))
        if cfg.workspace_lower is not None:
            clearance=min(clearance,float(np.min(p-np.asarray(cfg.workspace_lower))),float(np.min(np.asarray(cfg.workspace_upper)-p)))
        return clearance

    def solve(self,state,now,first):
        started=time.perf_counter();self._solve_time=now
        nominal=super().solve(state,now,first)
        if nominal.violation>=.003:return nominal
        self.refine_collocation(nominal)
        # Each template is generated from PVA, not a stored course trajectory.
        for follow in (.30,.60,.90,1.20):
            start=now+follow
            if start>=nominal.end_time:continue
            p,v,a=(nominal.sample(start,d) for d in range(3))
            base=max(.6,1.8*np.linalg.norm(v)/self.cfg.acceleration,
                     math.sqrt(8*np.linalg.norm(v)/self.cfg.jerk),2*np.linalg.norm(a)/self.cfg.jerk)
            for scale in (1.,1.25,1.6):
                if time.perf_counter()-started>=.090:return nominal
                T=base*scale;end=p+.5*T*v+(T*T/12)*a
                if not self.holdable(end):continue
                C=coefficients(p,v,a,end,np.zeros(3),np.zeros(3),T)
                brake=Plan(start,first,np.array([T]),C[None],np.zeros(9),0.,0.,'analytic_brake')
                candidate=GuardedPlan(nominal,brake,-np.inf)
                clearance=self.validate(candidate)
                if clearance>=self.cfg.tracking_margin:
                    candidate.clearance=clearance
                    candidate.solve_ms=1000*(time.perf_counter()-started)
                    return candidate
        return nominal

    def prefix_clearance(self,plan,now,duration=.30):
        return plan.clearance if isinstance(plan,GuardedPlan) else -np.inf


class CheckedPlanner(Planner):
    """Optimise the local stopping continuation jointly, then check it.

Two windows plus a rest endpoint: bounded local horizon, never a whole lap.
This avoids filtering for a straight analytic brake that the NLP cannot shape.
"""
    horizon_ids=ContinuationPlanner.horizon_ids
    terminal_position=ContinuationPlanner.terminal_position
    segment_frames=ContinuationPlanner.segment_frames
    seed_incumbent=ContinuationPlanner.seed_incumbent

    def solve(self,state,now,first):
        started=time.perf_counter();self._solve_time=now
        nominal=ContinuationPlanner.solve(self,state,now,first)
        if nominal.violation>=.003:return nominal
        self.refine_collocation(nominal)
        if time.perf_counter()-started>.090:return nominal
        clearance=self.validate(nominal)
        if clearance<self.cfg.tracking_margin:return nominal
        begin=nominal.end_time-nominal.durations[-1]
        brake=Plan(begin,first,nominal.durations[-1:],nominal.coefficients[-1:],np.zeros(9),0.,0.,'optimised_rest_continuation')
        checked=GuardedPlan(nominal,brake,clearance)
        checked.solve_ms=1000*(time.perf_counter()-started)
        return checked
