"""CTBR GPU controller with phase-constrained passage and feedback continuation.

Experimental: local empirical-domain checks are NOT a robustness certificate.
"""
import time
import numpy as np
from .controller import Controller,Selection
from .gpu import GPUController,GPURollout
from .ctbr import CTBRRollout
from .feedback_maneuver import FeedbackManeuver
from .rollout import MASS,hover_state
from .terminal_cost import velocity_correction_distance


class CTBRGPU(GPURollout):
    ctbr=True


class CommitmentController(Controller):
    sample_device=GPUController.sample_device

    def scores(self,batch,u,previous,prepared,reference,now,avoid):
        cost,recovered=GPUController.scores(self,batch,u,previous,prepared,reference,now,avoid)
        if avoid and self.config.terminal_velocity_cost:
            cost+=velocity_correction_distance(batch.states[:,-1],batch.indices[:,-1],prepared[0],
                self.gpu.device_constants[2],self.rollout.track.goal,self.config.acceleration,self.config.speed,self.cp,
                self.config.radius+self.config.error_margin+self.config.frame_thickness+.3)
        return cost,recovered

    def __init__(self,track,config,seed=0):
        super().__init__(track,config,seed)
        self.rollout=CTBRRollout(track,config);self.gpu=CTBRGPU(track,config);self.cp=self.gpu.cp
        self.gpu_rng=self.cp.random.RandomState(seed);self.bias=self.cp.zeros((6,4),dtype=self.cp.float32)
        self.maneuver=None;self.last_revalidation=None
        self.select(hover_state(track.start),0.,avoid=True)
        self.cp.cuda.get_current_stream().synchronize();self.gpu_rng=self.cp.random.RandomState(seed)
        self.last_diagnostics={}

    def seed_reference(self,*args):
        pass  # Rotor feedforward must never be reinterpreted as CTBR.

    def publish(self,selection,actual_state,now,gate=0):
        if selection is None or selection.cycle_ms>=100:return False
        policy=FeedbackManeuver(selection,gate)
        if not policy.within_domain(actual_state,now):return False
        self.cache=selection;self.maneuver=policy;return True

    def cached_action(self,state,now):
        if self.maneuver is None:return None
        try:return self.maneuver.command(state,now)
        except ValueError:return None

    def reference_risk(self,state,now,first,reference,crossing=0):
        if super().reference_risk(state,now,first,reference,crossing):return True
        if first>=len(self.rollout.track.order) or reference is None:return False
        elapsed=now-reference.made_at
        if elapsed<0:return False
        ts=np.linspace(elapsed,min(elapsed+1.,reference.trajectory.total_time),101)
        positions=reference.trajectory.evaluate(ts,0)
        w=self.rollout.track.windows[self.rollout.track.order[first]]
        slab=self.config.radius+self.config.error_margin+self.config.frame_thickness
        margin=self.config.radius+self.config.error_margin+self.config.feedback_geometry_reserve
        # A collision-free point path can skirt the frame and already lie
        # behind a gate the aircraft has NOT crossed. It is not a safe task
        # reference. Use the same whole-body passage contract as admission.
        for k,(p,t) in enumerate(zip(positions,ts+reference.made_at)):
            q,z=w.world_to_local(p,float(t));z*=self.rollout.signs[first]
            if k==0 and z>slab:return True
            if abs(z)<=slab and (not w.aperture.contains(q) or w.aperture.boundary_distance(q)<=margin):return True
        return False

    def continuation(self,state,now,prepared,first,crossing):
        """Re-simulate remaining feedback policy from actual FULL state.
        Never execute a repeated end control as an unchecked tail.
        """
        # The sender can activate another policy while this loop runs.
        # Revalidate one immutable policy, never splice two by accident.
        maneuver=self.maneuver
        if maneuver is None or not maneuver.within_domain(state,now):return None
        duration=maneuver.selection.expires_at-now
        n=min(self.nc,int(np.floor((duration+1e-9)/.02)))
        if n<5:return None
        x=state.copy();commands=[]
        try:
            for k in range(n):
                u=maneuver.command(x,now+k*.02);commands.append(u)
                x=self.rollout.integrate(x,u)
        except ValueError:return None
        commands=np.array(commands)
        checked=self.rollout(state,commands[None],prepared,first,crossing)
        if not checked.valid[0]:return None
        return commands,checked

    def select(self,state,now,first=0,crossing=False,avoid=False,reference=None,previous_input=None):
        started=time.perf_counter();cp=self.cp;cfg=self.config
        geometry=self.rollout.prepare(now);device=self.gpu.prepare_device(geometry)
        prior=self.continuation(state,now,geometry,first,crossing)
        deadline=None
        if prior is not None:
            u0,check=prior;exits=np.flatnonzero(check.indices[0]>first)
            if len(exits):deadline=now+.01*exits[0]
            base=np.concatenate((u0,np.tile(u0[-1],(self.nc-len(u0),1))))
        else:base=np.tile([MASS*9.8066,0,0,0],(self.nc,1))
        if previous_input is None:
            previous_input=self.cached_action(state,now)
            if previous_input is None:previous_input=base[0]
        previous=cp.asarray(previous_input,dtype=cp.float32)
        # Inside the gate, the current passage stays the task, never the
        # next reference waypoint. No references are read in this branch.
        avoid=bool(avoid or crossing or reference is None)
        candidates=[];tracking_count=0
        if not avoid:
            ts=np.clip(now+np.arange(self.ns+1)*.01-reference.made_at,0,reference.trajectory.total_time)
            pva=np.concatenate([reference.trajectory.evaluate(ts,d) for d in range(3)],axis=1)
            out=cp.empty((5,self.nc,4),dtype=cp.float32)
            self.gpu.tracking_kernel((1,),(32,),(np.int32(5),cp.asarray(state,dtype=cp.float32),cp.asarray(pva,dtype=cp.float32),self.gpu.device_constants[3],previous,out))
            candidates.append(out)
            tracking_count=5
        if first<len(self.rollout.track.order):
            count=360;out=cp.empty((count,self.nc,4),dtype=cp.float32)
            self.gpu.guided_kernel((3,),(128,),(np.int32(count),np.int32(len(self.rollout.track.order)),cp.asarray(state,dtype=cp.float32),device[0],self.gpu.device_constants[2],self.gpu.device_constants[3],np.int32(first),np.int32(crossing),previous,out))
            candidates.append(out)
        else:
            pva=np.zeros((self.ns+1,9));pva[:,:3]=self.rollout.track.goal
            out=cp.empty((5,self.nc,4),dtype=cp.float32)
            self.gpu.tracking_kernel((1,),(32,),(np.int32(5),cp.asarray(state,dtype=cp.float32),cp.asarray(pva,dtype=cp.float32),self.gpu.device_constants[3],previous,out))
            candidates.append(out)
        pilots=cp.concatenate([cp.asarray(base[None],dtype=cp.float32),*candidates])
        pilot=self.gpu.evaluate(state,pilots,device,first,crossing)

        def pool_for(batch):
            pool=batch.valid.copy();forward=batch.indices[:,-1]>first
            exitstep=cp.argmax(batch.indices>first,axis=1)
            # Forward exit must leave >=100 ms checked continuation.
            pool &= (~forward)|(exitstep<=self.ns-10)
            if crossing==2:pool &= forward
            if deadline is not None:pool &= forward&(now+.01*exitstep<=deadline+1e-7)
            if (crossing or avoid) and bool(cp.any(pool&forward)):pool &= forward
            return pool

        # One sampling round: if none of the tracking pilots is admissible,
        # select an autonomous shooting centre before generating samples.
        tracking_infeasible=bool(tracking_count and not bool(cp.any(pilot.valid[1:1+tracking_count])))
        if tracking_infeasible:avoid=True
        pmask=pool_for(pilot)
        costs,_=self.scores(pilot,pilots,previous,device,reference,now,avoid)
        if bool(cp.any(pmask)):
            best=int(cp.argmin(cp.where(pmask,costs,cp.inf)));base=cp.asnumpy(pilots[best]).astype(float)
        controls=self.sample_device(base,avoid,previous)
        count=min(len(pilots),cfg.samples//2);controls[-count:]=pilots[:count]
        batch=self.gpu.evaluate(state,controls,device,first,crossing);pool=pool_for(batch)
        costs,_=self.scores(batch,controls,previous,device,reference,now,avoid)
        safe=int(cp.count_nonzero(pool))
        self.last_diagnostics=dict(backend='cuda_ctbr',safe_samples=safe,samples=cfg.samples,
            avoidance=avoid,reference_weight=float(not avoid),crossing_phase=int(crossing),
            continuation_valid=prior is not None,passage_deadline=deadline,
            tracking_pilots_infeasible=tracking_infeasible)
        self.last_diagnostics['rejections']={name:int(cp.count_nonzero(batch.rejection_bits&bit)) for bit,name in
            ((1,'collision_or_workspace'),(2,'passage'),(4,'speed'),(8,'acceleration'),(16,'tilt'),(32,'body_rate'),(64,'actuation'),(128,'terminal'),(256,'nonfinite'))}
        selection=None
        if safe:
            costs=cp.where(pool,costs,cp.inf);minimum=cp.min(costs)
            weights=cp.exp(cp.clip(-(costs-minimum)/self.temperature,-80,0))*pool
            weights/=cp.sum(weights)
            proposals=[cp.asnumpy(cp.tensordot(weights,controls,axes=(0,0))).astype(float)]
            ids=cp.argsort(costs)[:min(8,safe)];proposals.extend(cp.asnumpy(controls[ids]).astype(float))
            for weighted,u in enumerate(proposals):
                b=self.rollout(state,u[None],geometry,first,crossing)
                if not b.valid[0]:continue
                exits=np.flatnonzero(b.indices[0]>first)
                if crossing==2 and not len(exits):continue
                if len(exits) and exits[0]>self.ns-10:continue
                if deadline is not None and (not len(exits) or now+.01*exits[0]>deadline+1e-7):continue
                # Averaging cannot replace a complete passage with no exit.
                if (avoid or crossing) and bool(cp.any(pool&(batch.indices[:,-1]>first))) and not len(exits):continue
                if weighted==0:
                    test=self.gpu.evaluate(state,cp.asarray(u[None],dtype=cp.float32),device,first,crossing)
                    score,_=self.scores(test,cp.asarray(u[None],dtype=cp.float32),previous,device,reference,now,avoid)
                    if float(score[0])>float(minimum)+1e-4:continue
                selection=Selection(now,u,b.states[0],b.clearances[0],b.indices[0],0.,safe,weighted==0,True,b.phases[0]);break
        if selection is None and prior is not None:
            u,b=prior
            selection=Selection(now,u,b.states[0],b.clearances[0],b.indices[0],0.,safe,False,True,b.phases[0])
            self.last_diagnostics['retained_feedback']=True
        elapsed=1000*(time.perf_counter()-started);self.last_diagnostics['cycle_ms']=elapsed
        if selection is not None:selection.cycle_ms=elapsed
        if elapsed>=100:self.last_diagnostics['deadline_miss']=True;return None
        return selection
