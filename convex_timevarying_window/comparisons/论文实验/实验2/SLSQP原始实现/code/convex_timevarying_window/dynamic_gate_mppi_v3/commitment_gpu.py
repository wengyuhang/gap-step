"""CTBR GPU controller with phase-constrained passage and feedback continuation.

Experimental: local empirical-domain checks are NOT a robustness certificate.
"""
import time
from dataclasses import replace
import numpy as np
from .controller import Controller,Selection
from .gpu import GPUController,GPURollout
from .ctbr import CTBRRollout
from .feedback_maneuver import FeedbackManeuver
from .rollout import MASS,hover_state
from .terminal_cost import velocity_correction_distance
from .passage_contract import candidate_pool,entry_merit


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
        config=replace(config,passage_event_tail=True,axis_workspace_bounds=True)
        super().__init__(track,config,seed)
        self.rollout=CTBRRollout(track,config);self.gpu=CTBRGPU(track,config);self.cp=self.gpu.cp
        self.gpu_rng=self.cp.random.RandomState(seed);self.bias=self.cp.zeros((6,4),dtype=self.cp.float32)
        self.maneuver=None;self.passage=None;self.last_revalidation=None
        self.select(hover_state(track.start),0.,avoid=True)
        self.cp.cuda.get_current_stream().synchronize();self.gpu_rng=self.cp.random.RandomState(seed)
        self.last_diagnostics={}

    def seed_reference(self,*args):
        pass  # Rotor feedforward must never be reinterpreted as CTBR.

    def publish(self,selection,actual_state,now,gate=0):
        if selection is None or selection.cycle_ms>=100:return False
        policy=FeedbackManeuver(selection,gate)
        if not policy.within_domain(actual_state,now):return False
        self.cache=selection;self.maneuver=policy
        if self.passage is not None and self.passage.gate!=gate:self.passage=None
        if np.any(selection.indices>gate):self.passage=policy
        return True

    def cached_action(self,state,now):
        if self.maneuver is None:return None
        try:return self.maneuver.command(state,now)
        except ValueError:return None

    def reference_risk(self,state,now,first,reference,crossing=0):
        # Near-term control risk is checked from actual-state tracking pilots
        # in select(), not inferred from a distant reference tail.
        if reference is None:return True
        elapsed=now-reference.made_at
        if elapsed>=reference.trajectory.total_time:return True
        if elapsed<0:return False
        if first<len(self.rollout.track.order):
            p=reference.trajectory.evaluate(np.array([elapsed]),0)[0]
            _,z=self.rollout.track.windows[self.rollout.track.order[first]].world_to_local(p,now)
            if z*self.rollout.signs[first]>self.config.radius+self.config.error_margin+self.config.frame_thickness:
                return True
        return False

    def reference_warning(self,state,now,first,reference,crossing=0):
        """Lookahead requests replanning; it is NOT an autonomous-mode flag."""
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

    def continuation(self,state,now,prepared,first,crossing,maneuver=None,previous=None):
        """Re-simulate remaining feedback policy from actual FULL state.
        Never execute a repeated end control as an unchecked tail.
        """
        # The sender can activate another policy while this loop runs.
        # Revalidate one immutable policy, never splice two by accident.
        maneuver=self.maneuver if maneuver is None else maneuver
        if maneuver is None:return None
        if not maneuver.within_domain(state,now):
            self.continuation_checks.append('feedback_domain');return None
        duration=maneuver.selection.expires_at-now
        n=min(self.nc,int(np.floor((duration+1e-9)/.02)))
        if n<5:
            self.continuation_checks.append('short_remaining_tail');return None
        try:
            commands=self.rollout.replay_feedback(state,maneuver,now,n,previous)
        except ValueError:
            self.continuation_checks.append('predicted_feedback_domain');return None
        if commands is None:
            self.continuation_checks.append('predicted_feedback_domain');return None
        checked=self.rollout(state,commands[None],prepared,first,crossing)
        if not checked.valid[0]:
            self.continuation_checks.append('hard_constraints:'+str(int(checked.rejection_bits[0])));return None
        exits=np.flatnonzero(checked.indices[0]>first)
        if len(exits) and int(checked.end_steps[0])-exits[0]<10:
            self.continuation_checks.append('short_exit_tail');return None
        self.continuation_checks.append('valid')
        return commands,checked

    @staticmethod
    def selection_from_batch(now,u,b,safe,weighted=False):
        end=int(b.end_steps[0])
        return Selection(now,u[:end//2],b.states[0,:end+1],b.clearances[0,:end],
                         b.indices[0,:end+1],0.,safe,weighted,True,b.phases[0,:end+1])

    def select(self,state,now,first=0,crossing=False,avoid=False,reference=None,previous_input=None):
        started=time.perf_counter();cp=self.cp;cfg=self.config
        self.continuation_checks=[]
        geometry=self.rollout.prepare(now);device=self.gpu.prepare_device(geometry)
        witness=self.passage
        prior=None
        witness_attempted=witness is not None and witness.gate==first
        if witness_attempted:
            prior=self.continuation(state,now,geometry,first,crossing,witness,previous_input)
        witness_valid=prior is not None
        if prior is None and (not witness_attempted or self.maneuver is not witness):
            prior=self.continuation(state,now,geometry,first,crossing,previous=previous_input)
        deadline=None
        if prior is not None:
            u0,check=prior;exits=np.flatnonzero(check.indices[0]>first)
            if len(exits):deadline=now+.01*exits[0]
            if deadline is not None and witness_valid:
                target=getattr(witness.selection,'passage_target',deadline)
                deadline=min(deadline,target)
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

        def recovery(batch,xp,poses):
            if first>=len(self.rollout.track.order):return xp.ones(len(batch.valid),dtype=bool),None
            pose=poses[-1,first];before=poses[-2,first]
            merit=entry_merit(batch.states[:,-1],pose,before,self.rollout.signs[first],cfg,xp)
            initial=entry_merit(batch.states[:,0],pose,before,self.rollout.signs[first],cfg,xp)
            return merit<initial-.02,merit

        def pool_for(batch,xp=cp,poses=device[0]):
            improving,_=recovery(batch,xp,poses)
            return candidate_pool(batch.valid,batch.indices,batch.phases,first,crossing,
                                  avoid,now,deadline,improving,xp)

        def score(batch,controls):
            costs,_=self.scores(batch,controls,previous,device,reference,now,avoid)
            if avoid and first<len(self.rollout.track.order):
                _,merit=recovery(batch,cp,device[0])
                forward=batch.indices[:,-1]>first
                # Exit time PLUS downstream entry effort: a fast exit with
                # velocity pointing away from the next gate is not free.
                exits=.01*cp.argmax(batch.indices>first,axis=1)
                if first+1<len(self.rollout.track.order):
                    ends=batch.end_steps
                    downstream=entry_merit(batch.states[:,-1],device[0][ends,first+1],
                        device[0][ends-1,first+1],self.rollout.signs[first+1],cfg,cp)
                else:
                    downstream=cp.linalg.norm(batch.states[:,-1,:3]-cp.asarray(self.rollout.track.goal),axis=1)/cfg.speed
                    downstream+=cp.linalg.norm(batch.states[:,-1,3:6],axis=1)/(.65*cfg.acceleration)
                costs=cp.where(forward,exits+downstream,merit)
            return costs

        tracking_infeasible=bool(tracking_count and not bool(cp.any(pilot.valid[1:1+tracking_count])))
        near_safe=pilot.near_valid[1:1+tracking_count] & cp.all(
            pilot.clearances[1:1+tracking_count,:10]>=cfg.computation_clearance,axis=1)
        tracking_near_safe=bool(tracking_count and bool(cp.any(near_safe)))
        if tracking_count and not tracking_near_safe:avoid=True
        capture_risk=False
        if tracking_count and first<len(self.rollout.track.order):
            # 100 ms is the computation guard, not the entire maneuvering
            # time. Before entering, require a complete exit witness while
            # there is still time to redirect/brake the current velocity.
            capture_time=min(1.,max(.1,float(np.linalg.norm(state[3:6]))/(.65*cfg.acceleration)))
            nodes=min(self.ns,int(np.ceil(capture_time/.01)))+1
            enters=cp.any(pilot.phases[1:1+tracking_count,:nodes]!=0,axis=1)
            full=pilot.valid[1:1+tracking_count]&(pilot.indices[1:1+tracking_count,-1]>first)
            capture_risk=bool(cp.any(enters)&~cp.any(full))
            if capture_risk:avoid=True
        pmask=pool_for(pilot)
        costs=score(pilot,pilots)
        if bool(cp.any(pmask)):
            best=int(cp.argmin(cp.where(pmask,costs,cp.inf)));base=cp.asnumpy(pilots[best]).astype(float)
        controls=self.sample_device(base,avoid,previous)
        count=min(len(pilots),cfg.samples//2);controls[-count:]=pilots[:count]
        batch=self.gpu.evaluate(state,controls,device,first,crossing);pool=pool_for(batch)
        costs=score(batch,controls)
        safe=int(cp.count_nonzero(pool))
        self.last_diagnostics=dict(backend='cuda_ctbr',safe_samples=safe,samples=cfg.samples,
            avoidance=avoid,reference_weight=float(not avoid),crossing_phase=int(crossing),
            continuation_valid=prior is not None,passage_deadline=deadline,
            witness_valid=witness_valid,tracking_near_safe=tracking_near_safe,
            capture_risk=capture_risk,
            continuation_checks=list(self.continuation_checks),
            replan_requested=tracking_infeasible,
            forward_samples=int(cp.count_nonzero(batch.valid&(batch.indices[:,-1]>first))),
            recovery_samples=int(cp.count_nonzero(pool&(batch.indices[:,-1]==first))),
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
                if not pool_for(b,np,geometry[0])[0]:continue
                exits=np.flatnonzero(b.indices[0]>first)
                if crossing==2 and not len(exits):continue
                if len(exits) and exits[0]>self.ns-10:continue
                if deadline is not None and (not len(exits) or now+.01*exits[0]>deadline+1e-7):continue
                # Averaging cannot replace a complete passage with no exit.
                if (avoid or crossing) and bool(cp.any(pool&(batch.indices[:,-1]>first))) and not len(exits):continue
                if weighted==0:
                    test=self.gpu.evaluate(state,cp.asarray(u[None],dtype=cp.float32),device,first,crossing)
                    check_cost=score(test,cp.asarray(u[None],dtype=cp.float32))
                    if float(check_cost[0])>float(minimum)+1e-4:continue
                selection=self.selection_from_batch(now,u,b,safe,weighted==0);break
        if selection is None and prior is not None:
            u,b=prior
            selection=self.selection_from_batch(now,u,b,safe)
            self.last_diagnostics['retained_feedback']=True
        self.last_diagnostics['selected_exit_time']=None
        if selection is not None:
            exits=np.flatnonzero(selection.indices>first)
            if len(exits):
                exit_time=now+.01*exits[0]
                self.last_diagnostics['selected_exit_time']=exit_time
                selection.passage_target=exit_time if deadline is None else min(deadline,exit_time)
                # A disturbed but safe old witness can be retained late;
                # never hide the delay by moving its absolute target forward.
                self.last_diagnostics['passage_timing_slip']=max(0.,exit_time-selection.passage_target)
            self.last_diagnostics['decision']='passage' if len(exits) else ('recovery' if avoid else 'tracking')
        else:self.last_diagnostics['decision']='no_admissible_update'
        elapsed=1000*(time.perf_counter()-started);self.last_diagnostics['cycle_ms']=elapsed
        if selection is not None:selection.cycle_ms=elapsed
        if elapsed>=100:self.last_diagnostics['deadline_miss']=True;return None
        return selection
