"""GPU MPPI over physical CTBR commands, not a PVA policy parameterization."""
import time
import numpy as np
from convex_timevarying_window.dynamic_gate_mppi_v3.rollout import MASS
from .engine import Engine

class Controller:
    def __init__(self,track,samples=16384,steps=100,seed=0,racing_guidance=False,coupled_racing=False):
        self.coupled_racing=coupled_racing
        self.track=track;self.racing_guidance=racing_guidance;self.engine=Engine(track,racing_guidance=racing_guidance,coupled_racing=coupled_racing);self.cp=self.engine.cp
        self.samples=samples;self.steps=steps;self.dt=.02;self.rng=self.cp.random.RandomState(seed)
        self.mean=None;self.prior_time=None;self.prepare_gate=None;self.stage='TRANSIT';self.last={};self.next_state=None
    def reference_passage_hint(self,x,now,first,ref):
        """Local handoff timing only, not a certificate or TOGT admission.

        A curved racing reference must not be rejected merely because a
        straight-line extrapolation misses the opening. Query only the short
        planned passage interval, including known future gate motion.
        """
        if ref is None or not hasattr(ref,'crossing_times'):return None
        index=first-ref.first
        if index<0 or index>=len(ref.crossing_times):return None
        crossing=float(ref.crossing_times[index]);eta=crossing-now
        if eta<=0:return None
        w=self.track.windows[self.track.order[first]];cfg=self.engine.config
        slab=cfg.radius+cfg.error_margin+cfg.frame_thickness
        at=crossing-ref.made_at
        if not 0<=at<=ref.trajectory.total_time:return None
        velocity=ref.trajectory.evaluate(at,1);_,rot,*_=w.state_at(crossing)
        vn=self.engine.signs[first]*np.dot(rot[:,2],velocity)
        if vn<=.5:return None
        span=min(.6,slab/vn+.08)
        times=crossing+np.linspace(-span,span,7)
        query=times-ref.made_at
        if query[0]<0 or query[-1]>ref.trajectory.total_time:return None
        current=ref.trajectory.evaluate(np.clip(now-ref.made_at,0.,ref.trajectory.total_time),0)
        # Preserve the measured tracking offset instead of testing an ideal
        # path while silently assuming perfect tracking.
        points=ref.trajectory.evaluate(query,0)+(x[:3]-current)
        centers,rotations=self.engine.geometry.poses(times)
        gate=self.track.order[first]
        local=np.einsum('tji,tj->ti',rotations[:,gate],points-centers[:,gate])
        frame=self.engine.geometry.frames[gate]
        clear=np.hypot(frame.boundary_distance(local[:,:2]),local[:,2])-slab
        aligned=bool(frame.contains(local[3,:2]) and clear.min()>.10)
        return dict(entry_eta_s=max(0.,eta-slab/vn),aligned=aligned,
                    minimum_local_reference_clearance_m=float(clear.min()))

    def stage_at(self,x,now,first,phase,ref=None):
        self.handoff_hint=None
        if first>=len(self.track.order):self.prepare_gate=None;return 'TRANSIT'
        if phase:return 'CROSS'
        w=self.track.windows[self.track.order[first]]
        center,_,*_=w.state_at(now)
        next_center,_,*_=w.state_at(now+.05)
        toward=center-x[:3];range_to_aperture=np.linalg.norm(toward)
        center_relative_speed=x[3:6]-(next_center-center)/.05
        closing=np.dot(center_relative_speed,toward)/max(range_to_aperture,1e-9)
        if self.prepare_gate==first:
            # With large but known gate translation, a briefly near opening
            # may swing away again.  Do not remain in a latched preparation
            # phase while the finite opening is remote.
            if range_to_aperture<=10. or range_to_aperture/max(closing,.1)<=3.5:return 'PREPARE'
            self.prepare_gate=None
        # A rotating plane's signed distance is not an arrival clock while
        # the drone is far from its finite opening. Retain TOGT in that part
        # of the inter-gate transit and release it within a vehicle-scale
        # three-second centre-approach horizon (or within 8 m regardless of
        # instantaneous velocity).
        # A known imminent reference crossing can require lateral capture
        # before the finite-aperture distance guard would normally activate.
        imminent_reference=False
        if getattr(self,'coupled_racing',False) and ref is not None and hasattr(ref,'crossing_times'):
            index=first-ref.first
            imminent_reference=0<=index<len(ref.crossing_times) and 0<float(ref.crossing_times[index])-now<4.
        if not imminent_reference and range_to_aperture>8. and range_to_aperture/max(closing,.1)>3.:
            return 'TRANSIT'
        self.handoff_hint=self.reference_passage_hint(x,now,first,ref) if (getattr(self,'racing_guidance',False) or getattr(self,'coupled_racing',False)) else None
        if self.handoff_hint is not None and self.handoff_hint['aligned']:
            if self.handoff_hint['entry_eta_s']>.8:return 'TRANSIT'
            self.prepare_gate=first;return 'PREPARE'
        q,z=w.world_to_local(x[:3],now)
        _,z_next=w.world_to_local(x[:3]+.05*x[3:6],now+.05)
        sign=self.engine.signs[first]
        slab=self.engine.config.radius+self.engine.config.error_margin+self.engine.config.frame_thickness
        normal_speed=sign*(z_next-z)/.05
        distance_to_entry=-slab-sign*z
        if distance_to_entry<=0:
            self.prepare_gate=first;return 'PREPARE'
        # A cheap, moving-gate prediction is used only to decide when to
        # release TOGT tracking. It is never accepted as a safety witness.
        if normal_speed>.1:
            time_to_entry=distance_to_entry/normal_speed
            if time_to_entry<=3.2:
                projected,_=w.world_to_local(x[:3]+time_to_entry*x[3:6],now+time_to_entry)
                frame=self.engine.geometry.frames[self.track.order[first]]
                reserve=slab+.15
                aligned=frame.contains(projected) and frame.boundary_distance(projected)>=reserve
                if time_to_entry<=1.0 or not aligned:
                    self.prepare_gate=first;return 'PREPARE'
        return 'TRANSIT'
    def reference(self,ref,now,x):
        if ref is None:return np.tile(np.r_[x[:3],np.zeros(6)],(self.steps+1,1))
        times=np.clip(now+np.arange(self.steps+1)*self.dt-ref.made_at,0.,ref.trajectory.total_time)
        return np.concatenate([ref.trajectory.evaluate(times,d) for d in range(3)],axis=-1)
    def avoidance_reference(self,x,now,first):
        # Escape along a new gate-centre rendezvous direction rather than
        # extrapolating the velocity that triggered a geometric violation.
        if first<len(self.track.order):
            window=self.track.windows[self.track.order[first]]
            rough=np.linalg.norm(window.state_at(now)[0]-x[:3])
            target=window.state_at(now+min(self.steps*self.dt,rough/8.))[0]
        else:target=self.track.goal
        delta=target-x[:3];length=np.linalg.norm(delta)
        desired=delta*(min(8.,length/1.5)/max(length,1e-9))
        times=np.arange(self.steps+1)*self.dt
        return np.column_stack((x[:3]+times[:,None]*desired,np.tile(desired,(len(times),1)),np.zeros((len(times),3))))
    def stabilization_reference(self,x):
        return np.tile(np.r_[x[:3],.4*x[3:6],np.zeros(3)],(self.steps+1,1))
    def select(self,x,now,first,phase,ref,previous_command=None):
        tick=time.perf_counter();cp=self.cp;self.next_state=None
        if previous_command is None:previous_command=np.array([MASS*9.8066,0.,0.,0.])
        stage=self.stage_at(x,now,first,phase,ref);mode={'TRANSIT':0,'PREPARE':1,'CROSS':2}[stage]
        poses=self.engine.poses(now+np.arange(4*self.steps+1)*self.dt/4)
        reference=self.reference(ref,now,x);seed=self.engine.seed(x,reference,poses,first,mode,self.dt)
        m,_,_=self.engine.rollout(x,seed[None],reference,poses,first,phase,mode,gpu=False,previous_command=previous_command)
        avoid=bool(stage=='TRANSIT' and (int(m[1])&1))
        if avoid:
            # Constant-velocity escape is not an obstacle-avoidance policy:
            # after a gate it can drive directly into a workspace face. Aim
            # along a new gate-centre rendezvous direction while the exact
            # frame and room constraints still reject unsafe CTBR tapes.
            reference=self.avoidance_reference(x,now,first)
            seed=self.engine.seed(x,reference,poses,first,0,self.dt)
        if self.mean is None:mean=seed
        else:
            shift=min(self.steps,max(1,int(round((now-self.prior_time)/self.dt))))
            mean=np.concatenate((self.mean[shift:],seed[-shift:]))
        anchors=[mean,seed]
        reference_seed=bool(self.coupled_racing and stage!='TRANSIT' and ref is not None and hasattr(ref,'crossing_times')
                            and first>=ref.first and first-ref.first<len(ref.crossing_times) and ref.crossing_times[first-ref.first]>now)
        if reference_seed:
            # Proposal only: retains the jointly planned crossing velocity and
            # turn, while the autonomous cost and output safety still decide.
            anchors.append(self.engine.seed(x,reference,poses,first,0,self.dt))
        if stage!='CROSS':
            # The same bounded plant can safely shed speed when a racing tape
            # approaches its tilt/acceleration envelope. This anchor is also
            # needed in ordinary transit, not only at a gate entrance.
            brake=self.engine.seed(x,self.stabilization_reference(x),poses,first,0,self.dt)
            anchors.append(brake)
        if stage=='TRANSIT':
            if avoid:
                times=np.arange(self.steps+1)*self.dt
                coast=np.column_stack((x[:3]+times[:,None]*x[3:6],np.tile(x[3:6],(len(times),1)),np.zeros((len(times),3))))
                alternative=self.engine.seed(x,coast,poses,first,0,self.dt)
            else:
                alternative=self.engine.seed(x,self.avoidance_reference(x,now,first),poses,first,0,self.dt)
            anchors.append(alternative)
        block=5;n=self.samples
        noise=self.rng.normal(0,1,(n,(self.steps+block-1)//block+1,4)).astype(cp.float32)
        noise*=cp.asarray([1.8,.14,.14,.06],dtype=cp.float32)
        noise[:n//4]*=.25;noise[3*n//4:]*=2.5
        # Continuous piecewise-linear input perturbations: block holding
        # injected a fresh body-rate jump every 100 ms into otherwise smooth
        # warm starts. The physical rollout still decides whether it is safe.
        knot=cp.arange(self.steps)//block
        fraction=(cp.arange(self.steps)%block).astype(cp.float32)/block
        noise=noise[:,knot]*(1-fraction[None,:,None])+noise[:,knot+1]*fraction[None,:,None]
        proposals=cp.empty((n,self.steps,4),dtype=cp.float32)
        for j,anchor in enumerate(anchors):
            left=j*n//len(anchors);right=(j+1)*n//len(anchors)
            proposals[left:right]=cp.asarray(anchor,dtype=cp.float32)[None]+noise[left:right]
        for j,anchor in enumerate(anchors[:n]):proposals[j]=cp.asarray(anchor,dtype=cp.float32)
        metrics=self.engine.rollout(x,proposals,reference,poses,first,phase,mode,soft_future=True,previous_command=previous_command)
        # Future violations are graded as warnings.  Only the command we will
        # actually publish must pass the current 20 ms safety filter.
        valid=metrics[:,7]>0;count=int(valid.sum().get());full_count=int(((metrics[:,1]==0)&valid).sum().get())
        chosen=None;action=None;chosen_metrics=None;used_best=False
        checked=[]
        if count:
            costs=cp.where(valid,metrics[:,0],cp.inf);best=int(cp.argmin(costs).get())
            weights=cp.where(valid,cp.exp(cp.clip(-(costs-costs[best])/.4,-80.,0.)),0.);weights/=weights.sum()
            weighted=cp.sum(weights[:,None,None]*proposals,axis=0).get()
            # FP32 finite-difference projections can disagree with FP64 near
            # an active constraint. Recheck several ranked candidates instead
            # of confusing two rejected representatives with zero safe samples.
            ranked=cp.argsort(costs)[:min(16,count)].get()
            candidates=proposals[ranked].get()
            for idx,proposal in enumerate([weighted,*candidates]):
                check,path,controls=self.engine.rollout(x,proposal[None],reference,poses,first,phase,mode,gpu=False,projection_iterations=4,soft_future=True,previous_command=previous_command)
                checked.append(check.tolist())
                if check[7]>0 and len(controls):
                    chosen=np.asarray(proposal,dtype=float).copy();chosen[:len(controls)]=controls
                    action=controls[0].copy();self.next_state=path[1].copy();chosen_metrics=check.tolist();used_best=bool(idx);break
        rejected={str(i):int(cp.sum((metrics[:,1].astype(cp.int32)&i)!=0).get()) for i in (1,2,4,8,16,32,64,128,256,512)}
        cp.cuda.get_current_stream().synchronize();ms=1000*(time.perf_counter()-tick)
        self.last=dict(time=now,gate=first,stage=stage,first_step_safe_samples=count,full_horizon_safe_samples=full_count,cycle_ms=ms,avoiding=avoid,
                       reference_seed=reference_seed,
                       reference_passage_hint=self.handoff_hint,
                       rejected=rejected,best_fallback=used_best,selected=chosen_metrics,cpu_checks=checked,command=None if action is None else action.tolist())
        if action is None:
            self.failure_data=dict(state=np.array(x),time=np.array(now),reference=reference,poses=poses,seed=seed,mean=mean,
                                   candidates=candidates if count else np.empty((0,self.steps,4)))
        if ms>=100:self.last['deadline_exceeded']=True;action=None
        if chosen is not None and action is not None:self.mean=chosen;self.prior_time=now
        self.stage=stage;return action
