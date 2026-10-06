"""Direct acceleration sequences, safe rollouts, no expired-policy fallback."""
import time
import numpy as np
from .engine import Engine,advance

class Controller:
    def __init__(self,track,samples=16384,seed=0,steps=100,dt=.02):
        self.engine=Engine(track);self.track=track;self.cp=self.engine.cp
        self.rng=self.cp.random.RandomState(seed);self.samples=samples;self.steps=steps;self.dt=dt
        self.mean=None;self.last={};self.stage='TRANSIT';self.prepare_gate=None;self.prior_time=None
    def stage_at(self,state,now,first,phase):
        if first>=len(self.track.order):self.prepare_gate=None;return 'TRANSIT'
        if phase:return 'CROSS'
        if self.prepare_gate==first:return 'PREPARE'
        delta=self.track.windows[self.track.order[first]].state_at(now)[0]-state[:3]
        approach=state[3:]@delta/max(1e-9,np.linalg.norm(delta))
        if np.linalg.norm(delta)<max(7.,2.5*max(0.,approach)):
            self.prepare_gate=first;return 'PREPARE'
        return 'TRANSIT'
    def reference(self,ref,now,state):
        if ref is None:return np.tile(np.r_[state[:3],np.zeros(6)],(self.steps+1,1))
        times=np.clip(now+np.arange(self.steps+1)*self.dt-ref.made_at,0,ref.trajectory.total_time)
        return np.concatenate([ref.trajectory.evaluate(times,d) for d in range(3)],axis=-1)
    def seed(self,state,now,first,reference,mode):
        """Feedback only initializes a CONTROL sequence; not the execution law."""
        x=state.copy();controls=[];gate=first
        for k in range(self.steps):
            if not mode or gate>=len(self.track.order):
                r=reference[k];a=r[6:]+2*(r[:3]-x[:3])+2*(r[3:6]-x[3:])
            else:
                w=self.track.windows[self.track.order[gate]];sign=self.engine.signs[gate]
                c,rot,*_=w.state_at(now+(k+1)*self.dt)
                q=rot.T@(x[:3]-c);distance=np.linalg.norm(c-x[:3]);eta=min(2.,distance/8.)
                center,rotation,*_=w.state_at(now+k*self.dt+eta)
                target=center+sign*rotation[:,2]*(self.engine.cfg[0]+1.)
                direction=target-x[:3];velocity=9.*direction/max(1.,np.linalg.norm(direction))
                a=2*(velocity-x[3:])
                if sign*q[2]>self.engine.cfg[0]+.2:gate+=1
            a*=min(1.,6.5/max(1e-9,np.linalg.norm(a)));controls.append(a);x=advance(x,a,self.dt)
        return np.array(controls)
    def select(self,state,now,first,phase,ref):
        started=time.perf_counter();cp=self.cp;stage=self.stage_at(state,now,first,phase);mode=int(stage!='TRANSIT')
        reference=self.reference(ref,now,state);poses=self.engine.poses(now+np.arange(self.steps+1)*self.dt)
        seed=self.seed(state,now,first,reference,mode)
        avoiding=False
        if stage=='TRANSIT':
            filtered,check=self.engine.filter(state,now,seed[0])
            avoiding=bool(np.linalg.norm(filtered-seed[0])>1e-3 or check['status']!=0)
            if avoiding:
                # Release the TOGT objective now. A local inertial objective
                # asks for a small deflection; the time-varying filter decides
                # which deflection is admissible. This is not a 100 ms gate.
                times=np.arange(self.steps+1)*self.dt
                reference=np.column_stack((state[:3]+times[:,None]*state[3:],
                                           np.tile(state[3:],(len(times),1)),np.zeros((len(times),3))))
                seed=self.seed(state,now,first,reference,0)
        if self.mean is not None:
            shift=max(1,int(round((now-self.prior_time)/self.dt)))
            mean=np.concatenate((self.mean[min(shift,self.steps):],np.tile(seed[-1],(min(shift,self.steps),1))))
        else:mean=seed
        # Piecewise-constant noise changes every 0.1s; all variables are actions.
        block=5;noise=self.rng.normal(0,2.5,(self.samples,(self.steps+block-1)//block,3)).astype(cp.float32)
        noise=cp.repeat(noise,block,axis=1)[:,:self.steps]
        proposals=cp.asarray(mean,dtype=cp.float32)[None]+noise
        proposals[self.samples//2:]=cp.asarray(seed,dtype=cp.float32)+noise[self.samples//2:]
        proposals[0]=cp.asarray(mean,dtype=cp.float32);proposals[1]=cp.asarray(seed,dtype=cp.float32)
        metrics=self.engine.rollout(state,proposals,reference,poses,first,phase,mode,self.dt)
        valid=metrics[:,1]==0;count=int(cp.sum(valid).get());action=None;chosen=None
        if count:
            costs=cp.where(valid,metrics[:,0],cp.inf);best=int(cp.argmin(costs).get())
            weights=cp.where(valid,cp.exp(cp.clip(-(costs-costs[best])/1.,-80.,0.)),0.);weights/=weights.sum()
            weighted=cp.sum(weights[:,None,None]*proposals,axis=0).get()
            # Safe samples may have an unsafe mean. Revalidate the new mean.
            for candidate in (weighted,proposals[best].get()):
                m,_,acts=self.engine.rollout(state,candidate[None],reference,poses,first,phase,mode,self.dt,False)
                if m[1]==0:chosen=candidate;action=acts[0];break
        emergency=False
        if action is None:
            # A NEW filtered action from current state, never old-policy continuation.
            emergency=True
            m,_,acts=self.engine.rollout(state,seed[None,:1],reference[:2],poses[:2],first,phase,mode,self.dt,False)
            if m[1]==0:action=acts[0]
        rejected={str(i):int(cp.sum((metrics[:,1].astype(cp.int32)&i)!=0).get()) for i in (1,2,4,8,16)}
        cp.cuda.get_current_stream().synchronize();elapsed=1000*(time.perf_counter()-started)
        self.last=dict(time=now,gate=first,stage=stage,safe_samples=count,cycle_ms=elapsed,
                       reactive_filter=emergency,avoiding=avoiding,rejected=rejected)
        if elapsed>=100:self.last['deadline_exceeded']=True;action=None
        if chosen is not None:self.mean=chosen;self.prior_time=now
        self.stage=stage;return action
