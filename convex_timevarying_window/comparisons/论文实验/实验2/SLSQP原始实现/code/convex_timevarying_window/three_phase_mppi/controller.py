from dataclasses import dataclass,replace
import time
import numpy as np
from convex_timevarying_window.dynamic_gate_mppi_v3.config import Config
from .engine import Engine

def config(samples=16384):
    return replace(Config(),samples=samples,horizon=6.,axis_workspace_bounds=True,feedback_geometry_reserve=.12)

class LinearReference:
    """Temporary local inertial target; contains NO unsafe TOGT samples."""
    def __init__(self,state,now,hold=False):
        self.position=state[:3].copy();self.velocity=np.zeros(3) if hold else state[3:6].copy()
        self.made_at=now;self.total_time=10.;self.trajectory=self
    def evaluate(self,t,d):
        t=np.asarray(t)
        if d==0:return self.position+t[...,None]*self.velocity
        return np.broadcast_to(self.velocity if d==1 else np.zeros(3),t.shape+(3,)).copy()

class Progress:
    """Actual events only. Prediction uses a separate local FSM in kernel.cpp."""
    def __init__(self,track,cfg,signs):
        self.track=track;self.cfg=cfg;self.signs=signs;self.first=0;self.phase=0
        self.violation=None;self.crossings=[]

    def update(self,p0,t0,p1,t1):
        if self.first==len(self.track.order):return False
        w=self.track.windows[self.track.order[self.first]];sign=self.signs[self.first]
        q0,z0=w.world_to_local(p0,t0);q1,z1=w.world_to_local(p1,t1)
        a,b=sign*z0,sign*z1;slab=self.cfg.radius+self.cfg.error_margin+self.cfg.frame_thickness
        extent=w.aperture.radius if w.aperture.kind=='circle' else np.linalg.norm(w.aperture.physical_vertices,axis=1).max()
        if self.phase==0 and a<=-slab<b and np.linalg.norm(q1)<extent+slab:self.phase=1
        if self.phase:
            if not w.aperture.contains(q1):self.violation='outside_aperture'
            if b<=-slab or (self.phase==2 and b<0):self.violation='forbidden_retreat'
            if a<=0<b:self.phase=2
            if self.phase==2 and b>slab and self.violation is None:
                self.crossings.append(dict(gate=self.first,time=t1,position=np.asarray(p1).tolist(),local_point=np.asarray(q1).tolist()))
                self.first+=1;self.phase=0;return True
        return False

@dataclass
class Policy:
    vector: np.ndarray
    made_at: float
    first: int
    reference: object
    valid_until: float
    mode: int
    parameters: np.ndarray|None=None

    def __post_init__(self):
        # GPU proposals are float32; final admission and execution must use
        # the SAME float64 polynomial arithmetic, not NumPy scalar downcasts.
        self.vector=np.array(self.vector,dtype=np.float64,copy=True)

    def at(self,now):
        v=self.vector.copy();v[19]+=now-self.made_at;return v

    def pva(self,now):
        v=self.at(now)
        if v[20]<.5:
            if self.reference is None:
                return np.r_[v[:3],np.zeros(6)]
            r=self.reference;at=np.clip(now-r.made_at,0.,r.trajectory.total_time)
            pva=np.concatenate([r.trajectory.evaluate(at,d) for d in (0,1,2)])
            pva[:3]+=.5*v[19]**2*v[21:24];pva[3:6]+=v[19]*v[21:24]
            pva[6:9]+=v[21:24];return pva
        at=v[19];q=min(at,v[18]);c=v[:18].reshape(3,6)
        p=sum(c[:,j]*q**j for j in range(6));vel=sum(j*c[:,j]*q**(j-1) for j in range(1,6))
        acc=sum(j*(j-1)*c[:,j]*q**(j-2) for j in range(2,6))
        return np.r_[p+max(0.,at-v[18])*vel,vel,acc if at<=v[18] else np.zeros(3)]

class Controller:
    def __init__(self,track,cfg=None,seed=0):
        self.cfg=cfg or config();self.track=track;self.engine=Engine(track,self.cfg)
        self.cp=self.engine.cp;self.rng=self.cp.random.RandomState(seed)
        self.active=None;self.prepare_gate=None;self.last={};self.stage='TRANSIT'

    def reference_array(self,ref,now,steps,state):
        if ref is None:return np.tile(np.r_[state[:3],np.zeros(6)],(steps+1,1))
        t=np.clip(now+np.arange(steps+1)*.01-ref.made_at,0.,ref.trajectory.total_time)
        return np.concatenate([ref.trajectory.evaluate(t,d) for d in (0,1,2)],axis=1)

    def choose_stage(self,state,now,first,phase,ref):
        if first>=len(self.track.order):self.prepare_gate=None;return 'TRANSIT'
        if phase:self.prepare_gate=first;return 'CROSS'
        if self.prepare_gate==first:return 'PREPARE'
        # Includes attitude response and transverse adjustment time, not the
        # solver's 100 ms budget. Query the MOVING plane throughout lookahead.
        speed=np.linalg.norm(state[3:6]);look=min(2.5,.65+speed/5.)
        t=np.linspace(0.,look,26);w=self.track.windows[self.track.order[first]]
        p=state[:3]+t[:,None]*state[3:6]
        if ref is not None:
            rt=np.clip(now+t-ref.made_at,0.,ref.trajectory.total_time)
            p=ref.trajectory.evaluate(rt,0)+(state[:3]-ref.trajectory.evaluate(rt[0],0))
        z=np.array([w.world_to_local(pi,now+ti)[1] for pi,ti in zip(p,t)])*self.engine.signs[first]
        slab=self.cfg.radius+self.cfg.error_margin+self.cfg.frame_thickness
        center=w.state_at(now)[0]
        delta=center-state[:3];distance=np.linalg.norm(delta)
        approach=float(state[3:6]@delta/max(distance,1e-9))
        travel=2*distance/(max(0.,approach)+8.)
        if np.max(z)>-slab-.7 and travel<2.5:self.prepare_gate=first;return 'PREPARE'
        return 'TRANSIT'

    def policies(self,state,previous,now,first,mode,ref,poses,avoid=False):
        cp=self.cp;n=self.cfg.samples;out=cp.zeros((n,25),dtype=cp.float32)
        params=None
        if mode==0:
            out[:,21:24]=self.rng.normal(0,2.5 if avoid else .5,(n,3)).astype(cp.float32);out[0,21:24]=0
            if ref is None:out[:,:3]=cp.asarray(state[:3],dtype=cp.float32)
        else:
            # Half broad exploration, half local refinement of previous
            # exit parameters. No gate-specific offsets or timing tables.
            params=cp.empty((n,9),dtype=cp.float32)
            max_duration=(len(poses)-1)*.01-1.
            params[:,0]=self.rng.uniform(.35,max_duration,n)
            params[:,1:3]=self.rng.normal(0,.45,(n,2))
            params[:,3]=self.rng.uniform(2.,10.,n)
            params[:,4:6]=self.rng.normal(0,1.,(n,2))
            params[:,6:9]=self.rng.normal(0,2.,(n,3))
            qref=np.zeros(2) if ref is None else ref.local_points[first]
            params[:n//4,1:3]+=cp.asarray(qref,dtype=cp.float32)
            if self.active is not None and self.active.first==first and self.active.parameters is not None:
                center=self.active.parameters.copy();center[0]=max(.15,center[0]+self.active.made_at-now)
                params[n//2:]=cp.asarray(center,dtype=cp.float32)+self.rng.normal(0,1,(n-n//2,9)).astype(cp.float32)*cp.asarray([.18,.12,.12,.8,.4,.4,1.,1.,1.],dtype=cp.float32)
                params[n//2:,0]=cp.clip(params[n//2:,0],.15,max_duration);params[:,3]=cp.clip(params[:,3],.5,10.)
            T=params[:,0];at=cp.minimum((T/.01).astype(cp.int32),len(poses)-2)
            data=cp.asarray(poses,dtype=cp.float32);pose=data[at,first];future=data[at+1,first]
            rot=pose[:,3:].reshape(n,3,3);rf=future[:,3:].reshape(n,3,3)
            local=cp.column_stack((params[:,1:3],cp.full(n,(self.cfg.radius+self.cfg.error_margin+self.cfg.frame_thickness+.3)*self.engine.signs[first])))
            end=pose[:,:3]+cp.einsum('bij,bj->bi',rot,local)
            pointvel=(future[:,:3]+cp.einsum('bij,bj->bi',rf,local)-end)/.01
            velocity=pointvel+rot[:,:,2]*self.engine.signs[first]*params[:,3,None]+rot[:,:,0]*params[:,4,None]+rot[:,:,1]*params[:,5,None]
            acc=(self.engine.integrate(state,previous,.002)[3:6]-state[3:6])/.002
            c=out[:,:18].reshape(n,3,6);p=cp.asarray(state[:3]);v=cp.asarray(state[3:6]);a=cp.asarray(acc)+params[:,6:9];t=T[:,None]
            c[:,:,0]=p;c[:,:,1]=v;c[:,:,2]=a/2
            dp=end-p-v*t-a*t*t/2;dv=velocity-v-a*t;da=-a
            c[:,:,3]=10*dp/t**3-4*dv/t**2+da/(2*t)
            c[:,:,4]=-15*dp/t**4+7*dv/t**3-da/t**2
            c[:,:,5]=6*dp/t**5-3*dv/t**4+da/(2*t**3)
            out[:,18]=T;out[:,20]=1
        if mode and self.active is not None and self.active.first==first and self.active.mode==mode and self.active.valid_until>now:
            out[0]=cp.asarray(self.active.at(now),dtype=cp.float32)
        elif not mode and self.active is not None and self.active.vector[20]>.999 and self.active.valid_until>now:
            # A gate event must not discard a still-executable exit policy.
            # Re-evaluate it against the NEW actual gate/task as a transit candidate.
            out[-1]=cp.asarray(self.active.at(now),dtype=cp.float32)
        return out,params

    def select(self,state,now,first,phase,ref,previous):
        tick=time.perf_counter();cp=self.cp
        stage=self.choose_stage(state,now,first,phase,ref);mode=int(stage!='TRANSIT')
        if ref is None and not mode:ref=LinearReference(state,now,hold=True)
        if mode:
            center=self.track.windows[self.track.order[first]].state_at(now)[0]
            delta=center-state[:3];distance=np.linalg.norm(delta);direction=delta/max(distance,1e-9)
            approach=float(state[3:6]@direction);transverse=np.linalg.norm(state[3:6]-approach*direction)
            duration=1.8*max(2*distance/(max(0.,approach)+8.),transverse/6.5)+1.2
            window=self.track.windows[self.track.order[first]]
            q,_=window.world_to_local(state[:3],now);_,rot,*_=window.state_at(now)
            lateral_speed=np.linalg.norm(rot[:,:2].T@state[3:6])
            alignment=2*np.sqrt(np.linalg.norm(q)/6.5)+lateral_speed/6.5+.4
            duration=max(duration,alignment+.7)
            if self.active is not None and self.active.first==first and self.active.mode==mode:
                duration=max(duration,self.active.valid_until-now+.1)
            steps=int(np.ceil(np.clip(duration,1.5,6.)/.1))*10
        else:steps=80
        poses=self.engine.prepare(now,steps)
        references=self.reference_array(ref,now,steps,state)
        policies,params=self.policies(state,previous,now,first,mode,ref,poses)
        metrics=self.engine.evaluate(state,previous,policies,references,poses,first,phase,mode)
        avoidance=False
        if mode==0 and int(metrics[0,1].get())!=0:
            # Transit danger: stop following TOGT, sample local inertial
            # corrections, then request a new reference once prefix is safe.
            avoidance=True;ref=LinearReference(state,now)
            references=self.reference_array(ref,now,steps,state)
            policies,params=self.policies(state,previous,now,first,mode,ref,poses,avoid=True)
            metrics=self.engine.evaluate(state,previous,policies,references,poses,first,phase,mode)
        safe=metrics[:,1]==0;count=int(cp.sum(safe).get());chosen=None
        retained=bool(mode and self.active is not None and self.active.first==first and self.active.mode==mode and self.active.valid_until>now)
        if count:
            costs=cp.where(safe,metrics[:,0],cp.inf);best=int(cp.argmin(costs).get())
            temp=.05 if mode else .2
            weights=cp.where(safe,cp.exp(cp.clip(-(costs-costs[best])/temp,-80,0)),0);weights/=cp.sum(weights)
            mean=cp.sum(weights[:,None]*policies,axis=0).get()
            bridge=not mode and float(policies[best,20].get())>.5
            if not mode:
                if bridge:mean=None
                else:
                    weights=cp.where(policies[:,20]<.5,weights,0);weights/=cp.sum(weights)
                    mean=cp.sum(weights[:,None]*policies,axis=0).get()
            # Differently aged polynomial coefficients must share one clock
            # before mixing. Exclude retained policy from the weighted mean.
            if retained:
                weights[0]=0;total=float(cp.sum(weights).get())
                if total>1e-20:
                    weights/=total;mean=cp.sum(weights[:,None]*policies,axis=0).get()
                else:mean=None
            winning=policies[best].get()
            for candidate in ((mean,winning) if mean is not None else (winning,)):
                if mode and candidate[20]<.999:continue
                result,path,u=self.engine.evaluate(state,previous,candidate[None],references,poses,first,phase,mode,False)
                if result[1]==0:
                    par=None if params is None else params[best].get()
                    if retained and best==0 and self.active.parameters is not None:
                        par=self.active.parameters.copy();par[0]+=self.active.made_at-now
                    chosen=Policy(candidate,now,first,ref,now+result[5]*.01,mode,par)
                    break
        prior_fallback=False
        if chosen is None and self.active is not None and self.active.valid_until>now and (not mode or self.active.vector[20]>.999):
            prior=self.active;prior_ref=self.reference_array(prior.reference,now,steps,state)
            m,_,_=self.engine.evaluate(state,previous,prior.at(now)[None],prior_ref,poses,first,phase,mode,False)
            if m[1]==0:
                parameters=None if prior.parameters is None else prior.parameters.copy()
                if parameters is not None:parameters[0]+=prior.made_at-now
                chosen=Policy(prior.at(now),now,first,prior.reference,now+m[5]*.01,mode,parameters)
                prior_fallback=True
        cp.cuda.get_current_stream().synchronize()
        reasons=metrics[:,1].astype(cp.int32)
        self.last=dict(time=now,gate=first,phase=phase,stage=stage,safe_samples=count,
            transit_avoidance=avoidance,prediction_horizon_s=steps*.01,
            exit_policy_bridge=bool(count and bridge) if not mode else False,
            prior_cpu_fallback=prior_fallback,
            rejected={str(bit):int(cp.sum((reasons&bit)!=0).get()) for bit in (1,2,4,8,16,32,64,128,256,512,1024,2048)},
            cycle_ms=1000*(time.perf_counter()-tick),selected=chosen is not None)
        self.stage=stage
        if self.last['cycle_ms']>=100:chosen=None;self.last['deadline_exceeded']=True
        return chosen

    def prefix(self,policy,state,now,first,phase,previous):
        poses=self.engine.prepare(now,10);ref=self.reference_array(policy.reference,now,10,state)
        return self.engine.evaluate(state,previous,policy.at(now)[None],ref,poses,first,phase,policy.mode,False)

    def action(self,state,now,previous):
        if self.active is None or now>self.active.valid_until+1e-9:raise RuntimeError('no_current_policy')
        return self.engine.command(state,self.active.pva(now),previous)
