"""CUDA MPPI with GPU-resident candidates and CPU FP64 final admission.

The CUDA dynamics/geometry/FSM are generated from the SAME native source.
The CPU only queries shared gate geometry, checks the winning control, and
manages execution. No CPU fallback is silently used when CUDA is unavailable.
"""
from pathlib import Path
import re
import time
import numpy as np
from .rollout import Rollout, Batch, hover_state, MIXER
from .controller import Controller, Selection


def cuda_source(controls=50,ctbr=False):
    code=(Path(__file__).parent/'native/rollout.cpp').read_text()
    code=re.sub(r'#include[^\n]*','',code)
    code=re.sub(r'^(double|void|bool) (\w+)\(',r'__device__ \1 \2(',code,flags=re.M)
    code=code.replace('constexpr double I[','__device__ const double I[')
    code=code.replace('constexpr double mix[','__device__ const double mix[')
    code=code.replace('extern "C" void','extern "C" __global__ void')
    code=re.sub(r'#pragma omp parallel for[^\n]*','',code)
    code=code.replace('for(int b=0;b<batch;b++){','{ int b=blockIdx.x*blockDim.x+threadIdx.x; if(b>=batch)return;')
    code=code.replace('std::clamp','clip_value').replace('std::memcpy','copy_bytes')
    code=code.replace('std::max','fmax').replace('std::min','fmin').replace('std::abs','fabs').replace('std::','')
    preamble='''
__device__ inline double clip_value(double x,double lo,double hi){return fmin(hi,fmax(lo,x));}
__device__ inline void copy_bytes(double *dst,const double *src,unsigned long n){
 for(unsigned long j=0;j<n/sizeof(double);j++)dst[j]=src[j];
}
'''
    code=(preamble+code).replace('double','float')
    code=re.sub(r'(?<![\w.])((?:\d+\.\d*|\.\d+)(?:[eE][+-]?\d+)?|\d+[eE][+-]?\d+)(?![\w.])',r'\1f',code)
    for name in ('fmin','fmax','fabs','sqrt','hypot'):
        code=re.sub(r'\b'+name+r'\(',name+'f(',code)
    return f"#define V3_GPU 1\n#define V3_CTBR {int(ctbr)}\n#define V3_CONTROLS {controls}\n#define V3_STEPS {2*controls}\n#define V3_NODES {2*controls+1}\n#define V3_KNOTS {controls//5+1}\n"+code+SAMPLING_AND_SCORE


SAMPLING_AND_SCORE=r'''
extern "C" __global__ void sample_controls(int n,int small,const float *baseline,
 const float *previous,const float *noise,const float *bias,float maxf,float minf,float *out){
 int b=blockIdx.x*blockDim.x+threadIdx.x;if(b>=n)return;
 float prev[4];for(int a=0;a<4;a++)prev[a]=previous[a];
 for(int k=0;k<V3_CONTROLS;k++)for(int a=0;a<4;a++){
  int j=k/5;float f=(k%5)*.2f;
#if V3_CTBR
  float scale=(a==0?1.f:(a==3?.03f:.10f))*(b<small?1.f:3.f);
  float perturb=((1-f)*noise[(b*V3_KNOTS+j)*4+a]+f*noise[(b*V3_KNOTS+j+1)*4+a])*scale;
  float u=b==0?baseline[k*4+a]:baseline[k*4+a]+perturb;
  u=clip_value(u,a==0?4*minf:-3.f,a==0?4*maxf:3.f);
  u=prev[a]+clip_value(u-prev[a],a==0?-6.4f:-.16f,a==0?6.4f:.16f);
  prev[a]=u;out[(b*V3_CONTROLS+k)*4+a]=u;
#else
  float perturb=((1-f)*noise[(b*V3_KNOTS+j)*4+a]+f*noise[(b*V3_KNOTS+j+1)*4+a])*(b<small?.05f:.15f)*maxf;
  if(b>=small)perturb+=bias[((b-small)%6)*4+a]*fmaxf(0.f,1.f-k*.1f);
  float u=b==0?baseline[k*4+a]:clip_value(baseline[k*4+a]+perturb,minf,maxf);
  u=prev[a]+clip_value(u-prev[a],-1.6f,1.6f);prev[a]=u;out[(b*V3_CONTROLS+k)*4+a]=u;
#endif
 }
}

extern "C" __global__ void score_controls(int n,int gates,const float *u,const float *previous,
 const float *states,const float *clear,const int *ids,const int *phases,const float *poses,
 const float *signs,const float *reference,const int *expected,float refweight,float margin,
 float slab,float maxf,const float *race,const int *ends,float *cost,int *recovered){
 int b=blockIdx.x*blockDim.x+threadIdx.x;if(b>=n)return;
 float total=refweight>0?0.f:race[b],smooth=0.f;int bad=0;recovered[b]=0;
 for(int k=0;k<ends[b]/2;k++)for(int a=0;a<4;a++){
  float delta=(u[(b*V3_CONTROLS+k)*4+a]-(k?u[(b*V3_CONTROLS+k-1)*4+a]:previous[a]))/maxf;smooth+=delta*delta;
 }
 total+=.02f*smooth/float(V3_CONTROLS);
 for(int j=0;j<10;j++)bad+=clear[b*V3_STEPS+j]<margin;
 for(int k=0;k<=ends[b];k++){
  const float *x=states+(b*V3_NODES+k)*17;int gate=ids[b*V3_NODES+k];
  if(refweight>0 && expected[k]>=0){
   const float *r=reference+k*6;float term=0;
   for(int a=0;a<3;a++){float dp=(x[a]-r[a])/.5f,dv=(x[3+a]-r[3+a])/2.f;term+=dp*dp+dv*dv;}
   total+=refweight*.01f*term;
  }
  if(k>0 && k<=(ends[b]-10)){bad-=clear[b*V3_STEPS+k-1]<margin;bad+=clear[b*V3_STEPS+k+9]<margin;}
  if(k<=(ends[b]-10) && bad==0 && phases[b*V3_NODES+k]==0){
   bool entry=gate>=gates;
   if(!entry){const float *p=poses+(k*gates+gate)*12;float z=0;
    for(int a=0;a<3;a++)z+=(x[a]-p[a])*p[5+3*a];entry=z*signs[gate]<=-slab;
   }
   if(entry)recovered[b]=1;
  }
 }
 cost[b]=total;
}
'''


class GPURollout(Rollout):
    def __init__(self,track,config):
        super().__init__(track,config)
        import cupy as cp
        if cp.cuda.runtime.getDeviceCount()<1:raise RuntimeError('CUDA requested but no device available')
        self.cp=cp
        self.module=cp.RawModule(code=cuda_source(self.nc,self.ctbr),options=('--std=c++17',))
        self.kernel=self.module.get_function('v3_rollout')
        self.guided_kernel=self.module.get_function('v3_guided')
        self.tracking_kernel=self.module.get_function('v3_tracking')
        self.sample_kernel=self.module.get_function('sample_controls')
        self.score_kernel=self.module.get_function('score_controls')
        self.device_constants=[cp.asarray(x,dtype=cp.float32) for x in (self.geom,self.frame_speeds,self.signs,self.cfg)]

    def prepare_device(self,prepared):
        return tuple(self.cp.asarray(x,dtype=self.cp.float32) for x in prepared)

    def evaluate(self,state,controls,prepared,first=0,phase=0):
        cp=self.cp;n=len(controls);gates=len(self.track.order)
        x=cp.asarray(state,dtype=cp.float32);u=cp.asarray(controls,dtype=cp.float32)
        poses,guides=prepared;geom,speeds,signs,cfg=self.device_constants
        states=cp.empty((n,self.ns+1,17),dtype=cp.float32);clear=cp.empty((n,self.ns),dtype=cp.float32)
        valid=cp.empty(n,dtype=cp.int32);ids=cp.empty((n,self.ns+1),dtype=cp.int32)
        race=cp.empty(n,dtype=cp.float32);phases=cp.empty((n,self.ns+1),dtype=cp.int32)
        reasons=cp.empty(n,dtype=cp.int32)
        near_valid=cp.empty(n,dtype=cp.int32)
        self.kernel(((n+127)//128,),(128,),(np.int32(n),np.int32(self.ns),np.int32(gates),
            x,u,poses,geom,speeds,signs,guides,cfg,np.int32(first),np.int32(phase),states,clear,valid,ids,race,phases,reasons,near_valid))
        from .passage_contract import checked_end_steps
        ends=checked_end_steps(ids,first,self.ctbr and self.config.passage_event_tail,cp)
        return Batch(states,clear,valid.astype(cp.bool_),ids,race,phases,reasons,ends,near_valid.astype(cp.bool_))


class GPUController(Controller):
    def __init__(self,track,config,seed=0):
        super().__init__(track,config,seed)
        self.gpu=GPURollout(track,config);self.cp=self.gpu.cp
        cp=self.cp;self.gpu_rng=cp.random.RandomState(seed)
        biases=np.array([[3.,0,0,0],[-3.,0,0,0],[0,.05,0,0],[0,-.05,0,0],[0,0,.05,0],[0,0,-.05,0]])
        self.bias=cp.asarray(np.linalg.solve(MIXER,biases.T).T,dtype=cp.float32)
        # Compilation, CUDA allocator and reduction warmup excluded from online time.
        self.select(hover_state(track.start),0.,avoid=True)
        from types import SimpleNamespace
        class WarmTrajectory:
            total_time=10.
            def evaluate(self,t,d):
                return np.broadcast_to(track.start if d==0 else np.zeros(3),np.shape(t)+(3,))
        warm=SimpleNamespace(trajectory=WarmTrajectory(),made_at=0.,local_points=np.zeros((len(track.order),2)))
        self.select(hover_state(track.start),0.,reference=warm)
        cp.cuda.get_current_stream().synchronize()
        self.gpu_rng=cp.random.RandomState(seed);self.last_diagnostics={}

    def sample_device(self,baseline,avoid,previous):
        cp=self.cp;cfg=self.config;n=cfg.samples
        noise=self.gpu_rng.standard_normal((n,self.nc//5+1,4),dtype=cp.float32)
        u=cp.empty((n,self.nc,4),dtype=cp.float32)
        self.gpu.sample_kernel(((n+127)//128,),(128,),(
            np.int32(n),np.int32(n*(.5 if avoid else .8)),cp.asarray(baseline,dtype=cp.float32),
            previous,noise,self.bias,np.float32(cfg.max_rotor_thrust),np.float32(cfg.min_rotor_thrust),u))
        return u

    def scores(self,batch,u,previous,prepared,reference,now,avoid):
        cp=self.cp;cfg=self.config;n=len(u)
        ref=np.zeros((self.ns+1,6),np.float32);expected=np.full(self.ns+1,-1,np.int32)
        if reference is not None and not avoid:
            times=now+np.arange(self.ns+1)*.01;elapsed=times-reference.made_at
            q=np.clip(elapsed,0.,reference.trajectory.total_time)
            ref[:,:3]=reference.trajectory.evaluate(q,0);ref[:,3:]=reference.trajectory.evaluate(q,1)
            expected=np.zeros(self.ns+1,dtype=np.int32)
            expected[(elapsed<0)|(elapsed>reference.trajectory.total_time)]=-1
        costs=cp.empty(n,dtype=cp.float32);recovered=cp.empty(n,dtype=cp.int32)
        self.gpu.score_kernel(((n+127)//128,),(128,),(
            np.int32(n),np.int32(len(self.rollout.track.order)),u,previous,batch.states,batch.clearances,
            batch.indices,batch.phases,prepared[0],self.gpu.device_constants[2],cp.asarray(ref),
            cp.asarray(expected,dtype=cp.int32),np.float32(not avoid and reference is not None),
            np.float32(cfg.computation_clearance),np.float32(cfg.radius+cfg.error_margin+cfg.frame_thickness),
            np.float32(cfg.max_rotor_thrust),batch.race,batch.end_steps,costs,recovered))
        return costs,recovered.astype(cp.bool_)

    def select(self,state,now,first=0,crossing=False,avoid=False,reference=None,previous_input=None):
        started=time.perf_counter()
        result=self._select(state,now,first,crossing,avoid,reference,previous_input)
        fallback=False
        if result is None and not self.last_diagnostics.get('avoidance',True) and not self.last_diagnostics.get('keep_validated_passage',False):
            fallback=True
            result=self._select(state,now,first,crossing,True,None,previous_input)
        elapsed=1000*(time.perf_counter()-started)
        self.last_diagnostics.update(cycle_ms=elapsed,tracking_infeasible_fallback=fallback,
                                     sampling_rounds=2 if fallback else 1)
        if result is not None:result.cycle_ms=elapsed
        return result

    def _select(self,state,now,first=0,crossing=False,avoid=False,reference=None,previous_input=None):
        cp=self.cp;cfg=self.config;started=time.perf_counter()
        baseline=self.baseline(now)
        if previous_input is None:previous_input=baseline[0]
        geometry=self.rollout.prepare(now)
        prior=self.rollout(state,baseline[None],geometry,first,crossing)
        imminent=self.imminent_risk(prior)
        avoid=bool(avoid or imminent or reference is None)
        prepared=geometry if reference is None or avoid else self.rollout.prepare(now,reference.local_points)
        deadline=self.passage_deadline(state,now,first,crossing,prepared)
        device=self.gpu.prepare_device(prepared);previous=cp.asarray(previous_input,dtype=cp.float32)
        # Small feedback shooting bank supplies a coherent motor-input seed.
        # Every seed is simulated and screened; an unsafe reference never
        # contributes even proposals in autonomous mode.
        proposals=None
        if reference is not None and not avoid:
            times=np.clip(now+np.arange(self.ns+1)*.01-reference.made_at,0.,reference.trajectory.total_time)
            pva=np.concatenate([reference.trajectory.evaluate(times,d) for d in (0,1,2)],axis=1)
            proposals=cp.empty((5,self.nc,4),dtype=cp.float32)
            self.gpu.tracking_kernel((1,),(32,),(np.int32(5),cp.asarray(state,dtype=cp.float32),
                cp.asarray(pva,dtype=cp.float32),self.gpu.device_constants[3],previous,proposals))
        elif first<len(self.rollout.track.order):
            count=360;proposals=cp.empty((count,self.nc,4),dtype=cp.float32)
            self.gpu.guided_kernel(((count+127)//128,),(128,),(
                np.int32(count),np.int32(len(self.rollout.track.order)),cp.asarray(state,dtype=cp.float32),
                device[0],self.gpu.device_constants[2],self.gpu.device_constants[3],np.int32(first),np.int32(crossing),previous,proposals))
        else:
            # The known terminal state is a task goal, not an old TOGT.
            # Autonomous return still needs coherent feedback proposals.
            pva=np.zeros((self.ns+1,9));pva[:,:3]=self.rollout.track.goal
            proposals=cp.empty((5,self.nc,4),dtype=cp.float32)
            self.gpu.tracking_kernel((1,),(32,),(np.int32(5),cp.asarray(state,dtype=cp.float32),
                cp.asarray(pva,dtype=cp.float32),self.gpu.device_constants[3],previous,proposals))
        if proposals is not None:
            # Never discard the shifted solution merely because a fresh
            # smooth shooting family exists: that repeatedly postpones its
            # acceleration. It competes under exactly the same objective.
            proposals=cp.concatenate((cp.asarray(baseline[None],dtype=cp.float32),proposals),axis=0)
            pilot=self.gpu.evaluate(state,proposals,device,first,crossing)
            pilot_cost,pilot_recovery=self.scores(pilot,proposals,previous,device,reference,now,avoid)
            admissible=pilot.valid.copy()
            if deadline is not None:
                exit_steps=cp.argmax(pilot.indices>first,axis=1)
                admissible &= (pilot.indices[:,-1]>first)&(now+.01*exit_steps<=deadline+1e-7)
            if avoid and cp.any(admissible&(pilot.indices[:,-1]>first)):
                admissible &= pilot.indices[:,-1]>first
            if cp.any(admissible):
                best=int(cp.argmin(cp.where(admissible,pilot_cost,cp.inf)))
                baseline=cp.asnumpy(proposals[best]).astype(np.float64)
        u=self.sample_device(baseline,avoid,previous)
        if proposals is not None:
            count=min(len(proposals),cfg.samples//2);u[-count:]=proposals[:count]
        batch=self.gpu.evaluate(state,u,device,first,crossing)
        count=int(cp.count_nonzero(batch.valid))
        self.last_diagnostics=dict(backend='cuda',samples=cfg.samples,safe_samples=count,
            baseline_valid=bool(prior.valid[0]),baseline_clearance=float(prior.clearances.min()),
            imminent_risk=imminent,
            avoidance=avoid,reference_weight=0. if avoid or reference is None else 1.,crossing_phase=int(crossing))
        self.last_diagnostics['passage_deadline']=deadline
        if not count:
            self.last_diagnostics['cycle_ms']=(time.perf_counter()-started)*1000
            return None
        costs,recovered=self.scores(batch,u,previous,device,reference,now,avoid)
        forward=batch.indices[:,-1]>first
        require_forward=bool((crossing or avoid) and cp.any(batch.valid&forward))
        pool=batch.valid.copy()
        if deadline is not None:
            exit_steps=cp.argmax(batch.indices>first,axis=1)
            pool &= forward&(now+.01*exit_steps<=deadline+1e-7)
            if not bool(cp.any(pool)):
                self.last_diagnostics.update(keep_validated_passage=True,cycle_ms=(time.perf_counter()-started)*1000)
                return None
        if require_forward:pool &= forward
        require_recovered=bool(avoid and not crossing and cp.any(pool&recovered))
        if require_recovered:pool &= recovered
        masked=cp.where(pool,costs,cp.inf)
        best_cost=float(cp.min(masked))
        weights=cp.exp(cp.clip(-(masked-cp.min(masked))/self.temperature,-700.,0.))
        weights=cp.where(pool,weights,0.);weights/=cp.sum(weights)
        weighted=cp.tensordot(weights,u,axes=(0,0))
        # Only selected controls cross to the host. Final admission always
        # uses CPU FP64, preventing FP32 near-boundary false acceptance.
        candidates=[cp.asnumpy(weighted).astype(np.float64)]
        ids=cp.asnumpy(cp.argsort(masked)[:min(8,count)])
        # Masked non-pool samples are never fallback candidates.
        host_pool=cp.asnumpy(pool[ids])
        ids=ids[host_pool]
        candidates.extend(cp.asnumpy(u[ids]).astype(np.float64))
        chosen=None
        for i,control in enumerate(candidates):
            control=np.clip(control,cfg.min_rotor_thrust,cfg.max_rotor_thrust)
            check=self.rollout(state,control[None],prepared,first,crossing)
            if not check.valid[0] or (require_forward and check.indices[0,-1]<=first):continue
            if deadline is not None:
                exits=np.flatnonzero(check.indices[0]>first)
                if not len(exits) or now+.01*exits[0]>deadline+1e-7:continue
            if i==0:
                # A safe arithmetic average can erase a coherent manoeuvre.
                # Keep the MPPI update only if its nonlinear rollout improves
                # on the best admitted sampled control (monotone safeguard).
                score=float(check.race[0]) if avoid or reference is None else 0.
                changes=np.diff(np.vstack((previous_input,control)),axis=0)
                score+=.02*np.mean(np.sum((changes/cfg.max_rotor_thrust)**2,axis=1))
                if not avoid and reference is not None:
                    elapsed=now+np.arange(self.ns+1)*.01-reference.made_at
                    active=(elapsed>=0)&(elapsed<=reference.trajectory.total_time)
                    t=np.clip(elapsed,0.,reference.trajectory.total_time)
                    dp=(check.states[0,:,:3]-reference.trajectory.evaluate(t,0))/.5
                    dv=(check.states[0,:,3:6]-reference.trajectory.evaluate(t,1))/2.
                    score+=.01*np.sum((np.sum(dp*dp,axis=1)+np.sum(dv*dv,axis=1))*active)
                if score>best_cost+1e-4:continue
            bad=check.clearances<cfg.computation_clearance
            prefix=np.pad(np.cumsum(bad,axis=1),((0,0),(1,0)))
            restored=bool(np.any((prefix[:,10:]-prefix[:,:-10]==0)&(check.phases[:,:self.ns-9]==0)
                                &self.computation_sides(check,prepared)[:,:self.ns-9]))
            if require_recovered and not restored:continue
            chosen=(control,check,restored,i==0);break
        elapsed=(time.perf_counter()-started)*1000
        self.last_diagnostics.update(cycle_ms=elapsed,forward_samples=int(cp.count_nonzero(batch.valid&forward)),
                                     cpu_final_rejected=chosen is None)
        # Include the diagnostic GPU synchronization in the reported wall time.
        elapsed=(time.perf_counter()-started)*1000;self.last_diagnostics['cycle_ms']=elapsed
        if chosen is None:return None
        control,check,restored,weighted_ok=chosen
        return Selection(now,control,check.states[0],check.clearances[0],check.indices[0],
                         elapsed,count,weighted_ok,restored,check.phases[0])
