"""Phase-aware, non-stalling independent MINCO starting trajectories."""
import numpy as np
from repair import Planner as Base,repair_derivatives,Parent,kernels
class Planner(Base):
    def align_turns(self,T):
        previous=0.;out=[]
        for i,w in enumerate(self.track.windows):
            omega=2*np.pi*w.motion.frequency
            k=round((omega*(previous+T[i])+w.motion.phase-np.pi/2)/np.pi)
            first=int(np.ceil((omega*(previous+np.exp(self.lower[i]))+w.motion.phase-np.pi/2)/np.pi))
            last=int(np.floor((omega*(previous+np.exp(self.upper[i]))+w.motion.phase-np.pi/2)/np.pi))
            k=int(np.clip(k,first,last));at=(np.pi/2+k*np.pi-w.motion.phase)/omega;out.append(at-previous);previous=at
        return np.r_[out,T[-1]]
    def seeds(self):
        original=Parent.seeds(self);result=[];self.initial_audits=[]
        for index,x0 in enumerate(original):
            x=x0.copy();baseT=np.exp(x[:self.n]);history=[]
            for it in range(40):
                T=self.align_turns(np.minimum(baseT*1.1**it,np.exp(self.upper[:self.n])*.99));x[:self.n]=np.log(T)
                points=np.array([w.state_at(t)[0] for w,t in zip(self.track.windows,np.cumsum(T))])
                d,qp=repair_derivatives(self.track,T,points,self.signs,self.cfg.workspace_lower,self.cfg.workspace_upper,self.cfg.nodes_per_piece,forward_min=6.5);x[self.velocity_start:]=d
                C=np.array(self.curve_values(x)).ravel().reshape(self.n,3,8).transpose(0,2,1);penalty=0.
                for c,duration in zip(C,T):
                    residual=np.asarray(kernels.dynamic_residuals('togt',.95)(np.r_[c.T.ravel(),duration])).ravel();penalty+=float(kernels.slack_integral('togt',.95)(np.maximum(residual,0.),duration))
                history.append(dict(iteration=it,time_s=float(T.sum()),dynamic_soft_integral=penalty,qp=qp))
                if penalty<1e-4:break
            self.initial_audits.append(dict(seed=index,history=history));result.append(x)
        return result
    def constrained_minimize(self,fun,initial,**kwargs):
        import finite_direct
        return finite_direct.minimize(self,fun,initial,**kwargs)
