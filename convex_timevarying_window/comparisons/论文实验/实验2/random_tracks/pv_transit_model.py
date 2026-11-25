"""Independent phase/endpoint-velocity starts using physical transition times."""
import numpy as np
from turning_model import Planner as Turning
from repair import Parent,kernels
class Planner(Turning):
    def seed_velocities(self):
        from convex_timevarying_window.comparisons.论文实验.实验2.safety import frame_aperture
        radius=self.cfg.radius+self.cfg.frame_thickness+self.cfg.passage_margin+self.cfg.witness_slack
        vv=[]
        for sign,w in zip(self.signs,self.track.windows):
            margin=float(frame_aperture(w.aperture).boundary_distance(np.zeros(2)))-radius;omega=2*np.pi*w.motion.frequency
            acc=w.motion.amplitude*omega**2 if w.motion.kind=='translation' else w.motion.length*(w.motion.angle+w.motion.angle**2)*omega**2
            delta=np.sqrt(2*margin/(acc+26.));speed=max(6.5,radius/delta+13*delta)
            vv.append(-sign*speed*w.R[:,2])
        return np.asarray(vv)
    def seeds(self):
        original=Parent.seeds(self);v=self.seed_velocities();nodes=np.vstack([self.track.start,*[w.center0 for w in self.track.windows],self.track.goal]);vel=np.vstack([np.zeros(3),v,np.zeros(3)])
        physics=np.maximum(np.linalg.norm(np.diff(vel,axis=0),axis=1)/8.,np.sqrt(6*np.linalg.norm(np.diff(nodes,axis=0),axis=1)/8.));physics=np.maximum(physics,.8)
        results=[];self.initial_audits=[]
        for idx,x0 in enumerate(original):
            x=x0.copy();proposal=physics+.15*np.exp(x[:self.n]);history=[]
            for it in range(48):
                T=self.align_turns(np.minimum(proposal,np.exp(self.upper[:self.n])*.99));x[:self.n]=np.log(T);x[self.velocity_start:self.velocity_start+3*self.m]=v.ravel()
                C=np.asarray(self.pv_seed_curve(x)).reshape(self.n,8,3)
                for order in [2,3]:x[self.velocity_start+3*self.m*(order-1):self.velocity_start+3*self.m*order]=np.concatenate([np.polynomial.polynomial.polyval(1.,np.polynomial.polynomial.polyder(c,order))/duration**order for c,duration in zip(C[:-1],T[:-1])])
                costs=[]
                for c,duration in zip(C,T):
                    residual=np.array(kernels.dynamic_residuals('togt',.95)(np.r_[c.T.ravel(),duration])).ravel();costs.append(float(kernels.slack_integral('togt',.95)(np.maximum(residual,0.),duration)))
                g,_,actual=self.evaluate(x,self.witnesses());np.testing.assert_allclose(actual,C,atol=1e-6,rtol=1e-8)
                history.append(dict(iteration=it,total_time_s=float(T.sum()),penalty=float(sum(costs)),minimum_finite=float(g.min())))
                if sum(costs)<1e-4 and g.min()>-1e-5:break
                # Endpoint velocities stay fixed, avoiding coupled MINCO speed blow-up.
                bad=np.array(costs)>1e-5;weights=bad.astype(float);weights[:-1]=np.maximum(weights[:-1],bad[1:]*.5);weights[1:]=np.maximum(weights[1:],bad[:-1]*.5)
                if g.min()<-1e-5:weights=np.maximum(weights,.5)
                proposal=np.maximum(T,proposal)*(1+.08*weights)
            results.append(x);self.initial_audits.append(dict(seed=idx,history=history,desired_velocities=v.tolist()))
        return results
