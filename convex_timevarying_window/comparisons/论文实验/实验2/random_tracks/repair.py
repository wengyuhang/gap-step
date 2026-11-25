"""Explicit layout directions and direction-feasible MINCO initialization."""
import itertools
import numpy as np
import osqp
from scipy import sparse
from nonconvex_timevarying_window.sc_dynatogt.minco import BoundaryState,MincoSnap
from convex_timevarying_window.global_gate_minco_probe import planner as core
from convex_timevarying_window.global_gate_minco_probe.trajectory import INVERSE,SNAP
from convex_timevarying_window.comparisons.论文实验.实验2.soft_dynamics.phase_initialization import Planner as Parent
from convex_timevarying_window.comparisons.论文实验.实验2.soft_dynamics import kernels

def explicit_signs(track):
    points=np.array([w.center0 for w in track.windows]);nodes=np.vstack([track.start,points,track.goal])
    chords=np.diff(nodes,axis=0);unit=chords/np.linalg.norm(chords,axis=1)[:,None]
    tangent=unit[:-1]+unit[1:]
    rates=np.array([v@w.R[:,2] for v,w in zip(tangent,track.windows)])
    if np.any(abs(rates)<1e-3):raise ValueError('ambiguous geometric crossing direction')
    return -np.sign(rates),rates

def repair_derivatives(track,times,points,signs,lower,upper,nodes,forward_min=.1):
    """Exact convex min-snap QP with seven linear knot-velocity inequalities."""
    m=len(track.windows);n=m+1
    curve=MincoSnap(BoundaryState(track.start),BoundaryState(track.goal),points,times)
    arrivals=np.cumsum(times)[:-1]
    d0=np.concatenate([curve.evaluate(arrivals,order).ravel() for order in [1,2,3]])
    H=np.zeros((3*m,3*m));maps=[]
    for piece,T in enumerate(times):
        B=np.zeros((8,3*m))
        for side,node in enumerate([piece,piece+1]):
            if node in [0,n]:continue
            for order in range(1,4):B[:,(order-1)*m+node-1]=INVERSE[:,4*side+order]*T**order
        H+=B.T@SNAP@B/T**7;maps.append(B)
    H=np.kron(H,np.eye(3));A=np.zeros((m,9*m))
    for i,w in enumerate(track.windows):A[i,3*i:3*i+3]=-signs[i]*w.R[:,2]
    rows=[A];lo=[np.full(m,forward_min)-A@d0];hi=[np.full(m,np.inf)]
    from convex_timevarying_window.togt_minco_mpc.trajectory import basis
    start=0.
    for piece,T in enumerate(times):
        for u in np.linspace(0,1,nodes+1):
            mapping=np.kron(np.asarray(basis(float(u))).ravel()@maps[piece],np.eye(3))
            position=curve.evaluate(float(start+u*T),0)
            rows.append(mapping);lo.append(np.asarray(lower)-position);hi.append(np.asarray(upper)-position)
        start+=T
    Aall=np.vstack(rows);lo=np.concatenate(lo);hi=np.concatenate(hi)
    scale=1/np.sqrt(np.diag(H));Hs=scale[:,None]*H*scale[None,:];Hs=(Hs+Hs.T)*.5
    solver=osqp.OSQP();solver.setup(P=sparse.csc_matrix(Hs),q=np.zeros(len(d0)),A=sparse.csc_matrix(Aall*scale),l=lo,u=hi,verbose=False,eps_abs=1e-8,eps_rel=1e-8,max_iter=30000,polish=True)
    result=solver.solve()
    if result.info.status_val not in [1,2]:raise RuntimeError('direction/workspace min-snap QP '+result.info.status)
    delta=scale*result.x;d=d0+delta
    margins=np.minimum(Aall@delta-lo,hi-Aall@delta)
    if np.min(margins)<-2e-6:raise RuntimeError('initial QP verification failed')
    return d,dict(minimum_forward_mps=float((A@d).min()),minimum_linear_margin=float(margins.min()),iterations=int(result.info.iter),status=result.info.status)

class Planner(Parent):
    def __init__(self,track,free_points=True,*,pose_function,restore=True):
        self.explicit_signs,self.geometric_reference_rates=explicit_signs(track)
        old=core.route_signs;core.route_signs=lambda _:self.explicit_signs.tolist()
        try:super().__init__(track,free_points,solver='ipopt',dynamics_model='togt',dynamic_limit_fraction=.95,extra_clearance_m=.05,initial_count=8,sampling_seed=0,pose_function=pose_function)
        finally:core.route_signs=old
        self.restore_initials=restore;self.initial_audits=[]
    def constrained_minimize(self,fun,initial,**kwargs):
        import finite_recovery
        return finite_recovery.minimize(self,fun,initial,
            primary_module='convex_timevarying_window.comparisons.论文实验.实验2.soft_dynamics.fast_ipopt',
            **kwargs)

    def seeds(self):
        original=super().seeds();results=[];self.initial_audits=[]
        for index,x0 in enumerate(original):
            x=x0.copy();T=np.exp(x[:self.n]);history=[]
            for iteration in range(36 if self.restore_initials else 1):
                x[:self.n]=np.log(T);points=np.array([w.state_at(t)[0] for w,t in zip(self.track.windows,np.cumsum(T))]);d,qp=repair_derivatives(self.track,T,points,self.signs,self.cfg.workspace_lower,self.cfg.workspace_upper,self.cfg.nodes_per_piece);x[self.velocity_start:]=d
                C=np.asarray(self.curve_values(x)).ravel().reshape(self.n,3,8).transpose(0,2,1)
                costs=[]
                for coeff,duration in zip(C,T):
                    v=np.r_[coeff.T.ravel(),duration]
                    residual=np.asarray(kernels.dynamic_residuals('togt',.95)(v)).ravel()
                    cost=float(kernels.slack_integral('togt',.95)(np.maximum(residual,0.),duration));costs.append(cost)
                history.append(dict(iteration=iteration,total_time_s=float(T.sum()),dynamic_penalty=float(sum(costs)),qp=qp))
                if sum(costs)<1e-4 or not self.restore_initials:break
                # Preserve each independent phase-sampled time ratio during feasibility
                # conditioning; all segment times remain independent in the NLP.
                T=np.minimum(T*1.10,np.exp(self.upper[:self.n])*.999)
            self.initial_audits.append(dict(seed=index,restored_dynamic_soft_integral=history[-1]['dynamic_penalty'],history=history));results.append(x)
        return results
