"""TOGT's MINCO-s4 trajectory family in a bounded checked online problem.

Two moving gate points plus free inter-gate points, two pieces per leg.
This is NOT a verbatim TOGT optimizer: the MINCO-s4/L-BFGS structure uses
the previous racing/clearance objective and checked safety/MPC protocol.
"""
from dataclasses import dataclass
import copy
import math
import tempfile
import time
import casadi as ca
import numpy as np
from scipy.optimize import minimize

from convex_timevarying_window.checked_soft_racing.planner import Config as ParentConfig, clearance_penalty
from convex_timevarying_window.receding_gate_racing.planner import symbolic_pose,symbolic_clearance_sq
from convex_timevarying_window.continuation_racing.planner import Planner as TerminalGeometry
from convex_timevarying_window.continuation_racing.guard import frame_clearances
from convex_timevarying_window.conditional_dual_constraint_cem.safety_penalty import polygon_halfspaces
from .trajectory import Plan,basis,minco_coefficients


@dataclass(frozen=True)
class Config(ParentConfig):
    horizon: int = 2
    nodes_per_piece: int = 6
    jit: bool = False
    solver_seconds: float = .055
    optimizer: str = 'lbfgs'
    constraint_penalty: float = 1e5


class BudgetEnded(Exception):pass


class ScalarSolver:
    """Budgeted scalar MINCO objective/adjoint, not the full constraint Jacobian.

Physical conditions become cubic violation costs while searching. Admission
still checks the original residuals and dense local geometry/dynamics.
"""
    def __init__(self,fg,residual,cfg):self.fg,self.residual,self.cfg=fg,residual,cfg;self._stats={}
    def stats(self):return self._stats
    def __call__(self,**args):
        start=time.perf_counter();best=np.array(args['x0']).copy();value=np.inf;calls=0;iterations=0
        feasible=None;feasible_value=np.inf
        def fun(x):
            nonlocal best,value,calls,feasible,feasible_value
            if calls and time.perf_counter()-start>=self.cfg.solver_seconds:raise BudgetEnded()
            f,g,v=self.fg(x,args['p']);f=float(f);calls+=1
            if np.isfinite(f) and f<value:best=x.copy();value=f
            if float(v)<.002 and np.isfinite(f) and f<feasible_value:feasible=x.copy();feasible_value=f
            return f,np.array(g).ravel()
        def callback(x):
            nonlocal iterations
            iterations+=1
        status='budget'
        try:
            result=minimize(fun,best,jac=True,method='L-BFGS-B',bounds=list(zip(args['lbx'],args['ubx'])),
                            callback=callback,options={'maxiter':200,'ftol':1e-10,'gtol':1e-6,'maxls':20,'maxcor':12})
            status=str(result.message)
        except BudgetEnded:pass
        self._stats=dict(iter_count=iterations,return_status=status,evaluations=calls)
        x=feasible if feasible is not None else best
        return dict(x=ca.DM(x),lam_x=ca.DM.zeros(len(x)),lam_g=ca.DM.zeros(len(args['lbg'])))


class Planner:
    swept_balls=TerminalGeometry.swept_balls
    terminal_position=TerminalGeometry.terminal_position
    workspace_inset=TerminalGeometry.workspace_inset

    def __init__(self,track,cfg=Config(),*,build_indices=None):
        self.track,self.cfg=track,cfg
        self.windows=[track.windows[i] for i in track.order]
        self.frames=[];self.signs=[];self.admitted=None;self.search={};self.dual_cache={}
        self._jit_dir=tempfile.TemporaryDirectory(prefix='togt_minco_jit_')
        for i,w in enumerate(self.windows):
            f=copy.copy(w);ap=copy.copy(w.aperture)
            if ap.kind=='circle':object.__setattr__(ap,'radius',ap.radius+cfg.frame_offset)
            else:
                hs=polygon_halfspaces(ap.physical_vertices)
                vertices=np.array([np.linalg.solve(np.stack([hs.normals[j-1],hs.normals[j]]),
                    np.array([hs.offsets[j-1],hs.offsets[j]])+cfg.frame_offset) for j in range(len(hs.offsets))])
                object.__setattr__(ap,'physical_vertices',vertices)
            f.aperture=ap;self.frames.append(f)
            source=track.start if i==0 else self.windows[i-1].center0
            self.signs.append(float(np.sign(w.state_at(0.)[1][:,2]@(source-w.center0)) or 1.))
        self.solvers=[None]*(len(self.windows)+1)
        for first in (range(len(self.windows)+1) if build_indices is None else build_indices):
            self.solvers[first]=self._build(first)

    def ids(self,first):return list(range(first,min(first+self.cfg.horizon,len(self.windows))))
    def commit(self,plan):self.admitted=plan

    def _build(self,first):
        cfg=self.cfg;ids=self.ids(first);gates=len(ids);legs=gates+1;n=2*legs
        # Per-leg duration variables, free 3D middle point, then gate-local 2D points.
        x=ca.MX.sym('x',n+3*legs+2*gates)
        par=ca.MX.sym('par',13+4*n)
        durations=x[:n];head=ca.reshape(par[1:13],3,4)
        tail=ca.DM.zeros(3,4);tail[:,0]=self.terminal_position(first)
        starts=[par[0]+ca.sum1(durations[:i]) if i else par[0] for i in range(n)]
        points=[];lo=[.035]*n;hi=[15.]*n
        for k in range(legs):
            points.append(x[n+3*k:n+3*k+3])
            lo.extend(cfg.workspace_lower or (-100.,-100.,.65))
            hi.extend(cfg.workspace_upper or (100.,100.,12.))
            if k<gates:
                at=par[0]+ca.sum1(durations[:2*k+2]);center,R=symbolic_pose(self.windows[ids[k]],at)
                q=x[n+3*legs+2*k:n+3*legs+2*k+2]
                points.append(center+R[:,:2]@q)
        lo.extend([-2.]*(2*gates));hi.extend([2.]*(2*gates))
        C=minco_coefficients(head,tail,ca.horzcat(*points),durations)
        constraints=[];lower=[];upper=[]
        def con(value,low,high):
            scale=max(1.,abs(low) if np.isfinite(low) else 0.,abs(high) if np.isfinite(high) else 0.)
            constraints.append(ca.vec(value)/scale)
            lower.extend([low/scale]*value.numel());upper.extend([high/scale]*value.numel())
        def state(piece,u,d=0):return C[8*piece:8*piece+8,:].T@basis(u,d)/durations[piece]**d
        def frame_ids(piece):
            k=piece//2
            if k<gates:return range(max(0,ids[k]-1),min(len(self.frames),ids[k]+1))
            return range(max(0,first+gates-1),min(len(self.frames),first+gates+1))
        hard=cfg.radius+cfg.frame_thickness+cfg.tracking_margin
        J=ca.sum1(durations)
        for k,wi in enumerate(ids):
            q=x[n+3*legs+2*k:n+3*legs+2*k+2];w=self.windows[wi]
            inset=hard-cfg.frame_offset
            if w.aperture.kind=='circle':con(ca.sumsqr(q),0.,(w.aperture.radius-inset)**2)
            else:
                hs=polygon_halfspaces(w.aperture.physical_vertices)
                for norm,b in zip(hs.normals,hs.offsets):con(ca.dot(norm,q),-np.inf,b-inset)
            tv=ca.MX.sym('tv');center,R=symbolic_pose(w,tv)
            rates=ca.Function(f'minco_rate_{first}_{k}',[tv],[ca.jacobian(center,tv),ca.reshape(ca.jacobian(ca.vec(R),tv),3,3)])
            at=starts[2*k+1]+durations[2*k+1];center,R=symbolic_pose(w,at);dc,dR=rates(at)
            pos,vel=state(2*k+1,1.),state(2*k+1,1.,1)
            con(-self.signs[wi]*(ca.dot(R[:,2],vel-dc)+ca.dot(dR[:,2],pos-center)),.1,np.inf)
            # Same 40..420 ms signed departure rule, evaluated on the actual adjacent pieces.
            for offset in (.04,.08,.16,.28,.42):
                for piece,endpoint,sign in ((2*k+1,1.,1.),(2*k+2,0.,-1.)):
                    fraction=ca.fmin(.45,offset/durations[piece]);u=endpoint-sign*fraction
                    t=starts[piece]+u*durations[piece];cc,rr=symbolic_pose(w,t)
                    distance=hard*ca.fmin(1.,fraction*durations[piece]/.28)
                    con(sign*self.signs[wi]*ca.dot(rr[:,2],state(piece,u)-cc)-distance,0.,np.inf)
        for piece in range(n):
            # Same total regular-grid interval count per leg as old 12-node implementation.
            nodes=list(np.linspace(0.,1.,cfg.nodes_per_piece+1))
            nodes+=list(ca.vertsplit(par[13+4*piece:13+4*piece+4]))
            for index,u in enumerate(nodes):
                pos,vel,acc,jerk=[state(piece,u,d) for d in range(4)]
                demand=acc+cfg.response_time*jerk;thrust=demand+ca.DM([0.,0.,9.8066])
                for value,limit in ((vel,cfg.speed),(acc,cfg.acceleration),(jerk,cfg.jerk),(demand,cfg.acceleration)):
                    con(ca.sumsqr(value),0.,limit**2)
                con(ca.sumsqr(thrust),4.,(4*8.54858/2.0643076923*.9)**2)
                con((ca.sumsqr(thrust[:2])-(math.tan(math.radians(42))*thrust[2])**2)/9.8066**2,-np.inf,0.)
                if cfg.workspace_lower is not None:
                    for axis in range(3):con(pos[axis],cfg.workspace_lower[axis]+cfg.tracking_margin,cfg.workspace_upper[axis]-cfg.tracking_margin)
                at=starts[piece]+u*durations[piece]
                for gj in frame_ids(piece):con(symbolic_clearance_sq(self.frames[gj],pos,at),hard**2,np.inf)
                if index<=cfg.nodes_per_piece:
                    weight=durations[piece]/cfg.nodes_per_piece*(.5 if index in (0,cfg.nodes_per_piece) else 1.)
                    J+=weight*(cfg.jerk_weight*ca.sumsqr(jerk)+sum(clearance_penalty(symbolic_clearance_sq(self.frames[gj],pos,at),cfg) for gj in frame_ids(piece)))
        gg=ca.vertcat(*constraints)
        opts={'print_time':False,'ipopt.print_level':0,'ipopt.sb':'yes','ipopt.max_iter':35,
              'ipopt.max_cpu_time':cfg.solver_seconds,'ipopt.tol':2e-3,'ipopt.acceptable_tol':1e-2,
              'ipopt.hessian_approximation':'limited-memory','ipopt.warm_start_init_point':'yes'}
        if cfg.jit:
            opts.update(jit=True,jit_name=f'minco_{first}',compiler='shell',
                        jit_options={'flags':['-O0'],'directory':self._jit_dir.name+'/'})
        if cfg.optimizer=='lbfgs':
            violations=[]
            for i,(low,high) in enumerate(zip(lower,upper)):
                if np.isfinite(low):violations.append(ca.fmax(low-gg[i],0.))
                if np.isfinite(high):violations.append(ca.fmax(gg[i]-high,0.))
            vv=ca.vertcat(*violations);cost=J+cfg.constraint_penalty*ca.sum1(vv**3)
            fg=ca.Function(f'minco_scalar_{first}',[x,par],[cost,ca.gradient(cost,x),ca.mmax(vv)])
            solver=ScalarSolver(fg,None,cfg)
        else:
            solver=ca.nlpsol(f'minco_online_{first}','ipopt',{'x':x,'p':par,'f':J,'g':gg},opts)
        evaluate=ca.Function(f'minco_out_{first}',[x,par],[gg,C,J])
        return solver,evaluate,np.array(lo),np.array(hi),np.array(lower),np.array(upper)

    def initial(self,state,now,first):
        # Search iterates have never been executed: preserve their relative
        # durations. Only an actually admitted plan can be trimmed by elapsed time.
        searched=self.search.get(first)
        if searched is not None and (self.admitted is None or self.admitted.first_gate!=first):
            return searched.x.copy()
        ids=self.ids(first);legs=len(ids)+1;n=2*legs
        old=self.admitted
        if old is None:old=self.search.get(first)
        old_ids=self.ids(old.first_gate) if old is not None else []
        times=[];middle=[];qs=[];source=state[:3];stamp=now
        for k in range(legs):
            terminal=k==len(ids)
            target=self.terminal_position(first) if terminal else self.windows[ids[k]].state_at(stamp)[0]
            old_k=(old_ids.index(ids[k]) if not terminal and ids[k] in old_ids else None)
            same_terminal=terminal and old is not None and old.first_gate==first
            end=(old.end_time if same_terminal else old.crossing_times[old_k] if old_k is not None else None)
            if end is not None and end>stamp+.15:
                total=max(.3,float(end-stamp))
                midpoint=old.sample(stamp+total*.5)
                if not terminal:
                    old_legs=len(old_ids)+1;qs.extend(old.x[5*old_legs+2*old_k:5*old_legs+2*old_k+2])
            else:
                distance=np.linalg.norm(target-source)
                total=max(.6,distance/6.5+(.5 if k==0 else 0.))
                if terminal:total=max(total,np.sqrt(6*distance/self.cfg.acceleration),(60*distance/self.cfg.jerk)**(1/3))
                if not terminal:target=self.windows[ids[k]].state_at(stamp+total)[0];qs.extend([0.,0.])
                midpoint=.5*(source+target)
            times.extend([total*.5]*2);middle.extend(midpoint);stamp+=total;source=target
        return np.r_[times,middle,qs]

    def validate(self,plan):
        cfg=self.cfg
        endpoint=plan.sample(plan.end_time)
        if any(np.linalg.norm(endpoint-center)<radius-1e-8 for center,radius in self.swept_balls()):return -np.inf
        tt=np.linspace(plan.made_at,plan.end_time,max(2,int(np.ceil((plan.end_time-plan.made_at)/.01))+1))
        p,v,a,j=[plan.sample_many(tt,d) for d in range(4)];demand=a+cfg.response_time*j;f=demand+np.array([0.,0.,9.8066])
        tol=1.006
        for values,limit in ((v,cfg.speed),(a,cfg.acceleration),(j,cfg.jerk),(demand,cfg.acceleration)):
            if np.linalg.norm(values,axis=1).max()>limit*tol:return -np.inf
        if np.linalg.norm(f,axis=1).max()>4*8.54858/2.0643076923*.9*tol or f[:,2].min()<=0:return -np.inf
        if (np.linalg.norm(f[:,:2],axis=1)/f[:,2]).max()>math.tan(math.radians(42))*tol:return -np.inf
        clear=float(frame_clearances(self.frames,p,tt,cfg.radius,cfg.frame_thickness).min())
        if cfg.workspace_lower is not None:clear=min(clear,float((p-np.array(cfg.workspace_lower)).min()),float((np.array(cfg.workspace_upper)-p).min()))
        return clear

    def refine(self,plan):
        slots=[];start=plan.made_at
        for T in plan.durations:
            tt=np.linspace(start,start+T,51);p=plan.sample_many(tt)
            score=frame_clearances(self.frames,p,tt,self.cfg.radius,self.cfg.frame_thickness)/self.cfg.tracking_margin-1
            for d,limit in ((1,self.cfg.speed),(2,self.cfg.acceleration),(3,self.cfg.jerk)):
                values=plan.sample_many(tt,d);score=np.minimum(score,1-np.sum(values*values,axis=1)/limit**2)
            chosen=[]
            for i in np.argsort(score):
                u=float((tt[i]-start)/T)
                if all(abs(u-q)>.025 for q in chosen):chosen.append(u)
                if len(chosen)==4:break
            slots.extend(chosen);start+=T
        return np.array(slots)

    def solve(self,state,now,first):
        began=time.perf_counter();state=np.asarray(state)
        if len(state)==9:state=np.r_[state,np.zeros(3)]
        solver,evaluate,lbx,ubx,lbg,ubg=self.solvers[first]
        seed=self.initial(state,now,first);n=2*(len(self.ids(first))+1)
        old=self.search.get(first)
        nodes=self.refine(old) if old is not None else np.tile([.15,.35,.65,.85],n)
        par=np.r_[now,state,nodes]
        args=dict(x0=np.clip(seed,lbx,ubx),p=par,lbx=lbx,ubx=ubx,lbg=lbg,ubg=ubg)
        if first in self.dual_cache:args.update(self.dual_cache[first])
        result=solver(**args);x=np.array(result['x']).ravel();g,C,J=evaluate(x,par)
        violation=float(np.maximum(np.maximum(lbg-np.array(g).ravel(),np.array(g).ravel()-ubg),0.).max())
        self.dual_cache[first]=dict(lam_x0=result['lam_x'],lam_g0=result['lam_g'])
        plan=Plan(now,first,x[:n].copy(),np.array(C).reshape(n,8,3),x,0.,violation,solver.stats()['return_status'],len(self.ids(first)))
        self.search[first]=plan
        if violation<.003:plan.clearance=self.validate(plan)
        plan.solve_ms=1000*(time.perf_counter()-began)
        return plan

    def prefix_clearance(self,plan,now,duration=.3):return plan.clearance
