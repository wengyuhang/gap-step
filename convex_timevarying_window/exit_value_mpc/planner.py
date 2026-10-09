"""Evaluate what is really executed: first body exit plus center value proxy."""
from dataclasses import dataclass
import time
import casadi as ca
import numpy as np
from convex_timevarying_window.event_free_terminal_mpc.planner import Planner as Parent,Config as ParentConfig
from convex_timevarying_window.event_free_terminal_mpc.core import Planner as Core
from convex_timevarying_window.race_to_go_mpc.planner import EnvelopeSolver
from convex_timevarying_window.wang_free_state_mpc.trajectory import Plan
from convex_timevarying_window.wang_free_state_mpc.events import plan_event_check
from .value import continuation,departure


@dataclass(frozen=True)
class Config(ParentConfig):
    center_seconds: float = .040
    free_seconds: float = .025


class ValueSolver(EnvelopeSolver):
    def candidate(self,x,args):
        p=self.planner;gates=len(p.ids(self.first));legs=gates+1
        g,C,_=self.evaluate(x,args['p']);g=np.asarray(g).ravel()
        violation=float(np.maximum(np.maximum(args['lbg']-g,g-args['ubg']),0.).max())
        plan=Plan(float(args['p'][0]),self.first,np.repeat(np.exp(x[:legs])/2,2),
                  np.asarray(C).reshape(2*legs,6,3),x,0.,violation,'value',gates)
        valid=False;reason='optimizer_residual';score=float('inf')
        if violation<.003:
            valid,reason=plan_event_check(plan,p.windows,p.signs,p.cfg)
            if valid:score=p.score(plan)
        return valid and np.isfinite(score),score,reason,violation


class Planner(Parent):
    def __init__(self,track,cfg=Config(),*,build_indices=None):
        self.values={}
        super().__init__(track,cfg,build_indices=build_indices)

    def _build(self,first):
        old,ev,lo,hi,lbg,ubg=super()._build(first)
        gates=len(self.ids(first));legs=gates+1;np0=old.solver.fg.size1_in(1)-gates
        x=ca.SX.sym('local_value_x',len(lo));par=ca.SX.sym('local_value_p',np0)
        cost,_,residual=old.solver.fg(x,ca.vertcat(par,ca.repmat(par[0]+1000,gates,1)))
        if gates:
            _,C,_=ev(x,par);T=ca.exp(x[1])/2
            c=C[12:18,:];normal=ca.DM(self.signs[first]*self.windows[first].state_at(0.)[1][:,2])
            def state(tau,d):
                import math
                u=tau/T
                return c.T@ca.vertcat(*[math.factorial(k)/math.factorial(k-d)*u**(k-d)/T**d if k>=d else 0. for k in range(6)])
            slab=self.cfg.radius+self.cfg.frame_thickness
            tau=ca.fmin(.9*T,slab/ca.fmax(.1,-ca.dot(normal,state(0.,1))))
            for _ in range(2):
                z=ca.dot(normal,state(tau,0)-ca.DM(self.windows[first].center0))+slab
                velocity=ca.fmin(-.1,ca.dot(normal,state(tau,1)))
                tau=ca.fmin(.95*T,ca.fmax(1e-4,tau-z/velocity))
            exit_at=par[0]+ca.exp(x[0])+tau
            z=ca.SX.sym('continuation_state',10)
            V=continuation(z[1:4],z[4:7],z[7:10],z[0],first+1,self.windows,self.signs,self.cfg,self.track.goal)
            fn=ca.Function(f'center_proxy_{first}',[z],[V],{'cse':True});self.values[first]=fn
            proxy=fn(ca.vertcat(exit_at,state(tau,0),state(tau,1),state(tau,2)))
            cost=cost-ca.sum1(ca.exp(x[:legs]))+(exit_at-par[0])+proxy
        fg=ca.Function(f'exit_value_fg_{first}',[x,par],[cost,ca.gradient(cost,x),residual],{'cse':True})
        return ValueSolver(fg,ev,self,first),ev,lo,hi,lbg,ubg

    def score(self,plan):
        if not plan.gate_count:return float(plan.end_time)
        i=plan.first_gate
        at=departure(plan,i,self.windows[i],self.signs[i],self.cfg.radius+self.cfg.frame_thickness)
        if not np.isfinite(at):return float('inf')
        return at+float(self.values[i](np.r_[at,*[plan.sample(at,d) for d in range(3)]]))

    def solve(self,state,now,first):
        began=time.perf_counter();plan=Core.solve(self,state,now,first)
        plan.race_selection=self.solvers[first][0].selection.copy()
        plan.value_score_s=self.score(plan)
        plan.solve_ms=1000*(time.perf_counter()-began)
        return plan
