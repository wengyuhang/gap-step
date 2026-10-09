"""Search subject to inherited common-event deadlines; geometry unchanged."""
import casadi as ca
import numpy as np

from convex_timevarying_window.wang_free_state_mpc.planner import Planner as BasePlanner,Config
from convex_timevarying_window.togt_minco_mpc.planner import ScalarSolver
from .contracts import shared_deadlines,comparison,TOLERANCE_S


class DeadlineSolver:
    def __init__(self,base,planner,first):
        self.planner,self.first=planner,first
        self.ids=planner.ids(first);gates=len(self.ids)
        fg=base.fg
        nx,np0=fg.size1_in(0),fg.size1_in(1)
        x=ca.SX.sym('x',nx);par=ca.SX.sym('par',np0+gates)
        original,_,residual=fg(x,par[:np0])
        cumulative=par[0];extra=[]
        for i in range(gates):
            cumulative+=ca.exp(x[i])
            extra.append(ca.fmax(cumulative-par[np0+i],0.))
        delay=ca.vertcat(0.,*extra)
        cost=original+planner.cfg.constraint_penalty*ca.sum1(delay**3)
        wrapped=ca.Function(f'progress_fg_{first}',[x,par],[cost,ca.gradient(cost,x),ca.fmax(residual,ca.mmax(delay))])
        self.solver=ScalarSolver(wrapped,None,planner.cfg)
        self.last_deadlines=None

    def stats(self): return self.solver.stats()

    def __call__(self,**args):
        now=float(args['p'][0]);cfg=self.planner.cfg
        deadlines=shared_deadlines(self.planner.admitted,self.ids)
        self.last_deadlines=deadlines.copy()
        ub=np.asarray(args['ubx']).copy()
        if len(deadlines) and np.isfinite(deadlines[0]):
            available=deadlines[0]-now
            if available<cfg.min_leg_seconds:
                raise ValueError('shared_passage_already_committed')
            ub[0]=min(ub[0],np.log(available))
        # Unshared events have no inherited deadline. All bounded durations
        # fit below this finite sentinel, so they incur no extra cost.
        finite=np.where(np.isfinite(deadlines),deadlines,now+1000.)
        changed=dict(args,ubx=ub,p=np.r_[args['p'],finite])
        changed['x0']=np.clip(args['x0'],args['lbx'],ub)
        return self.solver(**changed)


class Planner(BasePlanner):
    def _build(self,first):
        base,evaluate,lo,hi,lbg,ubg=super()._build(first)
        return DeadlineSolver(base,self,first),evaluate,lo,hi,lbg,ubg

    def solve(self,state,now,first):
        plan=super().solve(state,now,first)
        plan.shared_passage_contract=comparison(self.admitted,plan)
        if any(r['delay_s']>TOLERANCE_S+1e-9 for r in plan.shared_passage_contract):
            plan.event_valid=False
            plan.event_reason='delayed_shared_passage_search'
        return plan
