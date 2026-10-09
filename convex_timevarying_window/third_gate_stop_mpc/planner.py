"""True third-window trajectory terminal, not an approximate value function."""
from dataclasses import dataclass
import casadi as ca
import numpy as np
from convex_timevarying_window.event_free_terminal_mpc.planner import Planner as Dense, Config as ParentConfig
from convex_timevarying_window.event_free_terminal_mpc.core import Planner as Core, symbolic_pose
from convex_timevarying_window.wang_free_state_mpc.trajectory import arrival_jerk_for_middle
from convex_timevarying_window.third_gate_time_mpc.planner import ConvergedSolver


@dataclass(frozen=True)
class Config(ParentConfig):
    max_iterations: int = 10000
    optimizer_ftol: float = 1e-6
    optimizer_gtol: float = 1e-6
    compile_objective: bool = True


class Planner(Dense):
    def __init__(self,track,cfg=Config(),*,build_indices=None):
        super().__init__(track,cfg,build_indices=build_indices)

    def terminal_seed(self,first):
        if not self.free_terminal(first):return super().terminal_seed(first)
        third=self.windows[self.ids(first)[-1]+1]
        return np.array([third.center0,np.zeros(3),np.zeros(3)])

    def _build(self,first):
        wrapped,ev,lo,hi,lbg,ubg=super()._build(first)
        fg0=wrapped.solver.fg;gates=len(self.ids(first));legs=gates+1
        local=self.free_terminal(first)
        nx=ev.size1_in(0)-(9 if local else 0)
        x=ca.SX.sym('stop_x',nx);par=ca.SX.sym('stop_p',ev.size1_in(1))
        end=par[0]+ca.sum1(ca.exp(x[:legs]))
        if local:
            center,_=symbolic_pose(self.windows[self.ids(first)[-1]+1],end)
            # Exact endpoint equality through substitution: no nine unused tail
            # variables, no soft terminal penalty, and no extra tau variable.
            z=ca.vertcat(x,center/10,ca.DM.zeros(6))
        else:z=x
        no_deadlines=ca.vertcat(par,ca.repmat(par[0]+1000,gates,1))
        cost,_,residual=fg0(z,no_deadlines);g,C,obj=ev(z,par)
        if not local and getattr(self.cfg,'exclude_postrace_time',False):
            # The actual return leg is outside the W_last body-exit race metric.
            # Keep its curve, smoothing cost, geometry and dynamics constraints.
            cost-=ca.exp(x[gates]);obj-=ca.exp(x[gates])
        options={'cse':True}
        if self.cfg.compile_objective:
            options.update(jit=True,compiler='shell',jit_options={
                'flags':[getattr(self.cfg,'compiler_optimization','-O1')],'directory':self._jit_dir.name+'/'})
        fg=ca.Function(f'third_stop_fg_{first}',[x,par],
            [cost,ca.gradient(cost,x),residual],options)
        evaluate=ca.Function(f'third_stop_out_{first}',[x,par],[g,C,obj])
        return ConvergedSolver(fg,self.cfg),evaluate,lo[:nx],hi[:nx],lbg,ubg

    def initial(self,state,now,first):
        x=super().initial(state,now,first)
        if not self.free_terminal(first):return x
        gates=len(self.ids(first));legs=gates+1;vi,ai,ji,qi=self.offsets(gates)
        nx=4*legs+8*gates
        if len(x)==nx:return x
        x=x[:nx].copy()
        crossing=now+np.exp(x[:gates]).sum();end=crossing+np.exp(x[gates])
        last=self.windows[self.ids(first)[-1]];center,R=last.state_at(crossing)[:2]
        head=np.array([center+R[:,:2]@x[qi+2*(gates-1):qi+2*gates],
            self.cfg.speed*x[vi+3*(gates-1):vi+3*gates],
            self.cfg.acceleration*x[ai+3*(gates-1):ai+3*gates]])
        target=self.windows[self.ids(first)[-1]+1].state_at(end)[0]
        tail=np.array([target,np.zeros(3),np.zeros(3)])
        old=self.admitted
        middle=old.sample((crossing+end)/2) if old is not None and old.first_gate==first else .5*(head[0]+target)
        x[ji+3*gates:ji+3*gates+3]=arrival_jerk_for_middle(head,tail,middle,np.exp(x[gates]))/self.cfg.jerk
        return x

    def solve(self,state,now,first):
        plan=Core.solve(self,state,now,first)
        plan.optimizer_stats=self.solvers[first][0].stats().copy()
        if not plan.optimizer_stats['converged']:
            plan.event_valid=False;plan.event_reason='optimizer_not_converged'
        plan.terminal_gate=first+plan.gate_count if self.free_terminal(first) else None
        return plan
