"""Fixed-size local planning; no full-course optimization or candidate envelope."""
from dataclasses import dataclass
import time
import casadi as ca
import numpy as np
from scipy.optimize import minimize
from convex_timevarying_window.event_free_terminal_mpc.planner import Planner as Dense, Config as ParentConfig
from convex_timevarying_window.event_free_terminal_mpc.core import Planner as Core, symbolic_pose


@dataclass(frozen=True)
class Config(ParentConfig):
    use_third_gate: bool = True
    max_iterations: int = 10000
    optimizer_ftol: float = 1e-6
    optimizer_gtol: float = 1e-6
    exit_extra_m: float = .02


def reach_residual(p, v, center, tau, acceleration, speed):
    """Dimensionless necessary reachability conditions, not a safe flight plan."""
    delta = center-p
    return ca.vertcat((ca.sumsqr(delta-v*tau)-(.5*acceleration*tau**2)**2)/100.,
                      (ca.sumsqr(delta)-(speed*tau)**2)/100.)


class ConvergedSolver:
    """Return scipy's final iterate only. No best-iterate or wall-time truncation."""
    def __init__(self, fg, cfg):
        self.fg, self.cfg, self._stats = fg, cfg, {}

    def stats(self):
        return self._stats

    def __call__(self, **args):
        began=time.perf_counter()
        def fun(x):
            f,g,_=self.fg(x,args['p'])
            return float(f),np.asarray(g).ravel()
        result=minimize(fun,np.asarray(args['x0']),jac=True,method='L-BFGS-B',
            bounds=list(zip(args['lbx'],args['ubx'])),options=dict(maxiter=self.cfg.max_iterations,
            ftol=self.cfg.optimizer_ftol,gtol=self.cfg.optimizer_gtol,maxls=20,maxcor=12))
        self._stats=dict(iter_count=int(result.nit),evaluations=int(result.nfev),
            converged=bool(result.success),return_status=str(result.message),
            optimization_ms=1000*(time.perf_counter()-began),final_objective=float(result.fun))
        return dict(x=ca.DM(result.x))


class Planner(Dense):
    def terminal_seed(self,first):
        if not self.free_terminal(first):return super().terminal_seed(first)
        wi=self.ids(first)[-1];w=self.windows[wi]
        center,R=w.state_at(0.)[:2];outward=-self.signs[wi]*R[:,2]
        slab=self.cfg.radius+self.cfg.frame_thickness+self.cfg.tracking_margin+self.cfg.exit_extra_m
        seed=super().terminal_seed(first)
        seed[0]+=outward*(slab-np.dot(outward,seed[0]-center))
        return seed

    def _build(self,first):
        wrapped,ev,lo,hi,lbg,ubg=super()._build(first)
        # Dense geometry is reused; remove the old common-gate deadline wrapper.
        fg0=wrapped.solver.fg
        local=self.free_terminal(first)
        extra=bool(local and self.cfg.use_third_gate)
        nx=fg0.size1_in(0)
        x=ca.SX.sym('third_x',nx+int(extra));par=ca.SX.sym('third_p',ev.size1_in(1))
        no_deadlines=ca.vertcat(par,ca.repmat(par[0]+1000,len(self.ids(first)),1))
        z=x[:nx]
        if local:
            gates=len(self.ids(first));legs=gates+1;tail=4*legs+8*gates
            end=par[0]+ca.sum1(ca.exp(x[:legs]));wi=self.ids(first)[-1]
            center,R=symbolic_pose(self.windows[wi],end)
            outward=-self.signs[wi]*R[:,2]
            slab=self.cfg.radius+self.cfg.frame_thickness+self.cfg.tracking_margin+self.cfg.exit_extra_m
            raw_pos=10*x[tail:tail+3]
            pos=raw_pos+outward*(slab-ca.dot(outward,raw_pos-center))
            z=ca.vertcat(x[:tail],pos/10,x[tail+3:nx])
        cost,_,residual=fg0(z,no_deadlines);g,C,obj=ev(z,par)
        constraints=[];lower=[];upper=[];penalties=[]
        if local:
            gates=len(self.ids(first));legs=gates+1;tail=4*legs+8*gates
            end=par[0]+ca.sum1(ca.exp(x[:legs]))
            vel=self.cfg.speed*x[tail+3:tail+6]
            # Exit plane is imposed by exact coordinate projection, not a penalty.
            if extra:
                tau=ca.exp(x[-1]);third,_=symbolic_pose(self.windows[wi+1],end+tau)
                r=reach_residual(pos,vel,third,tau,self.cfg.acceleration,self.cfg.speed)
                constraints.extend(ca.vertsplit(r));lower.extend([-np.inf]*2);upper.extend([0.]*2)
                penalties.extend(ca.vertsplit(ca.fmax(r,0.)))
                cost+=tau;obj+=tau
                lo=np.r_[lo,np.log(.02)];hi=np.r_[hi,np.log(30.)]
        vv=ca.vertcat(0.,*penalties)
        cost+=self.cfg.constraint_penalty*ca.sum1(vv**3)
        fg=ca.Function(f'third_fg_{first}',[x,par],[cost,ca.gradient(cost,x),ca.fmax(residual,ca.mmax(vv))],
            {'cse':True,'jit':True,'compiler':'shell',
             'jit_options':{'flags':['-O1'],'directory':self._jit_dir.name+'/'}})
        evaluate=ca.Function(f'third_out_{first}',[x,par],[ca.vertcat(g,*constraints),C,obj])
        return ConvergedSolver(fg,self.cfg),evaluate,lo,hi,np.r_[lbg,lower],np.r_[ubg,upper]

    def initial(self,state,now,first):
        x=super().initial(state,now,first)
        expected=self.solvers[first][0].fg.size1_in(0)
        if len(x)==expected:return x
        gates=len(self.ids(first));legs=gates+1;tail=4*legs+8*gates
        end=now+np.exp(x[:legs]).sum();p=10*x[tail:tail+3];v=self.cfg.speed*x[tail+3:tail+6]
        w=self.windows[self.ids(first)[-1]+1]
        # Initialization only; scalar tau is jointly optimized with the two-gate plan.
        grid=np.linspace(.1,15.,300)
        tau=15.
        for dt in grid:
            d=w.state_at(end+dt)[0]-p
            if np.linalg.norm(d-v*dt)<=.5*self.cfg.acceleration*dt**2 and np.linalg.norm(d)<=self.cfg.speed*dt:
                tau=dt;break
        return np.r_[x,np.log(tau)]

    def solve(self,state,now,first):
        plan=Core.solve(self,state,now,first)
        stats=self.solvers[first][0].stats()
        plan.optimizer_stats=stats.copy()
        if not stats['converged']:
            plan.event_valid=False;plan.event_reason='optimizer_not_converged'
        plan.estimated_third_time_s=float(np.exp(plan.x[-1])) if self.free_terminal(first) and self.cfg.use_third_gate else None
        return plan
