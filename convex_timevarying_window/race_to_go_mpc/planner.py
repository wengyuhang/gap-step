"""Bounded center-then-free search; no claim of closed-loop global dominance."""
from dataclasses import dataclass, replace
import time
import casadi as ca
import numpy as np
from convex_timevarying_window.event_free_terminal_mpc.planner import Planner as Parent, Config as ParentConfig
from convex_timevarying_window.togt_minco_mpc.planner import ScalarSolver
from convex_timevarying_window.wang_free_state_mpc.trajectory import Plan
from convex_timevarying_window.wang_free_state_mpc.events import plan_event_check


@dataclass(frozen=True)
class Config(ParentConfig):
    center_seconds: float = .035
    free_seconds: float = .025
    compile_objective: bool = False


def finish_time(plan, window, sign, slab):
    """Polynomial departure root for a fixed plane and in-plane gate motion.

    Only solves the final two-piece tail, not a whole-course safety scan.
    These are numerical polynomial roots, not interval-certified roots.
    """
    if not plan.gate_count:
        return float(plan.end_time)
    normal=sign*window.state_at(0.)[1][:,2]
    start=float(plan.crossing_times[-1])
    for i in range(2*plan.gate_count,len(plan.durations)):
        coeff=plan.coefficients[i]@normal
        coeff=coeff.copy();coeff[0]+=slab-normal@window.center0
        if coeff[0]<=0:return start
        roots=np.polynomial.polynomial.polyroots(coeff)
        outgoing=sorted(float(r.real) for r in roots if abs(r.imag)<1e-7 and 0<=r.real<=1
            and np.polynomial.polynomial.polyval(r.real,np.polynomial.polynomial.polyder(coeff))<0)
        if outgoing:return start+outgoing[0]*plan.durations[i]
        start+=plan.durations[i]
    return float('inf')


def select_candidate(center, free):
    """(admissible, absolute_finish_time) pairs; ties retain the center."""
    if free[0] and np.isfinite(free[1]) and (not center[0] or free[1] < center[1]-1e-6):
        return 'free'
    return 'center'


class EnvelopeSolver:
    def __init__(self, fg, evaluate, planner, first):
        self.fg, self.evaluate, self.planner, self.first = fg, evaluate, planner, first
        self._stats = {}; self.selection = {}; self.center_memory=None; self.center_stamp=None

    def stats(self): return self._stats

    def candidate(self, x, args):
        p=self.planner; gates=len(p.ids(self.first)); legs=gates+1
        g,C,_=self.evaluate(x,args['p']);g=np.asarray(g).ravel()
        violation=float(np.maximum(np.maximum(args['lbg']-g,g-args['ubg']),0.).max())
        plan=Plan(float(args['p'][0]), self.first,np.repeat(np.exp(x[:legs])/2,2),
                  np.asarray(C).reshape(2*legs,6,3),x,0.,violation,'envelope',gates)
        valid=False; reason='optimizer_residual'
        if violation < .003:
            valid,reason=plan_event_check(plan,p.windows,p.signs,p.cfg)
        score=finish_time(plan,p.windows[-1],p.signs[-1],p.cfg.radius+p.cfg.frame_thickness) if valid else float('inf')
        return valid and np.isfinite(score),score,reason,violation

    def __call__(self, **args):
        cfg=self.planner.cfg; gates=len(self.planner.ids(self.first)); qi=self.planner.offsets(gates)[3]
        lo=np.asarray(args['lbx']).copy();hi=np.asarray(args['ubx']).copy()
        lo[qi:qi+2*gates]=hi[qi:qi+2*gates]=0.
        center_solver=ScalarSolver(self.fg,None,replace(cfg,solver_seconds=cfg.center_seconds))
        seed=np.asarray(args['x0'] if self.center_memory is None else self.center_memory).copy()
        if self.center_memory is not None:
            # Keep downstream ABSOLUTE arrival times when the head clock moves.
            # Reusing unchanged durations would shift every moving gate's phase.
            remaining=np.exp(seed[0])-(float(args['p'][0])-self.center_stamp)
            seed[0]=np.log(max(cfg.min_leg_seconds,remaining))
        center_args=dict(args,lbx=lo,ubx=hi,x0=np.clip(seed,lo,hi))
        center=center_solver(**center_args);cx=np.asarray(center['x']).ravel()
        self.center_memory=cx.copy()
        self.center_stamp=float(args['p'][0])
        cs=self.candidate(cx,args)
        chosen='center';result=center;fs=(False,float('inf'),'not_run',float('inf'));free_stats={}
        if cfg.free_points and gates:
            free_solver=ScalarSolver(self.fg,None,replace(cfg,solver_seconds=cfg.free_seconds))
            free=free_solver(**dict(args,x0=cx));fx=np.asarray(free['x']).ravel()
            fs=self.candidate(fx,args);free_stats=free_solver.stats()
            chosen=select_candidate(cs,fs)
            if not cs[0] and not fs[0] and fs[3]<cs[3]:
                # Search memory only: this invalid candidate still fails admission.
                chosen='free'
            if chosen=='free':result=free
        self.selection=dict(selected=chosen,center_valid=bool(cs[0]),center_finish_s=cs[1] if cs[0] else None,
                            free_valid=bool(fs[0]),free_finish_s=fs[1] if fs[0] else None,
                            center_reason=cs[2],free_reason=fs[2],center_residual=cs[3],free_residual=fs[3],
                            center_stats=center_solver.stats(),free_stats=free_stats)
        self._stats=dict(iter_count=center_solver.stats()['iter_count']+free_stats.get('iter_count',0),
                         return_status='race_to_go_'+chosen)
        return result


class Planner(Parent):
    def ids(self,first): return list(range(first,len(self.windows)))

    def _build(self,first):
        old,evaluate,lo,hi,lbg,ubg=super()._build(first)
        gates=len(self.ids(first));legs=gates+1
        nx=old.solver.fg.size1_in(0);np0=old.solver.fg.size1_in(1)-gates
        x=ca.SX.sym('race_x',nx);p=ca.SX.sym('race_p',np0)
        # Remove inherited per-window deadlines: an early window may be later
        # when the complete remaining race finishes earlier.
        cost,_,residual=old.solver.fg(x,ca.vertcat(p,ca.repmat(p[0]+1000,gates,1)))
        if gates:
            vi=self.offsets(gates)[0]
            normal=self.windows[-1].state_at(0.)[1][:,2]
            outward=-self.signs[-1]*ca.dot(ca.DM(normal),self.cfg.speed*x[vi+3*(gates-1):vi+3*gates])
            # Full remaining gate time + smooth departure-time approximation;
            # exact departure score, not this approximation, selects candidates.
            cost=cost-ca.exp(x[legs-1])+(self.cfg.radius+self.cfg.frame_thickness)/ca.fmax(.1,outward)
        opts={'cse':True}
        if self.cfg.compile_objective:
            opts.update(jit=True,compiler='shell',jit_options={'flags':['-O0'],'directory':self._jit_dir.name+'/'})
        fg=ca.Function(f'race_fg_{first}',[x,p],[cost,ca.gradient(cost,x),residual],opts)
        return EnvelopeSolver(fg,evaluate,self,first),evaluate,lo,hi,lbg,ubg

    def solve(self,state,now,first):
        # Use core construction, not Parent.solve's individual gate deadlines.
        from convex_timevarying_window.event_free_terminal_mpc.core import Planner as Core
        began=time.perf_counter();plan=Core.solve(self,state,now,first)
        solver=self.solvers[first][0]
        plan.race_selection=solver.selection.copy()
        plan.nominal_finish_s=finish_time(plan,self.windows[-1],self.signs[-1],self.cfg.radius+self.cfg.frame_thickness)
        plan.solve_ms=1000*(time.perf_counter()-began)
        return plan
