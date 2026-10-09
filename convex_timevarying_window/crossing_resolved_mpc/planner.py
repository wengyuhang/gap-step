"""Event-centred geometry quadrature in MINCO search, without post-plan scanning."""
from dataclasses import dataclass
import math
import casadi as ca
import numpy as np
from convex_timevarying_window.wang_free_state_mpc import planner as base
from convex_timevarying_window.wang_progress_mpc.planner import Planner as DeadlinePlanner, DeadlineSolver
from convex_timevarying_window.togt_minco_mpc.planner import ScalarSolver
from convex_timevarying_window.checked_soft_racing.planner import clearance_penalty


@dataclass(frozen=True)
class Config(base.Config):
    soft_clearance_buffer: float = .25
    soft_clearance_weight: float = 10.
    crossing_half_span_s: float = .5
    crossing_max_step_s: float = .02

    def __post_init__(self):
        super().__post_init__()
        if not (0 < self.crossing_max_step_s <= self.crossing_half_span_s <= 2.):
            raise ValueError('Require 0 < crossing step <= half span <= 2 seconds')


class Planner(DeadlinePlanner):
    def __init__(self, track, cfg=Config(), *, build_indices=None):
        self.crossing_geometry = {}
        super().__init__(track,cfg,build_indices=build_indices)

    def validate(self,plan):
        # Explicitly unchecked, not a fabricated positive clearance.
        plan.post_trajectory_validation_skipped=True
        return None

    def _build(self,first):
        old,ev,lo,hi,lbg,ubg=base.Planner._build(self,first)
        cfg=self.cfg; ids=self.ids(first)
        x=ca.SX.sym('x',old.fg.size1_in(0)); par=ca.SX.sym('p',old.fg.size1_in(1))
        cost,_,residual=old.fg(x,par); g,C,objective=ev(x,par)
        legs=len(ids)+1; durations=[ca.exp(x[k])/2 for k in range(legs) for _ in range(2)]
        starts=[];stamp=par[0]
        for T in durations:starts.append(stamp);stamp+=T
        intervals=math.ceil(cfg.crossing_half_span_s/cfg.crossing_max_step_s)
        hard=cfg.radius+cfg.frame_thickness+cfg.tracking_margin
        scale=max(1.,hard*hard)
        samples=[]; violations=[]; sample_times=[]; distances=[];extra_cost=0.
        for k,wi in enumerate(ids):
            # Both sides of the centre-plane event, including the body-clearance phase.
            for piece,ending in ((2*k+1,True),(2*k+2,False)):
                T=durations[piece];span=ca.fmin(cfg.crossing_half_span_s,T)
                for j in range(intervals+1):
                    offset=span*j/intervals
                    u=1.-offset/T if ending else offset/T
                    at=starts[piece]+u*T
                    B=ca.vertcat(*[u**power for power in range(6)])
                    pos=C[6*piece:6*piece+6,:].T@B
                    sq=base.symbolic_clearance_sq(self.frames[wi],pos,at)
                    samples.append(sq/scale);violations.append(ca.fmax(hard*hard-sq,0.)/scale)
                    sample_times.append(at);distances.append(sq)
                    weight=span/intervals*(.5 if j in (0,intervals) else 1.)
                    extra_cost+=weight*clearance_penalty(sq,cfg)
        v=ca.vertcat(0.,*violations)
        total=cost+extra_cost+cfg.constraint_penalty*ca.sum1(v**3)
        fg=ca.Function(f'crossing_fg_{first}',[x,par],[total,ca.gradient(total,x),ca.fmax(residual,ca.mmax(v))])
        evaluate=ca.Function(f'crossing_out_{first}',[x,par],[ca.vertcat(g,*samples),C,objective+extra_cost])
        self.crossing_geometry[first]=ca.Function(f'crossing_geometry_{first}',[x,par],
            [ca.vertcat(*sample_times),ca.vertcat(*distances),v])
        lower=np.r_[lbg,np.full(len(samples),hard*hard/scale)]
        upper=np.r_[ubg,np.full(len(samples),np.inf)]
        # Feed the augmented residual to the same best-feasible iterate selection and final admission.
        solver=DeadlineSolver(ScalarSolver(fg,None,cfg),self,first)
        return solver,evaluate,lo,hi,lower,upper
