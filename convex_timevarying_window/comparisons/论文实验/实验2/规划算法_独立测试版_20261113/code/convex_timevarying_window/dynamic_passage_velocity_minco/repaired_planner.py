"""Execution repair: attainable thrust in optimization and a true goal leg."""
from dataclasses import dataclass
import time
import casadi as ca
import numpy as np
from scipy.optimize import minimize
from .planner import Planner as Parent, Config as ParentConfig
from .trajectory import Plan
from convex_timevarying_window.dynamic_passage_minco.planner import Stats, BudgetExpired
from convex_timevarying_window.dynamic_passage_minco.safety import power_to_bernstein
from convex_timevarying_window.togt_minco_mpc.trajectory import minco_coefficients, basis
from convex_timevarying_window.checked_soft_mpc_racing.mpc import MASS, G, KF, UPPER


@dataclass(frozen=True)
class Config(ParentConfig):
    initial_tail_scale: float = 1.
    forward_tail: bool = True
    log_cost_scaling: bool = True
    physical_thrust_fraction: float = 1.
    hard_passage_thrust: bool = False
    full_task_error_recovery: bool = True
    abort_missed_gate: bool = True


def _flat_force_expression(velocity, acceleration):
    force=acceleration+ca.DM([0.,0.,G])
    for _ in range(3):
        axis=force/ca.sqrt(ca.sumsqr(force)+1e-12)
        motor=ca.sqrt(MASS*ca.sqrt(ca.sumsqr(force)+1e-12)/(4*KF))
        drag=4*8.06428e-5*motor/MASS
        force=acceleration+ca.DM([0.,0.,G])+drag*(velocity-ca.dot(velocity,axis)*axis)
    return MASS*force


_v,_a=ca.SX.sym('flat_velocity',3),ca.SX.sym('flat_acceleration',3)
flat_force=ca.Function('drag_corrected_flat_force',[_v,_a],[_flat_force_expression(_v,_a)])


class Planner(Parent):
    def __init__(self,track,cfg=Config(),**kwargs):
        super().__init__(track,cfg,**kwargs)
        self.solvers.append((Stats(),))
        logt=ca.MX.sym('goal_logt');head=ca.MX.sym('goal_head',12);T=ca.exp(logt)
        C=minco_coefficients(ca.reshape(head,3,4),ca.horzcat(ca.DM(track.goal),ca.DM.zeros(3,3)),
                             ca.DM.zeros(3,0),ca.vertcat(T))
        def state(piece,u,d=0):return C.T@basis(u,d)/T**d
        constraints=self.thrust_constraints(state,ca.vertcat(T))
        # A completed window stays behind the whole body throughout the goal leg.
        normal=-self.signs[0]*self.rotation[:,2]
        power=C@ca.DM(normal);power[0]-=float(track.windows[0].center0@normal)
        control=ca.DM(power_to_bernstein(np.eye(8)))@power
        constraints.extend(ca.vertsplit(control-(cfg.radius+cfg.frame_thickness)))
        self.goal_function=ca.Function('goal_eval',[logt,head],[T,ca.gradient(T,logt),
            ca.vertcat(*constraints),ca.jacobian(ca.vertcat(*constraints),logt),C],
            {'cse':True,'jit':cfg.compile_objective,'compiler':'shell',
             'jit_options':{'flags':[cfg.compiler_optimization],'directory':self._jit_dir.name+'/'}})

    def extra_constraints(self,state,times,coefficients):
        return self.thrust_constraints(state,times) if self.cfg.hard_passage_thrust else []

    def extra_dynamic_residuals(self,state,times):
        return [ca.fmax(-g,0.) for g in self.thrust_constraints(state,times)]

    def thrust_constraints(self,state,times):
        constraints=[];limit=UPPER[0]*self.cfg.physical_thrust_fraction
        for piece in range(int(times.numel())):
            for u in np.linspace(0.,1.,self.cfg.nodes_per_piece+1):
                force=flat_force(state(piece,float(u),1),state(piece,float(u),2))
                constraints.append(1.-ca.sumsqr(force)/limit**2)
        return constraints

    def order_seeds(self,seeds,state,now):
        offsets=np.tile(np.r_[[0.,.02,.04,.06,.08,.10,.14,.20],np.zeros(self.cfg.witness_slots-8)],2)
        par=np.r_[now,state,offsets]
        return sorted(seeds,key=lambda x:float(self.function(np.clip(x,self.lower,self.upper),par)[0]))

    def solve(self,state,now,first):
        state=np.asarray(state)
        if len(state)==9:state=np.r_[state,np.zeros(3)]
        if first==1:return self.goal(state,now)
        if first!=0:raise ValueError('invalid measured gate index')
        signed=self.signs[0]*float((state[:3]-self.windows[0].center0)@self.rotation[:,2])
        if signed<=0:raise RuntimeError('head_beyond_unpassed_window_no_reverse_reentry')
        return super().solve(state,now,first)

    def goal(self,state,now):
        began=time.perf_counter()
        deadline=float('inf') if self.cfg.max_solve_ms is None else began+self.cfg.max_solve_ms/1000
        initial=max(.5,np.linalg.norm(self.track.goal-state[:3])/5.+.5,
                    np.linalg.norm(state[3:6])/6.)
        cached_x=None;out=None
        def evaluate(x):
            nonlocal cached_x,out
            if time.perf_counter()>=deadline:raise BudgetExpired()
            if cached_x is None or not np.array_equal(cached_x,x):
                out=self.goal_function(x[0],state);cached_x=x.copy()
            return out
        result=minimize(lambda x:(float(evaluate(x)[0]),np.array(evaluate(x)[1]).ravel()),
            [np.log(initial)],jac=True,method='SLSQP',bounds=[(np.log(.25),np.log(15.))],
            constraints={'type':'ineq','fun':lambda x:np.array(evaluate(x)[2]).ravel(),
                         'jac':lambda x:np.array(evaluate(x)[3])},
            options={'maxiter':70,'ftol':1e-7})
        out=evaluate(result.x)
        if not result.success or np.min(np.array(out[2])) < -1e-5:
            raise RuntimeError('no_attainable_goal_leg_under_budget')
        elapsed=1000*(time.perf_counter()-began)
        plan=Plan(now,1,np.array([np.exp(result.x[0])]),np.array(out[4]).reshape(1,8,3),
                  result.x.copy(),elapsed,0.,'attainable_goal_leg',0)
        stats=dict(iter_count=int(result.nit),optimization_ms=elapsed,converged=True,
                   final_objective=float(out[0]),minimum_constraint=float(np.min(np.array(out[2]))),
                   return_status='attainable_goal_leg',formal_certificate=False)
        self.solvers[1][0].value=stats;plan.optimizer_stats=stats
        plan.event_valid=True;plan.event_reason='body_already_exited_goal_recovery';plan.terminal_gate=None
        return plan
