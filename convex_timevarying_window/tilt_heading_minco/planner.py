"""Minimal numerical repair of the existing global PVAJ constrained planner."""
import numpy as np
from convex_timevarying_window.global_gate_minco_probe import planner as core
from convex_timevarying_window.comparisons.论文实验.实验2 import planner as original
from convex_timevarying_window.togt_minco_mpc.trajectory import basis
from . import dynamics


class Planner(original.Planner):
    def __init__(self,track,free_points=True,*,pose_function,max_iterations=2000):
        if not isinstance(max_iterations,int) or max_iterations<1:raise ValueError('positive integer max_iterations required')
        self.max_iterations=max_iterations
        previous=(core.constraints,original.pose)
        core.constraints=dynamics.constraints;original.pose=pose_function
        try:super().__init__(track,free_points)
        finally:core.constraints,original.pose=previous

    def verify(self,plan,deadline=np.inf):
        for piece in range(self.n):
            for u in np.linspace(0.,1.,self.cfg.nodes_per_piece+1):
                v,a=[plan.coefficients[piece].T@np.asarray(basis(float(u),d)).ravel()/plan.durations[piece]**d for d in (1,2)]
                health=dynamics.domain(v,a)
                if not health['passed']:
                    return [dict(gate=-1,piece=piece,passed=False,outcome='attitude_chart_unresolved',
                                 witness_u=None,u=float(u),**{k:v for k,v in health.items() if k!='passed'})],False
        return super().verify(plan,deadline)

    @staticmethod
    def literal_time_objective(log_objective,y):
        value,gradient=log_objective(y)
        return np.expm1(value),np.exp(value)*gradient

    def solve_all(self,*args,**kwargs):
        previous=core.minimize
        def literal(fun,initial,*a,**kw):
            kw['options']=dict(kw.get('options',{}),maxiter=self.max_iterations)
            return previous(lambda y:self.literal_time_objective(fun,y),initial,*a,**kw)
        core.minimize=literal
        try:
            plan,result=super().solve_all(*args,**kwargs)
            result.update(nonlinear_solver='SLSQP',max_iterations=self.max_iterations,
                numerical_objective='literal sum of segment times',original_togt_point_map_preserved=True,
                attitude_model='north_tilt_heading_pi_over_2',optimizer_convergence_required=True)
            return plan,result
        finally:core.minimize=previous
