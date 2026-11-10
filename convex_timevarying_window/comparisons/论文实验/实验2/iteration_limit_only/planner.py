"""Only change SLSQP's iteration cap; original model and objective preserved."""
import inspect
import numpy as np
from .. import planner as original
from convex_timevarying_window.global_gate_minco_probe import planner as core


class Planner(original.Planner):
    def __init__(self, track, free_points=True, *, pose_function, max_iterations=2000):
        if not isinstance(max_iterations, int) or max_iterations < 1:
            raise ValueError('positive iteration limit required')
        self.max_iterations = max_iterations
        previous = original.pose
        original.pose = pose_function
        try:
            super().__init__(track, free_points)
        finally:
            original.pose = previous

    def solve_all(self, *args, **kwargs):
        previous = core.minimize
        calls = []

        def bounded_iterations(fun, initial, *a, **kw):
            assert kw.get('method') == 'SLSQP'
            before = dict(kw['options'])
            after = dict(before, maxiter=self.max_iterations)
            assert before == dict(maxiter=400, ftol=1e-7, disp=False)
            kw['options'] = after
            result = previous(fun, initial, *a, **kw)
            context = inspect.getclosurevars(fun).nonlocals
            calls.append(dict(options_before=before, options_after=after,
                status=str(result.message), optimizer_converged=bool(result.success), iterations=int(result.nit),
                terminal_variables=(context['origin']+context['scale']*result.x).tolist()))
            return result

        core.minimize = bounded_iterations
        try:
            plan, result = super().solve_all(*args, **kwargs)
            result.update(only_algorithm_change='SLSQP maxiter400->'+str(self.max_iterations),
                max_iterations=self.max_iterations,independent_cold_start=True,
                amplitude_continuation=False,cross_condition_warm_start=False,backend_calls=calls)
            return plan, result
        finally:
            core.minimize = previous
