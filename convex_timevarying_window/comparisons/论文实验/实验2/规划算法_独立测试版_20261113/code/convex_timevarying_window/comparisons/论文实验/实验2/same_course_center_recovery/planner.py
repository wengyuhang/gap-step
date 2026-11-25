"""Centre-first initialization on the same complete dynamic course."""
from dataclasses import asdict, replace
import time
import numpy as np
from scipy.optimize import minimize
from convex_timevarying_window.single_gate_stop_racing import point_mapping
from ..independent_repair.smooth_augmented import Planner as BasePlanner


def embed_center(planner, center_variables):
    center_variables = np.asarray(center_variables)
    if len(center_variables) != planner.n+9*planner.m:
        raise ValueError('expected complete centre PVAJ variables')
    points = np.concatenate([point_mapping.initial(w.aperture) for w in planner.track.windows])
    return np.r_[center_variables[:planner.n], points, center_variables[planner.n:]]


class Planner(BasePlanner):
    def refine_center(self, center, center_statistics, seconds_per_seed=300., progress=lambda r: None):
        x = embed_center(self, center.x)
        g, _, coefficients = self.evaluate(x, self.witnesses())
        np.testing.assert_allclose(coefficients, center.coefficients, rtol=1e-10, atol=1e-7)
        if g.min() < -1e-5:
            raise ValueError('centre initializer violates the original free-graph constraints')
        incumbent = replace(center, x=x.copy())
        checks, passed = self.verify(incumbent)
        if not passed:
            raise ValueError('centre initializer failed complete same-course verification')
        original_seeds = self.seeds
        original_backend = self.augmented_minimize

        def direct(fun, initial, **kwargs):
            initial_log_value, _ = fun(initial)
            normalization = 1.+np.expm1(initial_log_value)
            def objective(y):
                value, gradient = fun(y)
                return np.expm1(value)/normalization, np.exp(value)*gradient/normalization
            result = minimize(objective, initial, jac=True, method='SLSQP', bounds=kwargs['bounds'],
                constraints=kwargs['constraints'], callback=kwargs['callback'],
                options=dict(kwargs['options'], maxiter=self.max_iterations, ftol=1e-5))
            minimum = float(kwargs['constraints']['fun'](result.x).min())
            self.optimizer_calls.append(dict(stage='same_course_center_direct_sqp', success=bool(result.success),
                status=str(result.message), iterations=int(result.nit), minimum_constraint=minimum))
            if result.success and minimum >= -1e-5:
                return result
            return original_backend(fun, initial, **kwargs)

        self.seeds = lambda: [x.copy()]
        self.augmented_minimize = direct
        try:
            candidate, free_statistics = BasePlanner.solve_all(self, seconds_per_seed, progress)
        finally:
            self.seeds = original_seeds
            self.augmented_minimize = original_backend
        free_statistics.update(all_four_attempted=False, independent_cold_start=False,
                               same_course_center_initialization=True)
        improved = candidate is not None and candidate.durations.sum() < incumbent.durations.sum()-1e-8
        selected = candidate if improved else incumbent
        selected.solve_ms = 1000*(center_statistics['solve_s']+free_statistics['solve_s'])
        return selected, dict(found=True, algorithm='same_course_center_first',
            independent_course=True, cross_course_warm_start=False, amplitude_continuation=False,
            solve_s=center_statistics['solve_s']+free_statistics['solve_s'],
            center_stage=center_statistics, free_stage=free_statistics,
            center_time_s=float(center.durations.sum()),
            free_candidate_time_s=float(candidate.durations.sum()) if candidate else None,
            free_refinement_converged=candidate is not None,
            selected_stage='free_refinement' if improved else 'center_incumbent',
            returned_optimizer_scope='free_full_space' if improved else 'center_restricted',
            initial_curve_max_error=float(np.max(abs(coefficients-center.coefficients))),
            free_initial_variables=x.tolist(), center_reference=dict(variables=center.x.tolist(),
                durations=center.durations.tolist(), coefficients=center.coefficients.tolist()),
            all_56_initial_geometry_passed=len(checks) == 56, nominal_nonworsening=True)

    def solve_all(self, seconds_per_seed=300., progress=lambda r: None):
        if not self.free_points:
            return BasePlanner.solve_all(self, seconds_per_seed, progress)
        began = time.perf_counter()
        center_solver = BasePlanner(self.track, False, pose_function=self.pose_function,
                                    max_iterations=self.max_iterations)
        center_build_s = time.perf_counter()-began
        center, center_statistics = center_solver.solve_all(seconds_per_seed,
            lambda r: progress(dict(optimization_stage='center', **r)))
        if center is not None:
            selected, result = self.refine_center(center, center_statistics, seconds_per_seed,
                lambda r: progress(dict(optimization_stage='free_refinement', **r)))
        else:
            selected, free_statistics = BasePlanner.solve_all(self, seconds_per_seed,
                lambda r: progress(dict(optimization_stage='free_cold_fallback', **r)))
            result = dict(found=selected is not None, algorithm='same_course_center_first',
                independent_course=True, cross_course_warm_start=False, amplitude_continuation=False,
                center_stage=center_statistics, free_stage=free_statistics,
                selected_stage='free_cold_fallback' if selected else None,
                solve_s=center_statistics['solve_s']+free_statistics['solve_s'],
                free_refinement_converged=selected is not None, nominal_nonworsening=None)
        result['center_build_s'] = center_build_s
        result['center_config'] = asdict(center_solver.cfg)
        if selected is not None:
            selected.solve_ms = result['solve_s']*1000
        return selected, result
