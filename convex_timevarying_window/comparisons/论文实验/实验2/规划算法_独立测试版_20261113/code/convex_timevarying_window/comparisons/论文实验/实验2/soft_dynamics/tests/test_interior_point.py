import casadi as ca
import numpy as np

from convex_timevarying_window.comparisons.论文实验.实验2.soft_dynamics.interior_point import geometric_constraints
from convex_timevarying_window.comparisons.论文实验.实验2.soft_dynamics.symbolic_penalty import positive_penalty
from convex_timevarying_window.comparisons.论文实验.实验2.soft_dynamics.planner import Planner
from convex_timevarying_window.comparisons.论文实验.实验2.basic_group.whole_course_difficulty import scenario


def test_violation_variables_preserve_soft_objective_optimum():
    x = ca.MX.sym('epigraph_equivalence', 2)
    value = x[0]+1000.*positive_penalty(x[1])
    solver = ca.nlpsol('epigraph_equivalence_solver', 'ipopt',
        dict(x=x, f=value, g=x[1]-(2.-x[0])),
        {'ipopt.print_level': 0, 'ipopt.sb': 'yes', 'print_time': False,
         'ipopt.hessian_approximation': 'exact'})
    result = solver(x0=[1., 1.], lbx=[0., 0.], ubx=[3., np.inf], lbg=0., ubg=np.inf)
    assert solver.stats()['return_status'] == 'Solve_Succeeded'
    point = np.asarray(result['x']).ravel()
    direct = point[0]+1000.*float(positive_penalty(2.-point[0]))
    np.testing.assert_allclose(float(result['f']), direct, atol=1e-6)
    assert 1.99 < point[0] < 2.


def test_compressed_soft_geometry_preserves_all_constraint_violations():
    track, _ = scenario.build_case(scenario.Case(level='mixed_base'))
    planner = Planner(track, pose_function=scenario.pose)
    x = ca.MX.sym('geometry_equivalence', planner.nvar)
    witnesses = planner.witnesses()
    expression, upper = geometric_constraints(planner, x, witnesses)
    evaluate = ca.Function('geometry_equivalence_values', [x], [expression])
    rng = np.random.default_rng(20)
    for initial in planner.seeds()[:2]:
        for seed in [initial, initial+rng.normal(size=len(initial))*.03]:
            full = planner.evaluate(seed, witnesses)[0]
            compressed = np.asarray(evaluate(seed)).ravel()
            minimum = min(float(compressed.min()), float((upper-compressed).min()))
            np.testing.assert_allclose(minimum, full.min(), atol=1e-9, rtol=1e-9)
