from dataclasses import replace
import numpy as np
from convex_timevarying_window.single_gate_stop_racing.planner import Config
from nonconvex_timevarying_window.comparisons.asymmetric_dumbbell_sc_minco.spin.ipopt_benchmark import IpoptMinimize


def test_reusable_ipopt_preserves_inequality_sign_bounds_and_changed_objective():
    adapter=IpoptMinimize(1,replace(Config(),max_solve_ms=5000.,constrained_iterations=70))
    for target,minimum,expected in ((-1.,.5,.5),(1.5,.5,1.5)):
        centre=np.r_[target,np.zeros(6)]
        def objective(y): return float(np.sum((y-centre)**2)),2*(y-centre)
        constraint=dict(type='ineq',fun=lambda y:np.array([y[0]-minimum]),
                        jac=lambda y:np.array([[1.,0,0,0,0,0,0]]))
        result=adapter(objective,np.zeros(7),method='SLSQP',jac=True,
            bounds=[(-2.,2.)]*7,constraints=constraint,callback=lambda y:None,options={})
        assert result.success,result.message
        np.testing.assert_allclose(result.x,np.r_[expected,np.zeros(6)],atol=1e-5)
        assert constraint['fun'](result.x).min()>=-1e-7
