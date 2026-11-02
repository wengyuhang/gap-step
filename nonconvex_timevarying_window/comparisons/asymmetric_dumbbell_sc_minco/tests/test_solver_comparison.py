import casadi as ca
import numpy as np
from nonconvex_timevarying_window.comparisons.asymmetric_dumbbell_sc_minco.compare_solvers import NLPCallback


def test_ipopt_callback_passes_the_exact_objective_and_constraint_jacobian():
    def evaluate(y):
        g = np.array([np.sin(y[0])+y[1]**2, y[2]*y[3]])
        jac = np.zeros((2,7))
        jac[0,:2] = [np.cos(y[0]),2*y[1]]
        jac[1,2:4] = [y[3],y[2]]
        return y@y,2*y,g,jac
    callback = NLPCallback('callback_derivative_test',evaluate,2)
    symbolic = ca.MX.sym('y',7)
    output = callback(symbolic)
    function = ca.Function('callback_derivative_check',[symbolic],[output,ca.jacobian(output,symbolic)])
    y = np.array([.3,-.2,.5,.7,1.,-.4,.1])
    value,jacobian = function(y)
    cost,gradient,g,jac = evaluate(y)
    np.testing.assert_allclose(np.asarray(value).ravel(),np.r_[cost,g],atol=1e-13)
    np.testing.assert_allclose(np.asarray(jacobian),np.vstack([gradient,jac]),atol=1e-13)
