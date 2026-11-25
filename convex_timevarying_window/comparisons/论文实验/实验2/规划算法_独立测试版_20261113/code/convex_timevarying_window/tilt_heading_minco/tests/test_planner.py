import casadi as ca
import numpy as np
from convex_timevarying_window.tilt_heading_minco.planner import Planner
from convex_timevarying_window.comparisons.论文实验.实验2.basic_group.whole_course_difficulty import scenario
from convex_timevarying_window.global_gate_minco_probe import planner as core
from convex_timevarying_window.comparisons.论文实验.实验2 import planner as original
from convex_timevarying_window.tilt_heading_minco import dynamics


def test_literal_time_objective_is_equivalent_and_correctly_differentiated():
    y=np.array([1.,1.5,-.3])
    def log(x):
        T=np.exp(x);return np.log1p(T.sum()),T/(1+T.sum())
    value,gradient=Planner.literal_time_objective(log,y)
    np.testing.assert_allclose(value,np.exp(y).sum(),atol=1e-12)
    np.testing.assert_allclose(gradient,np.exp(y),atol=1e-12)
    for i in range(3):
        d=np.eye(3)[i]*1e-6
        numeric=(Planner.literal_time_objective(log,y+d)[0]-Planner.literal_time_objective(log,y-d)[0])/2e-6
        np.testing.assert_allclose(numeric,gradient[i],rtol=1e-8)


def test_formal_module_preserves_original_map_and_other_model_construction():
    tr,_=scenario.build_case(scenario.Case(level='a25_f3'))
    a=Planner(tr,True,pose_function=scenario.pose)
    previous=(core.constraints,original.pose)
    core.constraints=dynamics.constraints;original.pose=scenario.pose
    try:b=original.Planner(tr,True)
    finally:core.constraints,original.pose=previous
    assert a.nvar==b.nvar==96 and a.dims==[4,2,5,2,6,2,4]
    assert a.cfg==b.cfg and a.max_iterations==2000
    for x,y in zip(a.seeds(),b.seeds()):
        np.testing.assert_array_equal(x,y)
        g,J,C=a.evaluate(x,a.witnesses());h,K,D=b.evaluate(y,b.witnesses())
        np.testing.assert_array_equal(g,h);np.testing.assert_array_equal(J,K);np.testing.assert_array_equal(C,D)
