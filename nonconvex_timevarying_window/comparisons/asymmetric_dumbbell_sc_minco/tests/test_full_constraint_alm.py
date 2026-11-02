import numpy as np

from nonconvex_timevarying_window.comparisons.asymmetric_dumbbell_sc_minco.full_constraint_alm.solver import augmented_value_gradient


def test_phr_sign_and_chain_gradient_for_g_greater_equal_zero():
    jac=np.array([[1.,2.],[-3.,1.],[.2,-.4]])
    bias=np.array([.1,-.2,3.])
    lam=np.array([.7,.3,.1]);rho=2.
    x=np.array([.1,-.3])
    def fun(z):
        return augmented_value_gradient(.5*z@z,z,bias+jac@z,jac,lam,rho)
    value,grad=fun(x)
    numeric=np.array([(fun(x+np.eye(2)[k]*1e-6)[0]-fun(x-np.eye(2)[k]*1e-6)[0])/2e-6 for k in range(2)])
    assert np.isfinite(value)
    np.testing.assert_allclose(grad,numeric,atol=1e-8)


def test_same_full_objective_constraints_and_analytic_derivatives():
    from convex_timevarying_window.single_gate_stop_racing.planner import Config
    from nonconvex_timevarying_window.sc_dynatogt.sc_mapping import SCDiskMap
    from nonconvex_timevarying_window.comparisons.asymmetric_dumbbell_sc_minco.scenario import VERTICES,build_case
    from nonconvex_timevarying_window.comparisons.asymmetric_dumbbell_sc_minco.planner import Planner as Original
    from nonconvex_timevarying_window.comparisons.asymmetric_dumbbell_sc_minco.full_constraint_alm.solver import Planner,initial_variables
    cfg=Config(compile_objective=False,max_solve_ms=30000.)
    track=build_case(); mapping=SCDiskMap.fit(VERTICES)
    original=Original(track,cfg,mapping); candidate=Planner(track,cfg,mapping)
    state=np.r_[track.start,np.zeros(9)]
    x=initial_variables(candidate,state,0.)
    offsets=np.tile(np.r_[cfg.passage_seed_offsets,np.zeros(cfg.witness_slots-len(cfg.passage_seed_offsets))],(2,1))
    par=np.r_[0.,state,offsets.ravel()]
    for displacement in [np.zeros(7),np.array([.02,-.03,.01,-.02,.1,.2,-.1])]:
        a=original.function(x+displacement,par)
        b=candidate.function(x+displacement,par)
        for av,bv in zip(a,b):
            np.testing.assert_array_equal(np.asarray(av),np.asarray(bv))
        assert np.asarray(a[2]).size == 1155
