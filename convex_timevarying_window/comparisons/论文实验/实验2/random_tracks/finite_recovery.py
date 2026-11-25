"""Bounded recovery of the same finite NLP; never label failure ready for exchange."""
from importlib import import_module


def _clear_solver_state(planner):
    # The cached graph embeds a Hessian choice. Rebuild it when switching choices.
    for name in ['_fast_warm','_fast_problem','_fast_cold_problem']:
        planner.__dict__.pop(name,None)


def minimize(planner,fun,initial,*,primary_module='finite_direct',**kwargs):
    exact=bool(getattr(planner,'_prefer_exact_hessian',False))
    module=import_module('finite_direct_exact' if exact else primary_module)
    result=module.minimize(planner,fun,initial,**kwargs)
    planner.optimizer_calls[-1]['finite_recovery_role']='primary'
    if result.success:
        return result
    failed_status=planner.optimizer_calls[-1]['status']
    # Keep the failed solver's physical curve, but discard its multipliers,
    # barrier state and auxiliary slack initialization. Constraints stay fixed.
    _clear_solver_state(planner)
    planner._prefer_exact_hessian=True
    result=import_module('finite_direct_exact').minimize(planner,fun,result.x,**kwargs)
    planner.optimizer_calls[-1].update(finite_recovery_role='cold_exact_recovery',
        preceding_failure=failed_status,failed_duals_discarded=True)
    if not result.success:
        # The old outer loop retries an unchanged, geometry-safe failed NLP.
        # Fail explicitly here; it must not advance or repeat exchange as success.
        result.verification_ready=False
        _clear_solver_state(planner)
    return result
