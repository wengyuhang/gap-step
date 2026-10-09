from types import SimpleNamespace
import numpy as np
import pytest
from convex_timevarying_window.anytime_continuation_mpc.reference import Reference


def stub(bootstrap,first=0):
    return SimpleNamespace(is_bootstrap=bootstrap,first_gate=first,made_at=.15,end_time=5.,gate_count=2 if bootstrap else 7,
        nominal_finish_s=None if bootstrap else 25.,sample=lambda t,d:np.zeros(3))


def test_initial_local_candidate_does_not_fabricate_finish():
    r=Reference(np.zeros(3));p=stub(True)
    assert r.candidate_reason(p,0,'approach') is None
    assert r.contract_log[-1]['new_finish_s'] is None


def test_local_to_global_keeps_head_but_has_no_false_finish_bound():
    r=Reference(np.zeros(3));r.active=stub(True)
    assert r.candidate_reason(stub(False),0,'approach') is None
    assert r.contract_log[-1]['bootstrap_transition']
    candidate=stub(False);candidate.sample=lambda t,d:np.ones(3)
    assert r.candidate_reason(candidate,0,'approach')=='head_timeline_changed'


def test_no_fake_hover_on_bootstrap_expiry():
    r=Reference(np.zeros(3));r.active=stub(True)
    with pytest.raises(RuntimeError,match='bootstrap_reference_exhausted'):r.state_at(5.1)


def test_complete_plan_still_has_finish_contract():
    r=Reference(np.zeros(3));r.active=stub(False);candidate=stub(False);candidate.nominal_finish_s=26.
    assert r.candidate_reason(candidate,0,'approach')=='delayed_race_finish'
