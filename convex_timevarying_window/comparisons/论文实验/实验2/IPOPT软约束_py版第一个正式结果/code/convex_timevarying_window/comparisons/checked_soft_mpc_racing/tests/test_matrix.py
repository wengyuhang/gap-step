import numpy as np
import pytest
from convex_timevarying_window.comparisons.checked_soft_mpc_racing.run import conditions,condition_rows,control_audit
from convex_timevarying_window.comparisons.checked_soft_mpc_racing.report import groups,reason


def test_complete_union():
    cases=conditions()
    assert len(cases)==len(set(cases))==38
    assert (10,4,1,1) in cases
    assert all((r,1,1,2) in cases for r in (1,3,6,10))


def test_only_setup_retry_is_selectable():
    a=dict(condition_id=[1,1,1,0],attempt=1,infrastructure_failure=True,realtime_success=False)
    b=dict(a,attempt=2,infrastructure_failure=False,realtime_success=False)
    assert condition_rows([a,b])==[b]
    with pytest.raises(ValueError): condition_rows([dict(a,infrastructure_failure=False),b])


def test_sender_and_failed_cycles_count_toward_budget():
    r=dict(mpc_cycles=[dict(time=0.,cycle_ms=10.),dict(time=.02,cycle_ms=110.,failure='timeout')])
    audit=control_audit(r,np.array([[0.,5.]]))
    assert not audit['controller_deadline_passed'] and audit['mpc_failed_calls']==1
    r['mpc_cycles'][1]['cycle_ms']=20.
    assert not control_audit(r,np.array([[0.,101.]]))['controller_deadline_passed']
    assert control_audit(r,np.array([[0.,6.]]))['controller_deadline_passed']


def test_report_groups_are_same_38_condition_union():
    group=groups()
    assert {k:len(v) for k,v in group.items()}==dict(center24=24,amplitude16=16,rate12=12)
    assert set.union(*group.values())==set(conditions())


def test_noncompletion_is_not_an_infrastructure_retry():
    assert reason(dict(failure='timeout',infrastructure_failure=False))=='60s未完成'
    assert reason(dict(failure='MPC QP maximum iterations reached'))=='MPC数值求解中止'
