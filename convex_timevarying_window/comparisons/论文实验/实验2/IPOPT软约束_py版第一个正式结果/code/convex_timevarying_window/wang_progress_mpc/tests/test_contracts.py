from types import SimpleNamespace
import json
from pathlib import Path

import numpy as np
import pytest

from convex_timevarying_window.wang_progress_mpc.contracts import shared_deadlines,comparison
from convex_timevarying_window.wang_progress_mpc.reference import Reference
from convex_timevarying_window.wang_free_state_mpc.reference import Reference as OldReference
from convex_timevarying_window.comparisons.fixed_plane_wang.diagnose import plan_for


def plan(first,times,end=20.):
    return SimpleNamespace(first_gate=first,gate_count=len(times),crossing_times=np.array(times),
        made_at=2.,end_time=end,sample=lambda t,d=0:np.zeros(3))


def test_advance_compares_shared_window_not_horizon_number():
    old=plan(2,[7.,9.]);new=plan(3,[12.,14.],end=22.)
    ref=Reference(np.zeros(3));ref.active=old
    assert ref.candidate_reason(new,3,'approach')=='delayed_shared_passage:3'
    base=OldReference(np.zeros(3));base.active=old
    assert base.candidate_reason(new,3,'approach') is None


def test_extension_can_have_later_terminal_but_not_delayed_common_event():
    ref=Reference(np.zeros(3));ref.active=plan(2,[7.,9.],end=10.)
    assert ref.candidate_reason(plan(3,[8.99,12.],end=15.),3,'approach') is None


def test_every_shared_event_is_checked_and_tolerance_is_preserved():
    ref=Reference(np.zeros(3));ref.active=plan(2,[7.,9.])
    assert ref.candidate_reason(plan(2,[6.9,9.019]),2,'approach') is None
    assert ref.candidate_reason(plan(2,[6.9,9.03]),2,'approach')=='delayed_shared_passage:3'


def test_no_incumbent_or_no_overlap_does_not_invent_deadline():
    assert np.isinf(shared_deadlines(None,[0,1])).all()
    np.testing.assert_equal(shared_deadlines(plan(2,[7.,9.]),[3,4]),[9.02,np.inf])
    assert comparison(plan(0,[1.]),plan(2,[3.]))==[]


def test_saved_bad_transitions_are_rejected_by_identity():
    root=Path(__file__).resolve().parents[3]
    path=root/'convex_timevarying_window/comparisons/fixed_plane_wang/results/pilot_20261007/case00_ours_attempt1/result.json'
    if not path.exists():pytest.skip('Local development trace unavailable')
    raw=json.loads(path.read_text());accepted=[plan_for(s) for s in raw['solves'] if s['accepted']]
    found=[]
    for old,new in zip(accepted[:-1],accepted[1:]):
        if old.first_gate==new.first_gate:continue
        ref=Reference(np.zeros(3));ref.active=old
        reason=ref.candidate_reason(new,new.first_gate,'approach')
        common=comparison(old,new)
        if any(r['delay_s']>.02 for r in common):
            assert reason.startswith('delayed_shared_passage:')
            found.append(new.first_gate)
    assert 3 in found and 5 in found
