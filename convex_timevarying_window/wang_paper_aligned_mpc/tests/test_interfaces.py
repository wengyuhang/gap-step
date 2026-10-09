from types import SimpleNamespace
import numpy as np
import pytest
from convex_timevarying_window.comparisons.fixed_plane_wang.scenario import Case,build_case
from convex_timevarying_window.wang_paper_aligned_mpc.events import Progress,planning_block_reason
from convex_timevarying_window.wang_paper_aligned_mpc.mpc import MPC,covered_until
from convex_timevarying_window.wang_center_seed_mpc.reference import Reference
from convex_timevarying_window.checked_soft_mpc_racing.mpc import hover_state


def test_remote_plane_is_not_a_missed_window_or_progress():
    track,_=build_case(Case(.8,1.6,.4)); w=track.windows[0]
    center,R,*_=w.state_at(0.)
    progress=Progress([w],[1.],.386,.085)
    before=center+R@np.array([20.,0.,1.]);after=center+R@np.array([20.,0.,-1.])
    assert not progress.update(before,after,0.,0.)
    assert progress.violation is None and progress.gate==0 and not progress.crossed
    ref=SimpleNamespace(plan_at=lambda t:SimpleNamespace(first_gate=1,gate_count=0,
        sample=lambda t:after,crossing_times=[]))
    assert planning_block_reason(progress,ref,.15,.12) is None


def test_near_edge_miss_is_still_rejected():
    track,_=build_case(Case(.8,1.6,.4));w=track.windows[0];center,R,*_=w.state_at(0.)
    progress=Progress([w],[1.],.386,.085)
    q=np.asarray(w.aperture.physical_vertices[0])
    before=center+R@np.r_[q,1.];after=center+R@np.r_[q,-1.]
    assert not progress.update(before,after,0.,0.)
    assert progress.violation=='missed_current_opening'


class FakeReferencePlan:
    made_at=0.; end_time=.62; terminal_goal=False; first_gate=0; gate_count=1
    def sample(self,t,d=0):
        if t>self.end_time+1e-8:raise RuntimeError('requested undefined reference')
        return np.array([0.,0.,4.]) if d==0 else np.zeros(3)


def test_mpc_uses_short_valid_reference_without_extrapolation():
    ref=Reference(np.array([0.,0.,4.]));ref.active=FakeReferencePlan();mpc=MPC()
    u=mpc.command(hover_state(ref.start),.1,ref.sample)
    assert np.isfinite(u).all()
    assert .49<mpc.logs[-1]['reference_horizon_s']<.51
    assert mpc.cfg.dt==.04  # Nominal config stays immutable outside a call.
    with pytest.raises(RuntimeError,match='insufficient_valid_reference'):
        mpc.command(hover_state(ref.start),.5,ref.sample)


def test_contiguous_pending_extends_coverage_but_gap_does_not():
    ref=Reference(np.zeros(3));ref.active=FakeReferencePlan()
    ref.pending=SimpleNamespace(made_at=.5,end_time=2.,terminal_goal=False)
    assert covered_until(ref,.1)==2.
    ref.pending.made_at=.7
    assert covered_until(ref,.1)==.62
