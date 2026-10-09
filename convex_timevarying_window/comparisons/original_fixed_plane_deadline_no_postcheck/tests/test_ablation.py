from pathlib import Path
from types import SimpleNamespace
from convex_timevarying_window.comparisons.original_fixed_plane_deadline_no_postcheck import planner,px4
from convex_timevarying_window.wang_progress_mpc.planner import Planner as Original
from convex_timevarying_window.wang_free_state_mpc import px4 as OriginalPX4


def test_no_fabricated_clearance_or_geometry_evaluation():
    obj=planner.Planner.__new__(planner.Planner)
    plan=SimpleNamespace()
    assert obj.validate(plan) is None
    assert plan.post_trajectory_validation_skipped is True
    plan.clearance=None
    assert obj.prefix_clearance(plan,0.) is None


def test_optimizer_and_deadlines_and_refinement_are_inherited_unchanged():
    for name in ('_build','initial','solve','refine','terminal_position'):
        assert getattr(planner.Planner,name) is getattr(Original,name)


def test_px4_copy_has_only_declared_adapter_and_admission_changes():
    expected=Path(OriginalPX4.__file__).read_text()
    expected=expected.replace('"""Independent MINCO adapter; unchanged condensed MPC and PX4 session."""',
        '"""Simulation ablation: same MPC/PX4, no post-solve geometric/dynamic admission."""')
    expected=expected.replace('from .events import Progress, planning_block_reason',
        'from convex_timevarying_window.wang_free_state_mpc.events import Progress, planning_block_reason')
    expected=expected.replace('from .reference import Reference','from convex_timevarying_window.wang_progress_mpc.reference import Reference')
    expected=expected.replace('from .worker import Worker','from .adapter import Worker').replace('parents[2]','parents[3]')
    expected=expected.replace("                    elif clearance < cfg.tracking_margin: reason = 'geometry_or_dynamics'\n",
        '                    # Simulation ablation: post-solve geometry/dynamics was not evaluated.\n')
    expected=expected.replace('status=candidate.status if candidate else error, prefix_clearance=clearance)',
        'status=candidate.status if candidate else error, prefix_clearance=clearance,\n                             post_trajectory_validation_enabled=False)')
    assert Path(px4.__file__).read_text().strip()==expected.strip()


def test_event_and_residual_and_deadline_gates_remain():
    source=Path(px4.__file__).read_text()
    for text in ('candidate.violation >= .003','not candidate.event_valid','ref.candidate_reason(candidate, gate, progress.phase)',
                 "if elapsed >= 100.: reason = 'deadline'"):
        assert text in source
    assert 'clearance < cfg.tracking_margin' not in source
