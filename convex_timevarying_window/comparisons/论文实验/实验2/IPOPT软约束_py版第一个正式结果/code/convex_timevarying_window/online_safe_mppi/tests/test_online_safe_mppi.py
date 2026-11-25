import numpy as np
import torch
from dataclasses import replace

from convex_timevarying_window.online_safe_mppi.experiment import (
    BODY_RADIUS,
    Config,
    Course,
    GatePointPlanner,
    LocalSafeMPPI,
    command_is_one_step_safe,
    corridor_path_length,
    integrate_step,
    locally_relevant_gates,
    online_deadline_satisfied,
    passage,
    ordered_plane_filter,
    plan_point_at,
    select_verified_command,
    workspace_barrier_filter,
    torch_dynamics_step,
)
from convex_timevarying_window.conditional_dual_constraint_cem.safety_penalty import (
    polygon_halfspaces,
)


def test_planned_local_points_respect_shrunken_apertures():
    course = Course(torch.device("cpu"))
    cfg = Config(rollouts=8, rollout_steps=3)
    state = np.zeros(12)
    state[:3] = course.start
    planner = GatePointPlanner(course, cfg)
    for gate_index in range(len(course.windows)):
        plan = planner.plan(state, 0.0, gate_index, "test", np.inf)
        safe_kind, safe_aperture = course.safe_local_polygon(
            gate_index,
            BODY_RADIUS + cfg.safety_margin + cfg.swept_margin + 0.025
            + (0.18 if gate_index in (4, 5, 6) else 0.0),
        )
        if safe_kind == "circle":
            assert np.linalg.norm(plan.point_local) <= safe_aperture + 1e-9
        else:
            halfspaces = polygon_halfspaces(safe_aperture)
            assert np.all(halfspaces.normals @ plan.point_local
                          <= halfspaces.offsets + 1e-9)
        assert np.linalg.norm(plan.point_local) <= cfg.gate_point_trust_radius + 1e-9


def test_dynamic_gate_point_follows_gate_motion():
    course = Course(torch.device("cpu"))
    cfg = Config(rollouts=8, rollout_steps=3)
    state = np.zeros(12)
    state[:3] = course.start
    plan = GatePointPlanner(course, cfg).plan(state, 0.0, 0, "test", np.inf)
    p0 = plan_point_at(course, plan, 0.0)
    p1 = plan_point_at(course, plan, 1.0)
    assert np.linalg.norm(p1-p0) > 1e-4
    for t, point in ((0.0, p0), (1.0, p1)):
        local, normal = course.windows[0].world_to_local(point, t)
        assert np.allclose(local, plan.point_local, atol=1e-9)
        assert abs(normal) < 1e-9


def test_mppi_command_is_finite_on_small_cpu_smoke():
    course = Course(torch.device("cpu"))
    cfg = Config(rollouts=8, rollout_steps=3)
    state = np.zeros(12)
    state[:3] = course.start
    plan = GatePointPlanner(course, cfg).plan(state, 0.0, 0, "test", np.inf)
    command, safe_ratio = LocalSafeMPPI(course, cfg, seed=1).command(state, 0.0, plan)
    assert np.isfinite(command).all()
    assert np.linalg.norm(command) <= cfg.max_acceleration + 1e-6
    assert 0.0 <= safe_ratio <= 1.0


def test_one_step_check_rejects_motion_into_frame():
    """Catches executing a final filtered command without revalidation."""
    course = Course(torch.device("cpu"))
    cfg = Config()
    center, rotation, _, _ = course.pose(0, 0.0)
    state = np.zeros(12)
    # Body is just outside the W1 frame and already moving into it.  No
    # acceleration command can undo the next 40 ms collision.
    local = np.array([1.45, 0.0, BODY_RADIUS + 0.006])
    state[:3] = center + rotation @ local
    state[3:6] = -2.0 * rotation[:, 2]
    plan = GatePointPlanner(course, cfg).plan(state, 0.0, 0, "test", np.inf)
    command = np.zeros(3)
    assert not command_is_one_step_safe(
        course, cfg, state, 0.0, command, plan, departure_mode=False,
    )


def test_integrator_returns_applied_snap_for_audit():
    """Catches reconstructing a different snap signal with np.gradient."""
    cfg = Config()
    state = np.zeros(12)
    command = np.array([3.0, -1.0, 2.0])
    next_state, applied_snap = integrate_step(state, command, cfg, np.zeros(3))
    expected = (next_state[9:12] - state[9:12]) / cfg.controller_dt
    assert np.allclose(applied_snap, expected)
    assert np.linalg.norm(applied_snap) <= cfg.max_snap + 1e-9


def test_final_shield_refuses_unavoidable_next_step_collision():
    """Catches falling through to an unchecked 'least bad' action."""
    course = Course(torch.device("cpu"))
    cfg = Config()
    center, rotation, _, _ = course.pose(0, 0.0)
    state = np.zeros(12)
    state[:3] = center + rotation @ np.array([1.45, 0.0, BODY_RADIUS + 0.006])
    state[3:6] = -2.0 * rotation[:, 2]
    plan = GatePointPlanner(course, cfg).plan(state, 0.0, 0, "test", np.inf)
    selected, intervened = select_verified_command(
        course, cfg, state, 0.0, np.zeros(3), plan, departure_mode=False,
    )
    assert selected is None
    assert intervened


def test_final_shield_preserves_a_safe_command():
    course = Course(torch.device("cpu"))
    cfg = Config()
    state = np.zeros(12)
    state[:3] = course.start
    plan = GatePointPlanner(course, cfg).plan(state, 0.0, 0, "test", np.inf)
    command = np.array([0.3, -0.2, 0.1])
    selected, intervened = select_verified_command(
        course, cfg, state, 0.0, command, plan, departure_mode=False,
    )
    assert not intervened
    assert np.allclose(selected, command)


def test_passage_root_lies_on_the_moving_plane():
    """Catches freezing a moving gate plane at the interval midpoint."""
    course = Course(torch.device("cpu"))
    t0,t1=1.0,1.04
    c0,r0,_,_=course.pose(0,t0)
    c1,r1,_,_=course.pose(0,t1)
    p0=c0+0.12*r0[:,2]
    p1=c1-0.12*r1[:,2]
    crossed,tc,_=passage(course,0,p0,p1,t0,t1,1.0)
    assert crossed
    cross=p0+(tc-t0)/(t1-t0)*(p1-p0)
    _,normal_offset=course.windows[0].world_to_local(cross,tc)
    assert abs(normal_offset)<1e-10


def test_workspace_barrier_commands_upward_before_ground_is_unavoidable():
    """Catches treating altitude only as an MPPI rollout constraint."""
    cfg=Config()
    state=np.zeros(12)
    state[2]=1.0
    state[5]=-2.0
    filtered,intervened=workspace_barrier_filter(cfg,state,np.zeros(3))
    assert intervened
    assert filtered[2]>0.0


def test_integrator_preserves_snap_jerk_and_acceleration_limits_together():
    """Catches acceleration clipping that silently increases applied snap."""
    cfg=Config()
    state=np.zeros(12)
    state[6:9]=np.array([8.95,0.0,0.0])
    state[9:12]=np.array([3.0,0.0,0.0])
    next_state,applied_snap=integrate_step(
        state,np.array([9.0,2.0,0.0]),cfg,np.zeros(3))
    assert np.linalg.norm(next_state[6:9])<=cfg.max_acceleration+1e-8
    assert np.linalg.norm(next_state[9:12])<=cfg.max_jerk+1e-8
    assert np.linalg.norm(applied_snap)<=cfg.max_snap+1e-8


def test_torch_predictor_uses_the_same_joint_limits():
    """Catches MPPI predicting a transition the execution model cannot use."""
    cfg=Config()
    acceleration=torch.tensor([[8.95,0.0,0.0]])
    jerk=torch.tensor([[3.0,0.0,0.0]])
    command=torch.tensor([[9.0,2.0,0.0]])
    next_acceleration,next_jerk,snap=torch_dynamics_step(
        acceleration,jerk,command,cfg)
    assert torch.linalg.vector_norm(next_acceleration,dim=-1).max()<=cfg.max_acceleration+1e-5
    assert torch.linalg.vector_norm(next_jerk,dim=-1).max()<=cfg.max_jerk+1e-5
    assert torch.linalg.vector_norm(snap,dim=-1).max()<=cfg.max_snap+1e-5


def test_ordered_filter_brakes_before_plane_violation_is_imminent():
    """Catches a second-order CBF that ignores jerk/snap braking distance."""
    course=Course(torch.device("cpu")); cfg=Config()
    state=np.zeros(12)
    center,rotation,_,_=course.pose(0,0.0)
    incoming=1.0
    normal=rotation[:,2]
    state[:3]=center+6.0*normal+rotation[:,0]
    state[3:6]=-5.0*normal
    plan=GatePointPlanner(course,cfg,center_only=True).plan(
        state,0.0,0,"test",np.inf)
    plan.incoming_sign=incoming
    filtered,intervened,_=ordered_plane_filter(
        course,cfg,state,0.0,np.zeros(3),plan,enabled=True)
    assert intervened
    assert float(normal@filtered)>0.0


def test_cross_time_changes_mppi_command():
    """Catches computing a traversal time that the controller never uses."""
    course=Course(torch.device("cpu")); cfg=Config(rollouts=8,rollout_steps=6)
    state=np.zeros(12); state[:3]=course.start
    plan=GatePointPlanner(course,cfg).plan(state,0.0,0,"test",np.inf)
    early=replace(plan,cross_time=0.2)
    late=replace(plan,cross_time=6.0)
    early_mppi=LocalSafeMPPI(course,cfg,seed=4)
    late_mppi=LocalSafeMPPI(course,cfg,seed=4)
    early_command,_=early_mppi.command(state,0.0,early)
    late_command,_=late_mppi.command(state,0.0,late)
    assert np.linalg.norm(early_command-late_command)>1e-6


def test_corridor_objective_counts_the_path_the_controller_actually_flies():
    """Catches scoring a direct chord while execution uses normal corridors."""
    p=np.array([-2.0,1.0,0.5])
    point=np.array([0.0,0.0,0.0])
    normal=np.array([1.0,0.0,0.0])
    outgoing=np.array([4.0,3.0,0.0])
    value=corridor_path_length(
        p,point,normal,1.0,outgoing,approach_distance=2.0,
        departure_distance=1.0,current_signed_distance=3.0,
    )
    approach=np.array([2.0,0.0,0.0])
    departure=np.array([-1.0,0.0,0.0])
    expected=(np.linalg.norm(p-approach)+2.0+1.0
              +np.linalg.norm(outgoing-departure))
    assert np.isclose(value,expected)


def test_online_deadline_is_a_hard_success_condition():
    assert online_deadline_satisfied([99.9],[2.0],[70.0])
    assert not online_deadline_satisfied([100.01],[2.0],[70.0])
    assert not online_deadline_satisfied([90.0],[100.01],[70.0])
    assert not online_deadline_satisfied([90.0],[2.0],[100.01])


def test_time_replan_can_lock_the_gate_local_point():
    course=Course(torch.device("cpu")); cfg=Config(rollouts=8,rollout_steps=3)
    state=np.zeros(12); state[:3]=course.start
    planner=GatePointPlanner(course,cfg)
    first=planner.plan(state,0.0,0,"initial",np.inf)
    moved=state.copy(); moved[:3]+=np.array([1.0,-0.5,0.2])
    updated=planner.plan(
        moved,0.8,0,"max_dwell",np.inf,locked_local=first.point_local)
    assert np.allclose(updated.point_local,first.point_local)
    assert updated.cross_time>first.cross_time


def test_local_gate_culling_uses_a_conservative_reachability_bound():
    course=Course(torch.device("cpu")); cfg=Config()
    state=np.zeros(12); state[:3]=course.start
    relevant,bounds=locally_relevant_gates(course,cfg,state,0.0)
    assert len(bounds)==len(course.windows)
    for index,bound in enumerate(bounds):
        if index not in relevant:
            assert bound>0.0
