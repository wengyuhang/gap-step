import json
from pathlib import Path

import numpy as np

from nonconvex_timevarying_window.exact_area_sc_dynatogt.stress_case import WORLD_CLEARANCE, build_stress_case


def test_nominal_world_margin_is_safe_but_dynamic_pre_crossing_collides():
    case = build_stress_case()
    nominal_old = case.snapshot("Old-0.315", case.crossing_time, executed=False)
    collision_old = case.snapshot("Old-0.315", case.collision_time, executed=False)
    collision_ours = case.snapshot("Ours", case.collision_time, executed=False)
    assert case.cuboid.half_extents[0] == case.cuboid.half_extents[1]
    assert case.cuboid.half_extents[2] < case.cuboid.half_extents[0]
    assert case.start.tolist() == case.goal.tolist()
    assert nominal_old.center_legal
    assert nominal_old.boundary_distance >= WORLD_CLEARANCE
    assert nominal_old.center_normal_distance < 1.0e-10
    assert not nominal_old.metrics.whole_body_collision
    assert nominal_old.metrics.outside_area < 1.0e-12
    assert case.collision_time < case.crossing_time
    assert collision_old.center_normal_distance > 0.0
    assert not np.isclose(
        collision_old.center_normal_distance,
        collision_ours.center_normal_distance,
        atol=1.0e-3,
    )
    assert collision_old.metrics.whole_body_collision
    assert collision_old.metrics.outside_area > 0.0
    assert not collision_ours.metrics.whole_body_collision
    for method in ("Old-0.315", "Ours"):
        instant = case.collision_time
        step = 1.0e-4
        velocity = (
            case.planned_position(method, instant + step)
            - case.planned_position(method, instant - step)
        )
        velocity /= np.linalg.norm(velocity)
        assert float(case.body_rotation(method, instant)[:, 0] @ velocity) > 1.0 - 1.0e-9
    old_stop = case.trajectory_position("Old-0.315", case.collision_time)
    assert np.allclose(case.trajectory_position("Old-0.315", case.total_time), old_stop)
    assert not np.allclose(old_stop, case.goal)
    assert np.allclose(case.trajectory_position("Ours", case.total_time), case.goal)
    for instant in np.linspace(case.crossing_time - 1.0, case.crossing_time + 1.8, 121):
        snapshot = case.snapshot("Ours", float(instant), executed=False)
        if snapshot.section.area > 1.0e-12:
            assert snapshot.boundary_distance >= WORLD_CLEARANCE
            assert not snapshot.metrics.whole_body_collision


def test_saved_real_solver_run_and_artifacts():
    root = Path(__file__).parents[1] / "results" / "experiment_b"
    summary = json.loads((root / "summary.json").read_text())
    assert summary["same_start_and_goal"] is True
    assert summary["optimizer_claim"].startswith("both Old and Ours executed")
    rows = {row["method"]: row for row in summary["rows"]}
    assert rows["Old-0.315"]["minimum_world_center_clearance"] >= WORLD_CLEARANCE
    assert rows["Ours"]["minimum_world_center_clearance"] >= WORLD_CLEARANCE
    assert rows["Old-0.315"]["whole_body_collision"] is True
    assert rows["Ours"]["whole_body_collision"] is False
    assert (root / "experiment_b.csv").is_file()
    assert (root / "optimized_solutions.npz").is_file()
    assert (root / "figures" / "full_planned_timeline.png").is_file()
