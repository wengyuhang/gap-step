import json
from pathlib import Path

from nonconvex_timevarying_window.comparisons.seven_unique_sc_sphere.experiment import SHAPES


HERE = Path(__file__).resolve().parents[1]


def test_shape_sequence_has_seven_distinct_shapes():
    assert len(SHAPES) == 7
    assert len(set(SHAPES)) == 7


def test_frozen_scene_is_closed_and_spatially_dispersed():
    scene = json.loads(
        (HERE / "results" / "irregular_closed_20260909" / "scene.json").read_text(
            encoding="utf-8"))
    assert scene["closed_loop"]
    assert scene["start"] == scene["goal"]
    assert len(scene["windows"]) == 7
    assert min(scene["route_leg_distances"]) > 11.0


def test_frozen_result_has_at_least_two_sphere_collisions():
    result = json.loads(
        (HERE / "results" / "irregular_closed_20260909" / "result.json").read_text(
            encoding="utf-8"))
    failed = [row for row in result["sphere_audit"] if not row["passed"]]
    assert result["collision_window_count"] == len(failed)
    assert result["requirement_at_least_two_collisions"]
    assert len(failed) >= 2
    assert result["sphere_obstacle_model"] == "finite_zero_thickness_boundary_frame"
