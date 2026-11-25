from dataclasses import asdict
import numpy as np
from convex_timevarying_window.comparisons.论文实验.实验2.basic_group.whole_course_difficulty import scenario
from convex_timevarying_window.comparisons.论文实验.实验2.radius_ablation.planner import (
    Planner, ORIGINAL_RADIUS_M, SMALL_RADIUS_M)


def test_radius_is_applied_to_symbolic_safety_but_not_physics():
    track, _ = scenario.build_case(scenario.Case(level='a25_f6'))
    original = Planner(track, pose_function=scenario.pose, radius_m=ORIGINAL_RADIUS_M)
    smaller = Planner(track, pose_function=scenario.pose, radius_m=SMALL_RADIUS_M)
    original_config, smaller_config = asdict(original.cfg), asdict(smaller.cfg)
    assert smaller_config.pop('radius') == SMALL_RADIUS_M
    assert original_config.pop('radius') == ORIGINAL_RADIUS_M
    assert original_config == smaller_config
    for old, new in zip(original.seeds(), smaller.seeds()):
        np.testing.assert_array_equal(old, new)
    point = original.seeds()[0]
    old, _, old_curve = original.evaluate(point, original.witnesses())
    new, _, new_curve = smaller.evaluate(point, smaller.witnesses())
    np.testing.assert_array_equal(old_curve, new_curve)
    count = original.n*(original.cfg.nodes_per_piece+1)*17
    np.testing.assert_array_equal(old[:count], new[:count])
    assert (new[count:] >= old[count:]-1e-12).all()
    assert (new[count:]-old[count:]).max() > 1e-4
