import numpy as np
import pytest
from ..planner import Planner, embed_center
from ...independent_repair.smooth_augmented import Planner as BasePlanner
from ...basic_group.whole_course_difficulty import scenario


@pytest.mark.parametrize('level', ['a50_f6', 'a75_f3'])
def test_same_course_center_embedding_preserves_every_curve_and_constraint(level):
    track, _ = scenario.build_case(scenario.Case(level=level))
    center = BasePlanner(track, False, pose_function=scenario.pose)
    free = Planner(track, True, pose_function=scenario.pose)
    for seed in center.seeds():
        x = embed_center(free, seed)
        g0, _, c0 = center.evaluate(seed, center.witnesses())
        g1, _, c1 = free.evaluate(x, free.witnesses())
        np.testing.assert_allclose(c0, c1, atol=1e-7, rtol=1e-9)
        np.testing.assert_allclose(g0, g1, atol=1e-6, rtol=1e-7)
        np.testing.assert_array_equal(seed[:center.n], x[:free.n])
        np.testing.assert_array_equal(seed[center.n:], x[free.velocity_start:])
