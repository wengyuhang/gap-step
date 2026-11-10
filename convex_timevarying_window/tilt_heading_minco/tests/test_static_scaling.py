import numpy as np
from convex_timevarying_window.tilt_heading_minco.static_continuation import amplitude_track
from convex_timevarying_window.comparisons.论文实验.实验2.basic_group.whole_course_difficulty import scenario
from convex_timevarying_window.tilt_heading_minco.planner import Planner


def test_zero_motion_is_independent_of_previous_frequency_and_retains_motion_interface():
    cases=[scenario.build_case(scenario.Case(level=l))[0] for l in ('a25_f3','a25_f6','a75_f1')]
    aux=[amplitude_track(tr,0.) for tr in cases];planners=[Planner(tr,True,pose_function=scenario.pose) for tr in aux]
    for tr,p in zip(aux,planners):
        for w in tr.windows:
            assert np.isinf(w.motion.translation_period) and w.motion.frequency==0.
            for t in (0.,.7,13.2):np.testing.assert_array_equal(w.state_at(t)[0],w.center0)
        for x,y in zip(p.seeds(),planners[0].seeds()):
            np.testing.assert_array_equal(x,y)
            np.testing.assert_array_equal(p.evaluate(x,p.witnesses())[0],planners[0].evaluate(y,planners[0].witnesses())[0])
    for original in cases:
        restored=amplitude_track(original,1.)
        for a,b in zip(restored.windows,original.windows):
            assert type(a.motion) is type(b.motion) and a.motion.frequency==b.motion.frequency
            np.testing.assert_array_equal(a.state_at(.831)[0],b.state_at(.831)[0])
