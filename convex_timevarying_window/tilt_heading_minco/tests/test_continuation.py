import numpy as np
from convex_timevarying_window.tilt_heading_minco.continuation import amplitude_track
from convex_timevarying_window.comparisons.论文实验.实验2.basic_group.whole_course_difficulty import scenario


def test_continuation_does_not_mutate_target_or_change_any_other_scene_parameters():
    tr,_=scenario.build_case(scenario.Case(level='a50_f6'));amplitudes=[w.motion.amplitude for w in tr.windows]
    for factor in (0.,.25,.5,.75,1.):
        aux=amplitude_track(tr,factor)
        np.testing.assert_array_equal(aux.start,tr.start);np.testing.assert_array_equal(aux.goal,tr.goal)
        for a,b in zip(aux.windows,tr.windows):
            np.testing.assert_array_equal(a.R,b.R);np.testing.assert_array_equal(a.center0,b.center0)
            assert a.motion.amplitude==factor*b.motion.amplitude
            assert a.motion.frequency==b.motion.frequency and a.motion.phase==b.motion.phase
            assert a.motion.kind==b.motion.kind and a.motion.length==b.motion.length
        if factor==1.:
            for a,b in zip(aux.windows,tr.windows):
                for t in (0.,.137,1.723):
                    np.testing.assert_array_equal(a.state_at(t)[0],b.state_at(t)[0])
    assert [w.motion.amplitude for w in tr.windows]==amplitudes
