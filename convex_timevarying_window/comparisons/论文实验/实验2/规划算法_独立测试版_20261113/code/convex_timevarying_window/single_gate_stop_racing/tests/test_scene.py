import numpy as np
from 复现.polynomial_dynamic_racing_2023.px4_fixed_plane import scenario as original
from convex_timevarying_window.single_gate_stop_racing import scenario


def test_only_first_anchor_moves_in_plane():
    case = scenario.Case(3.,4.,0.)
    before,_ = original.build_case(case)
    after,_ = scenario.build_case(case,horizontal_spacing_scale=1.)
    np.testing.assert_allclose(after.windows[0].center0-before.windows[0].center0,[0.,-5.,0.])
    for i,(a,b) in enumerate(zip(before.windows,after.windows)):
        R=a.state_at(0.)[1]
        np.testing.assert_allclose(R,b.state_at(0.)[1])
        assert abs((b.center0-a.center0)@R[:,2])<1e-12
        np.testing.assert_allclose(a.motion.translation_amplitude,b.motion.translation_amplitude)
        assert a.motion.translation_period==b.motion.translation_period
        assert a.motion.phase==b.motion.phase
        if i:np.testing.assert_array_equal(a.center0,b.center0)
    np.testing.assert_array_equal(before.start,after.start)
    np.testing.assert_array_equal(before.goal,after.goal)


def test_manifest_and_historical_scene_match_actual_scene():
    case=scenario.Case(3.5,4.,0.)
    manifest=scenario.manifest(case)
    track,_=scenario.build_case(case)
    np.testing.assert_allclose(manifest['anchors'],[w.center0 for w in track.windows])
    assert not manifest['original_positions_unchanged']
    assert manifest['fixed_planes']
    assert manifest['original_orientations_unchanged']
    original_track,_=original.build_case(case)
    historical,_=scenario.build_case(case,0.,1.)
    np.testing.assert_allclose([w.center0 for w in original_track.windows],
                               [w.center0 for w in historical.windows])


def test_spacing_changes_only_horizontal_anchors_and_workspace():
    case=scenario.Case(3.,4.,0.)
    original_track,_=original.build_case(case)
    spread,_=scenario.build_case(case,first_gate_shift_m=0.)
    for a,b in zip(original_track.windows,spread.windows):
        np.testing.assert_allclose(b.center0[:2]-spread.start[:2],
                                   2*(a.center0[:2]-original_track.start[:2]))
        assert a.center0[2]==b.center0[2]
        np.testing.assert_allclose(a.state_at(0.)[1],b.state_at(0.)[1])
        np.testing.assert_allclose(a.motion.translation_amplitude,b.motion.translation_amplitude)
    np.testing.assert_array_equal(spread.start,original_track.start)
    np.testing.assert_array_equal(spread.goal,original_track.goal)
    lo,hi=scenario.workspace_bounds()
    for w in spread.windows:
        assert np.all(w.center0>lo) and np.all(w.center0<hi)
    assert lo[2]==.65 and hi[2]==12.
