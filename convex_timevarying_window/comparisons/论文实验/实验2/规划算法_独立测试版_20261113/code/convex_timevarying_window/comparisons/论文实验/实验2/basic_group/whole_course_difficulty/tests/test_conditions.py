import numpy as np
from convex_timevarying_window.comparisons.论文实验.实验2.basic_group.whole_course_difficulty import scenario,layout,run
from convex_timevarying_window.comparisons.论文实验.实验2.basic_group.togt_course import scenario as old


def test_complete_factorial_and_all_seven_participate():
    assert len(scenario.LEVELS)==10
    assert set(scenario.CONDITIONS.values())=={(a,f) for a in scenario.AMPLITUDE_RATIOS for f in scenario.FREQUENCY_FACTORS}
    for level in scenario.LEVELS[1:]:
        rho,scale=scenario.CONDITIONS[level];tr,_=scenario.build_case(scenario.Case(level=level))
        for i,w in enumerate(tr.windows):
            assert w.motion.amplitude>0 and w.motion.frequency>0
            np.testing.assert_allclose(w.motion.amplitude/scenario.horizontal_width(w),rho)
            np.testing.assert_allclose(w.motion.frequency/scenario.BASE_FREQUENCIES[i],scale)
            assert w.motion.kind==('translation' if i%2==0 else 'swing')


def test_geometry_and_zero_phase_unchanged():
    base,_=old.build_case(old.Case())
    for level in scenario.LEVELS:
        tr,_=scenario.build_case(scenario.Case(level=level))
        np.testing.assert_array_equal(tr.start,base.start);np.testing.assert_array_equal(tr.goal,base.goal)
        for w,b in zip(tr.windows,base.windows):
            np.testing.assert_array_equal(w.center0,b.center0);np.testing.assert_array_equal(w.R,b.R)
            assert w.aperture.kind==b.aperture.kind and w.aperture.margin==b.aperture.margin
            assert w.motion.phase==0 and w.motion.length==b.motion.length
            np.testing.assert_allclose(w.state_at(0.)[0],b.state_at(0.)[0])


def test_peak_speed_formula_and_motion_interfaces():
    for level in scenario.LEVELS[1:]:
        tr,_=scenario.build_case(scenario.Case(level=level))
        manifest=scenario.manifest(scenario.Case(level=level))
        for w,r in zip(tr.windows,manifest['windows']):
            _,v=w.motion.local(np.linspace(0,1/w.motion.frequency,1001))
            np.testing.assert_allclose(np.max(np.linalg.norm(v,axis=1)),r['peak_center_speed_mps'],rtol=1e-12)
    assert layout.motion_audit()['passed']


def test_geometry_screen_at_whole_course_maximum():
    r=layout.audit()
    assert r['passed'] and r['min_nonincident_gap_m']>2. and r['min_pair_sweep_gap_m']>0.
    assert len(r['per_piece_foreign_checks'])==42


def test_static_inheritance_checks_all_sources_and_scene():
    for m in run.driver.METHODS:
        e=run.static_reuse_evidence(m)
        assert e['all_prior_sources_unchanged'] and e['static_geometry_state_and_motion_equal']
        assert len(e['result_sha256'])==64
