import numpy as np
from convex_timevarying_window.comparisons.论文实验.实验2.basic_group import scenario as old
from convex_timevarying_window.comparisons.论文实验.实验2.basic_group.separated_course import scenario,layout


def test_maximum_envelopes_are_separated_from_nonincident_corridors():
    a=layout.audit();assert a['passed'];assert a['min_nonincident_gap_m']>3.8
    assert a['min_pair_sweep_gap_m']>9
    assert len(a['per_piece_foreign_checks'])==42


def test_apertures_and_inset_are_unchanged():
    a,_=old.build_case(old.Case('original','static'));b,_=scenario.build_case(scenario.Case())
    for x,y in zip(a.windows,b.windows):
        assert x.aperture.kind==y.aperture.kind and x.aperture.margin==y.aperture.margin
        if x.aperture.kind=='circle':assert x.aperture.radius==y.aperture.radius
        else:
            np.testing.assert_array_equal(x.aperture.physical_vertices,y.aperture.physical_vertices)
            np.testing.assert_array_equal(x.aperture.safe_vertices,y.aperture.safe_vertices)


def test_only_one_challenge_window_changes_per_difficulty_step():
    for before,after in zip(scenario.LEVELS[1:-1],scenario.LEVELS[2:]):
        a,b=scenario.profile(before),scenario.profile(after)
        changed=[i for i in range(7) if a[i]!=b[i]]
        assert len(changed)==1 and changed[0] in (2,5)
        assert b[changed[0]][0]>=a[changed[0]][0] and b[changed[0]][1]>=a[changed[0]][1]


def test_mixed_motion_and_planes_are_preserved():
    ref,_=scenario.build_case(scenario.Case())
    for level in scenario.LEVELS:
        tr,_=scenario.build_case(scenario.Case(level=level))
        assert [w.motion.kind for w in tr.windows]==['translation','swing','translation','swing','translation','swing','translation']
        for a,b in zip(ref.windows,tr.windows):
            np.testing.assert_array_equal(a.R,b.R);np.testing.assert_array_equal(a.center0,b.center0)
            assert b.motion.phase==0


def test_numeric_symbolic_motion_and_bounds():assert layout.motion_audit()['passed']


def test_nonincident_collision_cannot_be_reported_as_passage_collision_only():
    r=layout.foreign_diagnostics(dict(checks=[dict(gate='07_rectangle',piece=1,outcome='collision')]))
    assert not r['scene_assumption_passed'] and r['nonincident_collision_count']==1
