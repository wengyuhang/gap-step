import numpy as np

from convex_timevarying_window.comparisons.conditional_cem_vs_togt.benchmark import (
    DIFFICULTIES,
    _wilson,
    build_course,
    generate_courses,
)


def test_course_generator_is_deterministic_and_has_all_difficulties():
    first = generate_courses(2, 37)
    second = generate_courses(2, 37)
    assert first == second
    assert len(first) == 2 * len(DIFFICULTIES)
    assert {item.difficulty for item in first} == {item.name for item in DIFFICULTIES}


def test_all_generated_course_windows_have_valid_safe_apertures():
    for spec in generate_courses(1, 5):
        track, _ = build_course(spec)
        assert len(track.windows) == 7
        for window in track.windows:
            if window.aperture.kind == "circle":
                assert window.aperture.radius > window.aperture.margin / 2.0
            else:
                assert np.all(np.linalg.norm(window.aperture.safe_vertices, axis=1) > 0.0)


def test_wilson_interval_contains_observed_rate():
    lo, hi = _wilson(4, 6)
    assert lo < 4 / 6 < hi
