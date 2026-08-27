from nonconvex_timevarying_window.sc_dynatogt.boundary import Bezier, CircularArc, Line

from nonconvex_timevarying_window.planar_rs_dynatogt.scenario import (
    benchmark_boundaries,
)


def test_hard_benchmark_keeps_compact_original_primitives():
    definitions = benchmark_boundaries()
    assert [len(segments) for _, segments in definitions] == [6, 8, 10, 4, 6, 5]
    assert sum(len(segments) for _, segments in definitions) == 39
    assert any(
        isinstance(segment, CircularArc)
        for _, segments in definitions
        for segment in segments
    )
    assert any(
        isinstance(segment, Bezier)
        for _, segments in definitions
        for segment in segments
    )
    assert all(
        isinstance(segment, (Line, CircularArc, Bezier))
        for _, segments in definitions
        for segment in segments
    )
