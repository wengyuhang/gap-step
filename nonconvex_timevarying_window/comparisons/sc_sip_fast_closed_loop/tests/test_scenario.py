import numpy as np

from nonconvex_timevarying_window.sc_dynatogt.boundary import BSpline, Bezier, CircularArc, Line
from nonconvex_timevarying_window.sip_dynatogt.model import (
    CertificateResult,
    CertificateStatus,
    Witness,
)
from nonconvex_timevarying_window.comparisons.sc_sip_fast_closed_loop.experiment import (
    _collision_label,
)
from nonconvex_timevarying_window.comparisons.sc_sip_fast_closed_loop.scenario import (
    build_fast_closed_loop_scenario,
)


def test_fast_comparison_scenario_is_closed_diverse_and_fixed_margin():
    scenario = build_fast_closed_loop_scenario()
    track = scenario.track
    np.testing.assert_array_equal(track.start, track.goal)
    assert track.order == (3, 4, 1, 5, 0, 2)
    assert [window.name for window in track.windows] == [
        "L_polygon", "circle_arc", "bezier_notch", "bspline_wave",
        "arc_capsule", "bezier_diamond",
    ]
    assert sum(len(boundary) for boundary in scenario.sip_boundaries) == 24
    flat = tuple(segment for boundary in scenario.sip_boundaries for segment in boundary)
    assert any(isinstance(segment, Line) for segment in flat)
    assert any(isinstance(segment, CircularArc) for segment in flat)
    assert any(isinstance(segment, Bezier) for segment in flat)
    assert any(isinstance(segment, BSpline) for segment in flat)
    # SC may densify the same curves for mapping, but SIP keeps the 16 exact
    # primitive segments rather than receiving the dense vertices as lines.
    assert sum(len(window.physical_boundary) for window in track.windows) > len(flat)
    for gate, window in zip(scenario.preprocessed_gates, track.windows):
        assert max(
            window.motion.translation_period,
            window.motion.rotation_period,
            window.motion.scale_period,
        ) <= 3.1
        assert np.linalg.norm(window.motion.translation_amplitude) >= 1.9
        assert np.linalg.norm(window.motion.rotation_amplitude) >= 1.1
        assert window.motion.scale_amplitude >= 0.48
        assert window.motion.minimum_scale >= 0.40
        assert window.motion.minimum_scale > 0.0
        assert window.required_world_clearance == scenario.body.conservative_center_clearance(
            scenario.net_clearance
        )
        assert (
            window.motion.minimum_scale * gate.safe_region.distance
            >= window.required_world_clearance - 1.0e-12
        )
        assert len(gate.safe_polygon) >= 3

    centers = np.asarray([window.center0 for window in track.windows])
    spans = np.ptp(centers, axis=0)
    assert spans[0] >= 27.0 and spans[1] >= 26.0 and spans[2] >= 10.0


def test_positive_clearance_residual_is_not_mislabeled_as_physical_collision():
    report = CertificateResult(
        CertificateStatus.VIOLATED,
        "finite safety witness",
        0,
        0,
        0,
        None,
        None,
        (Witness("safety", 0, 0.5, 1.0e-6, 0, 0, 0.5, "coarse"),),
    )
    assert _collision_label(report) == "CLEARANCE_VIOLATION_PROVED"
