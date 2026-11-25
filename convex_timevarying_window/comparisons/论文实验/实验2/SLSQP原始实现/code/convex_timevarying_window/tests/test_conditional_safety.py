import numpy as np

from convex_timevarying_window.conditional_dual_constraint_cem.objective import (
    SafetyAugmentedTOGTObjective,
)
from convex_timevarying_window.conditional_dual_constraint_cem.safety_penalty import (
    ConvexSafetyConfig,
    HandwrittenConvexSafetyIntegral,
    polygon_halfspaces,
)
from convex_timevarying_window.togt.experiment import BODY_RADIUS, build_track
from convex_timevarying_window.togt.native_objective import NativeJointTOGTObjective
from nonconvex_timevarying_window.sc_dynatogt.minco import BoundaryState, MincoSnap


def _objects():
    track, config = build_track()
    base = NativeJointTOGTObjective(track, config)
    safety = HandwrittenConvexSafetyIntegral(
        track.windows, base.head, base.tail,
        ConvexSafetyConfig(BODY_RADIUS), base.core,
    )
    return track, base, safety


def test_polygon_halfspaces_contain_all_physical_vertices():
    track, _, _ = _objects()
    for window in track.windows:
        if window.aperture.kind != "polygon":
            continue
        halfspaces = polygon_halfspaces(window.aperture.physical_vertices)
        residuals = window.aperture.physical_vertices @ halfspaces.normals.T - halfspaces.offsets
        assert np.max(residuals) <= 1e-10
        np.testing.assert_allclose(
            np.linalg.norm(halfspaces.normals, axis=1), 1.0, atol=1e-14
        )


def test_handwritten_safety_minco_gradient_matches_directional_difference():
    track, base, safety = _objects()
    forward = base.forward(base.initial_guess())
    evaluation = safety.evaluate(forward.trajectory)
    rng = np.random.default_rng(2)
    point_direction = rng.normal(size=forward.waypoints.shape)
    duration_direction = 0.1*rng.normal(size=forward.durations.shape)
    step = 1e-5
    values = []
    for sign in (-1.0, 1.0):
        trajectory = MincoSnap(
            BoundaryState(track.start), BoundaryState(track.goal),
            forward.waypoints+sign*step*point_direction,
            forward.durations+sign*step*duration_direction,
        )
        values.append(safety.evaluate(trajectory, with_gradient=False).value)
    finite = (values[1]-values[0])/(2.0*step)
    analytic = (
        np.sum(evaluation.waypoint_gradient*point_direction)
        + evaluation.duration_gradient @ duration_direction
    )
    np.testing.assert_allclose(analytic, finite, rtol=5e-7, atol=5e-7)


def test_full_augmented_kd_gradient_matches_directional_difference():
    _, base, safety = _objects()
    objective = SafetyAugmentedTOGTObjective(base, safety)
    x = base.initial_guess()
    _, gradient = objective.value_and_gradient(x)
    direction = np.random.default_rng(4).normal(size=x.size)
    direction /= np.linalg.norm(direction)
    step = 1e-5
    finite = (
        objective.value_and_gradient(x+step*direction)[0]
        - objective.value_and_gradient(x-step*direction)[0]
    )/(2.0*step)
    np.testing.assert_allclose(gradient @ direction, finite, rtol=5e-7, atol=5e-4)
