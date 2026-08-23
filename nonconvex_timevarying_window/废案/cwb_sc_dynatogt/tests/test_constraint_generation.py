from __future__ import annotations

import numpy as np

from nonconvex_timevarying_window.cwb_sc_dynatogt.body_model import CuboidBody
from nonconvex_timevarying_window.cwb_sc_dynatogt.config import WholeBodySafetyConfig
from nonconvex_timevarying_window.cwb_sc_dynatogt.constraint_generation import (
    ActiveSafetyConstraint,
    WholeBodyConstrainedObjective,
)
from nonconvex_timevarying_window.cwb_sc_dynatogt.gate_frame import frame_at
from nonconvex_timevarying_window.cwb_sc_dynatogt.plane_section import (
    cuboid_world_vertices,
    plane_section_from_vertices,
)
from nonconvex_timevarying_window.cwb_sc_dynatogt.whole_body_safety import SafetyWitness
from nonconvex_timevarying_window.sc_dynatogt.environment import (
    MotionProfile,
    SCDynamicWindow,
    SCWindowTrack,
)
from nonconvex_timevarying_window.sc_dynatogt.optimizer import JointTOGTObjective, OptimizationConfig


def test_active_objective_preserves_k_d_dimension_and_has_finite_gradient(square_map) -> None:
    polygon = np.array([[-1.0, -1.0], [1.0, -1.0], [1.0, 1.0], [-1.0, 1.0]])
    window = SCDynamicWindow(
        "square", square_map, polygon, np.zeros(3),
        np.array([0.0, np.pi / 2.0, 0.0]), MotionProfile.static(),
    )
    track = SCWindowTrack(
        "single", np.array([-2.0, 0.0, 0.0]), np.array([2.0, 0.0, 0.0]),
        (window,), (0,),
    )
    base = JointTOGTObjective(track, OptimizationConfig(max_iterations=2, samples_per_segment=4))
    x = base.initial_guess()
    forward = base.forward(x)
    time = float(forward.traversal_times[0])
    body = CuboidBody(np.array([0.15, 0.5, 0.1]))
    vertices, _ = cuboid_world_vertices(forward.trajectory, time, body)
    section = plane_section_from_vertices(vertices, frame_at(window, time), body, time=time)
    first, second = section.vertices[:2]
    point = 0.5 * (first.local + second.local)
    witness = SafetyWitness(
        0, 1, 0.0, first.source_body_edge, second.source_body_edge, 0.5,
        None, -0.1, False, section.topology_key, point,
        frame_at(window, time).center + frame_at(window, time).basis @ point,
    )
    config = WholeBodySafetyConfig(sc_safe_radius=0.2, finite_difference_step=1e-5)
    constrained = WholeBodyConstrainedObjective(
        base, body, config, [ActiveSafetyConstraint(witness, 100.0)]
    )
    evaluation = constrained.evaluate(x)
    assert evaluation.gradient.shape == (base.dimension,)
    assert base.dimension == 4  # 2 K variables + 2 D variables for one gate.
    assert np.isfinite(evaluation.cost)
    assert np.all(np.isfinite(evaluation.gradient))
