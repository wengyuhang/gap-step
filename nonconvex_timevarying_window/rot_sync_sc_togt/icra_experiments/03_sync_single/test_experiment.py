from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path
import sys

import numpy as np


PATH = Path(__file__).with_name("run_experiment.py")
SPEC = spec_from_file_location("sync_single_experiment", PATH)
MODULE = module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def test_focused_grid_has_ten_unique_shape_speed_scenarios():
    names = {
        MODULE.scenario_name(shape, ratio, omega, phase)
        for shape in MODULE.SHAPES
        for ratio in MODULE.SIZE_RATIOS
        for omega in MODULE.OMEGAS
        for phase in MODULE.PHASES
    }
    assert len(names) == 10
    assert MODULE.OMEGAS == (0.0, 1.5, 3.0, 4.5, 6.0)


def test_planning_envelope_is_body_radius_plus_margin():
    assert MODULE.PLANNING_RHO == MODULE.BODY.circumscribed_radius + 0.015
    assert MODULE.MIN_SAFE_AREA == 1.0e-6


def test_optimized_minco_has_three_optimized_nodes():
    geometry = MODULE.prepare_geometry("L", 1.15, vertex_count=64, quadrature_order=32)
    scenario = MODULE.build_scenario(geometry, 1.5, 0.3, name="unit")
    config = MODULE.make_config({
        "smoothness_weight": 2.0e-4,
        "dynamics_weight": 0.1,
        "objective_samples_per_segment": 5,
    }, 1)
    objective = MODULE.OptimizedMincoObjective(scenario, config, 100.0)
    forward = objective.forward(objective.initial_guess())
    assert objective.dimension == 10
    assert forward.local_points.shape == (3, 2)
    assert forward.trajectory.num_segments == 4
    assert np.isclose(forward.trajectory.evaluate(forward.crossing_times[0])[0], 0.0)


def test_axis_is_locally_too_tight_but_off_axis_region_is_feasible():
    geometry = MODULE.prepare_geometry("L", 1.15, vertex_count=64, quadrature_order=32)
    assert 0.0 < geometry.axis_centered_incircle_radius < MODULE.BODY.circumscribed_radius
    assert geometry.physical_inradius > MODULE.PLANNING_RHO

    u_geometry = MODULE.prepare_geometry("U", 1.15, vertex_count=64, quadrature_order=32)
    assert u_geometry.axis_centered_incircle_radius == 0.0
    assert u_geometry.physical_inradius > MODULE.PLANNING_RHO
