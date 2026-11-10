from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from nonconvex_timevarying_window.sc_dynatogt.time_mapping import k_from_durations
from convex_timevarying_window.comparisons.论文实验.实验3 import planning, scenario


def test_single_gate_initial_curves_and_strict_centers():
    track, _ = scenario.build_case(scenario.Case(2))
    free = planning.make_planner(track, 'ours')
    center = planning.make_planner(track, 'center')
    instant = planning.make_planner(track, 'instant')
    objective = planning.CenterObjective(track, center)
    assert (free.nvar, center.nvar, objective.dimension) == (15, 11, 2)
    for a, b, c in zip(free.seeds(), center.seeds(), instant.seeds()):
        np.testing.assert_array_equal(a, c)
        target = np.asarray(center.curve_values(b)).ravel().reshape(2, 3, 8).transpose(0, 2, 1)
        np.testing.assert_allclose(free.curve_values(a), center.curve_values(b), atol=1e-8)
        forward = objective.forward(k_from_durations(np.exp(b[:2])))
        native = forward.trajectory.coefficients*forward.durations[:, None, None]**np.arange(8)[None, :, None]
        np.testing.assert_allclose(native, target, atol=1e-8)
        np.testing.assert_allclose(forward.waypoints[0], track.windows[0].state_at(forward.traversal_times[0])[0], atol=1e-12)
        np.testing.assert_allclose(objective.value_and_gradient(k_from_durations(np.exp(b[:2])))[0], center.objective(b)[0], rtol=1e-7, atol=1e-7)


def test_single_world_and_motion(tmp_path):
    track, _ = scenario.build_case(scenario.Case(4))
    world = ET.parse(scenario.write_world(track, scenario.ASSETS, tmp_path))
    models = [m for m in world.findall('world/model') if m.get('name', '').startswith('gate_')]
    assert len(models) == 1
    assert len(scenario.contact_topics(track)) == 2
    assert models[0].find('link/sensor/contact/collision').text == 'frame_collision'
    plugin = models[0].find("plugin[@name='paper_experiment::MixedGateMotion']")
    np.testing.assert_allclose(np.fromstring(plugin.find('axis0').text, sep=' '), [0., 1., 0.])
    assert float(plugin.find('amplitude').text) == float(plugin.find('frequency').text) == 4.
    for t in np.linspace(0., 1., 11):
        center, rotation = scenario.pose(track.windows[0], t)
        np.testing.assert_allclose(np.asarray(center).ravel(), track.windows[0].state_at(t)[0], atol=1e-12)
        np.testing.assert_allclose(rotation, scenario.ROTATION)


def test_instant_verification_uses_no_passage_oracle(monkeypatch):
    track, _ = scenario.build_case(scenario.Case())
    instant = planning.make_planner(track, 'instant')
    x = instant.seeds()[0]
    C = np.asarray(instant.curve_values(x)).ravel().reshape(2, 3, 8).transpose(0, 2, 1)
    plan = planning.frozen_plan(dict(coefficients=C, durations=np.exp(x[:2]), variables=x))
    from convex_timevarying_window.comparisons.论文实验.实验2 import planner as original
    def forbidden(*args, **kwargs):
        raise AssertionError('The instantaneous ablation called the continuous frame oracle')
    monkeypatch.setattr(original, 'passage', forbidden)
    checks, passed = instant.verify(plan)
    assert passed
    assert all(check['witness_u'] is None for check in checks)
    assert sum(len(v['values']) for v in instant.witnesses().values()) == 1
