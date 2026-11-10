import json
import numpy as np
from scipy.spatial.transform import Rotation
from convex_timevarying_window.comparisons.论文实验.实验2.gazebo_matrix import archive
from convex_timevarying_window.comparisons.论文实验.实验2.gazebo_matrix.reference import MPC, Reference
from convex_timevarying_window.tilt_heading_minco import dynamics
from convex_timevarying_window.togt_style_mpc.model import functions, MIXER
from convex_timevarying_window.togt_style_mpc.flatness import maps
from convex_timevarying_window.dynamic_passage_velocity_minco.trajectory import Plan


def test_reference_keeps_coefficients_clock_and_five_derivatives():
    rng = np.random.default_rng(4)
    c = rng.normal(size=(2, 8, 3))
    plan = Plan(0., 0, np.array([1., 2.]), c, np.empty(0), 0., 0., 'test', 7)
    reference = Reference(np.array([1., 2., 3.]), plan)
    np.testing.assert_array_equal(reference.sample([-.5])[0, :3], reference.start)
    np.testing.assert_array_equal(reference.sample([-.5])[0, 3:], np.zeros(12))
    reference.advance(.1)
    for at in (.1, .7, 1.1, 2.7):
        actual = reference.sample([at])[0]
        for d in range(5):
            np.testing.assert_allclose(actual[d*3:(d+1)*3], plan.sample(at, d))


def test_tracking_attitude_and_rates_match_nominal_north_chart():
    from types import SimpleNamespace
    tracker = MPC.__new__(MPC)
    tracker.cfg = SimpleNamespace(steps=3, dt=.04)
    tracker.flatness_map = maps().map(4)
    tracker.mixer_inverse = np.linalg.inv(MIXER)
    tracker.aero = functions()[2]
    rng = np.random.default_rng(33)
    pvaj = rng.normal(size=(4, 15))
    pvaj[:, 6:9] *= .3
    states, forces = tracker.references(lambda times: pvaj, .4, np.array([1., 0., 0., 0.]))
    assert states.shape == (4, 13) and forces.shape == (4, 4)
    for row, state, thrusts in zip(pvaj, states, forces):
        rot, magnitude, _ = dynamics.orientation(row[3:6], row[6:9])
        omega, _ = dynamics.rates(row[3:6], row[6:9], row[9:12], row[12:15])
        np.testing.assert_allclose(Rotation.from_quat(state[[7, 8, 9, 6]]).as_matrix(), rot, atol=1e-10)
        np.testing.assert_allclose(state[10:13], np.asarray(omega).ravel(), atol=1e-10)
        np.testing.assert_allclose(thrusts.sum(), float(magnitude), atol=1e-10)


def test_lossless_log_storage_and_temporary_restore(tmp_path):
    contents = {'gate_pose.pbtxt': b'header { stamp { sec: 30 } }\n'*100,
                'flight.ulg': bytes(range(256))*100}
    for name, value in contents.items():
        (tmp_path/name).write_bytes(value)
    records = archive.pack(tmp_path)
    assert len(records) == 2
    assert all(not (tmp_path/name).exists() for name in contents)
    with archive.restored(tmp_path):
        for name, value in contents.items():
            assert (tmp_path/name).read_bytes() == value
    assert all(not (tmp_path/name).exists() for name in contents)
    assert json.loads((tmp_path/'compression_manifest.json').read_text()) == records
