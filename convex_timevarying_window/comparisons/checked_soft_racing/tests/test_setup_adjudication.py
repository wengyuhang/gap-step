import json
from types import SimpleNamespace

import numpy as np

from convex_timevarying_window.comparisons.checked_soft_racing import adjudicate


def fixture_log(moving=False, offboard=True):
    times = np.linspace(10., 29., 200)
    zeros = np.zeros(len(times))
    altitude = np.linspace(6.013, -2.96, len(times)) if moving else zeros+6.013
    topics = {
        'vehicle_local_position_groundtruth': dict(timestamp=times*1e6, x=zeros+4., y=zeros-16.,
                                                  z=altitude, vx=zeros, vy=zeros, vz=zeros),
        'vehicle_attitude_groundtruth': dict(timestamp=times*1e6,
                                            **{'q[0]': zeros+1., 'q[1]': zeros, 'q[2]': zeros, 'q[3]': zeros}),
        'vehicle_status': dict(timestamp=times*1e6, arming_state=zeros+2., nav_state=zeros+(14. if offboard else 4.)),
        'trajectory_setpoint': dict(timestamp=times*1e6, **{'position[2]': zeros-9.}),
    }
    return SimpleNamespace(get_dataset=lambda name: SimpleNamespace(data=topics[name]))


def test_setup_failure_is_demonstrably_before_any_planning(tmp_path, monkeypatch):
    (tmp_path/'result.json').write_text(json.dumps({'solves': [{'time': .1}]}))
    monkeypatch.setattr(adjudicate, 'ULog', lambda _: fixture_log())
    result = adjudicate.assess(tmp_path)
    assert result['setup_failure']
    assert result['preflight_truth_displacement_m'] == 0.
    assert result['preflight_position_error_m'] > 8.


def test_flight_with_later_algorithm_failure_is_not_retried(tmp_path, monkeypatch):
    (tmp_path/'result.json').write_text(json.dumps({'solves': [{'time': .1}], 'failure': 'timeout'}))
    monkeypatch.setattr(adjudicate, 'ULog', lambda _: fixture_log(moving=True))
    assert not adjudicate.assess(tmp_path)['setup_failure']


def test_incomplete_causal_evidence_does_not_get_this_exemption(tmp_path, monkeypatch):
    (tmp_path/'result.json').write_text(json.dumps({'solves': [{'time': .1}]}))
    monkeypatch.setattr(adjudicate, 'ULog', lambda _: fixture_log(offboard=False))
    assert not adjudicate.assess(tmp_path)['setup_failure']
