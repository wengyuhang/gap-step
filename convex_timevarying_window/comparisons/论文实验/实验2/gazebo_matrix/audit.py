"""Re-evaluate actual body motion, eight physical sensors and known gate motion."""
import json
from pathlib import Path
import numpy as np
from pyulog import ULog
from .. import audit as physical
from ..basic_group.whole_course_difficulty import scenario
from ..basic_group import scenario as world


def audit(folder):
    folder = Path(folder)
    physical.build_case, physical.Case = scenario.build_case, scenario.Case
    physical.parse_all_contacts = world.parse_all_contacts
    result = physical.audit(folder)
    raw = json.loads((folder/'result.json').read_text())
    data = ULog(str(folder/'flight.ulg')).get_dataset('vehicle_local_position_groundtruth').data
    times = data['timestamp']*1e-6-30.
    mask = (times >= 0.) & (times <= raw['last_time_s'])
    velocities = np.column_stack([data[key][mask] for key in ('vx', 'vy', 'vz')])
    speed = float(np.linalg.norm(velocities[-1]))
    result['truth_goal_speed_mps'] = speed
    result['passed'] = bool(result['passed'] and speed < 1.)
    if result['verdict'] == 'completed_no_contact' and not result['passed']:
        result['verdict'] = 'terminal_speed_not_met'
        result['flight_time_s'] = None
    track, _ = scenario.build_case(scenario.Case(**raw['experiment_case']))
    truth = np.load(folder/'truth_trace.npz')
    reference = np.load(folder/'reference_trajectory.npz')
    from convex_timevarying_window.dynamic_passage_velocity_minco.trajectory import Plan
    plan = Plan(0., 0, reference['durations'], reference['coefficients'], np.empty(0), 0., 0., 'frozen', 7)
    errors = np.linalg.norm(truth['positions']-plan.sample_many(truth['times']), axis=1)
    result.update(truth_tracking_rms_m=float(np.sqrt(np.mean(errors**2))), truth_tracking_max_m=float(errors.max()),
        truth_start_error_m=float(np.linalg.norm(truth['positions'][0]-track.start)),
        reference_time_s=float(reference['durations'].sum()), reference_replanning_count=len(raw['solves']))
    (folder/'comparison_audit.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    return result
