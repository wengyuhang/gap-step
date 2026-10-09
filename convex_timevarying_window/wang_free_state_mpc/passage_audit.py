"""Independent, sampled all-event audit; never feeds truth into control."""
import json
from pathlib import Path
import numpy as np
from .events import plane_events


def audit_positions(windows, signs, positions, times, radius, thickness):
    events, exits, violations = [], [], []
    for wi, w in enumerate(windows):
        found, z = plane_events(w, signs[wi], positions, times, gate=wi,
                                nearby_margin=radius+thickness)
        events.extend(found)
        forward = [e for e in found if e['inside'] and e['direction']=='forward']
        if forward:
            first = forward[0]['time']
            if any(e['direction']=='backward' and e['time']>first for e in found):
                violations.append(f'returned_near_gate:{wi}')
            after = np.flatnonzero((times>first) & (z<=-(radius+thickness)))
            if len(after): exits.append(dict(gate=wi, time=float(times[after[0]])))
    events.sort(key=lambda e:e['time'])
    inside = [e for e in events if e['inside']]
    sequence = [(e['gate'], e['direction']) for e in inside]
    if sequence != [(wi, 'forward') for wi in range(len(windows))]:
        violations.append('not_exactly_one_ordered_forward_passage_per_gate')
    if len(exits) != len(windows): violations.append('incomplete_body_exit')
    if len(inside)==len(windows) and len(exits)==len(windows):
        for exit_event, next_event in zip(exits[:-1], inside[1:]):
            if exit_event['time']>=next_event['time']: violations.append('next_gate_before_body_exit')
    return dict(passed=not violations, violations=violations, events=events,
                aperture_events=inside, body_exits=exits,
                evidence='Sampled finite-gate event audit, not continuous certification',
                audit_step_s=float(np.max(np.diff(times))) if len(times)>1 else None)


def audit_run(path):
    from pyulog import ULog
    from scipy.spatial.transform import Rotation
    from convex_dynamic_seven_window_gazebo.x500_togt.experiment import build_track
    from convex_timevarying_window.receding_gate_racing.scenario import vary_track
    from convex_timevarying_window.three_phase_mppi.report import continuous_quaternions
    path = Path(path); r = json.loads((path/'result.json').read_text())
    duration = float(r.get('flight_time_s') or r['last_time_s'])
    times = np.arange(0., duration, .01); absolute = (times+30.)*1e6
    if len(times)<2: return dict(passed=False, violations=['insufficient_flight_data'])
    ulog = ULog(str(path/'flight.ulg'))
    p = ulog.get_dataset('vehicle_local_position_groundtruth').data
    a = ulog.get_dataset('vehicle_attitude_groundtruth').data
    if any(d['timestamp'][0]>absolute[0] or d['timestamp'][-1]<absolute[-1] for d in (p, a)):
        return dict(passed=False, violations=['truth_does_not_cover_flight'])
    real = np.column_stack([np.interp(absolute, p['timestamp'], p[k]) for k in ('x','y','z')])
    raw = continuous_quaternions(np.column_stack([a[k] for k in ('q[1]','q[2]','q[3]','q[0]')]))
    quat = np.column_stack([np.interp(absolute, a['timestamp'], raw[:,k]) for k in range(4)])
    real += Rotation.from_quat(quat).apply(np.tile([0.,0.,-.24], (len(times),1)))
    positions = real[:,[1,0,2]]*np.array([1.,1.,-1.])
    track, _ = build_track()
    vary_track(track, r['phase'], r['layout'], r['motion_rate'], r['translation_scale'], r['rotation_scale'])
    windows = [track.windows[i] for i in track.order]; signs = []
    for wi, w in enumerate(windows):
        source = track.start if wi==0 else windows[wi-1].center0
        signs.append(float(np.sign(w.state_at(0.)[1][:,2]@(source-w.center0)) or 1.))
    result = audit_positions(windows, signs, positions, times, r['config']['radius'], r['config']['frame_thickness'])
    result['groundtruth_used_for_control'] = False
    return result
