"""ULog true body-centre events, scored frame/ground contacts and full time."""
import json
from pathlib import Path
import numpy as np
from pyulog import ULog
from scipy.spatial.transform import Rotation
from convex_timevarying_window.three_phase_mppi.report import continuous_quaternions
from .scenario import Case, build_case


def audit(path, *, track=None, condition=None):
    path = Path(path)
    raw = json.loads((path/'result.json').read_text())
    if track is None:
        track,_ = build_case(Case(**raw['case']))
    window = track.windows[0]
    rotation = window.state_at(0.)[1]
    ulog = ULog(str(path/'flight.ulg'))
    d = ulog.get_dataset('vehicle_local_position_groundtruth').data
    a = ulog.get_dataset('vehicle_attitude_groundtruth').data
    absolute_t = d['timestamp']*1e-6
    selected = (absolute_t>=30.) & (absolute_t<=30.+raw['last_time_s'])
    absolute_t = absolute_t[selected]
    if len(absolute_t)<3:
        raise RuntimeError('insufficient truth samples')
    quats = continuous_quaternions(np.column_stack([a[k] for k in ('q[1]','q[2]','q[3]','q[0]')]))
    quats = np.column_stack([np.interp(absolute_t,a['timestamp']*1e-6,quats[:,i]) for i in range(4)])
    positions = np.column_stack([d[k][selected] for k in ('x','y','z')])
    positions += Rotation.from_quat(quats).apply(np.tile([0.,0.,-.24],(len(absolute_t),1)))
    positions = positions[:,[1,0,2]]*[1,1,-1]
    times = absolute_t-30.
    centers = np.array([window.state_at(float(t))[0] for t in times])
    gate_velocity = np.array([window.motion.translation(float(t))[1] for t in times])
    local = (positions-centers)@rotation
    sign = float(np.sign((track.start-window.center0)@rotation[:,2]))
    z = sign*local[:,2]
    velocity = np.gradient(positions,times,axis=0)
    relative = (velocity-gate_velocity)@rotation
    events = []
    for k in np.flatnonzero(z[:-1]*z[1:]<0):
        alpha = abs(z[k])/(abs(z[k])+abs(z[k+1]))
        t = times[k]+alpha*(times[k+1]-times[k])
        point = (1-alpha)*local[k]+alpha*local[k+1]
        events.append(dict(time_s=float(t),forward=bool(z[k]>0),inside=bool(window.aperture.contains(point[:2])),
            point_local_m=point[:2].tolist(),gate_speed_mps=float(np.linalg.norm(window.motion.translation(t)[1])),
            relative_lateral_speed_mps=float(np.interp(t,times,relative[:,1])),
            normal_speed_mps=float(-sign*np.interp(t,times,relative[:,2]))))
    once = len(events)==1 and events[0]['forward'] and events[0]['inside']
    h = raw['config']['radius']+raw['config']['frame_thickness']
    entry = exit = None
    if once:
        at = events[0]['time_s']
        before = np.flatnonzero((times<at)&(z>=h))
        after = np.flatnonzero((times>at)&(z<=-h))
        if len(before): entry = float(times[before[-1]])
        if len(after): exit = float(times[after[0]])
    goal_error = float(np.linalg.norm(positions[-1]-track.goal))
    collision = bool(raw['collision_detected'])
    accepted = sum(s['accepted'] for s in raw['solves'])
    passed = bool(raw['completed'] and once and exit is not None and goal_error<.55 and not collision)
    verdict = ('collision' if collision else 'completed_no_contact' if passed else
               'online_no_accepted_plan' if accepted==0 and raw['failure']=='timeout' else
               'missed_gate' if 'missed_opening' in str(raw['failure']) or events and not once else 'not_completed')
    planning = [s['cycle_ms'] for s in raw['solves']]
    mpc = [s['cycle_ms'] for s in raw['mpc_cycles'] if s.get('time',0)>=0]
    if condition is None:
        condition = dict(frequency_hz=raw['case']['frequency'],amplitude_m=raw['case']['amplitude'])
    result = dict(**condition,
        variant=raw['comparison_variant'],verdict=verdict,passed=passed,completed=raw['completed'],
        collision_detected=collision,frame_collision=raw['contacts'][0]['contact_pair_count']>0,
        ground_collision=raw['contacts'][1]['contact_pair_count']>0,contacts=raw['contacts'],
        flight_time_s=raw['flight_time_s'] if passed else None,raw_completion_time_s=raw['flight_time_s'],
        failure=raw['failure'],crossings=events,ordered_once=once,body_exit_confirmed=exit is not None,
        entry_time_s=entry,exit_time_s=exit,body_passage_duration_s=exit-entry if exit is not None and entry is not None else None,
        truth_goal_error_m=goal_error,truth_endpoint_speed_mps=float(np.linalg.norm(velocity[-1])),
        truth_step_median_s=float(np.median(np.diff(times))),
        actual_distance_m=float(np.linalg.norm(np.diff(positions,axis=0),axis=1).sum()),
        maximum_speed_mps=float(np.linalg.norm(velocity,axis=1).max()),accepted_plan_count=accepted,
        planner_max_ms=max(planning,default=None),planner_over_100ms=sum(x>100 for x in planning),
        mpc_max_ms=max(mpc,default=None),mpc_over_100ms=sum(x>100 for x in mpc),
        truth_start_position_m=positions[0].tolist(),truth_start_velocity_mps=velocity[0].tolist(),
        evidence='ULog true body centre; Gazebo frame/ground contacts in task interval; no closed-loop certificate')
    np.savez_compressed(path/'mechanism_trace.npz',times=times,positions=positions,gate_centers=centers,
        gate_velocity=gate_velocity,local_positions=local,relative_velocity_local=relative,velocity=velocity)
    (path/'comparison_audit.json').write_text(json.dumps(result,indent=2)+'\n')
    return result
