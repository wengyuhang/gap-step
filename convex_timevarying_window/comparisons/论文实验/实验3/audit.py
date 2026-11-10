"""Independent single-gate body truth, physical contact and motion audit."""
import json,re
from pathlib import Path
import numpy as np
from pyulog import ULog
from scipy.spatial.transform import Rotation
from convex_timevarying_window.three_phase_mppi.report import continuous_quaternions
from convex_timevarying_window.single_gate_stop_racing.planner import route_signs
from convex_timevarying_window.global_gate_minco_probe.flight import Progress
from .scenario import build_case,Case,parse_all_contacts
from convex_timevarying_window.comparisons.single_gate_swing.contacts import parse_contacts


def pose_audit(path,track,last_time):
    fields=lambda text:{k:float(v) for k,v in re.findall(r'([xyzw]):\s*([-+0-9.eE]+)',text)}
    rows={w.name:[] for w in track.windows}
    text=(path/'gate_pose.pbtxt').read_text(errors='replace')
    for message in re.split(r'(?m)(?=^header \{)',text):
        stamp=re.search(r'stamp\s*\{([^{}]*)\}',message)
        if stamp is None:continue
        sec=re.search(r'sec:\s*(\d+)',stamp[1]);ns=re.search(r'nsec:\s*(\d+)',stamp[1]);t=(int(sec[1]) if sec else 0)+(int(ns[1]) if ns else 0)*1e-9-30
        if t<0 or t>last_time:continue
        for block in re.split(r'(?m)^pose \{',message)[1:]:
            name=re.search(r'name:\s*"gate_([^"]+)"',block)
            if name is None or name[1] not in rows:continue
            pos=re.search(r'position\s*\{([^{}]*)\}',block);quat=re.search(r'orientation\s*\{([^{}]*)\}',block)
            if pos is None or quat is None:continue
            a,b=fields(pos[1]),fields(quat[1]);p=np.array([a.get(k,0) for k in ('x','y','z')]);q=np.array([b.get(k,0) for k in ('x','y','z','w')])
            w=next(w for w in track.windows if w.name==name[1]);c,R,*_=w.state_at(t)
            rows[w.name].append([t,np.linalg.norm(p-c),Rotation.from_matrix(Rotation.from_quat(q).as_matrix().T@R).magnitude()])
    result={}
    for w in track.windows:
        r=np.asarray(rows[w.name]);assert len(r)>3,'insufficient actual gate pose:'+w.name
        # Position command and pose publication can differ by one 1ms step.
        speed=2*np.pi*w.motion.frequency*(w.motion.amplitude if w.motion.kind=='translation' else w.motion.length*w.motion.angle)
        limit=2*speed*.001+1e-4
        result[w.name]=dict(samples=len(r),max_position_error_m=float(r[:,1].max()),position_limit_m=limit,max_rotation_error_rad=float(r[:,2].max()),passed=bool(r[:,1].max()<limit and r[:,2].max()<1e-6))
    (path/'gate_pose_audit.json').write_text(json.dumps(result,indent=2)+'\n')
    if not all(r['passed'] for r in result.values()):raise RuntimeError('physical_gate_pose_mismatch')
    return result


def audit(path,check_poses=True):
    path=Path(path);raw=json.loads((path/'result.json').read_text());track,_=build_case(Case(**raw['experiment_case']))
    u=ULog(str(path/'flight.ulg'));d=u.get_dataset('vehicle_local_position_groundtruth').data;a=u.get_dataset('vehicle_attitude_groundtruth').data
    t=d['timestamp']*1e-6;selection=(t>=30.)&(t<=30.+raw['last_time_s']);t=t[selection]
    if len(t)<3:raise RuntimeError('insufficient independent truth')
    q=continuous_quaternions(np.column_stack([a[k] for k in ('q[1]','q[2]','q[3]','q[0]')]))
    qs=np.column_stack([np.interp(t,a['timestamp']*1e-6,q[:,i]) for i in range(4)])
    positions=np.column_stack([d[k][selection] for k in ('x','y','z')])+Rotation.from_quat(qs).apply(np.tile([0.,0.,-.24],(len(t),1)))
    positions=positions[:,[1,0,2]]*[1,1,-1];times=t-30;cfg=raw['config'];signs=route_signs(track)
    progress=Progress(list(track.windows),signs,cfg['radius'],cfg['frame_thickness'])
    for at,pos in zip(times,positions):progress.observe(pos,float(at))
    all_events=[]
    for w,sign in zip(track.windows,signs):
        z=sign*((positions-w.center0)@w.R[:,2]);events=[]
        for k in np.flatnonzero(z[:-1]*z[1:]<0):
            fraction=abs(z[k])/(abs(z[k])+abs(z[k+1]));at=times[k]+fraction*(times[k+1]-times[k]);pos=(1-fraction)*positions[k]+fraction*positions[k+1]
            q,_=w.world_to_local(pos,at)
            if w.aperture.contains(q):events.append(dict(time_s=float(at),forward=bool(z[k]>0),q=q.tolist()))
        all_events.append(events)
    once=all(len(e)==1 and e[0]['forward'] for e in all_events)
    contacts=parse_all_contacts(path,track,30+raw['last_time_s'])
    collision=any(c['contact_pair_count'] for c in contacts[:-1]);goal_error=float(np.linalg.norm(positions[-1]-track.goal))
    passed=bool(raw['completed'] and progress.gate==len(track.windows) and once and not any(c['contact_pair_count'] for c in contacts[:-1]))
    verdict='collision' if collision else 'completed_no_contact' if passed else 'missed_gate' if 'missed' in str(raw['failure']) else 'not_completed'
    result=dict(case=raw['experiment_case'],variant=raw['experiment_variant'],passed=passed,verdict=verdict,completed=raw['completed'],collision_detected=collision,
        frame_collision=any(c['contact_pair_count'] for c in contacts[:-1]),ground_collision=bool(contacts[-1]['contact_pair_count']),
        flight_time_s=raw['flight_time_s'] if passed else None,last_time_s=raw['last_time_s'],failure=raw['failure'],actual_gate_count=progress.gate,
        crossings=progress.crossings,body_exits=progress.exits,all_opening_events=all_events,exactly_once=once,truth_goal_error_m=goal_error,
        actual_distance_m=float(np.linalg.norm(np.diff(positions,axis=0),axis=1).sum()),contacts=contacts,
        planner_requests=len(raw['solves']),planner_max_ms=max((s['cycle_ms'] for s in raw['solves']),default=None),
        mpc_max_ms=max((s['cycle_ms'] for s in raw['mpc_cycles'] if s.get('time',0)>=0),default=None),
        global_planning_s=raw.get('global_planning_s'),nominal_time_s=raw.get('nominal_time_s'),
        evidence='native ULog body truth + one full-sleeve and one ground contact channel')
    if check_poses:result['gate_pose_audit']=pose_audit(path,track,raw['last_time_s'])
    reference=np.load(path/'reference_trajectory.npz')
    from .planning import frozen_plan
    plan=frozen_plan(dict(durations=reference['durations'],coefficients=reference['coefficients']))
    errors=np.linalg.norm(positions-plan.sample_many(times),axis=1)
    velocity=np.column_stack([d[k][selection] for k in ('vx','vy','vz')])
    result.update(truth_tracking_rms_m=float(np.sqrt(np.mean(errors**2))),
        truth_tracking_max_m=float(errors.max()),truth_goal_speed_mps=float(np.linalg.norm(velocity[-1])),
        old_terminal_threshold_used=False,terminal_error_used_for_acceptance=False,
        finish_definition='full_reference_clock_and_measurement_and_one_actual_body_exit',
        full_reference_execution_time_s=raw['flight_time_s'] if passed else None)
    np.savez_compressed(path/'truth_trace.npz',times=times,positions=positions)
    (path/'comparison_audit.json').write_text(json.dumps(result,indent=2)+'\n');return result
