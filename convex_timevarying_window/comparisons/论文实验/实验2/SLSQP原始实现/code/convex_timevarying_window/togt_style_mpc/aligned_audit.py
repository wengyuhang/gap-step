"""Frozen-reference and first-received-setpoint audit after the flight."""
import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from pyulog import ULog
from .model import KF
from convex_timevarying_window.global_gate_minco_probe.phase_descent import load_plan


def main():
    p=argparse.ArgumentParser();p.add_argument('--flight-dir',required=True,type=Path)
    p.add_argument('--baseline-dir',required=True,type=Path);a=p.parse_args()
    flight=a.flight_dir; r=json.loads((flight/'result.json').read_text())
    old=json.loads((a.baseline_dir/'result.json').read_text())
    plan_dir=Path(r['global_plan_directory']); plan=load_plan(json.loads((plan_dir/'result.json').read_text()))
    truth=np.load(flight/'truth_trace.npz');t=truth['times'];pos=truth['positions']
    reference=np.array([plan.sample(float(at)) for at in t]);error=np.linalg.norm(pos-reference,axis=1)
    logs=[l for l in r['mpc_cycles'] if l.get('solver_status')==0 and l['time']>=0]
    lt=np.array([l['time'] for l in logs]); u=np.array([l['command'] for l in logs])
    encoded=np.column_stack([u[:,1:],(np.sqrt(u[:,0]/(4*KF))-150.)/850.])
    data={d.name:d.data for d in ULog(str(flight/'flight.ulg')).data_list if d.multi_id==0}
    sp=data['vehicle_rates_setpoint'];rt=sp['timestamp']*1e-6-30.
    ru=np.column_stack([sp['roll'],-sp['pitch'],-sp['yaw'],-sp['thrust_body[2]']])
    matches=[]
    for i,at in enumerate(lt):
        ids=np.flatnonzero((rt>=at)&(rt<=at+.1))
        if not len(ids):continue
        residual=np.max(abs(ru[ids]-encoded[i]),axis=1)
        good=ids[residual<2e-6]
        if len(good):
            first=good[0];matches.append([at,rt[first],logs[i]['execution_time_s'],
                rt[first]-logs[i]['execution_time_s'],float(np.max(abs(ru[first]-encoded[i])))])
    matches=np.asarray(matches)
    new_world=ET.parse(flight/'world.sdf');old_world=ET.parse(a.baseline_dir/'world.sdf')
    same=lambda path: [ET.tostring(x) for x in new_world.findall(path)]==[ET.tostring(x) for x in old_world.findall(path)]
    h=lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
    result=dict(reference_identical=h(flight/'reference_trajectory.npz')==h(a.baseline_dir/'reference_trajectory.npz'),
        reference_sha256=h(flight/'reference_trajectory.npz'),physics_identical=same('./world/physics'),
        world_models_identical=same('./world/model'),px4_parameter_overrides_identical=r['parameters']==old['parameters'],
        scene_identical=json.loads((flight/'scene.json').read_text())==json.loads((a.baseline_dir/'scene.json').read_text()),
        completed=r['completed'],flight_time_s=r['flight_time_s'],frame_contacts=sum(c['contact_pair_count'] for c in r['contacts']),
        nominal_duration_s=plan.end_time,finish_condition_unchanged='7 body exits, position <0.55 m, speed <1 m/s; not exact zero PVAJ',
        truth_position_error_max_m=float(error.max()),truth_position_error_rms_m=float(np.sqrt(np.mean(error**2))),
        truth_return_error_max_m=float(error[t>=49.76].max()),minimum_body_center_height_m=float(pos[:,2].min()),
        truth_goal_error_m=float(np.linalg.norm(pos[-1]-plan.sample(plan.end_time))),
        estimated_goal_speed_m_s=r['goal_speed_mps'],
        cycle_ms_p50_p95_max=np.percentile([l['cycle_ms'] for l in logs],[50,95,100]).tolist(),
        actual_mean_control_hz=float(len(logs)/(lt[-1]-lt[0])),
        prediction_lead_p50_p95_max_s=np.percentile([l['state_prediction_s'] for l in logs],[50,95,100]).tolist(),
        received_command_matches=len(matches),successful_racing_mpc_calls=len(logs),
        first_logged_receipt_minus_predicted_apply_p0_p50_p95_p100_s=np.percentile(matches[:,3],[0,50,95,100]).tolist(),
        first_logged_receipt_minus_predicted_apply_abs_p95_s=float(np.percentile(abs(matches[:,3]),95)),
        receipt_max_component_error=float(matches[:,4].max()),
        logged_intervals_median_s={name:float(np.median(np.diff(data[name]['timestamp']))*1e-6)
            for name in ['actuator_motors','vehicle_rates_setpoint','vehicle_angular_velocity','vehicle_local_position_groundtruth']},
        source_unchanged_during_flight=json.loads((flight/'verification.json').read_text())['sources_unchanged'],
        limitations=['One run, not a reliability or safety certificate.',
                    'First logged receiver updates have 4 ms resolution, not exact physical motor application timestamps.',
                    'No ground-contact sensor; body-center minimum height and successful return are separately reported.',
                    'Several execution alignment changes are combined; not a single-factor causal attribution.'])
    np.savez_compressed(flight/'tracking_error_audit.npz',times=t,positions=pos,reference=reference,errors=error)
    np.savez_compressed(flight/'first_receipt_audit.npz',rows=matches)
    (flight/'execution_alignment_audit.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()
