"""Offline body-centre audit; never supplies truth to the controller."""
import argparse
import contextlib
import io
import json
from pathlib import Path
import numpy as np
from pyulog import ULog
from scipy.spatial.transform import Rotation
from convex_timevarying_window.receding_gate_racing.localization_audit import audit


def body_center_position(origin_position,quat_xyzw):
    return np.asarray(origin_position)+Rotation.from_quat(quat_xyzw).apply(np.tile([0.,0.,-.24],(len(origin_position),1)))


def run(path):
    result=json.loads((path/'result.json').read_text())
    with contextlib.redirect_stdout(io.StringIO()):geometry=audit(path)
    log=ULog(str(path/'flight.ulg'))
    d=log.get_dataset('vehicle_local_position_groundtruth').data
    t=d['timestamp']*1e-6
    mask=(t>=30)&(t<=30+result['last_time_s']);t=t[mask]
    if len(t)<5:raise ValueError('insufficient flight truth samples')
    p=np.column_stack([d[k][mask] for k in ('x','y','z')])
    attitude=log.get_dataset('vehicle_attitude_groundtruth').data
    q=np.column_stack([np.interp(t,attitude['timestamp']*1e-6,attitude[k]) for k in ('q[1]','q[2]','q[3]','q[0]')])
    # The model origin is not the body/IMU centre. Differentiating origin
    # velocity alone includes rotation of the 0.24 m lever arm.
    p=body_center_position(p,q)
    v=np.gradient(p,t,axis=0);a=np.gradient(v,t,axis=0)
    ms=np.array([s['cycle_ms'] for s in result['solves']]);updates=result['reference_updates']
    report=dict(completed=result['completed'],failure=result['failure'],gates=len(result['crossings']),
                contacts=result['collision_detected'],last_time_s=result['last_time_s'],
                min_sampled_body_clearance_m=min(g['minimum_sampled_sphere_sleeve_clearance_m'] for g in geometry['groundtruth_sampled_sphere_sleeve_clearance']),
                max_truth_tilt_deg=geometry['maximum_groundtruth_tilt_deg'],
                max_sampled_body_speed_mps=float(np.linalg.norm(v[2:-2],axis=1).max()),
                max_sampled_body_acceleration_mps2=float(np.linalg.norm(a[2:-2],axis=1).max()),
                body_derivative_sampling_s=float(np.median(np.diff(t))),
                max_absolute_estimator_position_error_m=geometry['maximum_estimator_position_error_m'],
                mppi=dict(count=len(ms),median_ms=float(np.median(ms)),max_ms=float(ms.max()),over20=int(sum(ms>20)),over100=int(sum(ms>=100))),
                togt=dict(returned=len(updates),accepted=sum(bool(u['accepted']) for u in updates),
                          deadline_rejections=sum(isinstance(u['error'],dict) and u['error'].get('error')=='worker_deadline_exceeded' for u in updates)),
                evidence='Sampled ULog body-centre geometry/finite-difference dynamics. Not continuous certification; no truth in control.')
    (path/'ctbr_audit.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2));return report


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);args=p.parse_args();run(args.run)
