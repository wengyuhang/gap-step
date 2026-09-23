"""Read-only ULog diagnostic; ground truth never enters the controller."""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import numpy as np
from pyulog import ULog
from scipy.spatial.transform import Rotation
from convex_dynamic_seven_window_gazebo.x500_togt.experiment import build_track
from convex_timevarying_window.conditional_dual_constraint_cem.safety_penalty import polygon_halfspaces
from .scenario import vary_track


def audit(path):
    path=Path(path); result=json.loads((path/'result.json').read_text())
    log=ULog(str(path/'flight.ulg'))
    estimated=log.get_dataset('vehicle_local_position').data
    truth=log.get_dataset('vehicle_local_position_groundtruth').data
    attitude=log.get_dataset('vehicle_attitude_groundtruth').data
    end=30.+float(result.get('flight_time_s') or result['last_time_s'])
    times=np.arange(30.,end,.01)*1e6
    if len(times)==0: return {}
    def interpolate(data,fields):
        return np.column_stack([np.interp(times,data['timestamp'],data[k]) for k in fields])
    p=interpolate(estimated,['x','y','z']); v=interpolate(estimated,['vx','vy','vz'])
    real=interpolate(truth,['x','y','z'])
    q=interpolate(attitude,['q[1]','q[2]','q[3]','q[0]'])
    # Gazebo groundtruth is the x500 model origin; the merged base/IMU is
    # 0.24 m above it in FLU. Rotate that offset before comparing frames.
    real+=Rotation.from_quat(q).apply(np.tile([0.,0.,-.24],(len(times),1)))
    tilt=np.arccos(np.clip(Rotation.from_quat(q).as_matrix()[:,2,2],-1.,1.))
    origin=np.asarray(result.get('origin_enu_m',[-16.,4.,-5.76]))
    origin_ned=origin[[1,0,2]]*np.array([1.,1.,-1.])
    error=p+origin_ned-real
    gps=log.get_dataset('vehicle_gps_position').data
    mask=(gps['timestamp']>=times[0])&(gps['timestamp']<=times[-1])
    body_enu=real[:,[1,0,2]]*np.array([1.,1.,-1.])
    track,_=build_track(); vary_track(track,result.get('phase',0.),result.get('layout',0.),result.get('motion_rate',1.),result.get('translation_scale',1.),result.get('rotation_scale',1.))
    sleeve_audit=[]
    for i,w in enumerate(track.windows):
        local=np.array([np.r_[*w.world_to_local(p,float(t/1e6-30.))]
                        for p,t in zip(body_enu,times)])
        q=local[:,:2]; z=local[:,2]
        if w.aperture.kind=='circle':
            distance=np.abs(np.linalg.norm(q,axis=1)-(w.aperture.radius+.065))
        else:
            hs=polygon_halfspaces(w.aperture.physical_vertices)
            vertices=np.array([np.linalg.solve(np.stack([hs.normals[j-1],hs.normals[j]]),
                          np.array([hs.offsets[j-1],hs.offsets[j]])+.065) for j in range(len(hs.offsets))])
            distance=np.full(len(q),np.inf)
            for a,b in zip(vertices,np.roll(vertices,-1,axis=0)):
                edge=b-a; alpha=np.clip((q-a)@edge/(edge@edge),0.,1.)
                distance=np.minimum(distance,np.linalg.norm(q-a-alpha[:,None]*edge,axis=1))
        clearance=np.hypot(distance,z)-result['config']['radius']-.085
        sleeve_audit.append({'gate':i,'minimum_sampled_sphere_sleeve_clearance_m':float(clearance.min())})
    data={'estimator_position_rmse_m':float(np.sqrt(np.mean(np.sum(error**2,axis=1)))),
          'maximum_estimator_position_error_m':float(np.linalg.norm(error,axis=1).max()),
          'velocity_correlated_xy_bias_seconds':[float(np.linalg.lstsq(np.column_stack([v[:,k],np.ones(len(times))]),error[:,k],rcond=None)[0][0]) for k in (0,1)],
          'gps_timestamp_minus_sample_ms':float(np.median((gps['timestamp'][mask].astype(float)-gps['timestamp_sample'][mask])/1000)),
          'origin_from_run': 'origin_enu_m' in result,
          'world_body_position_min':body_enu.min(axis=0).tolist(),
          'world_body_position_max':body_enu.max(axis=0).tolist(),
          'groundtruth_sampled_sphere_sleeve_clearance':sleeve_audit,
          'audit_step_s':.01,
          'maximum_groundtruth_tilt_deg':float(np.rad2deg(tilt).max()),
          'room_clearance_passed':bool(np.all(body_enu[:,0]>-28.5) and np.all(body_enu[:,0]<28.5)
                                     and np.all(body_enu[:,1]<22.5) and np.all(body_enu[:,2]>-5.5)
                                     and np.all(body_enu[:,2]<15.5)),
          'groundtruth_used_for_control':False}
    (path/'localization_audit.json').write_text(json.dumps(data,indent=2)+'\n')
    print(json.dumps(data,indent=2)); return data


if __name__=='__main__':
    parser=argparse.ArgumentParser(); parser.add_argument('run'); args=parser.parse_args(); audit(args.run)
