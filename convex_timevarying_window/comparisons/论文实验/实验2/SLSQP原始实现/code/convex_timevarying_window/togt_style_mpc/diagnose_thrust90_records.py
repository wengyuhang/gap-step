"""Read-only flight audit. Derived outputs are not a new flight or certificate."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from scipy.signal import butter, lfilter
from scipy.spatial.transform import Rotation, Slerp
from pyulog import ULog
from convex_timevarying_window.single_gate_stop_racing.px4_model import native_dynamics
from .model import KF


def main():
    p=argparse.ArgumentParser(); p.add_argument('--flight-dir',type=Path,required=True)
    p.add_argument('--outdir',type=Path,required=True); a=p.parse_args()
    a.outdir.mkdir(parents=True,exist_ok=False)
    execution=np.load(a.flight_dir/'execution.npz')['rows']
    truth=np.load(a.flight_dir/'truth_trace.npz'); tt=truth['times']; pos=truth['positions']
    u=ULog(str(a.flight_dir/'flight.ulg'))
    ds={d.name:d.data for d in u.data_list if d.multi_id==0}
    interp=lambda t,ts,ys: np.column_stack([np.interp(t,ts,ys[:,i]) for i in range(ys.shape[1])])
    clock=lambda d: d['timestamp']*1e-6-30.
    t=execution[:,0]; truth_pos=interp(t,tt,pos)
    truth_vel=interp(t,tt,np.gradient(pos,tt,axis=0))
    ag=ds['vehicle_attitude_groundtruth']; at=clock(ag)
    q=np.column_stack([ag[f'q[{i}]'] for i in [1,2,3,0]])
    rotations=Rotation.from_matrix(np.array([[0,1,0],[1,0,0],[0,0,-1]])) * Rotation.from_quat(q) * Rotation.from_matrix(np.diag([1,-1,-1]))
    qt=Slerp(at,rotations)(np.clip(t,at[0],at[-1]))
    qe=Rotation.from_quat(execution[:,[8,9,10,7]])
    angle=np.degrees((qt.inv()*qe).magnitude())
    position_error=np.linalg.norm(execution[:,1:4]-truth_pos,axis=1)
    velocity_error=np.linalg.norm(execution[:,4:7]-truth_vel,axis=1)
    logs=[json.loads(line) for line in (a.flight_dir/'mpc_live.jsonl').read_text().splitlines()]
    logs=[r for r in logs if r.get('solver_status')==0 and r['time']>=0]
    lt=np.array([r['time'] for r in logs]); thrust=np.array([r['rotor_thrust_N'] for r in logs])
    intervals=[]
    for lo,hi in [(0,39),(39,44),(44,49.764),(49.764,51),(51,54)]:
        m=(t>=lo)&(t<hi); lm=(lt>=lo)&(lt<hi)
        intervals.append(dict(interval_s=[lo,hi],
            position_estimation_error_p50_p95_max_m=np.percentile(position_error[m],[50,95,100]).tolist(),
            velocity_estimation_error_p50_p95_max_m_s=np.percentile(velocity_error[m],[50,95,100]).tolist(),
            attitude_estimation_error_p50_p95_max_deg=np.percentile(angle[m],[50,95,100]).tolist(),
            mpc_any_rotor_at_limit_fraction=float(np.mean(np.any(thrust[lm]>8.54858-1e-4,axis=1))),
            mpc_all_rotors_at_limit_fraction=float(np.mean(np.all(thrust[lm]>8.54858-1e-4,axis=1)))))
    s=ds['sensor_combined']; w=ds['vehicle_angular_velocity']
    raw=np.column_stack([s[f'gyro_rad[{i}]'] for i in range(3)])
    st=clock(s); wt=w['timestamp_sample']*1e-6-30.
    actual_d=np.column_stack([w[f'xyz_derivative[{i}]'] for i in range(3)])
    bb,aa=butter(2,40.,fs=250.); filtered=lfilter(bb,aa,raw,axis=0)
    alpha=.004/(.004+1/(2*np.pi*20.)); dterms={}
    for name,rates in [('raw_then_derivative',raw),('lowpass_then_derivative',filtered)]:
        derivative=lfilter([alpha],[1,-(1-alpha)],np.gradient(rates,.004,axis=0),axis=0)
        residual=interp(wt,st,derivative)-actual_d; m=(wt>1)&(wt<49)
        dterms[name]=np.sqrt(np.mean(residual[m]**2,axis=0)).tolist()
    motors=ds['actuator_motors']; torque=ds['vehicle_torque_setpoint']; collective=ds['vehicle_thrust_setpoint']
    alloc=native_dynamics()[3]; checks=[]
    for j,stamp in enumerate(motors['timestamp']):
        ti=np.searchsorted(torque['timestamp'],stamp); ci=np.searchsorted(collective['timestamp'],stamp)
        if ti>=len(torque['timestamp']) or ci>=len(collective['timestamp']): continue
        if torque['timestamp'][ti]!=stamp or collective['timestamp'][ci]!=stamp: continue
        time=stamp*1e-6-30.
        if not 0<=time<54: continue
        throttle=-collective['xyz[2]'][ci]
        tor=np.array([torque[f'xyz[{i}]'][ti] for i in range(3)])*np.array([1,-1,-1])
        pred=(np.asarray(alloc(np.r_[throttle,tor])).ravel()-150.)/850.
        actual=np.array([motors[f'control[{i}]'][j] for i in range(4)])
        checks.append([time,np.max(abs(pred-actual)),4*KF*(150+850*throttle)**2,KF*np.sum((150+850*actual)**2)])
    checks=np.array(checks)
    sp=ds['vehicle_rates_setpoint']; received=np.column_stack([sp['roll'],-sp['pitch'],-sp['yaw'],-sp['thrust_body[2]']])
    expected=np.column_stack([execution[:,19:22],(np.sqrt(execution[:,18]/(4*KF))-150.)/850.])
    matches=[]
    for time,value in zip(clock(sp),received):
        if not 0<=time<54: continue
        ids=np.flatnonzero((t>=time-.1)&(t<=time+.004))
        if not len(ids): continue
        residual=np.max(abs(expected[ids]-value),axis=1); j=int(residual.argmin())
        matches.append([time,float(residual[j]),float(time-t[ids[j]])])
    matches=np.array(matches)
    audit=dict(intervals=intervals,derivative_log_rmse_rad_s2=dterms,
        derivative_log_method='250 Hz sensor_combined interval-averaged gyro; centered differences; compare same-time published derivative, 1<t<49. No fitted delay.',
        allocator_exact_timestamp_samples=len(checks),allocator_error_p50_p95_max=np.percentile(checks[:,1],[50,95,100]).tolist(),
        received_command_samples=len(matches),received_command_max_component_error=float(matches[:,1].max()),
        received_state_age_p50_p95_max_s=np.percentile(matches[:,2],[50,95,100]).tolist(),
        px4_source='https://raw.githubusercontent.com/PX4/PX4-Autopilot/83c4f4e580558816f0e398918f66358988004696/src/modules/sensors/vehicle_angular_velocity/VehicleAngularVelocity.cpp',
        caveats=['Body-centre velocity from 20 ms truth positions; not raw GT origin velocity.',
                 'PX4 angular_velocity_groundtruth is Euler-angle differentiation, not body angular velocity; deliberately not used.',
                 'Allocator rotor targets are not instantaneous physical rotor forces.',
                 'Received command age includes possible hold time: sampled ULog does not identify every first packet arrival.',
                 'No causal claim follows merely from saturation or estimation error.'],
        evidence='existing Gazebo flight records; no new flight',
        inputs={str(a.flight_dir/f):hashlib.sha256((a.flight_dir/f).read_bytes()).hexdigest()
                for f in ['execution.npz','truth_trace.npz','flight.ulg','mpc_live.jsonl']})
    np.savez_compressed(a.outdir/'measurement_errors.npz',times=t,position=position_error,velocity=velocity_error,attitude_deg=angle)
    np.savez_compressed(a.outdir/'allocator.npz',rows=checks)
    np.savez_compressed(a.outdir/'received_matches.npz',rows=matches)
    (a.outdir/'audit.json').write_text(json.dumps(audit,indent=2)+'\n')
    print(json.dumps(audit,indent=2))


if __name__=='__main__': main()
