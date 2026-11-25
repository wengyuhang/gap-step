"""Read-only source/record audit, with derived evidence in a new directory."""
import argparse
import json
from pathlib import Path
import numpy as np
from scipy.optimize import root
from scipy.spatial.transform import Rotation
from pyulog import ULog
from .model import functions, MIXER, FMAX, FMIN, KF
from convex_timevarying_window.dynamic_passage_velocity_minco.hard_dynamics import flat_actuators
from convex_timevarying_window.dynamic_passage_velocity_minco.repaired_planner import flat_force
from convex_timevarying_window.global_gate_minco_probe.phase_descent import load_plan


def main():
    p=argparse.ArgumentParser();p.add_argument('--outdir',type=Path,required=True)
    p.add_argument('--plan-dir',type=Path,required=True);p.add_argument('--flight-dir',type=Path,required=True)
    p.add_argument('--layer-dir',type=Path,required=True);a=p.parse_args()
    a.outdir.mkdir(parents=True,exist_ok=False)
    plan=load_plan(json.loads((a.plan_dir/'result.json').read_text()))
    rhs,_,aero=functions();rows=[];failures=0
    for t in np.arange(0.,plan.end_time,.01):
        pos,v,acc,j,snap=[plan.sample(float(t),d) for d in range(5)]
        force,w,original=[np.asarray(q).ravel() for q in flat_actuators(v,acc,j,snap)]
        f=np.asarray(flat_force(v,acc)).ravel(); z=f/np.linalg.norm(f)
        y=np.cross(z,[0.,1.,0.]);y/=np.linalg.norm(y)
        R=np.column_stack([np.cross(y,z),y,z]);q=Rotation.from_matrix(R).as_quat()[[3,0,1,2]]
        state=np.r_[pos,v,q,w]; target=MIXER@original
        def residual(u):
            return MIXER@u+np.r_[0.,np.asarray(aero(state,u)).ravel()]-target
        solved=root(residual,original,tol=1e-10)
        err=float(np.max(abs(residual(solved.x))))
        if err>1e-7 or not np.isfinite(solved.x).all(): failures+=1
        acceleration=np.asarray(rhs(state,solved.x)).ravel()[3:6]
        rows.append([t,*original,*solved.x,err,np.linalg.norm(acceleration-acc),*w,force[0]])
    rows=np.asarray(rows);np.savez_compressed(a.outdir/'actuator_inverse.npz',rows=rows)
    mask=(rows[:,0]>=44)&(rows[:,0]<=47)
    original=rows[mask,1:5]; corrected=rows[mask,5:9]
    idx=np.unravel_index(np.argmax(corrected),corrected.shape)
    inverse=dict(grid_s=.01,scope='planned fixed-yaw attitude/rates; torque restored including rotor-speed-dependent aero moments',
        caveat='not a proof of infeasibility under arbitrary alternative yaw/state/reference; no motor slew constraint here',
        root_residual_failures=failures,rotor_limit_N=FMAX,window_s=[44,47],
        original_max_N=float(original.max()),corrected_max_N=float(corrected.max()),
        corrected_max_time_s=float(rows[mask,0][idx[0]]),corrected_max_rotor=int(idx[1]),
        corrected_max_over_pct=float(100*(corrected.max()/FMAX-1)),
        corrected_samples_over_fraction=float(np.mean(np.any(corrected>FMAX,axis=1))),
        remaining_acceleration_residual_max_m_s2=float(rows[mask,10].max()),
        samples=[dict(time_s=float(rows[i,0]),original_N=rows[i,1:5].tolist(),
                      corrected_N=rows[i,5:9].tolist(),force_torque_residual=float(rows[i,9]),
                      remaining_acceleration_residual=float(rows[i,10]))
                 for i in [int(round(t/.01)) for t in [44,44.5,45,45.5,46,46.5,47]]])
    u=ULog(str(a.flight_dir/'flight.ulg'));ds={d.name:d.data for d in u.data_list if d.multi_id==0}
    execution=np.load(a.flight_dir/'execution.npz')['rows'];tt=execution[:,0];cmd=execution[:,18:22]
    sp=ds['vehicle_rates_setpoint'];st=sp['timestamp']*1e-6-30.
    value=np.c_[sp['roll'],-sp['pitch'],-sp['yaw'],-sp['thrust_body[2]']]
    expected=np.c_[cmd[:,1:],(np.sqrt(cmd[:,0]/(4*KF))-150)/850]
    matches=[]
    for t,val in zip(st,value):
        if not 0<=t<=45.8:continue
        ids=np.flatnonzero((tt<=t+.004)&(tt>=t-.1))
        if not len(ids):continue
        residual=np.max(abs(expected[ids]-val),axis=1);k=int(residual.argmin())
        matches.append([t,float(residual[k]),float(t-tt[ids[k]])])
    matches=np.asarray(matches);np.savez_compressed(a.outdir/'received_setpoint_matches.npz',rows=matches)
    interface=dict(compared_received_samples=len(matches),maximum_component_error=float(matches[:,1].max()),
        all_match_within_1e_5=bool(np.all(matches[:,1]<1e-5)),
        measured_state_to_received_setpoint_age_p50_p95_max_s=np.percentile(matches[:,2],[50,95,100]).tolist(),
        caveat='ULog is sampled; verifies these received setpoints, not every packet or arbitrary absolute calibration',
        rate_axis='FLU to FRD [p,-q,-r]',throttle='(sqrt(F/(4 KF))-150)/850')
    layers={}
    for path in sorted(a.layer_dir.glob('*_result.json')):
        result=json.loads(path.read_text()); name=result['layer']
        data=np.load(a.layer_dir/(name+'.npz'))['rows']; chunks=[]
        for lo,hi in [(0,39),(39,43),(43,44),(44,45),(45,46),(46,47)]:
            take=(data[:,0]>=lo)&(data[:,0]<hi)
            if take.any(): chunks.append(dict(interval_s=[lo,hi],max_m=float(data[take,-1].max()),
                rms_m=float(np.sqrt(np.mean(data[take,-1]**2))),recorded_end_s=float(data[take,0][-1])))
        layers[name]=dict(result=result,intervals=chunks)
    audit=dict(reference_dynamics=inverse,px4_interface=interface,layers=layers,
        evidence='offline numerical/record diagnosis only; production controller and reference unchanged')
    (a.outdir/'audit.json').write_text(json.dumps(audit,indent=2)+'\n')
    print(json.dumps(dict(reference_dynamics=inverse,px4_interface=interface),indent=2))


if __name__=='__main__':main()
