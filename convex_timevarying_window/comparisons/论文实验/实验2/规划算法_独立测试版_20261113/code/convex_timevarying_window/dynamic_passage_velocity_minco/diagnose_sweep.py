"""Read recorded flights only; never runs or changes a planner/controller."""
import json
from pathlib import Path
import numpy as np
from pyulog import ULog
from .trajectory import Plan
from convex_timevarying_window.checked_soft_mpc_racing.mpc import reference_states, LOWER, UPPER
from 复现.polynomial_dynamic_racing_2023.px4_fixed_plane.audit import truth


def diagnostic(path):
    raw = json.loads((path/'result.json').read_text())
    audits = json.loads((path/'comparison_audit.json').read_text()) if (path/'comparison_audit.json').exists() else json.loads((path/'physical_audit.json').read_text())
    goal = np.array(json.loads((path/'scene.json').read_text())['goal'])
    ts, ps = truth(path, raw['last_time_s'])
    accepted = [s for s in raw['solves'] if s['accepted']]
    plans = [Plan(s['made_at'], 0, np.array(s['durations']), np.array(s['coefficients']),
                  np.array(s['variables']), 0., 0., s['status'], 1) for s in accepted]
    cross = audits['crossings'][0]['time_s'] if len(audits['crossings']) == 1 else None
    v = np.gradient(ps, ts, axis=0)
    tr = np.genfromtxt(path/'telemetry_live.csv', delimiter=',', names=True)
    result = dict(case=raw['case'], completed=raw['completed'], failure=raw['failure'],
        flight_time_s=raw['flight_time_s'], crossing_s=cross,
        tail_time_s=None if cross is None or raw['flight_time_s'] is None else raw['flight_time_s']-cross,
        planning_requests=len(raw['solves']), accepted=len(accepted),
        first_tracking_error_trigger=next((e for e in raw['planning_events'] if e.get('reason')=='tracking_error'), None),
        first_error_above_035_s=next((float(x['t']) for x in tr if x['error']>.35),None),
        source='Saved PVA polynomials, telemetry, MPC command logs and 4ms body-center truth. No reruns.')
    last = plans[-1]
    tail_mask = ts >= (cross if cross is not None else last.crossing_times[0])
    result['actual'] = dict(goal_error_final_m=float(np.linalg.norm(ps[-1]-goal)),
        min_xyz_m=ps.min(axis=0).tolist(), max_xyz_m=ps.max(axis=0).tolist(),
        path_m=float(np.linalg.norm(np.diff(ps, axis=0),axis=1).sum()),
        tail_path_m=float(np.linalg.norm(np.diff(ps[tail_mask],axis=0),axis=1).sum()),
        tail_max_x_past_goal_m=float(max(0.,ps[tail_mask,0].max()-goal[0])) if tail_mask.any() else None,
        max_tracking_error_m=float(tr['error'].max()))
    snapshots=[]
    for t in (0.,1.,2.,3.,4.,5.,6.,7.,8.,10.,12.,15.,20.,30.,39.):
        if t < ts[-1]:
            i=int(np.argmin(abs(ts-t))); j=int(np.argmin(abs(tr['t']-t)))
            snapshots.append(dict(time_s=t, position_m=ps[i].tolist(), speed_mps=float(np.linalg.norm(v[i])),
                error_m=float(tr['error'][j]), gate=int(tr['gate'][j])))
    result['actual_snapshots']=snapshots
    nominal=[]
    for index in sorted(set([0,len(plans)-1])):
        p=plans[index]; grid=np.arange(p.made_at,p.end_time,.004)
        pos=p.sample_many(grid); vel=p.sample_many(grid,1); acc=p.sample_many(grid,2); jerk=p.sample_many(grid,3)
        refs,ff=reference_states(np.c_[pos,vel,acc],.004,np.array([1.,0.,0.,0.]))
        force=acc+np.array([0.,0.,9.8066]); tilt=np.degrees(np.arctan2(np.linalg.norm(force[:,:2],axis=1),force[:,2]))
        ref_thrust=8.54858e-6*np.sum(refs[:,13:17]**2,axis=1)
        before=grid<p.crossing_times[0]; after=~before
        stages={}
        for label,mask in [('before_gate',before),('after_gate',after)]:
            if not mask.any():continue
            stages[label]=dict(max_speed_mps=float(np.linalg.norm(vel[mask],axis=1).max()),
                max_acc_mps2=float(np.linalg.norm(acc[mask],axis=1).max()),
                max_jerk_mps3=float(np.linalg.norm(jerk[mask],axis=1).max()),
                min_vertical_total_acc_mps2=float(force[mask,2].min()),
                max_tilt_deg=float(tilt[mask].max()),max_drag_corrected_thrust_n=float(ref_thrust[mask].max()),
                thrust_over_physical_fraction=float(np.mean(ref_thrust[mask]>UPPER[0])),
                max_rate_abs_rps=np.max(abs(refs[mask,10:13]),axis=0).tolist())
        nominal.append(dict(index=index,accepted_at_s=accepted[index]['time'],made_at=p.made_at,
            durations_s=p.durations.tolist(), crossing_s=float(p.crossing_times[0]),end_s=p.end_time,
            dynamic_residual=accepted[index]['violation'],original_J=accepted[index]['optimizer_stats']['final_objective'],
            max_tilt_time_s=float(grid[np.argmax(tilt)]),stages=stages,
            tail_path_m=float(np.linalg.norm(np.diff(pos[after],axis=0),axis=1).sum()),
            tail_max_x_past_goal_m=float(max(0.,pos[after,0].max()-goal[0]))))
    result['nominal']=nominal
    logs=[s for s in raw['mpc_cycles'] if 'command' in s and s['time']>=0]
    lt=np.array([s['time'] for s in logs]); cmds=np.array([s['command'] for s in logs]); tilt=np.array([s['tilt_deg'] for s in logs])
    stages={}
    for label,mask in [('all',lt>=0),('before_gate',lt<cross if cross is not None else lt>=0),
                       ('after_gate',lt>=cross if cross is not None else lt<0)]:
        selected=[s for s,m in zip(logs,mask) if m]
        if not selected:continue
        saturated=np.isclose(cmds[mask],LOWER,atol=1e-5)|np.isclose(cmds[mask],UPPER,atol=1e-5)
        stages[label]=dict(max_actual_tilt_deg=float(tilt[mask].max()),input_saturated_fraction=saturated.mean(axis=0).tolist(),
            maximum_command_abs=np.max(abs(cmds[mask]),axis=0).tolist(),
            qp_nonconverged=sum(not q['converged'] for s in selected for q in s['box_solves']),
            rti_all_zero_steps=sum(all(a==0 for a in s['nonlinear_steps']) for s in selected),cycles=len(selected))
    result['control']=stages
    execution=np.load(path/'execution.npz')['rows']
    motor_force=8.54858e-6*np.sum(execution[:,14:18]**2,axis=1)
    cap=(execution[:,0]>3.2)&np.isclose(execution[:,18],UPPER[0],atol=1e-5)
    result['collective_allocation']=dict(
        force_is_telemetry_motor_lag_estimate_not_force_sensor=True,
        maximum_command_n=float(UPPER[0]), max_command_samples_after_3p2s=int(cap.sum()),
        median_rotor_estimated_force_under_max_command_n=float(np.median(motor_force[cap])) if cap.any() else None)
    allocator=ULog(str(path/'flight.ulg')).get_dataset('control_allocator_status').data
    at=allocator['timestamp']/1e6-30
    result['collective_allocation']['px4_stages']={}
    for lo,hi in ((0.,2.),(2.,3.),(3.2,5.),(5.,10.),(10.,13.)):
        keep=(at>=lo)&(at<min(hi,raw['last_time_s']))
        if keep.any():
            result['collective_allocation']['px4_stages'][f'{lo:g}-{hi:g}s']=dict(
                samples=int(keep.sum()),
                thrust_unachieved_fraction=float(np.mean(allocator['thrust_setpoint_achieved'][keep]==0)),
                torque_unachieved_fraction=float(np.mean(allocator['torque_setpoint_achieved'][keep]==0)))
    return result


def plot(paths, batch):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    selected=[paths[i] for i in (0,1,2,5)]
    fig,axes=plt.subplots(3,4,figsize=(15,8),squeeze=False)
    for col,path in enumerate(selected):
        raw=json.loads((path/'result.json').read_text());tr=np.genfromtxt(path/'telemetry_live.csv',delimiter=',',names=True)
        e=np.load(path/'execution.npz')['rows'];mask=(e[:,0]>=0)&(e[:,0]<=raw['last_time_s'])
        axes[0,col].plot(tr['t'],tr['rz'],'--',label='Reference')
        axes[0,col].plot(tr['t'],tr['z'],label='Measured')
        axes[0,col].set_title(f"{raw['case']['frequency']:g} Hz / {raw['case']['amplitude']:g} m")
        axes[1,col].plot(e[mask,0],e[mask,18],label='Command')
        axes[1,col].plot(e[mask,0],8.54858e-6*np.sum(e[mask,14:18]**2,axis=1),label='Motor-based estimate',alpha=.7)
        axes[2,col].plot(tr['t'],tr['error']);axes[2,col].axhline(.35,color='red',linestyle=':')
        for ax in axes[:,col]:ax.grid(alpha=.2);ax.set_xlabel('Flight time (s)')
    for row,label in enumerate(('Altitude (m)','Collective thrust (N)','Position tracking error (m)')):axes[row,0].set_ylabel(label)
    axes[0,0].legend();axes[1,0].legend(fontsize=8)
    fig.tight_layout();fig.savefig(batch/'successful_flights_diagnosis.png',dpi=160);plt.close(fig)


if __name__=='__main__':
    base=Path(__file__).parent/'results'
    batch=base/'difficulty_sweep_v1_20261009'
    paths=[base/'flight_forward_v2_f3_a4_20261009_01']+[Path(r['directory']) for r in json.loads((batch/'summary.json').read_text())]
    results=[dict(directory=str(p),**diagnostic(p)) for p in paths]
    (batch/'DIAGNOSIS.json').write_text(json.dumps(results,indent=2,ensure_ascii=False)+'\n')
    plot(paths,batch)
    for r in results:
        print(json.dumps({k:r[k] for k in ('case','flight_time_s','crossing_s','tail_time_s','actual','first_error_above_035_s','nominal','control')},ensure_ascii=False))
