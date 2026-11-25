"""Read-only comparison with matching frozen racing baselines.

Old acceptance labels are preserved. A separate common body-truth diagnostic
exposes differences in dynamics limits instead of silently relabelling history.
Only the explicitly supplied NEW output directory is written.
"""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from pyulog import ULog
from scipy.spatial.transform import Rotation
from convex_timevarying_window.three_phase_mppi.report import continuous_quaternions

CHECKED=Path('convex_timevarying_window/comparisons/amplitude_racing/results/center_motion_checked_20260923/summary.json')
WANG=Path('convex_timevarying_window/comparisons/paper_baselines_2026/results/center_motion_wang2023_20260923/summary.json')


def condition(row):
    return (float(row.get('motion_rate',row.get('rate'))),float(row['translation_scale']),
            float(row['rotation_scale']),float(row['phase']))


def body_dynamics(run,duration):
    u=ULog(str(run/'flight.ulg'))
    d=u.get_dataset('vehicle_local_position_groundtruth').data
    a=u.get_dataset('vehicle_attitude_groundtruth').data
    q=continuous_quaternions(np.column_stack([a[k] for k in ('q[1]','q[2]','q[3]','q[0]')]))
    stamps=d['timestamp']*1e-6;mask=(stamps>=30)&(stamps<=30+duration);t=stamps[mask]
    rotation=Rotation.from_quat(np.column_stack([np.interp(t,a['timestamp']*1e-6,q[:,i]) for i in range(4)]))
    positions=np.column_stack([d[k][mask] for k in ('x','y','z')])
    positions+=rotation.apply(np.tile([0.,0.,-.24],(len(t),1)))
    velocity=np.gradient(positions,t,axis=0);acceleration=np.gradient(velocity,t,axis=0)
    speed=np.linalg.norm(velocity,axis=1)
    peak_speed=float(speed[2:-2].max());peak_acc=float(np.linalg.norm(acceleration[2:-2],axis=1).max())
    tilt=float(np.degrees(np.arccos(np.clip(rotation.as_matrix()[:,2,2],-1,1))).max())
    return dict(max_speed_mps=peak_speed,max_acceleration_mps2=peak_acc,max_tilt_deg=tilt,
                body_path_length_m=float(np.linalg.norm(np.diff(positions,axis=0),axis=1).sum()),
                mean_body_speed_mps=float(np.trapz(speed,t)/(t[-1]-t[0])),
                satisfies_direct_dynamics_limits=bool(peak_speed<=12 and peak_acc<=8 and tilt<=42),
                sampling_s=float(np.median(np.diff(t))))


def main():
    p=argparse.ArgumentParser();p.add_argument('runs',type=Path,nargs='+');p.add_argument('--outdir',type=Path,required=True)
    args=p.parse_args();args.outdir.mkdir(parents=True,exist_ok=False)
    output=[];seen=set();deltas=[]
    for path in args.runs:
        result=json.loads((path/'result.json').read_text());key=condition(result)
        if key not in seen:
            for name,summary in [('checked',CHECKED),('Wang 2023 independent',WANG)]:
                matches=[r for r in json.loads(summary.read_text()) if not r.get('infrastructure_failure') and condition(r)==key]
                if len(matches)!=1:raise ValueError(f'Expected one valid attempt for {name} {key}, got {len(matches)}')
                row=matches[0];run=Path(row['run']);old=json.loads((run/'result.json').read_text())
                output.append(dict(method=name,condition=key,run=str(run.resolve()),
                                   historical_pass=bool(row['realtime_success']),
                                   race_time_s=row['flight_time_s'] if row['realtime_success'] else None,
                                   recorded_elapsed_s=row['last_time_s'],
                                   contact=bool(row['collision']),minimum_clearance_m=row['min_clearance_m'],
                                   center_times_s=[c['time'] for c in old['crossings']],
                                   dynamics=body_dynamics(run,row['last_time_s']),
                                   result_sha256=hashlib.sha256((run/'result.json').read_bytes()).hexdigest()))
            seen.add(key)
        verdict=json.loads((path/'RACING_EVALUATION.json').read_text())
        row=dict(method=path.name,condition=key,run=str(path.resolve()),
                 historical_pass=bool(verdict['realtime_success']),race_time_s=verdict['race_time_s'],
                 recorded_elapsed_s=result['last_time_s'],contact=result['collision_detected'],
                 minimum_clearance_m=verdict['min_sampled_body_clearance_m'],
                 center_times_s=[c['time'] for c in result['center_crossings']],
                 dynamics=body_dynamics(path,result['last_time_s']),
                 result_sha256=hashlib.sha256((path/'result.json').read_bytes()).hexdigest())
        output.append(row)
        for baseline in output:
            if baseline['method'] not in ('checked','Wang 2023 independent') or tuple(baseline['condition'])!=key:continue
            if row['race_time_s'] is None or baseline['race_time_s'] is None:continue
            deltas.append(dict(run=row['method'],baseline=baseline['method'],condition=key,
                               lap_delta_s=row['race_time_s']-baseline['race_time_s'],
                               center_event_deltas_s=(np.array(row['center_times_s'])-baseline['center_times_s']).tolist(),
                               common_dynamics_limits_pass=baseline['dynamics']['satisfies_direct_dynamics_limits'] and row['dynamics']['satisfies_direct_dynamics_limits']))
    (args.outdir/'comparison.json').write_text(json.dumps(dict(rows=output,deltas=deltas),indent=2)+'\n')
    lines=['# 旧方法目标与当前直接CTBR：同条件比较','',
           '窗口条件按频率、平移幅度、旋转幅度、相位精确匹配；计时均为七窗返航。',
           '旧方法的历史合格标签不改写。另用相同ULog机体中心偏移和原始时间网格差分复核动力学，',
           '暴露旧方法与当前12 m/s、8 m/s²、42°限制不完全一致，不能称为完全同约束消融。',
           '门事件比较使用中心过平面，不把当前整机退出与旧中心过平面混比。','',
           '| 方法/目录 | 条件 r/t/a/phase(rad) | 历史口径有效圈时 | 净空m | 峰速m/s | 峰加速度m/s² | 倾角° | 满足当前动力学界 |',
           '|---|---|---:|---:|---:|---:|---:|---|']
    for r in output:
        d=r['dynamics'];lap='FAIL' if r['race_time_s'] is None else f"{r['race_time_s']:.3f} s"
        lines.append(f"| {r['method']} | {'/'.join(f'{x:g}' for x in r['condition'])} | {lap} | {r['minimum_clearance_m']:.3f} | {d['max_speed_mps']:.3f} | {d['max_acceleration_mps2']:.3f} | {d['max_tilt_deg']:.3f} | {d['satisfies_direct_dynamics_limits']} |")
    lines+=['','这是少量固定条件开发对照，不估计总体成功概率；负结果不作为有效圈时。',
            'RL复现尚未通过同任务PX4迁移，没有可用安全圈时，不能用其失败夸大当前优势。',
            '旧TOGT 24.660806 s为不同历史设置的名义参考，不是安全PX4圈时。',
            '完整来源、原始结果SHA-256及逐门中心时间差见comparison.json。','']
    (args.outdir/'REPORT.md').write_text('\n'.join(lines))
    print(json.dumps(dict(outdir=str(args.outdir),rows=len(output),deltas=deltas),indent=2))


if __name__=='__main__':main()
