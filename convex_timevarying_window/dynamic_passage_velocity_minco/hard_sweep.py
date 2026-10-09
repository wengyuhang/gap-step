"""Frozen hard_v2 single-window escalation; each condition once, no retuning."""
import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import numpy as np
from .sweep import save,sources
from .trajectory import Plan
from .hard_dynamics import flat_actuators,KF
from convex_timevarying_window.simple_two_gate_soft_only_mpc import run as shared
from convex_timevarying_window.comparisons.single_gate_translation.audit import audit


ROUTES={
    'frequency':[(4.+.5*k,4.) for k in range(1,5)],
    'amplitude':[(3.5,4.5+.5*k) for k in range(1,5)],
    'joint':[(3.5+.5*k,4.5+.5*k) for k in range(1,5)],
}
MODULE='convex_timevarying_window.dynamic_passage_velocity_minco.hard_flight'


def nominal_diagnostic(raw):
    records=[]
    for s in raw['solves']:
        if not s['accepted']:continue
        p=Plan(s['made_at'],s['gate'],np.array(s['durations']),np.array(s['coefficients']),
               np.array(s['variables']),0.,0.,s['status'],int(s['gate']==0))
        t=np.linspace(p.made_at,p.end_time,int(np.ceil((p.end_time-p.made_at)/.001))+1)
        F,w,rotors=flat_actuators.map(len(t))(*[p.sample_many(t,d).T for d in (1,2,3,4)])
        F,w,rotors=map(np.array,(F,w,rotors))
        records.append(dict(accepted_at_s=s['time'],gate=s['gate'],
            max_step_s=float(np.max(np.diff(t))),peak_collective_n=float(F.max()),
            peak_rate_abs_rps=np.max(abs(w),axis=1).tolist(),
            min_rotor_n=float(rotors.min()),max_rotor_n=float(rotors.max()),
            rotor_relative_overshoot=max(0.,float(rotors.max()/(KF*1000**2)-1.))))
    return dict(offline_only=True,used_for_admission=False,formal_certificate=False,plans=records)


def report(out,rows):
    lines=['# hard_v2 单窗继续升档','',
        '冻结算法/小窗/运动/相位0/x500/PX4/跟踪器/碰撞/终点；每档一次，无重试、无在线调参。',
        '固定4m升频从4Hz继续；固定3.5Hz升幅从4.5m继续；从3.5Hz/4.5m同步提升。',
        '每步0.5、每路线最多4档，首次碰撞或未完成停止该路线；设施/证据故障整批停止。',
        '时间包含起点等待和窗后到终点，不是仅移动时间；本单窗不返航，不是七窗成绩。',
        '硬界仅优化节点强制；另存1ms离线名义诊断，不用它改变已飞条件/成绩。','',
        '|幅度 m|频率 Hz|结果|完整时间 s|最大误差 m|规划最大 ms|MPC最大 ms|',
        '|---:|---:|---|---:|---:|---:|---:|']
    for r in sorted(rows,key=lambda x:(x['trial']['amplitude'],x['trial']['frequency'])):
        verdict=('碰撞' if r.get('collision_detected') else '证据不足' if r.get('audit_error') or r.get('infrastructure_failure')
                 else '无窗框接触完赛' if r.get('passed') else '未完成')
        duration=f"{r['flight_time_s']:.3f}" if r.get('passed') else '—'
        vals=[f"{r['trial']['amplitude']:g}",f"{r['trial']['frequency']:g}",verdict,duration]
        for k in ('tracking_error_max_m','planner_max_ms','mpc_max_ms'):
            vals.append('—' if r.get(k) is None else f'{r[k]:.3f}')
        lines.append('| '+' | '.join(vals)+' |')
    lines+=['','未完成/碰撞不赋完成时间；首个失败档位不是物理可行性阈值，难度不保证单调。',
            '原始 result/接触/ULog、comparison_audit、nominal_diagnostic 和源码快照均保留。']
    (out/'REPORT.md').write_text('\n'.join(lines)+'\n')


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--outdir',type=Path,required=True)
    args=parser.parse_args();root=Path.cwd();shared.ROOT=root
    out=args.outdir.resolve();out.mkdir(parents=True,exist_ok=False)
    hashes=sources(root)
    save(out/'protocol.json',dict(routes=ROUTES,module=MODULE,source_sha256=hashes,
        phase=0.,repetitions=1,retries=0,max_flight_s=40.,algorithm_changes=False,
        primary_metrics=['complete_start_to_goal_time','physical_frame_contacts'],
        previous_baselines=['hard_v2_f4_a4_20261009_01','hard_v2_f3.5_a4.5_20261009_01'],
        stop_route='first collision or noncompletion',stop_batch='infrastructure or evidence failure',
        nominal_dense_audit='offline only; not a safety certificate or admission filter'))
    for rel in hashes:
        target=out/'source_snapshot'/rel;target.parent.mkdir(parents=True,exist_ok=True)
        shutil.copyfile(root/rel,target)
    rows=[];stopped={};save(out/'summary.json',rows);report(out,rows)
    try:
        for step in range(4):
            for route,cases in ROUTES.items():
                if route in stopped:continue
                if sources(root)!=hashes:
                    save(out/'status.json',dict(state='source_changed',completed=len(rows),pid=os.getpid()))
                    return
                f,a=cases[step];trial=shared.Trial(route,'ours',f,a,0.,0);path=out/trial.name
                save(out/'status.json',dict(state='running',pid=os.getpid(),completed=len(rows),
                     route=route,frequency=f,amplitude=a,directory=str(path)))
                print('START',route,f,a,flush=True)
                row=shared.execute(trial,path,out/(trial.name+'.log'),100,module=MODULE,audit_fn=audit)
                row['route']=route
                row['passed']=bool(row.get('passed') and not row.get('audit_error') and not row.get('infrastructure_failure'))
                rawpath=path/'result.json'
                if rawpath.exists():
                    raw=json.loads(rawpath.read_text())
                    row['planner_max_ms']=max((s['cycle_ms'] for s in raw['solves']),default=None)
                    row['mpc_max_ms']=max((s['cycle_ms'] for s in raw['mpc_cycles']),default=None)
                    try:save(path/'nominal_diagnostic.json',nominal_diagnostic(raw))
                    except Exception as exc:row['nominal_diagnostic_error']=repr(exc)
                    row['sources_unchanged_after_flight']=sources(root)==hashes
                rows.append(row);save(out/'summary.json',rows);report(out,rows)
                print('RESULT',route,f,a,row['passed'],row.get('collision_detected'),
                      row.get('flight_time_s'),row.get('failure'),flush=True)
                if row.get('infrastructure_failure') or row.get('audit_error') or row.get('nominal_diagnostic_error'):
                    save(out/'status.json',dict(state='setup_or_evidence_failure',completed=len(rows),pid=os.getpid()))
                    return
                if sources(root)!=hashes:
                    save(out/'status.json',dict(state='source_changed',completed=len(rows),pid=os.getpid()))
                    return
                if not row['passed']:
                    stopped[route]=dict(frequency=f,amplitude=a,collision=row.get('collision_detected'),failure=row.get('failure'))
                save(out/'boundaries.json',stopped)
        save(out/'verification.json',dict(source_count=len(hashes),sources_unchanged=sources(root)==hashes,
            snapshot_mismatches=[rel for rel,h in hashes.items()
                if __import__('hashlib').sha256((out/'source_snapshot'/rel).read_bytes()).hexdigest()!=h],
            algorithm_changes=[],retries=0))
        save(out/'status.json',dict(state='complete',completed=len(rows),pid=os.getpid(),stopped=stopped))
    except Exception as exc:
        save(out/'status.json',dict(state='coordinator_error',completed=len(rows),pid=os.getpid(),error=repr(exc)))
        raise
    finally:
        subprocess.run(['docker','exec','-e','GZ_PARTITION=convex_seven_togt_experiment_partition',
            'convex_seven_togt_experiment','gz','service','-s','/world/convex_seven_dynamic_px4_togt/control',
            '--reqtype','gz.msgs.WorldControl','--reptype','gz.msgs.Boolean','--timeout','2000','--req','pause: true'],
            capture_output=True,timeout=5)


if __name__=='__main__':main()
