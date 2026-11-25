"""One declared 1 Hz / 2 m PX4 flight with frozen sources and matched audit."""
import argparse
from collections import Counter
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import numpy as np
from convex_timevarying_window.comparisons.comprehensive_wang.run import sources
from convex_timevarying_window.comparisons.comprehensive_wang.audit import audit
from convex_timevarying_window.comparisons.checked_soft_racing.adjudicate import assess
from 复现.polynomial_dynamic_racing_2023.all_versions_current_track.run import infrastructure_arm_failure
from 复现.polynomial_dynamic_racing_2023.px4_original_track_legacy.run import ROOT, save, digest


BASELINE=ROOT/'convex_timevarying_window/event_triggered_mpc/results/f1_a2_v1_20261007/f1_a2_p000_attempt1'


def main():
    p=argparse.ArgumentParser();p.add_argument('--outdir',type=Path,required=True);a=p.parse_args()
    out=a.outdir.resolve();out.mkdir(parents=True,exist_ok=False)
    hashes=sources()
    for f in [*Path(__file__).parent.rglob('*'), *(ROOT/'convex_timevarying_window/event_triggered_mpc').rglob('*')]:
        if f.suffix in ('.py','.md') and 'results' not in f.parts:
            hashes[str(f.relative_to(ROOT))]=digest(f)
    save(out/'protocol.json',dict(case=dict(frequency=1.,amplitude=2.,phase=0.),
         source_sha256=hashes,baseline=str(BASELINE),repeat_count=1,
         protocol=Path(__file__).with_name('README.md').read_text()))
    for rel in hashes:
        target=out/'source_snapshot'/rel;target.parent.mkdir(parents=True,exist_ok=True)
        shutil.copyfile(ROOT/rel,target)
    env=dict(os.environ,OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1',GZ_SHOW_GUI='0')
    attempts=[]
    for attempt in (1,2):
        flight=out/f'f1_a2_p000_attempt{attempt}'
        save(out/'status.json',dict(state='running',attempt=attempt,pid=os.getpid()))
        command=[sys.executable,'-u','-m','convex_timevarying_window.event_free_terminal_mpc.flight',
                 '--method','ours','--frequency','1','--amplitude','2','--phase','0','--outdir',str(flight)]
        with (out/f'attempt{attempt}.log').open('w') as log:
            proc=subprocess.Popen(command,cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            try:code=proc.wait(timeout=300)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid,signal.SIGTERM)
                try:proc.wait(timeout=5)
                except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
                raise RuntimeError('Flight process timeout; artifacts preserved')
        with (out/f'attempt{attempt}.docker.log').open('w') as log:
            subprocess.run(['docker','logs','convex_seven_togt_experiment'],stdout=log,stderr=subprocess.STDOUT,timeout=15)
        raw=json.loads((flight/'result.json').read_text())
        preparation=assess(flight)
        infra=infrastructure_arm_failure(raw) or preparation['setup_failure']
        row=dict(attempt=attempt,returncode=code,directory=str(flight),infrastructure_failure=infra,preparation=preparation)
        if not infra:row.update(audit(flight))
        save(flight/'comparison_audit.json',row);attempts.append(row);save(out/'attempts.json',attempts)
        print('RESULT',json.dumps({k:row.get(k) for k in ('attempt','race_passed','race_time_s','body_exit_count','failure','infrastructure_failure')},ensure_ascii=False),flush=True)
        if not infra:break
    baseline=json.loads((BASELINE/'result.json').read_text())
    old=json.loads((BASELINE/'comparison_audit.json').read_text())
    checks={key:raw[key]==baseline[key] for key in ('case','config','mpc_config','parameters')}
    checks['world_bytes']=digest(flight/'world.sdf')==digest(BASELINE/'world.sdf')
    checks['scene_manifest']=json.loads((flight/'scene.json').read_text())==json.loads((BASELINE/'scene.json').read_text())
    triggers=Counter(e['reason'] for e in raw['planning_events'] if e['action']=='trigger')
    submits=[e for e in raw['planning_events'] if e['action']=='submit']
    known={e['event_id'] for e in raw['planning_events'] if e['action']=='trigger'}
    checks['all_requests_event_backed']=all(e['event_id'] in known for e in submits)
    checks['all_solve_logs_event_backed']=all(s.get('planning_trigger',{}).get('event_id') in known for s in raw['solves'])
    endpoints=[]
    for s in raw['solves']:
        if not s['accepted']:continue
        gates=min(raw['config']['horizon'],7-s['candidate_first_gate']);legs=gates+1
        free=s['candidate_first_gate']+gates<7
        C=np.array(s['coefficients'])[-1];T=s['durations'][-1];x=np.array(s['variables'])
        pvaj=np.array([np.ones(6)@C,np.arange(6)@C/T,
            np.array([0,0,2,6,12,20])@C/T**2,np.array([0,0,0,6,24,60])@C/T**3])
        if free:
            i=4*legs+8*gates;j=legs+6*gates+3*gates
            expected=np.array([10*x[i:i+3],raw['config']['speed']*x[i+3:i+6],
                raw['config']['acceleration']*x[i+6:i+9],raw['config']['jerk']*x[j:j+3]])
            checks[f"free_terminal_coordinates_at_{s['time']}"]=bool(np.max(abs(pvaj-expected))<1e-6)
        endpoints.append(dict(time=s['time'],first_gate=s['candidate_first_gate'],free_terminal=free,
             terminal_pvaj=pvaj.tolist(),terminal_speed_mps=float(np.linalg.norm(pvaj[1]))))
    save(out/'executed_terminal_boundaries.json',endpoints)
    metrics=dict(events=dict(triggers),submitted=len(submits),finished_solves=len(raw['solves']),
         retries=sum(e['attempts']>1 for e in submits),accepted=sum(s['accepted'] for s in raw['solves']),
         rejections=dict(Counter(s['rejection_reason'] for s in raw['solves'] if not s['accepted'])),
         planner_max_ms=max((s['cycle_ms'] for s in raw['solves']),default=0.),
         mpc_max_ms=max((s['cycle_ms'] for s in raw['mpc_cycles']),default=0.))
    verification=dict(source_count=len(hashes),source_changes=[p for p,h in hashes.items() if digest(ROOT/p)!=h],
         snapshot_changes=[p for p,h in hashes.items() if digest(out/'source_snapshot'/p)!=h],matched_checks=checks)
    save(out/'verification.json',verification)
    summary=dict(result=row,metrics=metrics,baseline={k:old.get(k) for k in ('race_passed','race_time_s','min_sampled_sphere_frame_clearance_m')},
         baseline_solves=len(baseline['solves']),verification=verification)
    save(out/'summary.json',summary)
    if verification['source_changes'] or verification['snapshot_changes'] or not all(checks.values()):
        raise RuntimeError('Frozen source, scene or mechanism verification failed')
    save(out/'status.json',dict(state='complete',attempts=len(attempts),race_passed=row.get('race_passed')))
    report=['# 自由局部末态＋事件触发：1Hz / 2m / 初始相位', '',
        '相对事件触发版释放非最终局部末态PVAJ，删除局部参考耗尽后伪造停车；其余目标/时限/MPC不改。',
        '规划/MPC/PX4/世界/场景配置一致性检查通过。采样审计不是连续安全认证。','',
        '| 项目 | 事件固定末态版 | 事件自由末态版 |','|---|---:|---:|',
        f"| 安全完成七窗 | {old.get('race_passed')} | {row.get('race_passed')} |",
        f"| 有效七窗时间/s | {old.get('race_time_s')} | {row.get('race_time_s')} |",
        f"| 最低全任务采样净空/m | {old.get('min_sampled_sphere_frame_clearance_m')} | {row.get('min_sampled_sphere_frame_clearance_m')} |",
        f"| 规划调用数（含返航） | {len(baseline['solves'])} | {len(raw['solves'])} |",'',
        f"事件：{dict(triggers)}；失败重试：{metrics['retries']}；拒绝：{metrics['rejections']}。",
        f"规划/MPC最大耗时：{metrics['planner_max_ms']:.3f}/{metrics['mpc_max_ms']:.3f} ms。",
        f"失败：{row.get('failure')}；实际接触：{row.get('collision_detected')}；整机退出数：{row.get('body_exit_count')}。",'',
        '没有加入中心候选保底，没有修改时刻限制、目标函数或旧参考150ms接续。单次结果不代表稳定成功率。']
    (out/'REPORT.md').write_text('\n'.join(report)+'\n')
    print('COMPLETE',out,flush=True)


if __name__=='__main__':main()
