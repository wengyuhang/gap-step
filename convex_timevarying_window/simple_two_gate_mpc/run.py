"""Frozen matched pair; complete closed-loop timing."""
import argparse
from dataclasses import asdict
import json,os,shutil,signal,subprocess,sys,time
from pathlib import Path
from convex_timevarying_window.comparisons.comprehensive_wang.run import sources as common_sources
from convex_timevarying_window.comparisons.comprehensive_wang.registry import Trial
from convex_timevarying_window.comparisons.comprehensive_wang.audit import audit
from convex_timevarying_window.comparisons.checked_soft_racing.adjudicate import assess
from 复现.polynomial_dynamic_racing_2023.all_versions_current_track.run import infrastructure_arm_failure
from 复现.polynomial_dynamic_racing_2023.px4_original_track_legacy.run import save,digest,ROOT


def sources():
    hashes=common_sources()
    for name in ('simple_two_gate_mpc','third_gate_racing_mpc','third_gate_stop_mpc','third_gate_time_mpc','event_free_terminal_mpc','event_triggered_mpc'):
        for p in (ROOT/'convex_timevarying_window'/name).rglob('*'):
            if 'results' not in p.parts and p.suffix in ('.py','.md'):
                hashes[str(p.relative_to(ROOT))]=digest(p)
    return hashes


def execute(trial,path,log_path,rho):
    module='convex_timevarying_window.simple_two_gate_mpc.flight'
    command=[sys.executable,'-u','-m',module]
    if trial.method!='wang':command+=['--method',trial.method]
    command+=['--frequency',str(trial.frequency),'--amplitude',str(trial.amplitude),'--phase',str(trial.phase),'--outdir',str(path)]
    env=dict(os.environ,OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1',GZ_SHOW_GUI='0',WANG_DEVELOPMENT_RHO=str(rho))
    start=time.time();error=None
    with log_path.open('w') as log:
        with subprocess.Popen(command,cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True) as proc:
            try:code=proc.wait(timeout=900)
            except subprocess.TimeoutExpired:
                error='process_timeout';os.killpg(proc.pid,signal.SIGTERM)
                try:proc.wait(timeout=5)
                except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
                code=proc.returncode
    row=dict(trial=asdict(trial),name=trial.name,pair=trial.pair,directory=str(path),wall_s=time.time()-start,
        returncode=code,process_error=error,race_passed=False,main_passed=False,infrastructure_failure=False)
    with log_path.with_suffix('.docker.log').open('w') as log:
        subprocess.run(['docker','logs','convex_seven_togt_experiment'],stdout=log,stderr=subprocess.STDOUT,timeout=15)
    if not (path/'result.json').exists():return dict(row,failure='missing_result',audit_error='missing_result')
    raw=json.loads((path/'result.json').read_text());row['failure']=raw.get('failure')
    row['infrastructure_failure']=infrastructure_arm_failure(raw)
    try:
        row['preparation_assessment']=assess(path)
        row['infrastructure_failure'] |= row['preparation_assessment']['setup_failure']
    except Exception as exc:row['preparation_assessment_error']=repr(exc)
    row['world_sha256']=digest(path/'world.sdf');row['mpc_config']=raw['mpc_config'];row['px4_parameters']=raw['parameters']
    row['scene_manifest']=json.loads((path/'scene.json').read_text())
    if not row['infrastructure_failure']:
        try:row.update(audit(path))
        except Exception as exc:row.update(audit_error=repr(exc))
    save(path/'comparison_audit.json',row)
    return row


def main():
    import numpy as np
    from collections import Counter
    from 复现.polynomial_dynamic_racing_2023.px4_fixed_plane.scenario import Case,build_case
    p=argparse.ArgumentParser();p.add_argument('--outdir',type=Path,required=True);a=p.parse_args()
    out=a.outdir.resolve();out.mkdir(parents=True,exist_ok=False)
    hashes=sources()
    save(out/'protocol.json',dict(source_sha256=hashes,methods=['ours','center'],case=dict(frequency=1.,amplitude=2.,phase=0.),
        primary_metric='full_race_time_s',repeat_count=1,protocol=Path(__file__).with_name('README.md').read_text()))
    for rel in hashes:
        dst=out/'source_snapshot'/rel;dst.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(ROOT/rel,dst)
    rows=[];attempts=[];raws=[];checks={}
    for method in ('ours','center'):
        trial=Trial('simple',method,1.,2.,0.,0)
        for attempt in (1,2):
            assert sources()==hashes,'Source changed during frozen experiment'
            path=out/(trial.name+f'_attempt{attempt}')
            save(out/'status.json',dict(state='running',method=method,attempt=attempt))
            print('START',method,attempt,flush=True)
            row=execute(trial,path,out/(path.name+'.log'),100)
            attempts.append(row);save(out/'attempts.json',attempts)
            if not row['infrastructure_failure']:break
        row['full_race_time_s']=row.get('flight_time_s') if row.get('main_passed') else None
        raw=json.loads((path/'result.json').read_text()) if (path/'result.json').exists() else None
        raws.append(raw)
        if raw is not None:
            solves=raw['solves']
            row.update(planning_calls=len(solves),accepted=sum(s['accepted'] for s in solves),
                planner_max_ms=max((s['cycle_ms'] for s in solves),default=0.),
                mpc_max_ms=max((s['cycle_ms'] for s in raw['mpc_cycles']),default=0.),
                deadline_misses=sum(s['cycle_ms']>=100 for s in solves),
                rejections=dict(Counter(s['rejection_reason'] for s in solves if not s['accepted'])))
            checks[method+'_time_objective']=raw['config']['jerk_weight']==0 and not raw['config']['exclude_postrace_time']
            checks[method+'_event_only']=all(e['reason'] in ('initial','body_exit') for e in raw['planning_events'] if e['action']=='trigger')
            track,_=build_case(Case(**raw['case']));terminals=[]
            for index,sol in enumerate(solves):
                if not sol['accepted']:continue
                gates=len(sol['durations'])//2-1;first=sol['candidate_first_gate'];end=sol['made_at']+sum(sol['durations'])
                C=np.asarray(sol['coefficients'])[-1];T=sol['durations'][-1]
                pva=np.array([C.sum(axis=0),np.arange(6)@C/T,np.array([0,0,2,6,12,20])@C/T**2])
                target=track.windows[first+gates].state_at(end)[0] if first+gates<7 else track.goal
                err=float(np.max(abs(pva-np.array([target,np.zeros(3),np.zeros(3)]))))
                checks[method+'_terminal_'+str(index)]=err<1e-7
                checks[method+'_converged_'+str(index)]=sol['optimizer_stats']['converged']
                if method=='center':
                    qi=4*(gates+1)+6*gates
                    checks['center_q_'+str(index)]=bool(np.max(abs(np.array(sol['variables'])[qi:qi+2*gates]))<1e-10)
                terminals.append(dict(index=index,first_gate=first,error=err,terminal_time=end,pva=pva.tolist()))
            save(path/'terminal_audit.json',terminals)
        rows.append(row);save(out/'summary.json',rows)
        print('COMPLETE',json.dumps({k:row.get(k) for k in ['full_race_time_s','flight_time_s','main_passed','failure','planner_max_ms','deadline_misses']},ensure_ascii=False),flush=True)
    if all(r is not None for r in raws):
        for k in ('case','mpc_config','parameters'):checks['matched_'+k]=raws[0][k]==raws[1][k]
        left,right=[dict(r['config']) for r in raws]
        checks['free_points_only']=left.pop('free_points') is True and right.pop('free_points') is False and left==right
        checks['same_world']=rows[0].get('world_sha256')==rows[1].get('world_sha256')
    verification=dict(source_count=len(hashes),source_changes=[k for k,v in hashes.items() if digest(ROOT/k)!=v],
        snapshot_changes=[k for k,v in hashes.items() if digest(out/'source_snapshot'/k)!=v],checks=checks)
    save(out/'verification.json',verification)
    assert not verification['source_changes'] and not verification['snapshot_changes'] and all(checks.values())
    report=['# 简化两窗：完整闭环竞速配对','',
        '原七窗1Hz/2m/初相，PX4/Gazebo/x500，真实飞控闭环；各方法一次，不做随机成功率推断。',
        '主成绩从起赛至七窗完成后返回终点并满足既有终态，包含返航；第七窗退出仅诊断。','',
        '| 方法 | 程序记录完整时间/s | 有效完整竞速/s | 全任务通过 | 出窗数 | 最小采样净空/m | 规划最大/ms | MPC最大/ms |',
        '|---|---:|---:|---|---:|---:|---:|---:|']
    for row in rows:
        report.append('| '+' | '.join(str(v) for v in [row['trial']['method'],row.get('flight_time_s'),row.get('full_race_time_s'),
            row.get('main_passed'),row.get('body_exit_count'),row.get('min_sampled_sphere_frame_clearance_m'),row.get('planner_max_ms'),row.get('mpc_max_ms')])+' |')
    report+=['','有效成绩为空表示失败或尚未通过严格验收，不能以较短终止时刻宣称更快。']
    if all(row['full_race_time_s'] is not None for row in rows):
        delta=rows[1]['full_race_time_s']-rows[0]['full_race_time_s']
        report.append(f'自由点相对同结构中心的时间差（正为自由点更快）：{delta:.3f}s；{100*delta/rows[1]["full_race_time_s"]:.2f}%。')
    for row in rows:report.append(f'{row["trial"]["method"]}: failure={row.get("failure")}; 拒绝={row.get("rejections")}; 超100ms={row.get("deadline_misses")}。')
    report+=['',f'{len(hashes)}源文件冻结核验；同场景/MPC/PX4，规划配置仅free_points不同。',
        '实际整机退出调度、接纳终点PVA、中心版q=0已重建检查。安全证据为4ms真值采样与物理接触，不是连续证明。']
    (out/'REPORT.md').write_text('\n'.join(report)+'\n')
    save(out/'status.json',dict(state='complete',methods=2,attempts=len(attempts)))


if __name__=='__main__':main()
