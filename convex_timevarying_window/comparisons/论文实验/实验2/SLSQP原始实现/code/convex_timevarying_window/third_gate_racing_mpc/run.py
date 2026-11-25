"""Frozen three-condition center-continuation comparison; every flight retained."""
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
    for name in ('third_gate_racing_mpc','third_gate_stop_mpc','third_gate_time_mpc','event_free_terminal_mpc','event_triggered_mpc'):
        for p in (ROOT/'convex_timevarying_window'/name).rglob('*'):
            if 'results' not in p.parts and p.suffix in ('.py','.md'):
                hashes[str(p.relative_to(ROOT))]=digest(p)
    return hashes


def execute(trial,path,log_path,rho):
    module='convex_timevarying_window.third_gate_racing_mpc.flight'
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


BASELINE=ROOT/'convex_timevarying_window/comparisons/comprehensive_wang/results/phase_zero_v1_20261007/ablation_center_f1_a2_p000_attempt1'


def main():
    import numpy as np
    from collections import Counter
    from 复现.polynomial_dynamic_racing_2023.px4_fixed_plane.scenario import Case,build_case
    p=argparse.ArgumentParser();p.add_argument('--outdir',type=Path,required=True);a=p.parse_args()
    out=a.outdir.resolve();out.mkdir(parents=True,exist_ok=False)
    trial=Trial('third_racing','ours',1.,2.,0.,0);hashes=sources()
    save(out/'protocol.json',dict(trial=asdict(trial),baseline=str(BASELINE),source_sha256=hashes,
        repeat_count=1,protocol=Path(__file__).with_name('README.md').read_text()))
    for rel in hashes:
        dst=out/'source_snapshot'/rel;dst.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(ROOT/rel,dst)
    attempts=[]
    for attempt in (1,2):
        assert sources()==hashes,'Source changed during frozen experiment'
        path=out/(trial.name+f'_attempt{attempt}')
        save(out/'status.json',dict(state='running',attempt=attempt))
        print('START',trial.name,attempt,flush=True)
        row=execute(trial,path,out/(path.name+'.log'),100)
        attempts.append(row);save(out/'attempts.json',attempts)
        if not row['infrastructure_failure']:break
    raw=json.loads((path/'result.json').read_text());old=json.loads((BASELINE/'result.json').read_text())
    old_audit=json.loads((BASELINE/'comparison_audit.json').read_text())
    checks={k:raw[k]==old[k] for k in ('case','mpc_config','parameters')}
    checks['world_bytes']=digest(path/'world.sdf')==digest(BASELINE/'world.sdf')
    track,_=build_case(Case(**raw['case']));terminals=[]
    for s in raw['solves']:
        if not s['accepted']:continue
        gates=len(s['durations'])//2-1;first=s['candidate_first_gate'];end=s['made_at']+sum(s['durations'])
        C=np.asarray(s['coefficients'])[-1];T=s['durations'][-1]
        pva=np.array([C.sum(axis=0),np.arange(6)@C/T,np.array([0,0,2,6,12,20])@C/T**2])
        target=track.windows[first+gates].state_at(end)[0] if first+gates<7 else track.goal
        err=float(np.max(abs(pva-np.array([target,np.zeros(3),np.zeros(3)]))))
        checks['terminal_'+str(s['time'])]=err<1e-7
        checks['converged_'+str(s['time'])]=s['optimizer_stats']['converged'] and s['cycle_ms']<100
        checks['dimension_'+str(s['time'])]=len(s['variables'])==4*(gates+1)+8*gates
        terminals.append(dict(time=s['time'],first_gate=first,target_gate=first+gates if first+gates<7 else None,
            end=end,terminal_pva=pva.tolist(),error=err))
    save(out/'terminal_audit.json',terminals)
    solves=raw['solves'];row.update(accepted=sum(s['accepted'] for s in solves),planning_calls=len(solves),
        converged=sum(s.get('optimizer_stats',{}).get('converged',False) for s in solves),
        planner_max_ms=max((s['cycle_ms'] for s in solves),default=0.),
        mpc_max_ms=max((s['cycle_ms'] for s in raw['mpc_cycles']),default=0.),
        deadline_misses=sum(s['cycle_ms']>=100 for s in solves),
        rejections=dict(Counter(s['rejection_reason'] for s in solves if not s['accepted'])))
    verification=dict(source_count=len(hashes),source_changes=[k for k,v in hashes.items() if digest(ROOT/k)!=v],
        snapshot_changes=[k for k,v in hashes.items() if digest(out/'source_snapshot'/k)!=v],checks=checks)
    save(out/'verification.json',verification)
    save(out/'summary.json',dict(new=row,historical_center=old_audit))
    assert not verification['source_changes'] and not verification['snapshot_changes'] and all(checks.values())
    report=['# 第三窗零VA＋竞速初始化：与历史中心对比','',
        '原七窗1Hz/2m/初相、真实PX4/Gazebo/x500，同世界/MPC/PX4；新算法一次，不重跑历史中心。',
        '实际W7整机退出时刻与严格在线有效成绩分开；采样安全不等于连续认证。','',
        '|项目|历史中心|第三窗中心零VA|','|---|---:|---:|',
        f"|实际W7整机退出/s|{old_audit.get('raw_seventh_exit_s',old_audit.get('race_time_s'))}|{row.get('raw_seventh_exit_s')}|",
        f"|严格竞速验收|{old_audit.get('race_passed')}|{row.get('race_passed')}|",
        f"|有效竞速成绩/s|{old_audit.get('race_time_s')}|{row.get('race_time_s')}|",
        f"|完整退出窗数|{old_audit.get('body_exit_count')}|{row.get('body_exit_count')}|",
        f"|最小采样净空/m|{old_audit.get('min_sampled_sphere_frame_clearance_m')}|{row.get('min_sampled_sphere_frame_clearance_m')}|",
        f"|Gazebo碰撞|{old_audit.get('collision_detected')}|{row.get('collision_detected')}|",'',
        f"规划正常停止/总数：{row['converged']}/{row['planning_calls']}；接纳{row['accepted']}；超100ms {row['deadline_misses']}次。",
        f"规划/MPC最大：{row['planner_max_ms']:.3f}/{row['mpc_max_ms']:.3f}ms。拒绝：{row['rejections']}。",
        f"程序失败：{row.get('failure')}；真值返航：{row.get('truth_returned')}；真值终速：{row.get('truth_goal_speed_mps')}。",'',
        f"{len(terminals)}条接纳曲线终点/零VA/正常停止/变量维度重建通过；{len(hashes)}源与快照未变。",
        '历史中心的周期重规划、限时最好迭代点和固定局部尾端机制不同，本轮不是单因素消融。',
        '新方法不使用第三窗时间估计，不生成第四窗之后轨迹；正常出第一窗即滚动，零速尾端不等于实际必停。']
    (out/'REPORT.md').write_text('\n'.join(report)+'\n')
    save(out/'status.json',dict(state='complete',race_passed=row.get('race_passed'),attempts=len(attempts)))
    print('COMPLETE',json.dumps({k:row.get(k) for k in ['raw_seventh_exit_s','race_passed','body_exit_count','failure','planner_max_ms','deadline_misses']},ensure_ascii=False),flush=True)


if __name__=='__main__':main()
