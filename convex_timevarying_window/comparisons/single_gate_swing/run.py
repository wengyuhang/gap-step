"""Serial frozen 17-condition paired experiment with preserved raw failures."""
import argparse
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
from convex_timevarying_window.comparisons.single_gate_constant_velocity.run import sources, save, digest
from .planner import Config

MODULE = 'convex_timevarying_window.comparisons.single_gate_swing.flight'
CASES = [(1.,0.)]+[(f,a) for a in (2.,4.) for f in (.5,1.,1.5,2.,2.5,3.,3.5,4.)]


def report(out, rows, cases=None):
    labels = dict(completed_no_contact='无碰撞完赛',collision='碰撞',missed_gate='漏窗',
                  not_completed='未完成',online_no_accepted_plan='任务截止前未接纳初始解',evidence_failure='证据不足')
    lines = ['# 单窗摆动配对实验','',
        '原 W1 2.90×2.30 m；固定平面、水平正弦往复、初相零。双方只差窗口内二维选点是否固定为中心。',
        '两方优化时间及穿越速度，同一硬约束、同一 execution-aligned MPC/PX4/x500。',
        '时间为任务开始到窗后固定终点，含等待及刹车。每条件各一次，不估计碰撞概率。','',
        '|振幅 m|频率 Hz|自由结果|自由完整 s|中心结果|中心完整 s|自由节省|',
        '|---:|---:|---|---:|---|---:|---:|']
    for frequency,amplitude in (CASES if cases is None else sorted(cases,key=lambda c:(c[1],c[0]))):
        pair = {r['variant']:r for r in rows if r['frequency_hz']==frequency and r['amplitude_m']==amplitude}
        values = [f'{amplitude:g}','静止' if amplitude==0 else f'{frequency:g}']
        for variant in ('ours','center'):
            row = pair.get(variant,{})
            values += [labels.get(row.get('verdict'),'待完成'),f"{row['flight_time_s']:.3f}" if row.get('passed') else '—']
        a,b = pair.get('ours',{}),pair.get('center',{})
        values += [f"{100*(b['flight_time_s']-a['flight_time_s'])/b['flight_time_s']:.2f}%"
                   if a.get('passed') and b.get('passed') else '—']
        lines.append('| '+' | '.join(values)+' |')
    lines += ['','## 原始记录与计算时间','',
        '|振幅|频率|方法|碰框/碰地|接受/请求|规划最大 ms|MPC最大 ms|原始结果|',
        '|---:|---:|---|---|---:|---:|---:|---|']
    for row in sorted(rows,key=lambda r:(r['amplitude_m'],r['frequency_hz'],r['variant'])):
        values = [row['amplitude_m'],row['frequency_hz'],row['variant'],
            f"{row.get('frame_collision','?')}/{row.get('ground_collision','?')}",
            f"{row.get('accepted_plan_count',0)}/{row.get('planning_requests',0)}",
            round(row.get('planner_max_ms') or 0,2),round(row.get('mpc_max_ms') or 0,2),
            f"[{Path(row['directory']).name}]({Path(row['directory']).name}/result.json)"]
        lines.append('| '+' | '.join(map(str,values))+' |')
    lines += ['','物理步长统一1 ms；日志按原生频率独立核对。零接触是本次仿真观察，不是闭环连续安全证明。',
        '未完成无有效竞速时间；仅双方无碰撞完成时比较。求解未找到不代表物理无解。',
        '若设施启动失败且未执行任务，最多额外一次启动，保留原记录；已执行条件不重跑、不择优。']
    (out/'REPORT.md').write_text('\n'.join(lines)+'\n')


def pair_check(out, rows, frequency, amplitude):
    pair = [r for r in rows if r['frequency_hz']==frequency and r['amplitude_m']==amplitude]
    if len(pair)!=2 or any(r['verdict']=='evidence_failure' for r in pair):
        return
    raw = [json.loads((Path(r['directory'])/'result.json').read_text()) for r in pair]
    configs = [dict(r['config']) for r in raw]
    for cfg in configs: cfg.pop('free_points')
    matching = (configs[0]==configs[1] and raw[0]['mpc_config']==raw[1]['mpc_config']
        and raw[0]['parameters']==raw[1]['parameters']
        and digest(Path(pair[0]['directory'])/'world.sdf')==digest(Path(pair[1]['directory'])/'world.sdf'))
    save(out/f'pair_a{amplitude:g}_f{frequency:g}.json',dict(equal_except_crossing_point=matching,
        source_directories=[r['directory'] for r in pair]))
    if not matching: raise RuntimeError('pair_configuration_mismatch')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--outdir',type=Path,required=True)
    parser.add_argument('--cases',nargs='+',help='Development only: frequency:amplitude')
    args = parser.parse_args()
    cases = [tuple(map(float,s.split(':'))) for s in args.cases] if args.cases else CASES
    root = Path(__file__).resolve().parents[3]
    out = args.outdir.resolve()
    out.mkdir(parents=True,exist_ok=False)
    hashes = sources(root)
    rows = []
    save(out/'protocol.json',dict(cases=[dict(frequency=f,amplitude=a,phase=0.) for f,a in cases],
        repetitions=1,unflown_facility_retries=1,source_sha256=hashes,module=MODULE,
        protocol=Path(__file__).with_name('README.md').read_text(),online_budget_ms=Config().max_solve_ms,
        max_task_s=40,physics_step_s=.001,offline_diagnostic_budget_ms=None if Config().max_solve_ms is None else 5000))
    for rel in hashes:
        target = out/'source_snapshot'/rel
        target.parent.mkdir(parents=True,exist_ok=True)
        shutil.copyfile(root/rel,target)
    save(out/'summary.json',rows)
    report(out,rows)
    try:
        for i,(frequency,amplitude) in enumerate(cases):
            for variant in (('ours','center') if i%2==0 else ('center','ours')):
                if sources(root)!=hashes: raise RuntimeError('frozen_source_changed')
                name = f'a{amplitude:g}_f{frequency:g}_{variant}'
                row = None
                for attempt in range(2):
                    path = out/(name if attempt==0 else name+'_facility_retry')
                    save(out/'status.json',dict(state='running',pid=os.getpid(),completed=len(rows),
                        total=2*len(cases),frequency_hz=frequency,amplitude_m=amplitude,variant=variant,
                        attempt=attempt,directory=str(path)))
                    print('START',path.name,flush=True)
                    command = [sys.executable,'-u','-m',MODULE,'--frequency',str(frequency),
                        '--amplitude',str(amplitude),'--method',variant,'--outdir',str(path)]
                    env = dict(os.environ,OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1',GZ_SHOW_GUI='0')
                    with (out/(path.name+'.log')).open('w') as log:
                        with subprocess.Popen(command,cwd=root,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True) as proc:
                            try: code = proc.wait(timeout=600)
                            except subprocess.TimeoutExpired:
                                os.killpg(proc.pid,signal.SIGTERM)
                                try: proc.wait(timeout=5)
                                except subprocess.TimeoutExpired: os.killpg(proc.pid,signal.SIGKILL);proc.wait()
                                raise RuntimeError('flight_process_timeout:'+path.name)
                    raw = json.loads((path/'result.json').read_text()) if (path/'result.json').exists() else None
                    unflown = (raw is not None and not raw['solves'] and raw['last_time_s']==0
                               and any(s in str(raw['failure']) for s in ('arm failed','no heartbeat','initial telemetry')))
                    if unflown and attempt==0:
                        save(path/'facility_retry_reason.json',dict(unflown=True,failure=raw['failure'],
                            original_retained=True,next_attempt=name+'_facility_retry'))
                        print('UNFLOWN_FACILITY_RETRY',name,raw['failure'],flush=True)
                        continue
                    if (path/'comparison_audit.json').exists():
                        row = json.loads((path/'comparison_audit.json').read_text())
                        row.update(directory=str(path),process_returncode=code,planning_requests=len(raw['solves']),
                            offline_found=raw['planner_process']['offline_diagnostic']['found'])
                        if code: row.update(verdict='evidence_failure',passed=False,flight_time_s=None)
                    else:
                        row = dict(frequency_hz=frequency,amplitude_m=amplitude,variant=variant,
                            directory=str(path),verdict='evidence_failure',passed=False,flight_time_s=None,
                            failure=raw['failure'] if raw else 'missing_result_or_audit',process_returncode=code)
                    break
                rows.append(row)
                save(out/'summary.json',rows)
                report(out,rows)
                print('RESULT',name,row['verdict'],row.get('flight_time_s'),flush=True)
                if row['verdict']=='evidence_failure': raise RuntimeError('infrastructure_or_audit_failure:'+name)
            pair_check(out,rows,frequency,amplitude)
        unchanged = sources(root)==hashes
        if not unchanged: raise RuntimeError('frozen_source_changed_at_end')
        save(out/'status.json',dict(state='complete',pid=os.getpid(),completed=len(rows),total=2*len(cases),sources_unchanged=unchanged))
    except Exception as exc:
        save(out/'status.json',dict(state='stopped',pid=os.getpid(),completed=len(rows),total=2*len(cases),error=repr(exc)))
        raise


if __name__=='__main__':
    main()
