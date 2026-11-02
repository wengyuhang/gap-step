"""Frozen paired Gazebo runs. Never silently retry, retune, or erase failures."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time

MODULE='convex_timevarying_window.comparisons.single_gate_constant_velocity.flight'
SPEEDS=(0.,5.,-5.,10.,-10.,15.,-15.,20.,-20.)


def save(path,value):path.write_text(json.dumps(value,indent=2,ensure_ascii=False)+'\n')
def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def sources(root):
    dirs=('convex_timevarying_window','convex_dynamic_seven_window_gazebo',
          'nonconvex_timevarying_window/sc_dynatogt','复现')
    files=[p for d in dirs for p in (root/d).rglob('*.py')
           if not {'results','__pycache__','build'}.intersection(p.parts) and not p.is_symlink()]
    files += [root/'convex_dynamic_seven_window_gazebo'/p for p in
              ('src/PeriodicGateMotion.cc','build/libPeriodicGateMotion.so','convex_seven_dynamic_px4_togt.sdf')]
    return {str(p.relative_to(root)):digest(p) for p in files}


def report(out,rows):
    labels={'completed_no_frame_contact':'无窗框接触完成','online_no_accepted_plan':'预算内未找到初始解',
            'collision':'碰撞','missed_gate':'漏窗','not_completed':'未完成','evidence_failure':'证据不足'}
    lines=['# 单窗匀速横移配对实验','',
        '原小窗/x500/PX4，双方同硬约束及 execution-aligned MPC。时间为完整起点→终点，含等待。',
        '每条件各一次；求解失败不是碰撞或无解证明，只有双方完成才比较时间。','',
        '|窗速 m/s（符号为方向）|自由结果|自由 s|中心结果|中心 s|自由节省|',
        '|---:|---|---:|---|---:|---:|']
    for speed in dict.fromkeys(r['speed_mps'] for r in rows):
        pair={r['variant']:r for r in rows if r['speed_mps']==speed};values=[f'{speed:g}']
        for variant in ('ours','center'):
            r=pair.get(variant,{})
            values += [labels.get(r.get('verdict'),'待完成'),f"{r['flight_time_s']:.3f}" if r.get('passed') else '—']
        a,b=pair.get('ours',{}),pair.get('center',{})
        values += [f"{100*(b['flight_time_s']-a['flight_time_s'])/b['flight_time_s']:.1f}%"
                   if a.get('passed') and b.get('passed') else '—']
        lines.append('| '+' | '.join(values)+' |')
    lines+=['','## 计算与失败诊断','','|窗速|方法|在线接受/请求|规划最大 ms|MPC最大 ms|5 s 名义离线诊断|',
        '|---:|---|---:|---:|---:|---|']
    for r in rows:
        lines.append('| '+' | '.join(map(str,[r['speed_mps'],r['variant'],
            f"{r.get('accepted_plan_count',0)}/{r.get('planning_requests',0)}",
            round(r.get('planner_max_ms') or 0,2),round(r.get('mpc_max_ms') or 0,2),
            '找到解' if r.get('offline_found') else '未找到']))+' |')
    lines+=['','5 s 为预算上限，不代表耗时恰好5 s；名义初态一致，实际在线初态微差保留在原始请求。',
        '4 ms 物理步长、约20 ms真值日志；高速接触离散误差未消除，不能宣称连续闭环安全保证。',
        '单次结果不能估计成功概率。拒飞不算碰撞，也不赋完成时间。']
    (out/'REPORT.md').write_text('\n'.join(lines)+'\n')


def main():
    p=argparse.ArgumentParser();p.add_argument('--outdir',type=Path,required=True)
    p.add_argument('--speeds',type=float,nargs='+',default=SPEEDS)
    a=p.parse_args();root=Path(__file__).resolve().parents[3];out=a.outdir.resolve()
    out.mkdir(parents=True,exist_ok=False);hashes=sources(root);rows=[]
    save(out/'protocol.json',dict(speeds_mps=a.speeds,repetitions=1,retries=0,source_sha256=hashes,
        protocol=Path(__file__).with_name('README.md').read_text(),module=MODULE,
        tracker_signature='e5fd428db5f6b1d6',offline_diagnostic_budget_ms=5000,
        online_budget_ms=90,max_flight_s=40,real_physics_step_s=.004))
    for rel in hashes:
        target=out/'source_snapshot'/rel;target.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(root/rel,target)
    save(out/'summary.json',rows);report(out,rows)
    try:
        for i,speed in enumerate(a.speeds):
            for variant in (('ours','center') if i%2==0 else ('center','ours')):
                if sources(root)!=hashes:raise RuntimeError('frozen_source_changed')
                name=f'v{speed:+g}_{variant}';path=out/name
                save(out/'status.json',dict(state='running',pid=os.getpid(),completed=len(rows),
                    total=2*len(a.speeds),speed_mps=speed,variant=variant,directory=str(path)))
                print('START',name,flush=True)
                command=[sys.executable,'-u','-m',MODULE,'--speed',str(speed),'--method',variant,'--outdir',str(path)]
                env=dict(os.environ,OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1',GZ_SHOW_GUI='0')
                with (out/(name+'.log')).open('w') as log:
                    with subprocess.Popen(command,cwd=root,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True) as proc:
                        try:code=proc.wait(timeout=600)
                        except subprocess.TimeoutExpired:
                            os.killpg(proc.pid,signal.SIGTERM)
                            try:proc.wait(timeout=5)
                            except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
                            raise RuntimeError('flight_process_timeout:'+name)
                if not (path/'comparison_audit.json').exists():raise RuntimeError('missing_audit:'+name)
                row=json.loads((path/'comparison_audit.json').read_text());raw=json.loads((path/'result.json').read_text())
                row.update(directory=str(path),process_returncode=code,planning_requests=len(raw['solves']),
                    offline_found=raw['planner_process']['offline_diagnostic']['found'],
                    offline_ms=raw['planner_process']['offline_diagnostic']['elapsed_ms'])
                save(path/'offline_diagnostic.json',raw['planner_process']['offline_diagnostic'])
                if code or raw['failure'] and any(s in raw['failure'] for s in ('arm failed','stale state','simulation clock')):
                    row['verdict']='evidence_failure';row['passed']=False
                rows.append(row);save(out/'summary.json',rows);report(out,rows)
                print('RESULT',name,row['verdict'],row.get('flight_time_s'),flush=True)
                if row['verdict']=='evidence_failure':raise RuntimeError('infrastructure_failure:'+name)
            pair=[json.loads((Path(r['directory'])/'result.json').read_text()) for r in rows if r['speed_mps']==speed]
            ca,cb=[dict(r['config']) for r in pair]
            ca.pop('free_points');cb.pop('free_points')
            matching=ca==cb and pair[0]['mpc_config']==pair[1]['mpc_config'] and pair[0]['parameters']==pair[1]['parameters']
            paths=[Path(r['directory']) for r in rows if r['speed_mps']==speed]
            matching &= digest(paths[0]/'world.sdf')==digest(paths[1]/'world.sdf')
            save(out/f'pair_v{speed:+g}.json',dict(equal_except_crossing_point=matching))
            if not matching:raise RuntimeError('pair_configuration_mismatch')
        save(out/'status.json',dict(state='complete',pid=os.getpid(),completed=len(rows),sources_unchanged=sources(root)==hashes))
    except Exception as exc:
        save(out/'status.json',dict(state='stopped',pid=os.getpid(),completed=len(rows),error=repr(exc)))
        raise


if __name__=='__main__':main()
