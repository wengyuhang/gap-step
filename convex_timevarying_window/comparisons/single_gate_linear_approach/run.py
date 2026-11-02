"""Frozen paired experiment, no solver or subprocess wall-time truncation."""
import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
from .planner import Config
from .scenario import Case, manifest
from convex_timevarying_window.comparisons.single_gate_constant_velocity.run import save, sources, digest
from convex_timevarying_window.comparisons.single_gate_swing.extension import unflown_reason

MODULE='convex_timevarying_window.comparisons.single_gate_linear_approach.flight'
SPEEDS=(0.,5.,-5.,10.,-10.,15.,-15.,20.,-20.)


def report(out,rows):
    labels=dict(completed_no_contact='无碰撞完赛',collision='碰撞',missed_gate='漏窗',
        online_no_accepted_plan='未接纳轨迹',not_completed='未完成',evidence_failure='证据不足')
    lines=['# 直线横移单窗配对实验','',
        '同起终点、固定初始侧向偏移、原小窗、当前硬约束PV-MINCO与单重射击MPC；每条件双方各一次。',
        '初始侧向偏移见protocol与各行原始审计。正速度在偏移/v秒后越过最近位置。仅双方无碰撞完成时比较时间。','',
        '|方向|窗速m/s|自由结果|自由完整s|中心结果|中心完整s|自由节省|',
        '|---|---:|---|---:|---|---:|---:|']
    for speed in sorted(set(r['speed_mps'] for r in rows),key=lambda s:(s<0,abs(s))):
        pair={r['variant']:r for r in rows if r['speed_mps']==speed}
        values=['靠近侧' if speed>0 else '远离侧' if speed<0 else '静止',f'{abs(speed):g}']
        for variant in ('ours','center'):
            r=pair.get(variant,{})
            values += [labels.get(r.get('verdict'),'待完成'),f"{r['flight_time_s']:.3f}" if r.get('passed') else '—']
        a,b=pair.get('ours',{}),pair.get('center',{})
        values += [f"{100*(b['flight_time_s']-a['flight_time_s'])/b['flight_time_s']:.2f}%" if a.get('passed') and b.get('passed') else '—']
        lines.append('| '+' | '.join(values)+' |')
    lines += ['','## 穿越机制与计算','','|有符号窗速|方法|过面时刻s|当时仍靠近起点|窗内点m|相对横速m/s|规划最大ms|原始审计|',
        '|---:|---|---:|---|---|---:|---:|---|']
    for r in sorted(rows,key=lambda r:(r['speed_mps']<0,abs(r['speed_mps']),r['variant'])):
        e=next(iter(r['crossings']),{})
        values=[r['speed_mps'],r['variant'],round(e['time_s'],4) if e else '—',
            e.get('approaching_start_at_crossing','—'),str(e.get('point_local_m','—')),
            round(e['relative_lateral_speed_mps'],3) if e else '—',round(r.get('planner_max_ms') or 0,2),
            f"[{Path(r['directory']).name}]({Path(r['directory']).name}/comparison_audit.json)"]
        lines.append('| '+' | '.join(map(str,values))+' |')
    lines += ['','未接纳解不等于碰撞或物理不可行；每条件一次不能估计成功概率。',
        '1ms物理接触/ULog为本次闭环观察，不是连续安全证书。任务时间包含等待及到终点刹车。']
    (out/'REPORT.md').write_text('\n'.join(lines)+'\n')


def main():
    p=argparse.ArgumentParser();p.add_argument('--outdir',type=Path,required=True)
    args=p.parse_args();root=Path(__file__).resolve().parents[3];out=args.outdir.resolve()
    out.mkdir(parents=True,exist_ok=False);hashes=sources(root);rows=[]
    save(out/'protocol.json',dict(cases=[manifest(Case(v)) for v in SPEEDS],
        repetitions=1,unflown_facility_retries=1,source_sha256=hashes,module=MODULE,
        planner_wall_limit_ms=Config().max_solve_ms,max_task_s=40,
        protocol=Path(__file__).with_name('README.md').read_text()))
    for rel in hashes:
        dest=out/'source_snapshot'/rel;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(root/rel,dest)
    save(out/'summary.json',rows);report(out,rows)
    try:
        for i,speed in enumerate(SPEEDS):
            for variant in (('ours','center') if i%2==0 else ('center','ours')):
                assert sources(root)==hashes,'frozen_source_changed'
                name=f'v{speed:+g}_{variant}'
                for attempt in range(2):
                    path=out/(name if not attempt else name+'_facility_retry')
                    save(out/'status.json',dict(state='running',pid=os.getpid(),completed=len(rows),total=2*len(SPEEDS),
                        speed_mps=speed,variant=variant,attempt=attempt,directory=str(path)))
                    print('START',path.name,flush=True)
                    command=[sys.executable,'-u','-m',MODULE,'--speed',str(speed),'--method',variant,'--outdir',str(path)]
                    with (out/(path.name+'.log')).open('w') as stream:
                        with subprocess.Popen(command,cwd=root,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True) as proc:
                            code=proc.wait()
                    raw=json.loads((path/'result.json').read_text()) if (path/'result.json').exists() else None
                    audit=json.loads((path/'comparison_audit.json').read_text()) if (path/'comparison_audit.json').exists() else None
                    reason=unflown_reason(path,raw,audit,(out/(path.name+'.log')).read_text())
                    if reason:
                        save(path/'facility_retry_reason.json',dict(reason=reason,unflown=True,attempt=attempt,original_audit=audit))
                        if not attempt:
                            print('UNFLOWN_FACILITY_RETRY',path.name,reason,flush=True);continue
                        raise RuntimeError('repeated_facility_failure:'+path.name)
                    if code or audit is None:raise RuntimeError('infrastructure_or_audit_failure:'+path.name)
                    audit.update(directory=str(path),process_returncode=code,planning_requests=len(raw['solves']),
                        offline_found=raw['planner_process']['offline_diagnostic']['found'])
                    break
                rows.append(audit);save(out/'summary.json',rows);report(out,rows)
                print('RESULT',name,audit['verdict'],audit.get('flight_time_s'),flush=True)
            pair=rows[-2:];raws=[json.loads((Path(r['directory'])/'result.json').read_text()) for r in pair]
            configs=[dict(r['config']) for r in raws]
            for cfg in configs:cfg.pop('free_points')
            matching=(configs[0]==configs[1] and raws[0]['mpc_config']==raws[1]['mpc_config']
                and raws[0]['parameters']==raws[1]['parameters']
                and digest(Path(pair[0]['directory'])/'world.sdf')==digest(Path(pair[1]['directory'])/'world.sdf'))
            save(out/f'pair_v{speed:+g}.json',dict(equal_except_crossing_point=matching))
            assert matching,'pair_configuration_mismatch'
        assert sources(root)==hashes,'frozen_source_changed_at_end'
        save(out/'status.json',dict(state='complete',pid=os.getpid(),completed=len(rows),total=2*len(SPEEDS),sources_unchanged=True))
    except Exception as exc:
        save(out/'status.json',dict(state='stopped',pid=os.getpid(),completed=len(rows),total=2*len(SPEEDS),error=repr(exc)))
        raise


if __name__=='__main__':main()
