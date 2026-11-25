"""Serial, source-frozen pendulum experiment; all outcomes retained."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
from convex_timevarying_window.comparisons.single_gate_constant_velocity.run import sources,save,digest
from . import scenario

MODULE=__package__+'.flight'
METHODS=('ours','center','high_mpc_author')

def report(out,rows):
    labels={'completed_no_contact':'无碰撞完赛','collision':'碰撞','missed_gate':'漏窗','not_completed':'未完成','online_no_accepted_plan':'未接纳规划','evidence_failure':'证据不足'}
    lines=['# 实验1B：按桨径匹配的摆动圆门','','门名义直径1.978800727m；作者阻尼单摆模型；x500/PX4物理仿真。',
        'High-MPC采用作者缓存问题与原权重，输入界匹配x500；未重新训练，属于明确标注的零样本适配。',
        '每档每方法一次。时间包括从任务开始到共同终点并减速；碰撞/未完成无有效竞速时间。','',
        '|初始角度°|方法|结果|完整时间s|碰框/碰地|最大规划ms|最大控制ms|','|---:|---|---|---:|---|---:|---:|']
    for r in sorted(rows,key=lambda z:(z['angle_deg'],METHODS.index(z['variant']))):
        lines.append('| '+' | '.join(map(str,[r['angle_deg'],r['variant'],labels.get(r['verdict'],r['verdict']),f"{r['flight_time_s']:.3f}" if r.get('passed') else '—',f"{r.get('frame_collision')}/{r.get('ground_collision')}",round(r.get('planner_max_ms') or 0,2),round(r.get('mpc_max_ms') or 0,2)]))+' |')
    lines+=['','原始ULog、接触发布端、接触记录、已知运动表、实际穿越审计与源快照分别保留。',
        '圆门和x500/PX4属于场景适配；门/整机外廓比例未核实，不能把按桨径匹配写成完全几何相似。']
    (out/'REPORT.md').write_text('\n'.join(lines)+'\n')

def main():
    p=argparse.ArgumentParser();p.add_argument('--outdir',type=Path,required=True);p.add_argument('--preflight',action='store_true');p.add_argument('--preflight-results',type=Path,default=scenario.HERE.parent/'results/preflight_pendulum_v3_20261105');a=p.parse_args()
    out=a.outdir.resolve();out.mkdir(parents=True,exist_ok=False)
    angles=[0.] if a.preflight else [0.,60.,120.]
    if not a.preflight:
        pre=a.preflight_results.resolve()
        status=json.loads((pre/'status.json').read_text());rows0=json.loads((pre/'summary.json').read_text())
        if status['state']!='complete' or len(rows0)!=3 or not all(r['passed'] for r in rows0):raise RuntimeError('three-method static interface preflight incomplete')
    if digest(scenario.HERE/'PendulumGateMotion.cc')!=digest(scenario.ASSETS/'PendulumGateMotion.cc'):raise RuntimeError('compiled plugin source differs from recorded source')
    if not a.preflight:
        validation=scenario.HERE.parent/'results/pendulum_dynamic_interface_v1_20261105/gate_pose_audit.json'
        if not json.loads(validation.read_text())['passed']:raise RuntimeError('dynamic physical gate query not validated')
    hashes=sources(scenario.ROOT)
    for path in (scenario.HERE/'PendulumGateMotion.cc',scenario.ASSETS/'build/libPendulumGateMotion.so',scenario.ASSETS/'circle_frame.stl',
                 scenario.ROOT/'复现/high_mpc_author/high_mpc/mpc/saved/high_mpc.so',
                 scenario.ROOT/'复现/high_mpc_author/high_mpc/Dataset/act_net/weights_999.h5',
                 scenario.HERE.parent/'results/author_reproduction_v4_20261105/author_cache_metadata_compat.so'):
        hashes[str(path.relative_to(scenario.ROOT))]=digest(path)
    save(out/'protocol.json',dict(angles_deg=angles,methods=METHODS,source_sha256=hashes,geometry=scenario.manifest(scenario.Case(0)),
        preflight=a.preflight,task_limit_s=40,repetitions=1,policy_retrained=False,original_author_performance_claim=False))
    for rel in hashes:
        target=out/'source_snapshot'/rel;target.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(scenario.ROOT/rel,target)
    rows=[];save(out/'summary.json',rows);report(out,rows)
    try:
        for i,angle in enumerate(angles):
            for method in METHODS[i%3:]+METHODS[:i%3]:
                if any(digest(scenario.ROOT/rel)!=h for rel,h in hashes.items()):raise RuntimeError('frozen source changed')
                name=f'angle{angle:g}_{method}'
                for attempt in range(2):
                    path=out/(name if not attempt else name+'_facility_retry')
                    save(out/'status.json',dict(state='running',pid=os.getpid(),completed=len(rows),total=3*len(angles),angle_deg=angle,variant=method,directory=str(path)))
                    print('START',path.name,flush=True)
                    with (out/(path.name+'.log')).open('w') as stream:
                        proc=subprocess.Popen([sys.executable,'-u','-m',MODULE,'--angle',str(angle),'--method',method,'--outdir',str(path)],cwd=scenario.ROOT,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
                        try:code=proc.wait(timeout=600)
                        except subprocess.TimeoutExpired:
                            os.killpg(proc.pid,signal.SIGTERM)
                            try:proc.wait(timeout=5)
                            except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
                            raise RuntimeError('flight process timeout')
                    raw=json.loads((path/'result.json').read_text()) if (path/'result.json').exists() else None
                    unflown=raw is not None and raw['last_time_s']==0 and any(x in str(raw['failure']) for x in ('arm failed','no heartbeat','initial telemetry'))
                    if unflown and attempt==0:
                        save(path/'facility_retry_reason.json',dict(failure=raw['failure'],unflown=True,original_retained=True));continue
                    if not (path/'comparison_audit.json').exists():raise RuntimeError('missing physical flight audit:'+str(path))
                    row=json.loads((path/'comparison_audit.json').read_text());row.update(directory=str(path),process_returncode=code)
                    if code:row.update(verdict='evidence_failure',passed=False,flight_time_s=None)
                    rows.append(row);save(out/'summary.json',rows);report(out,rows);print('RESULT',name,row['verdict'],row.get('flight_time_s'),flush=True)
                    if code:raise RuntimeError('flight/audit infrastructure failure')
                    break
            paths=[Path(r['directory']) for r in rows if r['angle_deg']==angle]
            if len({digest(path/'world.sdf') for path in paths})!=1 or len({digest(path/'pendulum_table.txt') for path in paths})!=1:raise RuntimeError('unmatched physical scene/motion')
            raw=[json.loads((path/'result.json').read_text()) for path in paths]
            pair=[r for r in raw if r['comparison_variant'] in ('ours','center')]
            configs=[{k:v for k,v in r['config'].items() if k!='free_points'} for r in pair]
            if configs[0]!=configs[1] or pair[0]['mpc_config']!=pair[1]['mpc_config']:raise RuntimeError('unmatched free/centre ablation')
            save(out/f'pair_angle{angle:g}.json',dict(same_world_and_motion=True,same_free_center_except_point=True))
        if any(digest(scenario.ROOT/rel)!=h for rel,h in hashes.items()):raise RuntimeError('source changed at end')
        save(out/'status.json',dict(state='complete',completed=len(rows),total=len(rows),sources_unchanged=True))
    except Exception as exc:
        save(out/'status.json',dict(state='stopped',completed=len(rows),total=3*len(angles),error=repr(exc)));raise

if __name__=='__main__':main()
