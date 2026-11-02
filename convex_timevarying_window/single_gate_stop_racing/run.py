"""Frozen five-case seven-gate single-stop experiment."""
import argparse
from dataclasses import asdict
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time
from convex_timevarying_window.comparisons.comprehensive_wang.registry import Trial
from .audit import audit
from .scenario import FIRST_GATE_SHIFT_M,HORIZONTAL_SPACING_SCALE

CASES=((.5,1.),(1.,2.),(2.,3.),(3.,4.),(3.5,4.))


def save(path,value):
    path.write_text(json.dumps(value,indent=2,ensure_ascii=False)+'\n')


def sources(root):
    directories=('convex_timevarying_window','convex_dynamic_seven_window_gazebo',
                 'nonconvex_timevarying_window/sc_dynatogt','复现')
    files=[p for d in directories for p in (root/d).rglob('*.py')
           if 'results' not in p.parts and '__pycache__' not in p.parts and not p.is_symlink()]
    return {str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest() for p in files}


def execute(root,trial,path,log_path):
    command=[sys.executable,'-u','-m','convex_timevarying_window.single_gate_stop_racing.flight',
             '--tracker','px4-model','--method',trial.method,'--frequency',str(trial.frequency),
             '--amplitude',str(trial.amplitude),'--phase',str(trial.phase),'--outdir',str(path)]
    env=dict(os.environ,OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1',GZ_SHOW_GUI='0')
    start=time.monotonic();error=None
    with log_path.open('w') as log:
        with subprocess.Popen(command,cwd=root,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True) as proc:
            try:code=proc.wait(timeout=900)
            except subprocess.TimeoutExpired:
                error='process_timeout';os.killpg(proc.pid,signal.SIGTERM)
                try:proc.wait(timeout=5)
                except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
                code=proc.returncode
    row=dict(trial=asdict(trial),name=trial.name,directory=str(path),wall_s=time.monotonic()-start,
             returncode=code,process_error=error,infrastructure_failure=bool(error))
    with log_path.with_suffix('.docker.log').open('w') as log:
        subprocess.run(['docker','logs','convex_seven_togt_experiment'],stdout=log,stderr=subprocess.STDOUT,timeout=15)
    if not (path/'result.json').exists():return dict(row,audit_error='missing_result')
    raw=json.loads((path/'result.json').read_text())
    row.update(failure=raw.get('failure'),collision_detected=raw.get('collision_detected',False))
    row['infrastructure_failure'] |= raw.get('failure')=="RuntimeError('arm failed')"
    row['world_sha256']=hashlib.sha256((path/'world.sdf').read_bytes()).hexdigest()
    row['mpc_config']=raw['mpc_config'];row['px4_parameters']=raw['parameters']
    row['scene_manifest']=json.loads((path/'scene.json').read_text())
    try:row.update(audit(path))
    except Exception as exc:row['audit_error']=repr(exc)
    row['passed']=bool(row.get('passed') and not row.get('audit_error') and not row['infrastructure_failure'])
    save(path/'comparison_audit.json',row)
    return row


def report(out,rows):
    lines=['# 单窗停车点滚动七窗实验','',
        '七窗水平锚点相对固定起终点拉开2倍，第一窗额外侧移-5m；高度/开口/朝向/实际米数幅度不变。',
        '固定平面/水平正弦、初相0各一次，仅自由点。MPC预测实际PX4 PID/归一化分配/x500阻力力矩，修正GPS投影坐标单位；同一跟踪代价用直接L-BFGS-B求解。规划/物理界不变。无条件调参/重飞。',
        '整机出窗后重规划；末窗后不重规划，完整返回起点停稳。真实实体接触判碰撞。','',
        '|幅度m|频率Hz|结果|完整七窗返航s|实际完成窗数|规划最大ms|MPC最大ms|',
        '|---:|---:|---|---:|---:|---:|---:|']
    for r in sorted(rows,key=lambda x:(x['trial']['amplitude'],x['trial']['frequency'])):
        label='碰撞' if r.get('collision_detected') else '证据不足' if r.get('audit_error') or r.get('infrastructure_failure') else '无碰撞完赛' if r.get('passed') else '未完成'
        time=f"{r['flight_time_s']:.3f}" if r.get('passed') else '—'
        vals=[str(r['trial']['amplitude']),str(r['trial']['frequency']),label,time,str(r.get('actual_gate_count','—'))]
        for k in ('planner_max_ms','mpc_max_ms'):vals.append(f"{r[k]:.2f}" if r.get(k) is not None else '—')
        lines.append('| '+' | '.join(vals)+' |')
    lines+=['','不以出窗/到达目标/采样净空单独判安全完赛；每条件一次，不估计成功率。',
            '初始规划等待包含在竞速时间；飞前编译、起飞准备不包含。节点界/浮点局部包络不是形式化保证。']
    (out/'REPORT.md').write_text('\n'.join(lines)+'\n')


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--outdir',required=True,type=Path)
    parser.add_argument('--last-two',action='store_true',help='Only the two previously unfinished conditions')
    args=parser.parse_args();cases=CASES[-2:] if args.last_two else CASES
    root=Path.cwd();out=args.outdir.resolve();out.mkdir(parents=True,exist_ok=False)
    hashes=sources(root);rows=[]
    save(out/'protocol.json',dict(cases=cases,repetitions=1,phase=0.,max_flight_s=120,retries=0,
        source_sha256=hashes,metrics=['complete_seven_gate_return_time','physical_contacts'],
        only_nearest_window=True,first_gate_sideways_shift_m=FIRST_GATE_SHIFT_M,
        horizontal_spacing_scale=HORIZONTAL_SPACING_SCALE,
        other_gate_avoidance_enabled=False,
        tracking_error_replan_enabled=False,
        prediction_model='PX483c4f4e_PID_normalized_allocation_x500_drag_moments',
        position_coordinate_mapping='PX4_spherical_GPS_map_to_Gazebo_WGS84_metres',
        ground_truth_feedback_enabled=False,
        stop_batch='source/infrastructure/audit failure, not algorithm failure'))
    for rel in hashes:
        dst=out/'source_snapshot'/rel;dst.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(root/rel,dst)
    try:
        for f,a in cases:
            if sources(root)!=hashes:raise RuntimeError('source_changed')
            trial=Trial('seven','ours',f,a,0.,0);path=out/trial.name
            save(out/'status.json',dict(state='running',completed=len(rows),pid=os.getpid(),frequency=f,amplitude=a))
            print('START',f,a,flush=True)
            row=execute(root,trial,path,out/(trial.name+'.log'))
            rows.append(row);save(out/'summary.json',rows);report(out,rows)
            print('RESULT',f,a,row.get('passed'),row.get('collision_detected'),row.get('flight_time_s'),row.get('actual_gate_count'),flush=True)
            if row.get('audit_error') or row.get('infrastructure_failure'):raise RuntimeError('infrastructure_or_audit_failure')
        if sources(root)!=hashes:raise RuntimeError('source_changed')
        save(out/'verification.json',dict(sources_unchanged=True,retries=0,flights=len(rows)))
        save(out/'status.json',dict(state='complete',completed=len(rows),pid=os.getpid()))
    except Exception as exc:
        save(out/'status.json',dict(state='error',completed=len(rows),pid=os.getpid(),error=repr(exc)));raise
    finally:
        subprocess.run(['docker','exec','-e','GZ_PARTITION=convex_seven_togt_experiment_partition',
            'convex_seven_togt_experiment','gz','service','-s','/world/convex_seven_dynamic_px4_togt/control',
            '--reqtype','gz.msgs.WorldControl','--reptype','gz.msgs.Boolean','--timeout','2000','--req','pause: true'],
            capture_output=True,timeout=5)


if __name__=='__main__':main()
