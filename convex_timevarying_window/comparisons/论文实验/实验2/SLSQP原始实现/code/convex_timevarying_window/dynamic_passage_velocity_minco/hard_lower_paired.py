"""Fresh hard_v2 free/center flights at lower previously unpaired conditions."""
import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
from .hard_sweep import nominal_diagnostic
from .sweep import save, sources
from convex_timevarying_window.simple_two_gate_soft_only_mpc import run as shared
from convex_timevarying_window.comparisons.single_gate_translation.audit import audit

CASES=((.5,1.),(1.,2.),(2.,3.),(3.,4.),(3.5,4.),(3.,4.5),(3.,5.),(3.,5.5))
MODULE='convex_timevarying_window.dynamic_passage_velocity_minco.hard_flight'


def report(out,rows):
    lines=['# hard_v2低档自由点/中心配对','',
        '双方均新飞，每条件初相0各一次，无重试；仅free_points不同，穿窗速度均可优化。',
        '保持原小窗、x500/PX4、90ms求解预算、40s飞行上限、MPC、接触及终点判据。',
        '完整时间包含起点等首解和窗后到目标，不含编译/起飞准备；单窗不返航。','',
        '|幅度 m|频率 Hz|自由点|强制中心|','|---:|---:|---|---|']
    def label(r):
        if r is None:return '待测'
        if r.get('audit_error') or r.get('infrastructure_failure'):return '证据不足'
        if r.get('collision_detected'):return '碰撞'
        if r.get('passed'):return f"无碰撞 {r['flight_time_s']:.3f}s"
        return '未完成'
    for f,a in sorted(CASES,key=lambda x:(x[1],x[0])):
        pair={r['trial']['method']:r for r in rows if r['trial']['frequency']==f and r['trial']['amplitude']==a}
        lines.append(f"|{a:g}|{f:g}|{label(pair.get('ours'))}|{label(pair.get('center'))}|")
    (out/'REPORT.md').write_text('\n'.join(lines)+'\n')


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--outdir',type=Path,required=True)
    args=parser.parse_args();root=Path.cwd();shared.ROOT=root
    out=args.outdir.resolve();out.mkdir(parents=True,exist_ok=False)
    hashes=sources(root);rows=[]
    save(out/'protocol.json',dict(cases=CASES,methods=['ours','center'],module=MODULE,
        phase=0,repetitions=1,retries=0,max_flight_s=40,source_sha256=hashes,
        algorithm_changes=False,only_difference='free_points',
        stop_batch='source, infrastructure or audit error; not algorithm failure'))
    for rel in hashes:
        target=out/'source_snapshot'/rel;target.parent.mkdir(parents=True,exist_ok=True)
        shutil.copyfile(root/rel,target)
    save(out/'summary.json',rows);report(out,rows)
    try:
        for f,a in CASES:
            reference=None
            for method in ('ours','center'):
                if sources(root)!=hashes:raise RuntimeError('source_changed')
                trial=shared.Trial('lower',method,f,a,0.,0);path=out/trial.name
                save(out/'status.json',dict(state='running',pid=os.getpid(),completed=len(rows),
                    total=2*len(CASES),frequency=f,amplitude=a,method=method,directory=str(path)))
                print('START',method,f,a,flush=True)
                row=shared.execute(trial,path,out/(trial.name+'.log'),100,module=MODULE,audit_fn=audit)
                rows.append(row);save(out/'summary.json',rows);report(out,rows)
                print('RESULT',method,f,a,row.get('passed'),row.get('collision_detected'),row.get('flight_time_s'),flush=True)
                if row.get('infrastructure_failure') or row.get('audit_error'):raise RuntimeError('infrastructure_or_audit_error')
                raw=json.loads((path/'result.json').read_text())
                if raw['config']['free_points']!=(method=='ours'):raise RuntimeError('wrong_variant')
                if reference is None:reference=raw
                else:
                    differences={k:[reference['config'].get(k),raw['config'].get(k)]
                        for k in set(reference['config'])|set(raw['config'])
                        if reference['config'].get(k)!=raw['config'].get(k)}
                    matched=all(reference[k]==raw[k] for k in ('case','parameters','mpc_config'))
                    save(path/'paired_config_audit.json',dict(config_differences=differences,mpc_px4_case_equal=matched))
                    if set(differences)!={'free_points'} or not matched:raise RuntimeError('pair_mismatch')
                save(path/'nominal_diagnostic.json',nominal_diagnostic(raw))
        if sources(root)!=hashes:raise RuntimeError('source_changed')
        save(out/'verification.json',dict(sources_unchanged=True,retries=0,flights=len(rows)))
        save(out/'status.json',dict(state='complete',pid=os.getpid(),completed=len(rows)))
    except Exception as exc:
        save(out/'status.json',dict(state='error',pid=os.getpid(),completed=len(rows),error=repr(exc)))
        raise
    finally:
        subprocess.run(['docker','exec','-e','GZ_PARTITION=convex_seven_togt_experiment_partition',
            'convex_seven_togt_experiment','gz','service','-s','/world/convex_seven_dynamic_px4_togt/control',
            '--reqtype','gz.msgs.WorldControl','--reptype','gz.msgs.Boolean','--timeout','2000','--req','pause: true'],
            capture_output=True,timeout=5)


if __name__=='__main__':main()
