"""Center-only ablation of all nine frozen hard_v2 flight conditions."""
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


def baselines(root):
    base=root/'convex_timevarying_window/dynamic_passage_velocity_minco/results'
    rows=json.loads((base/'hard_sweep_v1_20261009/summary.json').read_text())
    for name in ('hard_v2_f4_a4_20261009_01','hard_v2_f3.5_a4.5_20261009_01'):
        path=base/name
        row=audit(path)
        raw=json.loads((path/'result.json').read_text())
        row.update(trial=raw['case'],directory=str(path))
        rows.append(row)
    return sorted(rows,key=lambda r:(r['trial']['amplitude'],r['trial']['frequency']))


def report(out,pairs):
    lines=['# hard_v2自由点与强制中心：九条件配对','',
           '仅锁定中心版q0=q1=0；共用hard_flight入口，其余配置不变。每条件一次，不重试。',
           '自由版复用已完成记录，不重飞；中心版失败仍继续其他条件。',
           '时间包含起点等待和窗后到原终点，单窗、不返航。只有双方无碰撞完赛才比较时间。','',
           '|幅度 m|频率 Hz|自由点|中心|','|---:|---:|---|---|']
    def label(row):
        if row is None:return '待测'
        if row.get('collision_detected'):return '碰撞'
        if row.get('audit_error') or row.get('infrastructure_failure'):return '证据不足'
        if row.get('passed'):return f"无碰撞 {row['flight_time_s']:.3f}s"
        return '未完成'
    for p in pairs:
        t=p['free']['trial']
        lines.append(f"|{t['amplitude']:g}|{t['frequency']:g}|{label(p['free'])}|{label(p['center'])}|")
    (out/'REPORT.md').write_text('\n'.join(lines)+'\n')


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--outdir',type=Path,required=True)
    args=parser.parse_args();root=Path.cwd();shared.ROOT=root
    out=args.outdir.resolve();out.mkdir(parents=True,exist_ok=False)
    pairs=[dict(free=r,center=None) for r in baselines(root)]
    hashes=sources(root)
    save(out/'protocol.json',dict(source_sha256=hashes,conditions=[p['free']['trial'] for p in pairs],
        only_difference='free_points=False fixes q0=q1=0',phase=0,repetitions=1,retries=0,
        module='convex_timevarying_window.dynamic_passage_velocity_minco.hard_flight',
        max_flight_s=40,stop_batch='infrastructure, audit or source failure; not algorithm failure'))
    for rel in hashes:
        target=out/'source_snapshot'/rel;target.parent.mkdir(parents=True,exist_ok=True)
        shutil.copyfile(root/rel,target)
    try:
        for i,pair in enumerate(pairs):
            if sources(root)!=hashes:raise RuntimeError('source_changed')
            t=pair['free']['trial'];f,a=t['frequency'],t['amplitude']
            trial=shared.Trial('center_ablation','center',f,a,0.,0);path=out/trial.name
            save(out/'status.json',dict(state='running',pid=os.getpid(),completed=i,frequency=f,amplitude=a))
            save(out/'summary.json',pairs);report(out,pairs)
            print('START',f,a,flush=True)
            row=shared.execute(trial,path,out/(trial.name+'.log'),100,
                module='convex_timevarying_window.dynamic_passage_velocity_minco.hard_flight',audit_fn=audit)
            pair['center']=row
            rawpath=path/'result.json'
            if rawpath.exists():
                raw=json.loads(rawpath.read_text())
                if raw['config']['free_points']:raise RuntimeError('center_not_locked')
                reference=json.loads((Path(pair['free']['directory'])/'result.json').read_text())
                differences={k:[reference['config'].get(k),raw['config'].get(k)]
                    for k in set(reference['config'])|set(raw['config'])
                    if reference['config'].get(k)!=raw['config'].get(k)}
                matched=all(raw[k]==reference[k] for k in ('mpc_config','parameters','case'))
                save(path/'paired_config_audit.json',dict(config_differences=differences,
                    mpc_px4_case_equal=matched))
                if set(differences)!={'free_points'} or not matched:
                    raise RuntimeError('unexpected_paired_config_difference')
                save(path/'nominal_diagnostic.json',nominal_diagnostic(raw))
            save(out/'summary.json',pairs);report(out,pairs)
            print('RESULT',f,a,row.get('passed'),row.get('collision_detected'),row.get('flight_time_s'),flush=True)
            if row.get('infrastructure_failure') or row.get('audit_error'):raise RuntimeError('infrastructure_or_audit_failure')
        if sources(root)!=hashes:raise RuntimeError('source_changed')
        save(out/'verification.json',dict(sources_unchanged=True,retries=0,conditions=len(pairs)))
        save(out/'status.json',dict(state='complete',pid=os.getpid(),completed=len(pairs)))
    except Exception as exc:
        save(out/'status.json',dict(state='error',pid=os.getpid(),error=repr(exc)))
        raise
    finally:
        subprocess.run(['docker','exec','-e','GZ_PARTITION=convex_seven_togt_experiment_partition',
            'convex_seven_togt_experiment','gz','service','-s','/world/convex_seven_dynamic_px4_togt/control',
            '--reqtype','gz.msgs.WorldControl','--reptype','gz.msgs.Boolean','--timeout','2000','--req','pause: true'],
            capture_output=True,timeout=5)


if __name__=='__main__':main()
