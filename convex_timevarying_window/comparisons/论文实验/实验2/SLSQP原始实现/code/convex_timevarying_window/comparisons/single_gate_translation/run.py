"""Frozen sequential pilot: static and two fast planar translations, paired methods."""
import argparse
from dataclasses import asdict
import hashlib
import json
import os
from pathlib import Path
from convex_timevarying_window.simple_two_gate_soft_only_mpc.run import execute,Trial
from convex_timevarying_window.simple_two_gate_soft_only_mpc import run as shared_run
from .audit import audit

CASES=((1.,0.),(1.,2.),(2.,2.))
METHODS=('center','ours')
FLIGHT='convex_timevarying_window.comparisons.single_gate_translation.flight'


def save(path,obj):path.write_text(json.dumps(obj,indent=2,ensure_ascii=False)+'\n')


def report(out,rows):
    pairs={(r['trial']['frequency'],r['trial']['amplitude'],r['trial']['method']):r for r in rows}
    lines=['# 原尺寸单窗高速平移机制实验','',
      '完整时间从起点至单窗后方目标；本实验不返航。每条件每方法一次，碰撞依Gazebo接触。','',
      '|幅度 m|频率 Hz|自由点结果|自由点用时 s|中心结果|中心用时 s|',
      '|---:|---:|---|---:|---|---:|']
    for f,a in CASES:
        vals=[]
        for method in ('ours','center'):
            r=pairs.get((f,a,method),{})
            outcome=('碰撞' if r.get('collision_detected') else
                     '完成' if r.get('passed') else '未完成' if r else '待运行')
            val=f"{r['flight_time_s']:.3f}" if r.get('passed') else '—'
            vals.extend([outcome,val])
        lines.append(f'| {a:g} | {f:g} | '+' | '.join(vals)+' |')
    (out/'REPORT.md').write_text('\n'.join(lines)+'\n')


def main():
    p=argparse.ArgumentParser();p.add_argument('--outdir',type=Path,required=True);a=p.parse_args()
    # The ignored reproduction package is read through a symlink in this
    # worktree. Its historical ROOT points at the original checkout; the
    # child flight must start here so it loads this experiment package.
    shared_run.ROOT=Path.cwd()
    out=a.outdir.resolve();out.mkdir(parents=True,exist_ok=False)
    src=Path(__file__).parent
    hashes={x.name:hashlib.sha256(x.read_bytes()).hexdigest() for x in src.glob('*.py')}
    save(out/'protocol.json',dict(cases=CASES,methods=METHODS,phase=0.,
       source_hashes=hashes,text=(src/'PROTOCOL.md').read_text(),repetitions=1))
    rows=[];report(out,rows)
    for f,amplitude in CASES:
        pair=[]
        for method in METHODS:
            assert hashes=={x.name:hashlib.sha256(x.read_bytes()).hexdigest() for x in src.glob('*.py')}
            trial=Trial('single_gate',method,f,amplitude,0.,0)
            path=out/trial.name
            save(out/'status.json',dict(state='running',pid=os.getpid(),frequency=f,
                                       amplitude=amplitude,method=method,completed=len(rows)))
            row=execute(trial,path,out/(trial.name+'.log'),100,module=FLIGHT,audit_fn=audit)
            if row.get('audit_error'):
                row['passed']=False
            else:
                row['passed']=bool(row.get('passed') and not row.get('infrastructure_failure'))
            rows.append(row);pair.append(row)
            save(out/'summary.json',rows);report(out,rows)
            print('RESULT',trial.name,row.get('passed'),row.get('collision_detected'),flush=True)
            if row.get('infrastructure_failure') or row.get('audit_error'):
                save(out/'status.json',dict(state='evidence_or_setup_failure',completed=len(rows),
                                            directory=str(path)))
                return
        assert pair[0]['world_sha256']==pair[1]['world_sha256']
        assert pair[0]['mpc_config']==pair[1]['mpc_config']
        assert pair[0]['px4_parameters']==pair[1]['px4_parameters']
        raw=[json.loads((Path(r['directory'])/'result.json').read_text()) for r in pair]
        configs=[dict(r['config']) for r in raw]
        assert configs[0].pop('free_points')!=configs[1].pop('free_points')
        assert configs[0]==configs[1]
    save(out/'status.json',dict(state='complete',completed=len(rows)))


if __name__=='__main__':main()
