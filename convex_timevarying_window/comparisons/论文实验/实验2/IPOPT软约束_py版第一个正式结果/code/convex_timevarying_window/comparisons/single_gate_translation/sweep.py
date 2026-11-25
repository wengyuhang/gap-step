"""Frozen paired sweep; contact is the primary safety observation."""
import argparse
import hashlib
import json
import os
from pathlib import Path
from convex_timevarying_window.simple_two_gate_soft_only_mpc import run as shared
from convex_timevarying_window.simple_two_gate_soft_only_mpc.run import execute,Trial
from .audit import audit

FREQUENCY=((1.5,2.),(2.5,2.),(3.,2.),(3.5,2.),(4.,2.))
AMPLITUDE=((1.,2.5),(1.,3.),(1.,3.5),(1.,4.))
JOINT=((1.5,2.5),(2.,3.),(2.5,3.5),(3.,4.))
ROUTES={'frequency':FREQUENCY,'amplitude':AMPLITUDE,'joint':JOINT}
FLIGHT='convex_timevarying_window.comparisons.single_gate_translation.flight'


def save(path,value):path.write_text(json.dumps(value,indent=2,ensure_ascii=False)+'\n')


def source_hashes():
    root=Path(__file__).resolve().parents[3]
    selected=[]
    selected.extend(x for x in Path(__file__).parent.glob('*.py'))
    selected.extend(x for x in (root/'convex_timevarying_window/simple_two_gate_soft_only_mpc').glob('*.py'))
    selected.extend(x for x in (root/'convex_timevarying_window/simple_two_gate_mpc').glob('*.py'))
    selected.extend([root/'convex_dynamic_seven_window_gazebo/src/PeriodicGateMotion.cc',
                     root/'复现/polynomial_dynamic_racing_2023/px4_fixed_plane/scenario.py',
                     root/'convex_dynamic_seven_window_gazebo/convex_seven_dynamic_px4_togt.sdf'])
    return {str(x.relative_to(root)):hashlib.sha256(x.read_bytes()).hexdigest() for x in selected}


def report(out,rows):
    pairs={(r['trial']['frequency'],r['trial']['amplitude'],r['trial']['method']):r for r in rows}
    lines=['# 原尺寸单窗：动态窗口频率与振幅升档','',
      '安全主判据为Gazebo实测窗框接触。每条件每方一次，初相0；时间为起点到窗后目标。','',
      '|振幅m|频率Hz|自由点结果|自由点时间s|中心结果|中心时间s|时间优势s|',
      '|---:|---:|---|---:|---|---:|---:|']
    cases=sorted({(a,f) for route in ROUTES.values() for f,a in route})
    for a,f in cases:
        pair={m:pairs.get((f,a,m)) for m in ('ours','center')}
        def label(row):
            if row is None:return '待运行'
            if row.get('collision_detected'):return '碰撞'
            if row.get('passed'):return '无碰撞完赛'
            if row.get('audit_error'):return '证据不足'
            return '未完成'
        def time(row):return '—' if not row or not row.get('passed') else f"{row['flight_time_s']:.3f}"
        ours,center=pair['ours'],pair['center']
        advantage=(f"{center['flight_time_s']-ours['flight_time_s']:.3f}"
                   if ours and center and ours.get('passed') and center.get('passed') else '—')
        lines.append(f'| {a:g} | {f:g} | {label(ours)} | {time(ours)} | '
                     f'{label(center)} | {time(center)} | {advantage} |')
    lines+=['','碰撞和未完成不赋予有效完成时间；时间优势仅对双方安全完赛计算。']
    (out/'REPORT.md').write_text('\n'.join(lines)+'\n')


def check_pair(pair):
    left,right=pair
    for field in ('world_sha256','scene_manifest','mpc_config','px4_parameters'):
        assert left[field]==right[field],field
    configs=[json.loads((Path(row['directory'])/'result.json').read_text())['config'] for row in pair]
    assert configs[0].pop('free_points')!=configs[1].pop('free_points')
    assert configs[0]==configs[1]


def main():
    p=argparse.ArgumentParser();p.add_argument('--outdir',type=Path,required=True);a=p.parse_args()
    shared.ROOT=Path.cwd()
    out=a.outdir.resolve();out.mkdir(parents=True,exist_ok=False)
    hashes=source_hashes()
    save(out/'protocol.json',dict(routes=ROUTES,methods=('center','ours'),phase=0.,
         repeat_count=1,source_sha256=hashes,text=Path(__file__).with_name('SWEEP_PROTOCOL.md').read_text(),
         prior_pilot='pilot_v2_20261008; no old flights copied into this batch'))
    rows=[];attempts=[];report(out,rows)
    steps=[(route,ROUTES[route][i]) for i in range(5) for route in ROUTES if i<len(ROUTES[route])]
    for route,(f,amplitude) in steps:
        pair=[]
        for method in ('center','ours'):
            assert source_hashes()==hashes,'Experiment source changed after freeze'
            trial=Trial('single_'+route,method,f,amplitude,0.,0)
            for attempt in range(2):
                name=trial.name+(f'_setup_retry{attempt}' if attempt else '')
                path=out/name
                save(out/'status.json',dict(state='running',pid=os.getpid(),route=route,
                  frequency=f,amplitude=amplitude,method=method,attempt=attempt,completed=len(rows)))
                print('START',name,flush=True)
                row=execute(trial,path,out/(name+'.log'),100,module=FLIGHT,audit_fn=audit)
                row.update(route=route,attempt=attempt)
                attempts.append(row);save(out/'attempts.json',attempts)
                if not row.get('infrastructure_failure') or attempt==1:break
            row['passed']=bool(row.get('passed') and not row.get('collision_detected')
                               and not row.get('infrastructure_failure') and not row.get('audit_error'))
            rows.append(row);pair.append(row);save(out/'summary.json',rows);report(out,rows)
            print('RESULT',name,'collision',row.get('collision_detected'),
                  'passed',row['passed'],'time',row.get('flight_time_s'),flush=True)
            if (row.get('infrastructure_failure') or
                (row.get('audit_error') and not row.get('collision_detected'))):
                save(out/'status.json',dict(state='evidence_or_setup_failure',directory=str(path),
                                            completed=len(rows)))
                return
        check_pair(pair)
    save(out/'status.json',dict(state='complete',completed=len(rows)))


if __name__=='__main__':main()
