"""Frozen paired sweep; independent enlarged-world audit and collision evidence."""
import argparse
import json
import os
from pathlib import Path
import shutil
from convex_timevarying_window.simple_two_gate_soft_only_mpc.run import (
    sources as algorithm_sources,execute,evaluate,save,ROOT,Trial)
from 复现.polynomial_dynamic_racing_2023.px4_original_track_legacy.run import digest
from .audit import audit
from .scenario import ASSETS,Case,build_case,write_world

ROUTES={
    'frequency':[(1.+.5*k,2.) for k in range(1,7)],
    'amplitude':[(1.,2.+.5*k) for k in range(1,7)],
    'joint':[(1.+.5*k,2.+.5*k) for k in range(1,7)],
}


def sources():
    values=algorithm_sources()
    for path in Path(__file__).parent.rglob('*'):
        if 'results' not in path.parts and path.suffix in ('.py','.md'):
            values[str(path.relative_to(ROOT))]=digest(path)
    for path in (ASSETS/'results'/'large_window_meshes_scale3_v1').glob('*.stl'):
        values[str(path.relative_to(ROOT))]=digest(path)
    for name in ('export_world.py','course.py'):
        path=ASSETS/name;values[str(path.relative_to(ROOT))]=digest(path)
    return values


def adjudicate(row):
    row=evaluate(row)
    # Require the enlarged-geometry truth audit to confirm ordering as well as count.
    passed=bool(row.get('simulation_passed') and row.get('ordered_once')
                and row.get('complete_body_exits') and not row.get('audit_error')
                and not row.get('infrastructure_failure'))
    row.update(simulation_passed=passed,
               simulation_race_time_s=row.get('flight_time_s') if passed else None)
    row['simulation_outcome']=('collision' if row.get('collision_detected') else
                              'complete' if passed else 'incomplete')
    return row


def report(out,rows,states):
    lines=['# 3倍开口：闭环自由点与强制中心','',
        '每条件每方法一次，初相固定；完整时间含返回终点。物理窗框接触判碰撞，超时无有效成绩。',
        '双方成功才比较时间；不把缺证据或碰撞后返回当成功。不宣称随机成功率/单调阈值。','',
        '|振幅m|频率Hz|自由点时间s|自由点结果|中心时间s|中心结果|自由点时间优势s|',
        '|---:|---:|---:|---|---:|---|---:|']
    pairs={}
    for row in rows:
        t=row['trial'];pairs.setdefault((t['amplitude'],t['frequency']),{})[t['method']]=row
    for (a,f),pair in sorted(pairs.items()):
        left,right=pair.get('ours',{}),pair.get('center',{})
        lt,rt=left.get('simulation_race_time_s'),right.get('simulation_race_time_s')
        fmt=lambda x:'—' if x is None else f'{x:.3f}'
        lines.append('| '+' | '.join(map(str,[a,f,fmt(lt),left.get('simulation_outcome','待运行'),
                  fmt(rt),right.get('simulation_outcome','待运行'),fmt(rt-lt if rt is not None and lt is not None else None)]))+' |')
    lines+=['','正时间优势表示自由点更快。','',json.dumps(states,ensure_ascii=False,indent=2)]
    (out/'REPORT.md').write_text('\n'.join(lines)+'\n')


def check_pair(left,right):
    assert left['world_sha256']==right['world_sha256'],'Different physical scenes'
    assert left['scene_manifest']==right['scene_manifest'],'Different scene metadata'
    assert left['mpc_config']==right['mpc_config'],'Different trackers'
    assert left['px4_parameters']==right['px4_parameters'],'Different PX4 parameters'
    a=json.loads((Path(left['directory'])/'result.json').read_text())['config']
    b=json.loads((Path(right['directory'])/'result.json').read_text())['config']
    assert a['free_points']!=b['free_points']
    assert {k:v for k,v in a.items() if k!='free_points'}=={k:v for k,v in b.items() if k!='free_points'}


def main(*,scene_build=build_case,world_writer=write_world,source_fn=sources,
         flight_module='convex_timevarying_window.comparisons.large_window_sweep.flight',
         audit_fn=audit,protocol_path=None):
    p=argparse.ArgumentParser();p.add_argument('--outdir',type=Path,required=True)
    args=p.parse_args();out=args.outdir.resolve();out.mkdir(parents=True,exist_ok=False)
    preview=out/'geometry_check';preview.mkdir()
    world_writer(scene_build(Case(1.,2.,0.))[0],ASSETS,preview)
    hashes=source_fn()
    save(out/'protocol.json',dict(aperture_scale=3.,routes=ROUTES,baseline=[1.,2.],
        max_trials=38,phase=0.,repeats=1,source_sha256=hashes,
        stop='Both methods fail at same route step, or finite test cap',
        text=(protocol_path or Path(__file__).with_name('PROTOCOL.md')).read_text()))
    for rel in hashes:
        dest=out/'source_snapshot'/rel;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(ROOT/rel,dest)
    rows=[];states={route:dict(state='pending') for route in ROUTES}
    sequence=[('baseline',1.,2.)]+[(route,*ROUTES[route][k]) for k in range(6) for route in ROUTES]
    report(out,rows,states)
    for index,(route,f,a) in enumerate(sequence):
        if route!='baseline' and states[route]['state']=='both_failed':continue
        pair=[]
        for method in (('ours','center') if index%2==0 else ('center','ours')):
            assert source_fn()==hashes,'Frozen sources changed'
            trial=Trial('large3_'+route,method,f,a,0.,0)
            for attempt in range(2):
                path=out/(trial.name+(f'_setup_retry{attempt}' if attempt else ''))
                save(out/'status.json',dict(state='running',pid=os.getpid(),route=route,
                    frequency=f,amplitude=a,method=method,attempt=attempt,completed=len(rows),boundaries=states))
                print('START',path.name,flush=True)
                row=execute(trial,path,out/(path.name+'.log'),100,
                    module=flight_module,audit_fn=audit_fn)
                row=adjudicate(row);row.update(route=route,attempt=attempt)
                if row.get('infrastructure_failure') and attempt==0:
                    save(out/(path.name+'_setup_failure.json'),row);continue
                break
            raw_path=path/'result.json'
            if raw_path.exists():
                raw=json.loads(raw_path.read_text())
                row['tracking_error_events']=sum(e['action']=='trigger' and e['reason']=='tracking_error' for e in raw['planning_events'])
                row['contacts_during_flight']=[dict(window=c['raw_log'],time_s=c['first_contact_sim_time_s']-30)
                    for c in raw['contacts'] if c['first_contact_sim_time_s'] is not None
                    and 30<=c['first_contact_sim_time_s']<=30+raw['last_time_s']]
            rows.append(row);pair.append(row);save(out/'summary.json',rows);report(out,rows,states)
            print('RESULT',path.name,row['simulation_outcome'],row.get('simulation_race_time_s'),flush=True)
            if row.get('infrastructure_failure') or not raw_path.exists() or (row.get('audit_error') and not row.get('contacts_during_flight')):
                save(out/'status.json',dict(state='evidence_or_setup_failure',directory=str(path),completed=len(rows)))
                raise RuntimeError('Trial has unresolved evidence/setup failure')
        check_pair(*pair)
        if route!='baseline':
            stopped=all(not row['simulation_passed'] for row in pair)
            states[route]=dict(state='both_failed' if stopped else 'continue',last_case=[f,a])
        save(out/'boundaries.json',states);report(out,rows,states)
    for state in states.values():
        if state['state']=='continue':state['state']='test_cap_reached'
    report(out,rows,states)
    save(out/'status.json',dict(state='complete',completed=len(rows),boundaries=states,source_unchanged=source_fn()==hashes))


if __name__=='__main__':main()
