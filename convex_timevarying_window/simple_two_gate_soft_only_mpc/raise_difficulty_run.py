"""Frozen free-point upward sweep: observed contacts, not nominal vetoes."""
import argparse
import json
import shutil
from pathlib import Path
from .run import sources, execute, evaluate, save, ROOT, Trial


ROUTES = {
    'joint': [(2.+.5*k,3.+.5*k) for k in range(1,5)],
    'frequency': [(2.+.5*k,2.) for k in range(1,5)],
    'amplitude': [(1.,3.+.5*k) for k in range(1,5)],
}


def report(out, rows, boundaries):
    lines=['# 自由选点：纯跟踪MPC继续升档','',
      '算法/控制器冻结，原初相，每条件一次；完整时间含返航。每条路线首次实测窗框碰撞后停止该路线。',
      '超时但未记录接触不是碰撞，继续下一档；无碰撞不自动等于完成。',
      '本批最多四轮（三路线最多12趟）；到达测试上限未碰撞，记录未观察到，不声称找到了临界值。','',
      '|幅度m|频率Hz|路线|完整时间|结果|首次窗框接触s|误差重规划|',
      '|---:|---:|---|---:|---|---:|---:|']
    for r in sorted(rows,key=lambda r:(r['trial']['amplitude'],r['trial']['frequency'])):
        t=r['trial'];duration=r.get('simulation_race_time_s')
        contacts=r.get('contacts_during_flight',[])
        first=min((x['time_s'] for x in contacts),default=None)
        lines.append('| '+' | '.join([str(t['amplitude']),str(t['frequency']),r['route'],
          f'{duration:.3f}' if duration is not None else '—',r.get('simulation_outcome','证据不足'),
          f'{first:.3f}' if first is not None else '—',str(r.get('tracking_error_events','—'))])+' |')
    lines+=['','## 路线状态','']
    for name,state in boundaries.items():lines.append(f'- {name}: {json.dumps(state,ensure_ascii=False)}')
    lines+=['','单次固定初相的首次观察，不是成功率、连续安全证明或单调碰撞阈值。无窗框接触不代表全环境无碰撞。']
    (out/'REPORT.md').write_text('\n'.join(lines)+'\n')


def main():
    p=argparse.ArgumentParser();p.add_argument('--outdir',type=Path,required=True)
    p.add_argument('--baseline',type=Path,required=True);a=p.parse_args()
    baseline=a.baseline.resolve();out=a.outdir.resolve();hashes=sources()
    old=json.loads((baseline/'protocol.json').read_text())['source_sha256']
    assert all(hashes[k]==v for k,v in old.items()), 'Baseline sources changed'
    original_rows=json.loads((baseline/'summary.json').read_text())
    base=json.loads((Path(original_rows[0]['directory'])/'result.json').read_text())
    assert base['config']['free_points']
    out.mkdir(parents=True)
    save(out/'protocol.json',dict(routes=ROUTES,phase=0.,repeat_count=1,method='ours',
         baseline=str(baseline),source_sha256=hashes,stop_each_route='first physical window-frame contact',
         max_rounds=4,noncollision_incomplete='retain and continue upward',
         metrics=['complete start-seven-gates-return time','physical window-frame contact']))
    for rel in hashes:
        dst=out/'source_snapshot'/rel;dst.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(ROOT/rel,dst)
    rows=[];boundaries={name:dict(state='pending') for name in ROUTES};report(out,rows,boundaries)
    for k in range(4):
        for route,cases in ROUTES.items():
            if boundaries[route]['state']=='observed_collision':continue
            assert sources()==hashes,'Frozen sources changed'
            f,amp=cases[k];trial=Trial('raise_'+route,'ours',f,amp,0.,0);path=out/trial.name
            save(out/'status.json',dict(state='running',route=route,case=[f,amp],completed=len(rows),boundaries=boundaries))
            print('START',trial.name,flush=True)
            row=evaluate(execute(trial,path,out/(trial.name+'.log'),100));row['route']=route
            if (path/'result.json').exists():
                raw=json.loads((path/'result.json').read_text())
                for key in ('config','mpc_config','parameters'):assert raw[key]==base[key],key
                row['tracking_error_events']=sum(e['action']=='trigger' and e['reason']=='tracking_error' for e in raw['planning_events'])
                row['contacts_during_flight']=[dict(log=c['raw_log'],time_s=c['first_contact_sim_time_s']-30)
                    for c in raw['contacts'] if c.get('first_contact_sim_time_s') is not None
                    and 30<=c['first_contact_sim_time_s']<=30+raw['last_time_s']]
            rows.append(row);save(out/'summary.json',rows)
            if row.get('contacts_during_flight'):
                boundaries[route]=dict(state='observed_collision',frequency=f,amplitude=amp,directory=str(path))
            else:
                boundaries[route]=dict(state='continue' if k<3 else 'no_collision_observed_to_test_cap',
                                      tested_frequency=f,tested_amplitude=amp,outcome=row.get('simulation_outcome'))
            save(out/'boundaries.json',boundaries);report(out,rows,boundaries)
            print('RESULT',trial.name,row.get('simulation_outcome'),row.get('simulation_race_time_s'),flush=True)
            if row.get('infrastructure_failure') or row.get('failure')=='missing_result' or (row.get('audit_error') and not row.get('contacts_during_flight')):
                save(out/'status.json',dict(state='evidence_or_infrastructure_failure',directory=str(path),boundaries=boundaries))
                raise RuntimeError(f'Unresolved evidence: {path}')
    save(out/'status.json',dict(state='complete',trials=len(rows),boundaries=boundaries))


if __name__=='__main__':main()
