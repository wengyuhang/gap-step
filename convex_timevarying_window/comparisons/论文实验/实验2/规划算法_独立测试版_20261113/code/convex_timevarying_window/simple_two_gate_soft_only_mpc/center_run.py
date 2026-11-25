"""Matched center ablation using the frozen pure-tracking free-point flights."""
import argparse
import json
import shutil
from pathlib import Path
from .run import sources, execute, evaluate, save, ROOT, Trial, digest


def check_pair(free, center):
    a, b = dict(free['config']), dict(center['config'])
    assert a.pop('free_points') is True and b.pop('free_points') is False
    assert a == b, 'Planner settings differ beyond free_points'
    for key in ('mpc_config', 'parameters', 'case'):
        assert free[key] == center[key], key
    assert all(c.get('controller') == 'trajectory_tracking_input_box_rti'
               for c in center['mpc_cycles'] if not c.get('failure'))


def report(out, free_rows, rows):
    lines=['# 纯跟踪MPC：自由选点与强制中心', '',
      '七个相同条件、相同初相，每方法每条件一次。自由数据引用已完成pure_tracking_v1批次，中心重新飞行。',
      '完整竞速时间包含七窗及返航；物理窗框接触与未完成保留，不按名义限制判失败。', '',
      '|幅度m|频率Hz|自由完整时间|中心完整时间|中心结果|中心误差重规划|',
      '|---:|---:|---:|---:|---|---:|']
    for f in free_rows:
        t=f['trial']; c=next((r for r in rows if r['trial']['frequency']==t['frequency']
                              and r['trial']['amplitude']==t['amplitude']),None)
        ft=f.get('simulation_race_time_s');ct=c.get('simulation_race_time_s') if c else None
        lines.append('| '+ ' | '.join([f"{t['amplitude']:g}",f"{t['frequency']:g}",
          f'{ft:.3f}s' if ft is not None else '未完成', f'{ct:.3f}s' if ct is not None else '—',
          c.get('simulation_outcome','未知') if c else '待运行',str(c.get('tracking_error_events','—')) if c else '—'])+' |')
    lines += ['', '单次结果不等于成功率或稳定优越性；1Hz/3m等幅度不是单调难度保证。',
              '不新增2Hz/2.5m和1.5Hz/3m两组，以保证本轮与已有七组逐项匹配。']
    (out/'REPORT.md').write_text('\n'.join(lines)+'\n')


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--free-results',type=Path,required=True)
    p.add_argument('--outdir',type=Path,required=True)
    a=p.parse_args();free_dir=a.free_results.resolve();out=a.outdir.resolve()
    free_rows=sorted(json.loads((free_dir/'summary.json').read_text()),
                     key=lambda r:(r['trial']['amplitude'],r['trial']['frequency']))
    assert len(free_rows)==7 and all(r['trial']['method']=='ours' for r in free_rows)
    assert json.loads((free_dir/'status.json').read_text())['state']=='complete'
    hashes=sources();old=json.loads((free_dir/'protocol.json').read_text())['source_sha256']
    # No flight/optimizer/controller change between the existing free and new center flights.
    assert all(hashes[k]==v for k,v in old.items()), 'Existing frozen sources changed'
    out.mkdir(parents=True)
    save(out/'protocol.json',dict(method='center',free_results=str(free_dir),phase=0.,repeats=1,
         cases=[r['trial'] for r in free_rows],source_sha256=hashes,
         identical_to_free_sources=True,only_algorithm_difference='free_points=False',
         primary_metrics=['complete seven-gates plus return time','physical frame contact']))
    for rel in hashes:
        dst=out/'source_snapshot'/rel;dst.parent.mkdir(parents=True,exist_ok=True)
        shutil.copyfile(ROOT/rel,dst)
    rows=[];report(out,free_rows,rows)
    for free in free_rows:
        assert sources()==hashes, 'Frozen source changed'
        t=free['trial'];trial=Trial('center_comparison','center',t['frequency'],t['amplitude'],t['phase'],0)
        path=out/trial.name
        save(out/'status.json',dict(state='running',case=[trial.frequency,trial.amplitude],completed=len(rows)))
        print('START',trial.name,flush=True)
        row=evaluate(execute(trial,path,out/(trial.name+'.log'),100))
        if (path/'result.json').exists():
            raw=json.loads((path/'result.json').read_text())
            free_path=Path(free['directory']);free_raw=json.loads((free_path/'result.json').read_text())
            check_pair(free_raw,raw)
            assert digest(path/'world.sdf')==digest(free_path/'world.sdf'), 'World differs'
            row.update(matched_config_and_world=True,tracking_error_events=sum(
                e['action']=='trigger' and e['reason']=='tracking_error' for e in raw['planning_events']))
        rows.append(row);save(out/'summary.json',rows);report(out,free_rows,rows)
        print('RESULT',trial.name,row.get('simulation_outcome'),row.get('simulation_race_time_s'),flush=True)
        if row.get('infrastructure_failure') or row.get('failure')=='missing_result' or (row.get('audit_error') and row.get('completed')):
            save(out/'status.json',dict(state='evidence_or_infrastructure_failure',directory=str(path)))
            raise RuntimeError(f'Unresolved evidence: {path}')
    save(out/'status.json',dict(state='complete',trials=len(rows)))


if __name__=='__main__':main()
