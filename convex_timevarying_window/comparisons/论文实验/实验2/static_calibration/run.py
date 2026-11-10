"""Freeze and execute six calibration flights, retaining all attempts."""
import argparse
import json
import os
import shutil
from pathlib import Path
from .scenario import ROOT,HERE,LAYOUTS,Case,manifest,prepare_assets,ASSETS
from ..run import execute
from ..run_four import sources as base_sources
from convex_timevarying_window.comparisons.single_gate_constant_velocity.run import save,digest

MODULE='convex_timevarying_window.comparisons.论文实验.实验2.static_calibration'
METHODS=('dynamic_togt_center','dynamic_togt')


def sources():
    result=base_sources()
    for p in ASSETS.rglob('*'):
        if p.is_file():result[str(p.relative_to(ROOT))]=digest(p)
    return result


def report(out,rows,final=False):
    lines=['# 历史七窗形状：三布局静态校准','',f'状态：{"完成" if final else "运行中"}，{len(rows)}/6。',
        '这是用户指定的布局校准，不是未调过的泛化评估。窗口形状、尺寸、TOGT内缩规则、原生x500目标、MPC/PX4均固定。',
        '全局七窗后返航，无在线规划；有效时间只填物理无碰撞且真值七窗整机退出、返航完成。','',
        '|布局|TOGT自由点|TOGT中心|','|---|---|---|']
    for layout in LAYOUTS:
        items=[]
        for m in ('dynamic_togt','dynamic_togt_center'):
            r=next((r for r in rows if r['case']['layout']==layout and r['variant']==m),{})
            items.append(f"{r['flight_time_s']:.3f}s，无碰撞" if r.get('passed') else r.get('verdict','待运行'))
        lines.append('|'+layout+'|'+'|'.join(items)+'|')
    lines+=['','## 原始证据','']
    for r in rows:
        p=Path(r['directory']);lines.append('- '+r['case']['layout']+' / '+r['variant']+'：['+r['verdict']+']('+os.path.relpath(p/'result.json',out)+')')
    (out/'REPORT.md').write_text('\n'.join(lines)+'\n')


def main():
    p=argparse.ArgumentParser();p.add_argument('--plans',type=Path,required=True);p.add_argument('--outdir',type=Path,required=True);a=p.parse_args();out=a.outdir.resolve();out.mkdir(parents=True,exist_ok=False)
    prepare_assets();frozen=sources();plans={};rows=[]
    for l in LAYOUTS:
        for m in METHODS:
            path=a.plans.resolve()/(l+'_'+m);pr=json.loads((path/'result.json').read_text())
            if not pr['found'] or not all(pr['safe_point_membership']):raise RuntimeError('calibration plan missing:'+str(path))
            if json.loads((path/'protocol.json').read_text())!=json.loads(json.dumps(manifest(Case(l,'static')))):raise RuntimeError('probe scene changed')
            plans[l+'_'+m]=dict(directory=str(path),result_sha256=digest(path/'result.json'),trajectory_sha256=digest(path/'trajectory.npz'))
    save(out/'protocol.json',dict(purpose='static layout calibration',methods=METHODS,cases=[manifest(Case(l,'static')) for l in LAYOUTS],total=6,
                                source_sha256=frozen,plans=plans,planner_backend='historical SciPy L-BFGS-B; original x500 native objective and shape maps',no_outcome_based_reflight=True))
    for rel in frozen:
        target=out/'source_snapshot'/rel;target.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(ROOT/rel,target)
    save(out/'summary.json',rows);report(out,rows)
    try:
        for layout in LAYOUTS:
            for method in METHODS:
                if sources()!=frozen:raise RuntimeError('frozen_source_changed')
                name=layout+'_'+method;plan=Path(plans[name]['directory'])
                for attempt in range(2):
                    path=out/(name if attempt==0 else name+'_facility_retry')
                    save(out/'status.json',dict(state='flying',pid=os.getpid(),completed=len(rows),total=6,layout=layout,method=method,directory=str(path)))
                    print('FLY',path.name,flush=True)
                    code=execute([MODULE+'.flight','--layout',layout,'--method',method,'--plan-dir',str(plan),'--outdir',str(path)],out/(path.name+'.log'),600)
                    raw=json.loads((path/'result.json').read_text()) if (path/'result.json').exists() else None
                    unflown=raw is not None and raw['last_time_s']==0 and not raw['solves'] and any(s in str(raw['failure']) for s in ('arm failed','no heartbeat','initial telemetry'))
                    if unflown and attempt==0:
                        save(path/'facility_retry_reason.json',dict(unflown=True,original_retained=True,failure=raw['failure']));continue
                    if code or not (path/'comparison_audit.json').exists():raise RuntimeError('flight_or_audit_error:'+name)
                    row=json.loads((path/'comparison_audit.json').read_text());row['directory']=str(path);break
                rows.append(row);save(out/'summary.json',rows);report(out,rows);print('RESULT',name,row['verdict'],row.get('flight_time_s'),flush=True)
                if not row['passed']:raise RuntimeError('calibration_did_not_pass:'+name)
            pair=[r for r in rows if r['case']['layout']==layout];raw=[json.loads((Path(r['directory'])/'result.json').read_text()) for r in pair]
            cfg=[dict(x['config']) for x in raw]
            for c in cfg:c.pop('free_points')
            equal=cfg[0]==cfg[1] and raw[0]['mpc_config']==raw[1]['mpc_config'] and raw[0]['parameters']==raw[1]['parameters'] and digest(Path(pair[0]['directory'])/'world.sdf')==digest(Path(pair[1]['directory'])/'world.sdf')
            save(out/('pair_'+layout+'.json'),dict(equal_except_free_points=equal))
            if not equal:raise RuntimeError('paired_configuration_mismatch')
        for v in plans.values():
            path=Path(v['directory']);assert digest(path/'result.json')==v['result_sha256'] and digest(path/'trajectory.npz')==v['trajectory_sha256']
        assert sources()==frozen;report(out,rows,True);save(out/'status.json',dict(state='complete',pid=os.getpid(),completed=6,total=6,sources_unchanged=True,plans_unchanged=True));print('COMPLETE',flush=True)
    except Exception as exc:
        save(out/'status.json',dict(state='stopped',pid=os.getpid(),completed=len(rows),total=6,error=repr(exc)));raise


if __name__=='__main__':main()
