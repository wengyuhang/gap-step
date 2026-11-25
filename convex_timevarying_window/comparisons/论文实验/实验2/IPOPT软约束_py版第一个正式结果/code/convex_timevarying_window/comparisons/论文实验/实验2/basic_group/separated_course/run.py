"""Preflight-gated full nominal experiment; no simulator or algorithm changes."""
import argparse
from dataclasses import asdict
import fcntl
import os
from pathlib import Path
import shutil
import subprocess
import sys
import numpy as np
import trimesh
from . import scenario,layout
from ..trajectory_only import run as common
from ..trajectory_only.collision import evaluate,CheckConfig

HERE=Path(__file__).resolve().parent;ROOT=scenario.ROOT
MODULE='convex_timevarying_window.comparisons.论文实验.实验2.basic_group.separated_course.run'
METHODS=common.METHODS;NAMES=common.NAMES;LABELS=common.LABELS
save=common.save;read=common.read;digest=common.digest


def sources():
    result=common.sources()
    for p in HERE.rglob('*'):
        if p.is_file() and p.suffix in ('.py','.md'):result[str(p.relative_to(ROOT))]=digest(p)
    return result


def plan(level,method,out):
    case=scenario.Case(level=level)
    if method in ('ours','center'):
        from ... import plan as entry,planner
        entry.Case=scenario.Case;entry.build_case=scenario.build_case;entry.manifest=scenario.manifest
        planner.pose=scenario.pose
        sys.argv=[entry.__name__,'--layout','separated','--level',level,'--method',method,'--outdir',str(out)]
        entry.main()
    else:
        from ...static_calibration import planner as entry
        entry.build_case=scenario.build_case;entry.manifest=scenario.manifest
        # Kept as non-decisional historical metadata; correct moving-frame query.
        from ..plan import nominal_audit
        entry.nominal_audit=nominal_audit
        entry.solve(case,method,out)


def assess(path,level,method):
    raw=read(path/'result.json');cs=raw['candidates']
    good=[c for c in cs if c['found' if method in ('ours','center') else 'success']]
    assert len(cs)==4 and bool(good)==raw['found']
    assert raw['case']==dict(layout='separated',level=level) and raw['method']==method
    row=dict(level=level,variant=method,profile=[list(x) for x in scenario.profile(level)],
             plan_directory=str(path),plan_result_sha256=digest(path/'result.json'),
             build_s=raw['build_s'],solve_s=raw['solve_s'],gazebo_run=False)
    if not raw['found']:
        return dict(row,passed=False,verdict='planning_not_found',collision_detected=None,
                    full_trajectory_time_s=None,check_s=0.,scene_assumption_passed=None)
    metric='nominal_time_s' if method in ('ours','center') else 'objective'
    assert abs(cs[raw['selected_seed']][metric]-min(c[metric] for c in good))<1e-8
    tr,_=scenario.build_case(scenario.Case(level=level))
    result=evaluate(raw['coefficients'],raw['durations'],tr,scenario.ASSETS/'meshes')
    assert abs(result['full_trajectory_time_s']-raw['nominal_time_s'])<1e-8
    return dict(row,**result,**layout.foreign_diagnostics(result))


def report(out,rows,state='running'):
    lines=['# 实验二：独立穿窗区间的混合动态七窗', '',
        f'状态：{state}，{len(rows)}/40条。', '',
        '固定非自交折线+外侧返航；四平移/三摆动。仅W3与W6交替升档，其余五窗维持各自低速运动，初相均零。',
        '完整参考时间含七窗返航至零VAJ，不是实飞时间。名义球/STL碰撞；全部窗口全轨迹检查，非目标窗口风险不忽略。', '',
        '|档位|W3平移 Hz/m|W6摆动 Hz/m|本文自由点|本文中心|动态TOGT自由点|动态TOGT中心|',
        '|---|---|---|---:|---:|---:|---:|']
    data=[]
    for level in scenario.LEVELS:
        by={r['variant']:r for r in rows if r['level']==level};profile=scenario.profile(level);cells=[]
        item=dict(level=level,profile=[list(x) for x in profile])
        for m in METHODS:
            r=by.get(m,{});v=r.get('verdict','pending');t=r.get('full_trajectory_time_s')
            cells.append((f'{t:.3f}s，' if t is not None else '')+LABELS[v])
            item[m]=dict(verdict=v,full_trajectory_time_s=t)
        for free,center,key in [('ours','center','ours_saving_percent'),('dynamic_togt','dynamic_togt_center','togt_saving_percent'),('ours','dynamic_togt','ours_vs_togt_saving_percent')]:
            x,y=by.get(free,{}),by.get(center,{})
            item[key]=100*(y['full_trajectory_time_s']-x['full_trajectory_time_s'])/y['full_trajectory_time_s'] if x.get('passed') and y.get('passed') else None
        data.append(item)
        fmt=lambda x:f'{x[0]:g}/{x[1]:g}'
        lines.append('|'+level+'|'+fmt(profile[2])+'|'+fmt(profile[5])+'|'+'|'.join(cells)+'|')
    lines+=['','## 计算与轨迹证据','', '|档位|方法|建图 s|四初值求解 s|独立检查 s|非相邻窗风险|原始结果|', '|---|---|---:|---:|---:|---|---|']
    for r in sorted(rows,key=lambda r:(scenario.LEVELS.index(r['level']),METHODS.index(r['variant']))):
        foreign='—' if r['scene_assumption_passed'] is None else '无' if r['scene_assumption_passed'] else '发现，需核查赛道假设'
        link=f"assessments/{r['level']}_{r['variant']}.json"
        lines.append(f"|{r['level']}|{NAMES[r['variant']]}|{r['build_s']:.3f}|{r['solve_s']:.3f}|{r['check_s']:.3f}|{foreign}|[记录]({link})|")
    lines+=['','几何图/核验：[布局](course_layout.png)、[运动包络](geometry_audit.json)、[正式实验准入](preflight_gate.json)。',
        '几何走廊分离只覆盖声明的航路走廊，不证明任意绕行轨迹安全；所有实际参考仍独立检查。',
        '未找到、检查待定、任务未完成、名义碰撞分别保留；只有双方完整名义通过才比较时间。',
        '静态四方法与最高动态TOGT预检通过“无非相邻窗风险”后才进行其余正式条件；预检规划原样复用，不重新求解择优。',
        '旧紧凑赛道与旧实飞结果不混入本批，规划目标/约束/四时间初值/预算/选解规则未改。']
    (out/'REPORT.md').write_text('\n'.join(lines)+'\n')
    save(out/'analysis.json',dict(state=state,completed=len(rows),total=40,conditions=data,
        foreign_risk_records=sum(r.get('scene_assumption_passed') is False for r in rows),
        by_method={m:{v:sum(r['variant']==m and r['verdict']==v for r in rows) for v in LABELS if v!='pending'} for m in METHODS}))


def validate(out):
    frozen=read(out/'protocol.json')['source_sha256'];assert sources()==frozen
    assert read(out/'preflight_gate.json')['passed']
    assert read(out/'geometry_audit.json')['passed'] and read(out/'motion_audit.json')['passed']
    rows=read(out/'summary.json');assert len(rows)==40 and len({(r['level'],r['variant']) for r in rows})==40
    checks=[]
    for r in rows:
        path=Path(r['plan_directory']);assert digest(path/'result.json')==r['plan_result_sha256']
        again=assess(path,r['level'],r['variant'])
        assert again['verdict']==r['verdict'] and again['full_trajectory_time_s']==r['full_trajectory_time_s']
        assert r['scene_assumption_passed'] is not False
        for check in r.get('checks',[]):
            if check['outcome']!='collision':continue
            mesh=trimesh.load_mesh(scenario.ASSETS/'meshes'/f"gate_{check['gate']}_sleeve.stl")
            distance=trimesh.proximity.closest_point_naive(mesh,np.array(check['witness']['local_position'])[None])[1][0]
            assert distance-CheckConfig().radius < -CheckConfig().numerical_guard_m
        checks.append(dict(level=r['level'],method=r['variant'],verdict=r['verdict'],plan_sha256=r['plan_result_sha256']))
    save(out/'validation.json',dict(passed=True,records=40,frozen_sources_unchanged=True,
        all_assessments_recomputed=True,foreign_collision_records=0,all_collision_witnesses_full_triangles_verified=True,checks=checks))


def run(out):
    with open('/tmp/convex_exp2_trajectory_only.lock','a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        out.mkdir(parents=True,exist_ok=False);(out/'plans').mkdir();(out/'assessments').mkdir()
        frozen=sources();rows=[]
        save(out/'protocol.json',dict(protocol=(HERE/'PROTOCOL.md').read_text(),source_sha256=frozen,
            cases=[scenario.manifest(scenario.Case(level=l)) for l in scenario.LEVELS],methods=METHODS,total=40,
            checker=asdict(CheckConfig()),seconds_per_seed=180.,gazebo_run=False,algorithm_changed=False))
        prior=HERE.parents[1]/'results/basic_trajectory_f0p5_to3_a2_v1_20261106/source_snapshot'
        for rel,h in frozen.items():
            target=out/'source_snapshot'/rel;target.parent.mkdir(parents=True,exist_ok=True);old=prior/rel
            if old.is_file() and digest(old)==h:os.link(old,target)
            else:shutil.copyfile(ROOT/rel,target)
        save(out/'summary.json',rows);report(out,rows,'preflight')
        def one(level,method,stage):
            if sources()!=frozen:raise RuntimeError('frozen_source_changed')
            existing=next((r for r in rows if (r['level'],r['variant'])==(level,method)),None)
            if existing:return existing
            path=out/'plans'/f'{level}_{method}'
            save(out/'status.json',dict(state=stage,pid=os.getpid(),completed=len(rows),total=40,level=level,method=method,directory=str(path)))
            print('PLAN',stage,level,method,flush=True)
            with (out/f'{level}_{method}_plan.log').open('w') as log:
                subprocess.run([sys.executable,'-u','-m',MODULE,'plan','--level',level,'--method',method,'--outdir',str(path)],
                               cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=1000)
            row=assess(path,level,method);save(out/'assessments'/f'{level}_{method}.json',row)
            rows.append(row);save(out/'summary.json',rows);report(out,rows,stage)
            print('RESULT',level,method,row['verdict'],row['full_trajectory_time_s'],'foreign_clear',row['scene_assumption_passed'],flush=True)
            if row['scene_assumption_passed'] is False:raise RuntimeError('nonincident_frame_risk_stop_without_ignoring_contact')
            return row
        try:
            save(out/'status.json',dict(state='geometry_preflight',pid=os.getpid(),completed=0,total=40))
            geometry=layout.audit();save(out/'geometry_audit.json',geometry)
            if not geometry['passed']:raise RuntimeError('layout_envelopes_not_separated')
            motion=layout.motion_audit();save(out/'motion_audit.json',motion);layout.figure(out,geometry)
            static=[one('static',m,'static_preflight') for m in METHODS]
            if not all(r['passed'] for r in static):raise RuntimeError('static_four_method_preflight_did_not_pass')
            dynamic=[one(l,m,'dynamic_preflight') for l in ('mixed_base',scenario.LEVELS[-1]) for m in ('dynamic_togt','dynamic_togt_center')]
            if not all(r['scene_assumption_passed'] is True for r in dynamic):raise RuntimeError('dynamic_preflight_missing_or_foreign_risk')
            save(out/'preflight_gate.json',dict(passed=True,geometry_passed=True,motion_passed=True,
                static_four_methods_nominal_safe=True,mild_and_max_dynamic_togt_nonincident_clear=True,
                optimization_based_safety_proof=False,all_references_still_fully_checked=True))
            print('PREFLIGHT_PASSED_START_FULL_MATRIX',flush=True)
            for level in scenario.LEVELS:
                for method in METHODS:one(level,method,'formal_planning')
            save(out/'status.json',dict(state='validating',pid=os.getpid(),completed=len(rows),total=40))
            validate(out);report(out,rows,'complete')
            save(out/'status.json',dict(state='complete',pid=os.getpid(),completed=40,total=40,validation_passed=True))
            print('COMPLETE',40,flush=True)
        except BaseException as exc:
            report(out,rows,'stopped')
            save(out/'status.json',dict(state='stopped',pid=os.getpid(),completed=len(rows),total=40,error=repr(exc)))
            raise


def main():
    p=argparse.ArgumentParser();p.add_argument('action',choices=('run','plan','validate'))
    p.add_argument('--outdir',required=True,type=Path);p.add_argument('--level',choices=scenario.LEVELS);p.add_argument('--method',choices=METHODS)
    a=p.parse_args()
    if a.action=='run':run(a.outdir.resolve())
    elif a.action=='plan':plan(a.level,a.method,a.outdir.resolve())
    else:validate(a.outdir.resolve())


if __name__=='__main__':main()
