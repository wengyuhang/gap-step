"""Single coordinator: freeze sources, retain old plans, assess, ascend frequency."""
import argparse
from dataclasses import asdict
import fcntl
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time
import numpy as np
import trimesh
from .. import scenario
from ..run import sources as baseline_sources
from ..report import METHODS, NAMES
from .collision import evaluate, CheckConfig

HERE = Path(__file__).resolve().parent
ROOT = scenario.ROOT
RESULTS = HERE.parents[1]/'results'
MODULE = 'convex_timevarying_window.comparisons.论文实验.实验2.basic_group.trajectory_only.run'
GRID = [('static', .5, 0.), ('f0p5_a2', .5, 2.), ('medium', 1., 2.),
        ('f1p5_a2', 1.5, 2.), ('f2_a2', 2., 2.), ('f2p5_a2', 2.5, 2.), ('f3_a2', 3., 2.)]
LABELS = dict(nominal_safe_complete='名义无碰撞', nominal_collision='名义碰撞',
              nominal_unresolved='检查待定', nominal_task_invalid='名义任务未完成',
              planning_not_found='未找到合格规划', pending='待完成')


def read(path): return json.loads(Path(path).read_text())
def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        while block := f.read(4*1024**2): h.update(block)
    return h.hexdigest()
def save(path, value):
    path = Path(path); temporary = path.with_name(path.name+'.new')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2)+'\n')
    temporary.replace(path)
def canonical(value): return json.loads(json.dumps(value))


def register():
    for key, f, a in GRID:
        if key in scenario.LEVELS and scenario.LEVELS[key] != (f, a):
            raise RuntimeError('existing level changed: '+key)
        scenario.LEVELS[key] = (f, a)


def sources():
    result = baseline_sources()
    for path in HERE.rglob('*'):
        if path.is_file() and path.suffix in ('.py', '.md'):
            result[str(path.relative_to(ROOT))] = digest(path)
    return result


def retained_plans(priors, frozen):
    result = {}; evidence = []
    for prior in priors:
        protocol = read(prior/'protocol.json')
        changed = [p for p, h in protocol['source_sha256'].items() if frozen.get(p) != h]
        if changed: raise RuntimeError('retained baseline source changed: '+repr(changed))
        for row in read(prior/'summary.json'):
            level = row['case']['level']; method = row['variant']
            if level not in {r[0] for r in GRID} or method not in METHODS: continue
            path = Path(row['directory']); raw = read(path/'result.json')
            if 'global_plan_directory' in raw: path = Path(raw['global_plan_directory'])
            raw = read(path/'result.json'); scene = read(path/'protocol.json')
            if 'scene' in scene: scene = scene['scene']
            expected = canonical(scenario.manifest(scenario.Case('original', level)))
            if scene != expected: raise RuntimeError('retained scene mismatch: '+str(path))
            if raw['method'] != method: raise RuntimeError('retained method mismatch')
            key = (level, method)
            if key in result: raise RuntimeError('duplicate retained condition')
            result[key] = path
            evidence.append(dict(level=level, method=method, directory=str(path),
                                 result_sha256=digest(path/'result.json'), prior=str(prior),
                                 reason='same frozen planner, scene and four-start selection; old flight verdict is NOT reused'))
    return result, evidence


def assess(path, level, method):
    raw = read(path/'result.json'); candidates = raw['candidates']
    qualified = [c for c in candidates if c['found' if method in ('ours', 'center') else 'success']]
    assert len(candidates) == 4 and bool(qualified) == raw['found']
    row = dict(level=level, variant=method, frequency_hz=scenario.LEVELS[level][0],
               amplitude_m=scenario.LEVELS[level][1], plan_directory=str(path),
               plan_result_sha256=digest(path/'result.json'), build_s=raw['build_s'],
               solve_s=raw['solve_s'], gazebo_run=False)
    if not raw['found']:
        return dict(row, passed=False, verdict='planning_not_found', collision_detected=None,
                    full_trajectory_time_s=None, check_s=0.)
    metric = 'nominal_time_s' if method in ('ours', 'center') else 'objective'
    chosen = candidates[raw['selected_seed']]
    assert abs(chosen[metric]-min(c[metric] for c in qualified)) < 1e-8
    track, _ = scenario.build_case(scenario.Case('original', level))
    result = evaluate(raw['coefficients'], raw['durations'], track, scenario.ASSETS/'meshes')
    assert abs(result['full_trajectory_time_s']-raw['nominal_time_s']) < 1e-8
    return dict(row, **result)


def report(out, rows, final=False):
    lines = ['# 实验二基础组：纯规划轨迹对比', '',
             f"状态：{'完成' if final else '进行中'}，{len(rows)}/{len(GRID)*4}项。", '',
             '时间为七窗后返航至零PVAJ终态的完整参考时长，不是Gazebo实飞时间。',
             '碰撞模型：x500外接球与原实体STL窗框/地面；无额外评估裕量。每段检查所有窗口，含返航。',
             '已按固定判据自适应覆盖连续时间；普通浮点数值证据，不是跟踪/实体接触或严格区间证书。', '',
             '|振幅|频率|本文自由点|本文中心|动态TOGT自由点|动态TOGT中心|', '|---|---|---:|---:|---:|---:|']
    comparisons = []
    for level, f, a in GRID:
        pair = {r['variant']:r for r in rows if r['level'] == level}; cells = []
        item = dict(level=level, frequency_hz=f, amplitude_m=a)
        for m in METHODS:
            r = pair.get(m, {}); verdict = r.get('verdict', 'pending'); t = r.get('full_trajectory_time_s')
            cells.append((f'{t:.3f} s，' if t is not None else '')+LABELS[verdict])
            item[m] = dict(verdict=verdict, full_trajectory_time_s=t)
        for free, center, key in [('ours','center','ours_saving_percent'), ('dynamic_togt','dynamic_togt_center','togt_saving_percent'), ('ours','dynamic_togt','ours_vs_togt_saving_percent')]:
            x, y = pair.get(free, {}), pair.get(center, {})
            item[key] = 100*(y['full_trajectory_time_s']-x['full_trajectory_time_s'])/y['full_trajectory_time_s'] if x.get('passed') and y.get('passed') else None
        comparisons.append(item)
        lines.append('|'+('静态|—' if a == 0 else f'{a:g} m|{f:g} Hz')+'|'+'|'.join(cells)+'|')
    lines += ['', '## 计算耗时和原始记录', '', '|条件|方法|建图 s|四初值求解 s|独立评估 s|规划来源|原始评估|', '|---|---|---:|---:|---:|---|---|']
    for r in sorted(rows, key=lambda r:([g[0] for g in GRID].index(r['level']), METHODS.index(r['variant']))):
        label = '静态' if r['level'] == 'static' else f"{r['amplitude_m']:g}m/{r['frequency_hz']:g}Hz"
        link = os.path.relpath(out/'assessments'/f"{r['level']}_{r['variant']}.json", out)
        lines.append(f"|{label}|{NAMES[r['variant']]}|{r['build_s']:.3f}|{r['solve_s']:.3f}|{r['check_s']:.3f}|{'复用冻结规划' if r['reused_plan'] else '新规划'}|[记录]({link})|")
    lines += ['', '未找到规划≠碰撞；检查待定≠通过。碰撞轨迹的时长保留展示，但不计有效时间优势。',
              '四方法保持旧目标/约束/选解规则。复用规划原求解耗时照录，不作为本轮新的计时重复实验。',
              '历史实飞数据不覆盖；不启动Gazebo/PX4，不操作容器。协议见protocol.json及源PROTOCOL.md。']
    (out/'REPORT.md').write_text('\n'.join(lines)+'\n')
    save(out/'analysis.json', dict(final=final, completed=len(rows), total=len(GRID)*4, conditions=comparisons,
        by_method={m:{v:sum(r['variant']==m and r['verdict']==v for r in rows) for v in LABELS if v!='pending'} for m in METHODS}))


def validate(out):
    protocol = read(out/'protocol.json'); rows = read(out/'summary.json')
    assert sources() == protocol['source_sha256']
    assert len(rows) == len(GRID)*4 and len({(r['level'],r['variant']) for r in rows}) == len(rows)
    audits = []
    for r in rows:
        path = Path(r['plan_directory']); assert digest(path/'result.json') == r['plan_result_sha256']
        again = assess(path, r['level'], r['variant'])
        assert again['verdict'] == r['verdict'] and again['full_trajectory_time_s'] == r['full_trajectory_time_s']
        assert r['gazebo_run'] is False
        # Independently verify every claimed frame-contact witness against ALL
        # triangles, rather than the accelerated nearest-neighbour candidate set.
        for check in r.get('checks', []):
            if check['outcome'] != 'collision': continue
            mesh = trimesh.load_mesh(scenario.ASSETS/'meshes'/f"gate_{check['gate']}_sleeve.stl")
            p = np.array(check['witness']['local_position'])[None]
            clearance = float(trimesh.proximity.closest_point_naive(mesh,p)[1][0])-CheckConfig().radius
            assert clearance < -CheckConfig().numerical_guard_m
        audits.append(dict(level=r['level'], method=r['variant'], verdict=r['verdict'], plan_sha256=r['plan_result_sha256']))
    save(out/'validation.json', dict(passed=True, records=len(rows), frozen_sources_unchanged=True,
                                     every_nominal_assessment_recomputed=True, collision_witnesses_all_triangles_verified=True, checks=audits))


def run(out, priors):
    with open('/tmp/convex_exp2_trajectory_only.lock', 'a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX|fcntl.LOCK_NB)
        out.mkdir(parents=True, exist_ok=False)
        (out/'plans').mkdir(); (out/'assessments').mkdir()
        frozen = sources(); retained, evidence = retained_plans(priors, frozen)
        save(out/'reuse_audit.json', evidence)
        save(out/'protocol.json', dict(protocol=(HERE/'PROTOCOL.md').read_text(), source_sha256=frozen,
             cases=[canonical(scenario.manifest(scenario.Case('original',k))) for k,_,_ in GRID],
             methods=METHODS, total=len(GRID)*4, initial_phase=0., trials=1, gazebo_run=False,
             checker=asdict(CheckConfig()), selection_unchanged=True, seconds_per_seed=180.))
        for rel, h in frozen.items():
            target = out/'source_snapshot'/rel; target.parent.mkdir(parents=True, exist_ok=True)
            archived = next((p/'source_snapshot'/rel for p in priors if (p/'source_snapshot'/rel).is_file()), None)
            if archived is not None and digest(archived) == h: os.link(archived, target)
            else: shutil.copyfile(ROOT/rel, target)
        rows=[]; save(out/'summary.json', rows); report(out, rows)
        try:
            for level, _, _ in GRID:
                for method in METHODS:
                    if sources() != frozen: raise RuntimeError('frozen_source_changed')
                    reused = (level,method) in retained
                    path = retained.get((level,method), out/'plans'/f'{level}_{method}')
                    if not reused:
                        save(out/'status.json', dict(state='planning',pid=os.getpid(),completed=len(rows),total=len(GRID)*4,
                                                    level=level,method=method,directory=str(path)))
                        print('PLAN',level,method,flush=True)
                        command=[sys.executable,'-u','-m',MODULE,'plan','--level',level,'--method',method,'--outdir',str(path)]
                        with (out/f'{level}_{method}_plan.log').open('w') as log:
                            subprocess.run(command,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=1000)
                    save(out/'status.json',dict(state='assessing',pid=os.getpid(),completed=len(rows),total=len(GRID)*4,level=level,method=method))
                    row=assess(path,level,method);row['reused_plan']=reused
                    save(out/'assessments'/f'{level}_{method}.json',row)
                    rows.append(row);save(out/'summary.json',rows);report(out,rows)
                    print('RESULT',level,method,row['verdict'],row['full_trajectory_time_s'],flush=True)
            save(out/'status.json',dict(state='validating',pid=os.getpid(),completed=len(rows),total=len(GRID)*4))
            validate(out);report(out,rows,True)
            save(out/'status.json',dict(state='complete',pid=os.getpid(),completed=len(rows),total=len(GRID)*4,validation_passed=True))
            print('COMPLETE',len(rows),flush=True)
        except Exception as exc:
            save(out/'status.json',dict(state='stopped',pid=os.getpid(),completed=len(rows),total=len(GRID)*4,error=repr(exc)))
            raise


def main():
    register()
    p=argparse.ArgumentParser();p.add_argument('action',choices=['run','plan','validate']);p.add_argument('--outdir',type=Path,required=True)
    p.add_argument('--level',choices=[g[0] for g in GRID]);p.add_argument('--method',choices=METHODS)
    p.add_argument('--reuse',type=Path,nargs='*',default=[RESULTS/'basic_group_full_v1_20261106',RESULTS/'basic_group_f2_a2_v1_20261106'])
    a=p.parse_args()
    if a.action=='run': run(a.outdir.resolve(),[x.resolve() for x in a.reuse]);return
    if a.action=='validate': validate(a.outdir.resolve());return
    from .. import plan
    sys.argv=[plan.__name__,'--layout','original','--level',a.level,'--method',a.method,'--outdir',str(a.outdir)]
    plan.main()


if __name__=='__main__':
    main()
