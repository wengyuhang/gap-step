"""Whole-course factorial adapter; all four mathematical planners unchanged."""
import json
import shutil
import sys
from pathlib import Path
from . import scenario,layout
from ..separated_course import run as driver
from ..togt_course import scenario as old_scene
from ..trajectory_only.collision import evaluate

HERE=Path(__file__).resolve().parent
MODULE='convex_timevarying_window.comparisons.论文实验.实验2.basic_group.whole_course_difficulty.run'
PRIOR=HERE.parents[1]/'results/togt_layout_mixed_v1_20261106'


def sources():
    result=driver.common.sources()
    for directory in (HERE,HERE.parent/'togt_course',HERE.parent/'separated_course'):
        for p in directory.rglob('*'):
            if p.is_file() and p.suffix in ('.py','.md','.json'):
                result[str(p.relative_to(scenario.ROOT))]=driver.digest(p)
    return result


def static_reuse_evidence(method):
    frozen=driver.read(PRIOR/'protocol.json')['source_sha256'];now=sources()
    changed=[p for p,h in frozen.items() if now.get(p)!=h]
    if changed:raise RuntimeError('retained static source changed: '+repr(changed))
    source=PRIOR/'plans'/('static_'+method)
    previous=driver.read(source/'protocol.json')
    if 'scene' in previous:previous=previous['scene']
    expected=old_scene.manifest(old_scene.Case())
    assert previous==json.loads(json.dumps(expected))
    new=scenario.manifest(scenario.Case())
    # All physical/state inputs are identical; descriptive experiment metadata differs.
    for key in ('start','goal','windows','reference_polyline_length_m','inter_gate_distances_m'):
        if key=='windows':
            assert all(all(w[k]==new['windows'][i][k] for k in w) for i,w in enumerate(expected[key]))
        else:assert new[key]==expected[key]
    original_row=next(r for r in driver.read(PRIOR/'summary.json')
                      if r['level']=='static' and r['variant']==method)
    assert original_row['passed'] and driver.digest(source/'result.json')==original_row['plan_result_sha256']
    return dict(source_directory=str(source),prior=str(PRIOR),
        result_sha256=driver.digest(source/'result.json'),
        protocol_sha256=driver.digest(source/'protocol.json'),
        prior_frozen_source_count=len(frozen),all_prior_sources_unchanged=True,
        static_geometry_state_and_motion_equal=True,original_candidates_unchanged=True,
        cost_origin='prior original build/solve measurement; independent assessment repeated now')


def plan(level,method,out):
    if level=='static':
        evidence=static_reuse_evidence(method)
        shutil.copytree(evidence['source_directory'],out)
        driver.save(out/'reuse.json',evidence)
        print('RETAIN_STATIC',method,evidence['result_sha256'],flush=True)
        return
    if method in ('ours','center'):
        from ... import plan as entry,planner
        entry.Case=scenario.Case;entry.build_case=scenario.build_case
        entry.manifest=scenario.manifest;planner.pose=scenario.pose
        sys.argv=[entry.__name__,'--layout','togt_whole','--level',level,'--method',method,'--outdir',str(out)]
        entry.main()
    else:
        from ...static_calibration import planner as entry
        from ..plan import nominal_audit
        entry.build_case=scenario.build_case;entry.manifest=scenario.manifest;entry.nominal_audit=nominal_audit
        entry.solve(scenario.Case(level=level),method,out)


def assess(path,level,method):
    raw=driver.read(path/'result.json');cs=raw['candidates']
    good=[c for c in cs if c['found' if method in ('ours','center') else 'success']]
    assert len(cs)==4 and bool(good)==raw['found'] and raw['method']==method
    if level=='static':
        evidence=static_reuse_evidence(method)
        assert driver.read(path/'reuse.json')==evidence
        assert driver.digest(path/'result.json')==evidence['result_sha256']
        assert driver.digest(path/'protocol.json')==evidence['protocol_sha256']
        assert raw['case']==dict(layout='togt',level='static')
    else:assert raw['case']==dict(layout='togt_whole',level=level)
    rho,scale=(0.,1.) if level=='static' else scenario.CONDITIONS[level]
    row=dict(level=level,variant=method,amplitude_ratio=rho,frequency_factor=scale,
        profile=[list(x) for x in scenario.profile(level)],plan_directory=str(path),
        plan_result_sha256=driver.digest(path/'result.json'),build_s=raw['build_s'],
        solve_s=raw['solve_s'],gazebo_run=False,reused_plan=level=='static',
        cost_origin='prior_original_measurement' if level=='static' else 'new_measurement')
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
    lines=['# 实验二：七窗全局规划，幅度×频率矩阵','',f'状态：{state}，{len(rows)}/40项。','',
        '保留TOGT交错/上下叠窗布局、现有小窗/x500；四平移三摆动，每个动态条件七窗全部运动。',
        '幅度按每窗水平开口宽度归一化，频率按各窗基准同比例提升。初相均0，各格各方法一次。',
        '时间是完整七窗后返航零VAJ的参考时长；不是Gazebo实飞。碰撞候选的时长不计有效完赛。','',
        '|幅度/窗宽|频率倍数（实际Hz范围）|本文自由点|本文中心|动态TOGT自由点|动态TOGT中心|',
        '|---|---|---:|---:|---:|---:|']
    data=[]
    for level in scenario.LEVELS:
        manifest=scenario.manifest(scenario.Case(level=level));by={r['variant']:r for r in rows if r['level']==level}
        item=dict(level=level,amplitude_ratio=manifest['amplitude_to_horizontal_opening_width'],
                  frequency_factor=manifest['frequency_factor'],amplitude_range_m=manifest['amplitude_range_m'],
                  frequency_range_hz=manifest['frequency_range_hz'],peak_center_speed_range_mps=manifest['peak_center_speed_range_mps'])
        cells=[]
        for m in driver.METHODS:
            r=by.get(m,{});v=r.get('verdict','pending');t=r.get('full_trajectory_time_s')
            cells.append((f'{t:.3f}s，' if t is not None else '')+driver.LABELS[v])
            item[m]=dict(verdict=v,full_trajectory_time_s=t)
        for a,b,key in [('ours','center','ours_saving_percent'),('dynamic_togt','dynamic_togt_center','togt_saving_percent'),('ours','dynamic_togt','ours_vs_togt_saving_percent')]:
            x,y=by.get(a,{}),by.get(b,{})
            item[key]=100*(y['full_trajectory_time_s']-x['full_trajectory_time_s'])/y['full_trajectory_time_s'] if x.get('passed') and y.get('passed') else None
        data.append(item)
        rho=manifest['amplitude_to_horizontal_opening_width'];s=manifest['frequency_factor'];lo,hi=manifest['frequency_range_hz']
        label='静态|—' if level=='static' else f'{rho:g}|{s:g}（{lo:g}–{hi:g}）'
        lines.append('|'+label+'|'+'|'.join(cells)+'|')
    lines+=['','## 逐条件运动强度','', '|条件|实际幅度范围 m|中心峰值速度范围 m/s|','|---|---:|---:|']
    for d in data:
        a,b=d['amplitude_range_m'];v,w=d['peak_center_speed_range_mps']
        lines.append(f"|{d['level']}|{a:.3f}–{b:.3f}|{v:.3f}–{w:.3f}|")
    lines+=['','## 计算与原始证据','', '|条件|方法|建图 s|四初值求解 s|检查 s|规划来源|非相邻风险|记录|', '|---|---|---:|---:|---:|---|---|---|']
    for r in sorted(rows,key=lambda r:(scenario.LEVELS.index(r['level']),driver.METHODS.index(r['variant']))):
        foreign='—' if r['scene_assumption_passed'] is None else '无' if r['scene_assumption_passed'] else '发现，停批'
        origin='静态原测量继承' if r['reused_plan'] else '本轮新规划'
        lines.append(f"|{r['level']}|{driver.NAMES[r['variant']]}|{r['build_s']:.3f}|{r['solve_s']:.3f}|{r['check_s']:.3f}|{origin}|{foreign}|[记录](assessments/{r['level']}_{r['variant']}.json)|")
    lines+=['','![赛道布局](course_layout.png)','',
        '完整参数/源码哈希见protocol.json；静态原始规划与四候选不改，新独立检查，详见reuse.json。动态条件全部新算。',
        '不按成绩挑条件或重跑择优；全部失败保留，仅双方名义完成才比较时间。',
        '运动强度不等于失败必然单调；本实验不估计随机成功率。几何筛查不替代实际全轨迹检查。',
        '当前为浮点球/STL规划评估，不宣称闭环/实体接触或严格区间证书。']
    (out/'REPORT.md').write_text('\n'.join(lines)+'\n')
    driver.save(out/'analysis.json',dict(state=state,completed=len(rows),total=40,conditions=data,
        foreign_risk_records=sum(r.get('scene_assumption_passed') is False for r in rows),
        by_method={m:{v:sum(r['variant']==m and r['verdict']==v for r in rows) for v in driver.LABELS if v!='pending'} for m in driver.METHODS}))


def configure():
    driver.HERE=HERE;driver.MODULE=MODULE;driver.scenario=scenario;driver.layout=layout
    driver.sources=sources;driver.plan=plan;driver.assess=assess;driver.report=report


if __name__=='__main__':configure();driver.main()
