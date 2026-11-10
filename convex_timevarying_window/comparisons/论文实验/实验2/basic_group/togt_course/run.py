"""TOGT-layout scene adapter to the unchanged, preflight-gated experiment."""
import sys
from pathlib import Path
from . import scenario,layout
from ..separated_course import run as driver
from ..trajectory_only.collision import evaluate,CheckConfig

HERE=Path(__file__).resolve().parent
MODULE='convex_timevarying_window.comparisons.论文实验.实验2.basic_group.togt_course.run'
_sources=driver.sources


def sources():
    result=_sources();p=HERE/'upstream.json';result[str(p.relative_to(scenario.ROOT))]=driver.digest(p)
    return result


def plan(level,method,out):
    if method in ('ours','center'):
        from ... import plan as entry,planner
        entry.Case=scenario.Case;entry.build_case=scenario.build_case;entry.manifest=scenario.manifest;planner.pose=scenario.pose
        sys.argv=[entry.__name__,'--layout','togt','--level',level,'--method',method,'--outdir',str(out)];entry.main()
    else:
        from ...static_calibration import planner as entry
        from ..plan import nominal_audit
        entry.build_case=scenario.build_case;entry.manifest=scenario.manifest;entry.nominal_audit=nominal_audit
        entry.solve(scenario.Case(level=level),method,out)


def assess(path,level,method):
    raw=driver.read(path/'result.json');cs=raw['candidates'];good=[c for c in cs if c['found' if method in ('ours','center') else 'success']]
    assert len(cs)==4 and bool(good)==raw['found'] and raw['case']==dict(layout='togt',level=level) and raw['method']==method
    row=dict(level=level,variant=method,profile=[list(x) for x in scenario.profile(level)],plan_directory=str(path),
        plan_result_sha256=driver.digest(path/'result.json'),build_s=raw['build_s'],solve_s=raw['solve_s'],gazebo_run=False)
    if not raw['found']:return dict(row,passed=False,verdict='planning_not_found',collision_detected=None,full_trajectory_time_s=None,check_s=0.,scene_assumption_passed=None)
    metric='nominal_time_s' if method in ('ours','center') else 'objective'
    assert abs(cs[raw['selected_seed']][metric]-min(c[metric] for c in good))<1e-8
    tr,_=scenario.build_case(scenario.Case(level=level));r=evaluate(raw['coefficients'],raw['durations'],tr,scenario.ASSETS/'meshes')
    assert abs(r['full_trajectory_time_s']-raw['nominal_time_s'])<1e-8
    return dict(row,**r,**layout.foreign_diagnostics(r))


def report(out,rows,state='running'):
    lines=['# 实验二：TOGT原始布局的动态小窗适配','',f'状态：{state}，{len(rows)}/40条。','',
        '官方race_uzh_7g_multiprisma位置/顺序/前六姿态，2.2倍场地尺度与统一高度平移，保留上下叠窗和交错折返。',
        '仍用现有七种实体小窗和x500；原第7个SingleBall改为第7物理窗、终点改返航，故是场景适配而非原场景完整复刻。',
        'W3平移、W4摆动交替升级，其余五窗固定低速运动，初相0。时间含完整七窗返航零VAJ。','',
        '|档位|W3平移 Hz/m|W4摆动 Hz/m|本文自由点|本文中心|动态TOGT自由点|动态TOGT中心|',
        '|---|---|---|---:|---:|---:|---:|']
    data=[]
    for level in scenario.LEVELS:
        by={r['variant']:r for r in rows if r['level']==level};p=scenario.profile(level);cells=[];item=dict(level=level,profile=[list(x) for x in p])
        for m in driver.METHODS:
            r=by.get(m,{});v=r.get('verdict','pending');t=r.get('full_trajectory_time_s');cells.append((f'{t:.3f}s，' if t is not None else '')+driver.LABELS[v]);item[m]=dict(verdict=v,full_trajectory_time_s=t)
        for free,center,key in [('ours','center','ours_saving_percent'),('dynamic_togt','dynamic_togt_center','togt_saving_percent'),('ours','dynamic_togt','ours_vs_togt_saving_percent')]:
            x,y=by.get(free,{}),by.get(center,{})
            item[key]=100*(y['full_trajectory_time_s']-x['full_trajectory_time_s'])/y['full_trajectory_time_s'] if x.get('passed') and y.get('passed') else None
        data.append(item);lines.append('|'+level+f'|{p[2][0]:g}/{p[2][1]:g}|{p[3][0]:g}/{p[3][1]:g}|'+'|'.join(cells)+'|')
    lines+=['','## 原始证据与计算','', '|档位|方法|建图 s|四初值求解 s|检查 s|非相邻窗风险|记录|','|---|---|---:|---:|---:|---|---|']
    for r in sorted(rows,key=lambda r:(scenario.LEVELS.index(r['level']),driver.METHODS.index(r['variant']))):
        foreign='—' if r['scene_assumption_passed'] is None else '无' if r['scene_assumption_passed'] else '发现，批次停止'
        lines.append(f"|{r['level']}|{driver.NAMES[r['variant']]}|{r['build_s']:.3f}|{r['solve_s']:.3f}|{r['check_s']:.3f}|{foreign}|[记录](assessments/{r['level']}_{r['variant']}.json)|")
    lines+=['','![TOGT三维布局](course_layout.png)','',
        '先geometry/motion核验、静态四方全通过、低/最高动态TOGT无非相邻风险，preflight_gate通过才完整矩阵。',
        '运动扫掠与1m锚点航段管道分离是布局筛查，不是任意轨迹证明；全部实际参考仍检查所有窗口和地面。',
        '未找到/碰撞/待定/任务未完成分开；共同名义成功才计算时间优势，不把结果当Gazebo接触或跟踪安全。',
        '仅场景适配，不改规划器/代价/约束/四初值/预算/择解；不复用被否决赛道成绩，不启动Gazebo。']
    (out/'REPORT.md').write_text('\n'.join(lines)+'\n')
    driver.save(out/'analysis.json',dict(state=state,completed=len(rows),total=40,conditions=data,
        foreign_risk_records=sum(r.get('scene_assumption_passed') is False for r in rows)))


def configure():
    driver.HERE=HERE;driver.MODULE=MODULE;driver.scenario=scenario;driver.layout=layout
    driver.sources=sources;driver.plan=plan;driver.assess=assess;driver.report=report


if __name__=='__main__':configure();driver.main()
