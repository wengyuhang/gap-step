"""Recompute repaired trajectories, not just solver/status success flags."""
import argparse,json,os,shutil
from pathlib import Path
import numpy as np
from convex_timevarying_window.tilt_heading_minco.planner import Planner
from convex_timevarying_window.dynamic_passage_velocity_minco.trajectory import Plan
from .run import scenario,save,read,digest,evaluate,attitude_audit

FREE_LEVELS=('a25_f3','a25_f6','a50_f6','a75_f1','a75_f3','a75_f6')


def validate(root,require_free=False):
    previous=root.parent/'togt_whole_course_factorial_v1_20261106'
    prior_hashes=read(previous/'protocol.json')['source_sha256']
    assert all(digest(scenario.ROOT/p)==h for p,h in prior_hashes.items())
    old={ (r['level'],r['variant']):r for r in read(previous/'summary.json') }
    rows=[];all_sources={};snapshots=root/'validated_source_snapshot'
    for path in sorted(root.glob('static_scaled_continuation_*')):
        protocol=read(path/'protocol.json');level=protocol['scene']['case']['level'];method=protocol['method']
        tr,_=scenario.build_case(scenario.Case(level=level))
        assert protocol['scene']==json.loads(json.dumps(scenario.manifest(scenario.Case(level=level))))
        assert protocol['static_auxiliary_frequency_zero'] and protocol['static_effective_period_infinite']
        for rel,h in protocol['source_sha256'].items():
            if rel in all_sources:assert all_sources[rel]==h
            else:assert digest(scenario.ROOT/rel)==h;all_sources[rel]=h
        row=dict(level=level,method=method,directory=str(path),previous_verdict=old[(level,method)]['verdict'])
        if not (path/'result.json').exists():
            rows.append(dict(row,verdict='pending',passed=False));continue
        raw=read(path/'result.json');row.update(result_sha256=digest(path/'result.json'),solve_total_s=raw['solve_s'],build_included_s=raw['build_s'],
            found=raw['found'],lambda_reached=raw['lambda_reached'],stage_count=len(raw['stages']))
        if not raw['found']:
            rows.append(dict(row,passed=False,verdict='planning_not_found'));continue
        assert raw['lambda_reached']==1. and raw['stages'][-1]['lambda_target']==1. and raw['stages'][-1]['found']
        for stage in raw['stages']:
            if not stage['found']:continue
            last=stage['statistics']['records'][-1]
            assert last['optimizer_converged'] and last['minimum_constraint']>=-1e-5 and last['global_passed']
        p=Planner(tr,method=='ours',pose_function=scenario.pose);x=np.array(raw['variables']);T=np.array(raw['durations']);C=np.array(raw['coefficients'])
        g,_,computed=p.evaluate(x,p.witnesses())
        np.testing.assert_allclose(np.exp(x[:8]),T,atol=1e-12,rtol=1e-12)
        np.testing.assert_allclose(computed,C,atol=1e-8,rtol=1e-10);assert g.min()>=-1e-5
        curve=Plan(0.,0,T,C,x,0.,max(0.,-float(g.min())),'independent regression validation',7)
        checks,passed=p.verify(curve);assert passed and len(checks)==56
        assessment=evaluate(C,T,tr,scenario.ASSETS/'meshes');audit=attitude_audit(curve)
        assert assessment['verdict']==read(path/'assessment.json')['verdict'] and audit['passed']==read(path/'attitude_audit.json')['passed']
        assert assessment['passed'] and audit['passed']
        row.update(passed=True,verdict=assessment['verdict'],full_race_time_s=float(T.sum()),
            trajectory_distance_m=assessment['trajectory_distance_m'],node_max_violation=max(0.,-float(g.min())),
            independent_geometry_checks=56,nominal_mesh_and_ground_passed=True,attitude_domain_passed=True,
            dense_motor_max_n=audit['maximum_motor_n'],north_chart_margin=audit['north_chart_min_margin'],
            max_stage_iterations=max((z.get('iterations',0) for s in raw['stages'] for z in s['statistics']['records']),default=0))
        rows.append(row)
    if require_free:
        assert all(any(r['level']==l and r['method']=='ours' and r['passed'] for r in rows) for l in FREE_LEVELS)
    for rel,h in all_sources.items():
        target=snapshots/rel
        if target.exists():assert digest(target)==h;continue
        target.parent.mkdir(parents=True,exist_ok=True);archived=previous/'source_snapshot'/rel
        if archived.is_file() and digest(archived)==h:os.link(archived,target)
        else:shutil.copyfile(scenario.ROOT/rel,target)
        assert digest(target)==h
    validation=dict(passed=True,prior_2386_entries_unchanged=True,prior_source_count=len(prior_hashes),
        all_frozen_trial_sources_unchanged=True,source_snapshot_entries=len(all_sources),
        target_scenes_exact=True,all_found_trajectories_independently_recomputed=True,
        repaired_previous_free_failures=sum(r['method']=='ours' and r['previous_verdict']=='planning_not_found' and r['passed'] for r in rows),
        expected_previous_free_failures=5,checks=rows,gazebo_run=False,formal_interval_certificate=False)
    save(root/'regression_validation.json',validation);save(root/'regression_summary.json',rows)
    report(root,rows)
    print(json.dumps({k:v for k,v in validation.items() if k!='checks'},ensure_ascii=False,indent=2))


def report(root,rows):
    lines=['# 未找到合格解：最终修复回归','',
        '同一版本：保留原TOGT选点映射，倾斜/航向姿态重建，直接总时间目标，SLSQP2000，统一静态辅助缩放的幅度连续求解。',
        '最终完整幅度lambda=1才算成功；没有放宽安全/节点动力学约束，没有中心回退或未收敛候选放行。',
        '这里是纯规划验证。竞速时间包含七窗返航零VAJ；求解总耗时包含全部辅助阶段和建图，不能称100ms在线。','',
        '|幅度/窗宽|频率范围 Hz|方法|旧结果|新结果|完整竞速 s|总计算 s（含建图）|路程 m|',
        '|---|---|---|---|---|---:|---:|---:|']
    order={l:i for i,l in enumerate(scenario.LEVELS)}
    for r in sorted(rows,key=lambda r:(order[r['level']],r['method'])):
        m=scenario.manifest(scenario.Case(level=r['level']));rho=m['amplitude_to_horizontal_opening_width'];lo,hi=m['frequency_range_hz']
        old='未找到' if r['previous_verdict']=='planning_not_found' else '名义通过' if r['previous_verdict']=='nominal_safe_complete' else r['previous_verdict']
        label='名义无碰撞完成' if r['passed'] else '仍计算中' if r['verdict']=='pending' else '未找到完整目标解'
        t=f"{r['full_race_time_s']:.6f}" if r['passed'] else '—';d=f"{r['trajectory_distance_m']:.3f}" if r['passed'] else '—'
        solve=f"{r['solve_total_s']:.3f}" if 'solve_total_s' in r else '—'
        lines.append(f"|{rho:g}|{lo:g}–{hi:g}|{'自由点' if r['method']=='ours' else '中心'}|{old}|{label}|{t}|{solve}|{d}|")
    lines+=['','## 证据与版本边界','',
        'regression_validation.json逐条重建最终曲线、原节点约束、7窗×8段几何复查、独立球/STL/地面碰撞和5ms姿态域/连续性。全部56项不是只检查目标窗口节点。',
        '全程浮点名义证据，不是Arb区间证书或Gazebo实体接触；节点动力学通过不宣称连续动力学严格认证。',
        '本轮所有原失败、未收敛与中止诊断版本均保留，详见各目录result/progress/status；不将它们混成同一算法最优成绩。',
        '源协议与代码快照validated_source_snapshot保留，原40项矩阵与其9项未找到结果没有覆盖。',
        '当前中心高动态若未到lambda1，不能拿低幅度辅助解作完赛时间或宣称物理不可行。']
    (root/'REGRESSION.md').write_text('\n'.join(lines)+'\n')


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);p.add_argument('--require-complete-free',action='store_true')
    a=p.parse_args();validate(a.root.resolve(),a.require_complete_free)
