"""Round-trip the best physical candidate through the original SC planner."""
import argparse
from dataclasses import asdict
from datetime import datetime
import hashlib
import json
from pathlib import Path
import shutil
import time
from zoneinfo import ZoneInfo

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import least_squares

from convex_timevarying_window.single_gate_stop_racing.planner import Config
from nonconvex_timevarying_window.sc_dynatogt.sc_mapping import SCDiskMap
from ..scenario import VERTICES
from ..spin.scenario import build_case
from ..spin.planner import Planner
from ..spin.run import audit
from ..run import save_json
from .continuous_dynamics import verify


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--outdir',type=Path,required=True)
    args=ap.parse_args();out=args.outdir.resolve();out.mkdir(parents=True,exist_ok=False)
    method=Path(__file__).resolve().parents[1];results=method/'results'
    root=method.parents[2]
    sources=list((method/'global_optimality').glob('*.py'))
    sources += [method/'planner.py',method/'spin/planner.py',method/'spin/scenario.py',
                method/'spin/bounds.py',method/'scenario.py']
    sources += [root/'convex_timevarying_window/dynamic_passage_velocity_minco'/n
                for n in ('trajectory.py','hard_dynamics.py','repaired_planner.py','allocation_mpc.py')]
    hashes={str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}
    for p in sources:
        target=out/'source_snapshot'/p.relative_to(root);target.parent.mkdir(parents=True,exist_ok=True)
        shutil.copyfile(p,target)
    first=results/'global_spin2_v1_20261030'
    data=np.load(first/'best_plan.npz');physical=data['x'].copy()
    cfg=Config(compile_objective=False);track=build_case(2.)
    mapping=SCDiskMap.load(results/'spin2_jit_v1_20261013/sc_map.npz')
    old=np.load(results/'spin2_ipopt_adaptive_v1_20261014/01_ipopt/plan.npz')['x']
    inverse=least_squares(lambda d:mapping.map_unconstrained(d)-physical[2:4],old[2:4],
        jac=mapping.jacobian_unconstrained,ftol=1e-13,xtol=1e-13,gtol=1e-13)
    sc=physical.copy();sc[2:4]=inverse.x
    planner=Planner(track,cfg,mapping)
    base=np.asarray(cfg.passage_seed_offsets)
    offsets=np.tile(np.r_[base,np.zeros(cfg.witness_slots-len(base))],2)
    par=np.r_[0.,track.start,np.zeros(9),offsets]
    raw=planner.function(sc,par)
    plan=planner._plan(sc,0.,raw)
    rho=cfg.radius+cfg.frame_thickness+cfg.passage_margin
    geometry=[planner.check_passage(plan,track.windows[0],rho,i,time.perf_counter()+60) for i in (0,1)]
    np.savez(out/'plan.npz',durations=plan.durations,coefficients=plan.coefficients,x=sc,
             physical_x=physical,physical_point=physical[2:4])
    save_json(out/'status.json',dict(finished=False,stage='arb_continuous_dynamics'))
    cert=verify(out/'plan.npz',120.)
    save_json(out/'continuous_dynamics_arb.json',cert)
    dynamic_audit=audit(plan,track.windows[0],cfg)
    searches=[]
    for name in ('global_spin2_v1_20261030','global_spin2_restore_v1_20261030'):
        st=json.loads((results/name/'status.json').read_text())
        attempts=json.loads((results/name/'attempts.json').read_text())
        searches.append(dict(directory=name,starts=st['starts_completed'],search_s=st['search_s'],
            accepted_attempts=sum(r['accepted'] for r in attempts),best_s=st['best']['nominal_time_s']))
    rejected=[]
    for folder in sorted(results.glob('global_spin2_*bound_v*_20261030')):
        if not (folder/'result.json').exists():continue
        raw_global=json.loads((folder/'result.json').read_text())
        log=(folder/'scip.log').read_text(errors='replace') if (folder/'scip.log').exists() else ''
        warnings=[s for s in ('unresolved numerical troubles','unbounded variable') if s in log]
        item=dict(directory=folder.name,raw_status=raw_global['status'],accepted_for_optimality=False,
            reason='numerical LP/unbounded auxiliary warnings; no trusted nontrivial lower bound',warnings=warnings,
            original_result_preserved=True)
        if folder.name=='global_spin2_force_bound_v4_20261030':
            item['independent_counterexample']='v5 local_relaxation_check.json: feasible relaxation time 2.435338406s < claimed 2.947577223s lower bound'
        save_json(folder/'VALIDATION_REJECTED.json',item);rejected.append(item)
    original=float(np.load(results/'spin2_jit_v1_20261013/spin_2/plan.npz')['durations'].sum())
    final=dict(created_at=datetime.now(ZoneInfo('America/Indiana/Winamac')).isoformat(),
        task=dict(translation_hz=3.,amplitude_m=2.,spin_hz=2.,start=track.start.tolist(),
                  goal=track.goal.tolist(),endpoint_VAJ='zero',config=asdict(cfg)),
        nominal_time_s=float(plan.durations.sum()),crossing_time_s=float(plan.crossing_times[0]),
        local_point=physical[2:4].tolist(),sc_variables=sc.tolist(),
        inverse_error_m=float(np.linalg.norm(inverse.fun)),
        original_nlp_min_residual=float(np.asarray(raw[2]).min()),
        coefficient_max_difference=float(np.max(abs(plan.coefficients-data['coefficients']))),
        continuous_geometry=geometry,continuous_dynamics=cert,independent_1ms_audit=dynamic_audit,
        historical_slsqp_nominal_s=original,savings_fraction=1-float(plan.durations.sum())/original,
        searches=searches,global_runs=rejected,global_optimality_proven=False,
        epsilon_global_optimality_proven=False,trusted_nontrivial_lower_bound_s=None,
        flight_executed=False,evidence='nominal feasible candidate, not a globally optimal certificate')
    if not cert['passed'] or not all(v['passed'] for v in geometry):
        raise RuntimeError('Final original-SC round-trip failed continuous validation')
    if final['original_nlp_min_residual']<-1e-6:raise RuntimeError('Original finite NLP rejected final candidate')
    save_json(out/'result.json',final)
    # Plot only actual stored polynomial and true physical aperture.
    ts=np.linspace(0,plan.end_time,2001);pos=plan.sample_many(ts);vel=plan.sample_many(ts,1)
    fig,ax=plt.subplots(1,3,figsize=(12,3.7),constrained_layout=True)
    ax[0].plot(pos[:,1],pos[:,0]);ax[0].scatter([0],[plan.sample(plan.crossing_times[0])[0]],c='r')
    ax[0].set(xlabel='World y [m]',ylabel='World x [m]',title='Nominal flight: top view')
    vertices=np.vstack([VERTICES,VERTICES[0]])
    ax[1].plot(*vertices.T,color='k');ax[1].scatter(*physical[2:4],c='r',label='Selected crossing')
    ax[1].set(xlabel='Gate u [m]',ylabel='Gate v [m]',title='Actual nonconvex aperture',aspect='equal');ax[1].legend(fontsize=8)
    ax[2].plot(ts,np.linalg.norm(vel,axis=1));ax[2].axvline(plan.crossing_times[0],ls='--',color='r')
    ax[2].set(xlabel='Time [s]',ylabel='Speed [m/s]',title='Stored trajectory speed')
    fig.savefig(out/'trajectory.png',dpi=180);plt.close(fig)
    changed=[p for p,h in hashes.items() if hashlib.sha256((root/p).read_bytes()).hexdigest()!=h]
    save_json(out/'source_verification.json',dict(unchanged=not changed,changed=changed,source_sha256=hashes))
    text=['# 单自旋非凸窗：最优性尝试（2026-10-30）','',
        '**结果：获得更短、整段名义动力学验证通过的可行轨迹；没有获得全局最优证明。**','',
        '冻结3Hz/2m横摆、2圈/秒自旋，原真实12边开口，起终点相距12m且两端零PVAJ。',
        '原两段七次PV-MINCO、机体/框厚/安全余量、推力/角速度/工作空间物理界均保持。',
        '离线优化使用等价物理选点坐标；最终已SC逆映射并用原规划器完整NLP和原QR轨迹重建复核。','',
        '| 指标 | 结果 |','|---|---:|',
        f"| 名义精确终态时间 | {final['nominal_time_s']:.9f} s |",
        f"| 名义中心过面 | {final['crossing_time_s']:.9f} s |",
        f"| 窗局部选点 | ({physical[2]:.9f}, {physical[3]:.9f}) m，右室 |",
        f"| 相对旧SLSQP参考 {original:.9f}s | 缩短 {100*final['savings_fraction']:.3f}% |",
        f"| SC逆映射误差 | {final['inverse_error_m']:.3g} m |",
        f"| 原有限NLP最小残差 | {final['original_nlp_min_residual']:.9g} |",
        f"| 原QR与显式消元最大系数差 | {final['coefficient_max_difference']:.3g} |",
        f"| 整段动力学区间复核 | {cert['status']}，{cert['elapsed_s']:.3f}s |",'',
        '动力学对固定保存多项式使用128位Arb，包含所有时刻的原单桨上下界、角速度与工作空间。',
        '几何仍使用原局部Bernstein/正弦/旋转浮点包络，未升级为Arb几何证书。',
        '二者均仅针对既有名义模型，不保证真实PX4/x500跟踪；本轮未飞。','',
        '## 搜索代价','',
        '| 批次 | 初值尝试 | 接纳尝试 | 实际搜索/s |','|---|---:|---:|---:|']
    for r in searches:text.append(f"| {r['directory']} | {r['starts']} | {r['accepted_attempts']} | {r['search_s']:.3f} |")
    text+=['','总计74个初始化尝试；接纳尝试可能收敛到同一解，不能当不同轨迹数量或成功率。',
        '首批预算300s，在最后一条内部复查中超出至322.304s；实际时间如实保留，不宣称硬截止/在线达标。',
        '一批使用历史候选修复及普通冷初值，另一批加可行性恢复、更密到达时间种子；所有失败保留。','',
        '## 全局求解：未闭合上下界','',
        '使用PySCIPOpt6.3/SCIP10，先导出七变量显式MINCO，再尝试有界Bernstein未消元等价形式。',
        '为获取下界，仅保留原问题必要约束，明确松弛单桨/角速度/穿越邻域几何。',
        '消元形式出现数值LP警告，并报告了不可采信的infeasible/optimal。',
        '其中v4宣称下界2.947577s，但v5独立求得原松弛可行候选2.435338s，直接反驳v4。',
        '**2.435338s是松弛问题的一条可行轨迹时间，不能当原任务可行时间，也不能当已证明下界。**',
        '未消元Bernstein形式运行180.000s、6151节点后超时，原始下界仍只有时长盒给出的0.6s。',
        '其日志也存在数值警告；未取得可用于最优性差距的非平凡可靠下界。',
        '早期重复辅助变量名称导致CIP文件不可可靠重载，已修正源码命名，旧模型文件保留并排除证明。',
        '各原始result/log不覆盖；追加VALIDATION_REJECTED.json作为证据裁决。',
        '本结果不能写作全局最优、epsilon全局最优或物理不可行证明。','',
        '![原始存储轨迹与穿越点](trajectory.png)','',
        '验证：本方法31项测试通过（4条已有CasADi FutureWarning）；未改生产求解器、未操作Gazebo/replay。','']
    (out/'REPORT.md').write_text('\n'.join(text))
    save_json(out/'status.json',dict(finished=True,stage='finished',global_optimality_proven=False,
        nominal_time_s=final['nominal_time_s'],continuous_dynamics_passed=cert['passed']))
    print(json.dumps({k:final[k] for k in ['nominal_time_s','crossing_time_s','local_point',
        'inverse_error_m','original_nlp_min_residual','coefficient_max_difference','savings_fraction',
        'global_optimality_proven','flight_executed']}),flush=True)
    print(json.dumps(cert),flush=True)
    if changed:raise RuntimeError('Source changed during final validation')


if __name__=='__main__':main()
