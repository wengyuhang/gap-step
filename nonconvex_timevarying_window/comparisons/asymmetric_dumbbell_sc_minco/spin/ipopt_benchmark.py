"""Matched cold solves: replace only SLSQP by IPOPT in the original loop.

Both methods share a precompiled physical NLP, the same SC map, geometric
component seeds, restoration, finite constraints and local witness exchange.
The reusable IPOPT object is constructed outside the measured online calls.
"""
import argparse
from dataclasses import asdict
from datetime import datetime
import hashlib
import json
from pathlib import Path
import platform
import time
from types import SimpleNamespace
from zoneinfo import ZoneInfo

import casadi as ca
import numpy as np
import scipy

from convex_timevarying_window.single_gate_stop_racing.planner import Config
from convex_timevarying_window.dynamic_passage_minco.planner import BudgetExpired
from nonconvex_timevarying_window.sc_dynatogt.sc_mapping import SCDiskMap
from .. import solver as local_solver
from ..compare_solvers import NLPCallback
from ..run import save_json
from .planner import Planner
from .scenario import build_case
from .run import audit


class IpoptMinimize:
    """SciPy-compatible first-order adapter with no change to the finite NLP."""
    def __init__(self, count, cfg, mu_strategy='monotone'):
        self.context = None
        self.logs = []
        self.max_solve_s = cfg.max_solve_ms/1000.
        self.options = {
            'print_time': False, 'error_on_fail': False,
            'ipopt.print_level': 0, 'ipopt.sb': 'yes',
            'ipopt.hessian_approximation': 'limited-memory',
            'ipopt.mu_strategy': mu_strategy,
            'ipopt.max_iter': cfg.constrained_iterations,
            'ipopt.max_wall_time': self.max_solve_s,
            'ipopt.tol': 1e-7, 'ipopt.constr_viol_tol': 1e-7,
            'ipopt.acceptable_iter': 0, 'ipopt.bound_relax_factor': 0.,
        }
        self.callback = NLPCallback('spin_ipopt_eval', self.evaluate, count)
        y = ca.MX.sym('spin_y', 7)
        fg = self.callback(y)
        self.solver = ca.nlpsol('spin_ipopt', 'ipopt',
            {'x': y, 'f': fg[0], 'g': fg[1:]}, self.options)

    def evaluate(self, y):
        objective, constraints = self.context
        value, gradient = objective(y)
        return value, gradient, constraints['fun'](y), constraints['jac'](y)

    def __call__(self, objective, initial, *, method, jac, bounds,
                 constraints, callback, options):
        if method != 'SLSQP' or jac is not True or constraints['type'] != 'ineq':
            raise ValueError('Only the original analytic inequality NLP is supported')
        self.context = (objective, constraints)
        began = time.perf_counter()
        try:
            solution = self.solver(x0=initial, lbx=np.asarray(bounds)[:, 0],
                                   ubx=np.asarray(bounds)[:, 1], lbg=0., ubg=np.inf)
        except RuntimeError as exc:
            # The original closure enforces the shared outer wall budget.
            # CasADi wraps Python callback exceptions; retain its budget meaning.
            if 'BudgetExpired' in str(exc):
                raise BudgetExpired() from exc
            raise
        stats = self.solver.stats()
        y = np.asarray(solution['x']).ravel()
        callback(y)
        self.logs.append(dict(solve_ms=1000*(time.perf_counter()-began),
                              iterations=int(stats['iter_count']),
                              status=stats['return_status'], success=bool(stats['success'])))
        return SimpleNamespace(x=y, nit=int(stats['iter_count']),
                               success=bool(stats['success']), message=stats['return_status'])


def report(out, rows, build_ms, ipopt_setup_ms):
    lines=['# 同一自旋单窗：SLSQP 与 IPOPT', '',
        '3 Hz / 2 m 平移 + 2 圈/秒自旋；原几何、分区初值、SC、PV-MINCO、约束与见证交换。',
        '每次从相同几何冷初值开始；计时包含所有区域/时间种子、恢复、优化和局部复查。',
        '两方共用同一个预编译模型。IPOPT 使用解析一阶导数与 limited-memory Hessian。',
        f'一次性模型准备 {build_ms/1000:.3f}s；IPOPT 求解对象准备 {ipopt_setup_ms:.3f}ms，均不计入下表。', '',
        '| 方法 | 次序 | 接纳 | 整套求解/ms | 左区/ms | 右区/ms | 名义起终点/s |',
        '|---|---:|---|---:|---:|---:|---:|']
    for r in rows:
        branches=r['stats']['branches']
        times=[b['stats'].get('optimization_ms') for b in branches]
        fmt=lambda v: '—' if v is None else f'{v:.3f}'
        lines.append(f"| {r['method']} | {r['repeat']+1} | {'通过' if r['accepted'] else '未接纳'} | "
            f"{r['wall_ms']:.3f} | {fmt(times[0])} | {fmt(times[1])} | {fmt(r.get('nominal_time_s'))} |")
    lines += ['', '| 方法 | 接纳/尝试 | 求解中位数/ms | 求解范围/ms |', '|---|---:|---:|---|']
    for method in ('SLSQP','IPOPT'):
        rr=[r for r in rows if r['method']==method]
        if not rr: continue
        tt=[r['wall_ms'] for r in rr]
        lines.append(f"| {method} | {sum(r['accepted'] for r in rr)}/{len(rr)} | "
                     f"{np.median(tt):.3f} | {min(tt):.3f}–{max(tt):.3f} |")
    lines += ['', '这里的通过沿用原有限动力学约束与局部浮点几何复查，独立1ms审计另存。',
              '本实验只比较规划求解，未重新实飞；名义时间不能当成闭环竞速时间。',
              'IPOPT与SLSQP的停止条件数值含义不同，保留各自状态、有限约束残差及局部复查记录。', '']
    (out/'REPORT.md').write_text('\n'.join(lines))


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--outdir',type=Path,required=True)
    ap.add_argument('--repeats',type=int,default=3)
    ap.add_argument('--budget-ms',type=float,default=5000.)
    ap.add_argument('--iterations',type=int,default=70)
    ap.add_argument('--mu-strategy',choices=('monotone','adaptive'),default='monotone')
    ap.add_argument('--no-jit',action='store_true')
    ap.add_argument('--map-path',type=Path,default=Path(__file__).parents[1]/
                    'results/spin2_jit_v1_20261013/sc_map.npz')
    args=ap.parse_args();out=args.outdir.resolve()
    out.mkdir(parents=True,exist_ok=False)
    root=Path(__file__).resolve().parents[4]
    sources=list(Path(__file__).parents[1].rglob('*.py'))
    # Do not recursively include historical source snapshots in ignored results.
    sources=[p for p in sources if 'results' not in p.relative_to(Path(__file__).parents[1]).parts]
    sources += [root/'convex_timevarying_window/single_gate_stop_racing'/name
                for name in ('planner.py','geometry.py','local_bounds.py','solver.py')]
    sources += [root/'convex_timevarying_window/dynamic_passage_velocity_minco'/name
                for name in ('trajectory.py','hard_dynamics.py','hard_planner.py')]
    sources += [root/'nonconvex_timevarying_window/sc_dynatogt/sc_mapping.py']
    hashes={str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}
    cfg=Config(compile_objective=not args.no_jit,max_solve_ms=args.budget_ms,
               constrained_iterations=args.iterations)
    mapping=SCDiskMap.load(args.map_path)
    track=build_case(2.)
    save_json(out/'status.json',dict(stage='building_shared_model',finished=False))
    print('Building shared native model',flush=True)
    began=time.perf_counter();planner=Planner(track,cfg,mapping);build_ms=1000*(time.perf_counter()-began)
    state=np.r_[track.start,np.zeros(9)]
    base=np.asarray(cfg.passage_seed_offsets)
    offsets=np.tile(np.r_[base,np.zeros(cfg.witness_slots-len(base))],2)
    test_x=np.r_[np.log([1.4,2.]),planner.seed_coordinates[1]['d'],[0.,9.6,0.]]
    count=int(planner.function(test_x,np.r_[0.,state,offsets])[2].numel())
    began=time.perf_counter();ipopt=IpoptMinimize(count,cfg,args.mu_strategy);ipopt_setup_ms=1000*(time.perf_counter()-began)
    save_json(out/'manifest.json',dict(created_at=datetime.now(ZoneInfo('America/Indiana/Winamac')).isoformat(),
        config=asdict(cfg),spin_hz=2.,translation_frequency_hz=3.,translation_amplitude_m=2.,
        repeats=args.repeats,cold_start_each_call=True,shared_compiled_nlp=True,
        offline_build_ms=build_ms,ipopt_setup_ms=ipopt_setup_ms,ipopt_options=ipopt.options,
        sc_map_source=str(args.map_path.resolve()),source_sha256=hashes,
        numpy=np.__version__,scipy=scipy.__version__,casadi=ca.__version__,platform=platform.platform(),
        flight_executed=False))
    import shutil
    for source in sources:
        dst=out/'source_snapshot'/source.relative_to(root)
        dst.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(source,dst)
    print(json.dumps(dict(stage='ready',build_ms=build_ms,ipopt_setup_ms=ipopt_setup_ms,
                          constraint_count=count)),flush=True)
    original=local_solver.minimize
    rows=[]
    for repeat in range(args.repeats):
        # Alternate ordering, while keeping the model and cold initial values fixed.
        order=('SLSQP','IPOPT') if repeat%2==0 else ('IPOPT','SLSQP')
        for method in order:
            folder=out/f'{repeat+1:02d}_{method.lower()}';folder.mkdir()
            row=dict(method=method,repeat=repeat,accepted=False)
            ipopt.logs=[]
            local_solver.minimize=original if method=='SLSQP' else ipopt
            save_json(out/'status.json',dict(stage='solving',method=method,repeat=repeat,finished=False))
            began=time.perf_counter()
            try:
                plan=planner.solve(state,0.,0)
                row['wall_ms']=1000*(time.perf_counter()-began)
                row.update(accepted=True,nominal_time_s=float(plan.durations.sum()),
                    crossing_time_s=float(plan.crossing_times[0]),
                    final_q=mapping.map_unconstrained(plan.x[2:4]).tolist())
                np.savez(folder/'plan.npz',durations=plan.durations,coefficients=plan.coefficients,x=plan.x)
                row['posthoc_audit']=audit(plan,track.windows[0],cfg)
            except RuntimeError as exc:
                row['wall_ms']=1000*(time.perf_counter()-began)
                if str(exc)!='no_accepted_component': raise
                row['failure']=str(exc)
            finally:
                local_solver.minimize=original
            row['stats']=planner.solvers[0][0].stats()
            row['ipopt_calls']=ipopt.logs
            save_json(folder/'result.json',row);rows.append(row)
            save_json(out/'summary.json',rows);report(out,rows,build_ms,ipopt_setup_ms)
            print(json.dumps({k:row[k] for k in ('method','repeat','accepted','wall_ms','nominal_time_s')
                              if k in row}),flush=True)
    changed=[p for p,h in hashes.items() if hashlib.sha256((root/p).read_bytes()).hexdigest()!=h]
    save_json(out/'source_verification.json',dict(unchanged=not changed,changed=changed))
    save_json(out/'status.json',dict(stage='finished',finished=True,trial_count=len(rows)))
    if changed: raise RuntimeError('Sources changed during benchmark')


if __name__=='__main__': main()
