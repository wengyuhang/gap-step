"""Diagnostic: identical finite NLP, one left seed, different local solvers.

Raw vs common least-squares-restored initialization is a separate factor.
No poll, spatial restart, constraint exchange or change of physical limits.
"""
import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import time
import casadi as ca
import numpy as np
import scipy
from scipy.optimize import Bounds, NonlinearConstraint, least_squares, minimize
from scipy.sparse import csr_matrix

from convex_timevarying_window.single_gate_stop_racing.planner import Config
from convex_timevarying_window.single_gate_stop_racing.geometry import passage
from nonconvex_timevarying_window.sc_dynatogt.sc_mapping import SCDiskMap
from .planner import Planner
from .scenario import VERTICES, Case, build_case, room
from .run import save_json
from .analyze import audit_plan


class NLPCallback(ca.Callback):
    def __init__(self, name, evaluate, count, derivative=False):
        self.evaluate, self.count, self.derivative = evaluate, count, derivative
        self.children = []
        ca.Callback.__init__(self)
        self.construct(name)

    def get_n_in(self):
        return 2 if self.derivative else 1

    def get_n_out(self):
        return 1

    def get_sparsity_in(self, i):
        return ca.Sparsity.dense(7 if i == 0 else self.count+1, 1)

    def get_sparsity_out(self, i):
        return ca.Sparsity.dense(self.count+1, 7 if self.derivative else 1)

    def eval(self, args):
        cost, grad, g, jac = self.evaluate(np.asarray(args[0]).ravel())
        return [np.vstack([grad, jac]) if self.derivative else np.r_[cost, g]]

    def has_jacobian(self):
        return not self.derivative

    def get_jacobian(self, name, inames, onames, opts):
        child = NLPCallback(name, self.evaluate, self.count, True)
        self.children.append(child)
        return child


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--outdir', type=Path, required=True)
    ap.add_argument('--budget-s', type=float, default=10.)
    args = ap.parse_args()
    if args.outdir.exists():
        raise FileExistsError('Preserve old results')
    args.outdir.mkdir(parents=True)
    cfg, case = Config(compile_objective=False), Case(3., 2.)
    mapping = SCDiskMap.fit(VERTICES)
    track = build_case(case)
    p = Planner(track, cfg, mapping)
    state = np.r_[track.start, np.zeros(9)]
    omega = 2*np.pi*case.frequency
    t_guess = max(.8, np.linalg.norm(track.windows[0].center0-track.start)/cfg.arrival_seed_speed+.5)
    nearest = round((omega*t_guess-np.pi/2)/np.pi)
    first = max(.3, (np.pi/2+nearest*np.pi)/omega)
    distance = np.linalg.norm(track.goal-track.windows[0].center0)
    tail = max(.8, np.sqrt(6*distance/cfg.acceleration), (60*distance/cfg.jerk)**(1/3))
    origin = np.r_[np.log([first, tail*cfg.initial_tail_scale]), p.initial_d,
                   -.8*cfg.speed*p.signs[0]*p.rotation[:, 2]]
    scale = np.r_[1/max(1.,omega*first), 1., np.ones(2), np.full(3,cfg.speed)]
    low, high = (p.lower-origin)/scale, (p.upper-origin)/scale
    offsets = np.tile(np.r_[cfg.passage_seed_offsets, np.zeros(cfg.witness_slots-len(cfg.passage_seed_offsets))],2)
    par = np.r_[0.,state,offsets]
    cache_y, cache_value = None, None
    last = np.zeros(7)
    visited = []

    def evaluate(y):
        nonlocal cache_y, cache_value, last
        y = np.asarray(y, float)
        last = y.copy()
        if cache_y is None or not np.array_equal(cache_y,y):
            x = origin+scale*y
            raw = p.function(x,par)
            cost = 1000*float(raw[0])
            cache_y = y.copy()
            cache_value = (np.log1p(cost), 1000*np.asarray(raw[1]).ravel()*scale/(1+cost),
                           np.asarray(raw[2]).ravel(), np.asarray(raw[3])*scale)
            visited.append(dict(x=x.tolist(), q=mapping.map_unconstrained(x[2:4]).tolist(),
                                gmin=float(cache_value[2].min())))
        return cache_value

    restore_start = time.perf_counter()
    restored = least_squares(lambda y:np.minimum(evaluate(y)[2],0.), np.zeros(7),
        jac=lambda y:evaluate(y)[3]*(evaluate(y)[2]<0)[:,None], bounds=(low,high),
        max_nfev=40,ftol=1e-8,xtol=1e-8,gtol=1e-8)
    restore_ms = 1000*(time.perf_counter()-restore_start)
    manifest = dict(case=asdict(case),config=asdict(cfg),origin=origin,scale=scale,
        restoration_ms=restore_ms,restored_variables=origin+scale*restored.x,
        scipy=scipy.__version__,budget_s=args.budget_s,derivatives='Analytic first derivatives, SC chain rule',
        constraints='Identical fixed witness set; no exchange; 1e-5 acceptance tolerance',
        source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),flight_executed=False)
    save_json(args.outdir/'manifest.json',manifest)
    results=[]
    for initialization,y0 in [('raw_left',np.zeros(7)),('restored_left',restored.x)]:
        for method in ['SLSQP','IPOPT','trust-constr']:
            folder=args.outdir/(initialization+'_'+method)
            folder.mkdir()
            visited=[]
            cache_y=None
            last=y0.copy()
            callback_obj=None
            setup_ms=0.
            if method=='IPOPT':
                setup_start=time.perf_counter()
                count=len(evaluate(y0)[2])
                callback_obj=NLPCallback('eval_'+initialization,evaluate,count)
                y=ca.MX.sym('y',7)
                fg=callback_obj(y)
                solver=ca.nlpsol('ipopt_'+initialization,'ipopt',{'x':y,'f':fg[0],'g':fg[1:]},
                    {'ipopt.print_level':0,'print_time':False,'ipopt.hessian_approximation':'limited-memory',
                     'ipopt.max_iter':500,'ipopt.max_wall_time':args.budget_s,
                     'ipopt.tol':1e-7,'ipopt.constr_viol_tol':1e-7})
                setup_ms=1000*(time.perf_counter()-setup_start)
                visited=[]
                cache_y=None
            began=time.perf_counter()
            def timed(y):
                if time.perf_counter()-began>args.budget_s:
                    raise TimeoutError('diagnostic_wall_budget')
                return evaluate(y)
            try:
                if method=='IPOPT':
                    solution=solver(x0=y0,lbx=low,ubx=high,lbg=0.,ubg=np.inf)
                    yfinal=np.asarray(solution['x']).ravel()
                    stats=solver.stats()
                    success,status=bool(stats['success']),stats['return_status']
                    iterations=int(stats['iter_count'])
                else:
                    if method=='SLSQP':
                        constraints={'type':'ineq','fun':lambda y:timed(y)[2],
                                     'jac':lambda y:timed(y)[3]}
                        options={'maxiter':500,'ftol':1e-7}
                    else:
                        constraints=NonlinearConstraint(lambda y:timed(y)[2],0.,np.inf,
                                                       jac=lambda y:csr_matrix(timed(y)[3]))
                        options={'maxiter':500,'gtol':1e-7,'sparse_jacobian':True}
                    result=minimize(lambda y:timed(y)[:2],y0,method=method,jac=True,
                                    bounds=Bounds(low,high),constraints=constraints,options=options)
                    yfinal=result.x
                    success,status=bool(result.success),str(result.message)
                    iterations=int(result.nit)
            except TimeoutError as exc:
                yfinal=last.copy()
                success,status,iterations=False,str(exc),None
            elapsed=1000*(time.perf_counter()-began)
            x=origin+scale*yfinal
            g=evaluate(yfinal)[2]
            q=mapping.map_unconstrained(x[2:4])
            row=dict(method=method,initialization=initialization,solver_success=success,status=status,
                     solve_ms=elapsed,solver_setup_ms=setup_ms,iterations=iterations,x=x,q=q,room=room(q),
                     minimum_residual=float(g.min()),finite_feasible=bool(g.min()>=-1e-5),
                     evaluated_rooms=sorted(set(room(v['q']) for v in visited)),evaluations=visited)
            if row['finite_feasible']:
                raw=p.function(x,par)
                plan=p._plan(x,0.,raw)
                radius=cfg.radius+cfg.frame_thickness+cfg.passage_margin
                parts=[passage(plan,track.windows[0],radius,k) for k in (0,1)]
                row.update(local_geometry_passed=all(v['passed'] for v in parts),passage=parts,
                           nominal_total_time_s=float(plan.durations.sum()))
                np.savez(folder/'plan.npz',coefficients=plan.coefficients,durations=plan.durations,x=plan.x)
                row['posthoc_audit']=audit_plan(folder,case,radius)
            save_json(folder/'result.json',row)
            results.append(row)
            save_json(args.outdir/'summary.json',results)
            print(json.dumps({k:v for k,v in row.items() if k not in ['x','q','evaluations','passage','posthoc_audit']},ensure_ascii=False),flush=True)


if __name__=='__main__':
    main()
