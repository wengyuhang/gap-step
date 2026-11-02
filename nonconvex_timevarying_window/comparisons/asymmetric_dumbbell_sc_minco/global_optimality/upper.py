"""Search all time seeds; refine dynamics peaks and continuous local geometry."""
import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import time

import casadi as ca
import numpy as np
from scipy.optimize import minimize,least_squares

from convex_timevarying_window.single_gate_stop_racing.planner import Config
from convex_timevarying_window.dynamic_passage_velocity_minco.hard_dynamics import flat_actuators
from convex_timevarying_window.dynamic_passage_velocity_minco.allocation_mpc import KF
from ..run import save_json
from ..spin.scenario import build_case
from ..spin.bounds import passage,warmup
from ..spin.run import audit
from .model import PhysicalModel,bound_knot_velocity


def dynamics_peaks(plan,cfg):
    """Find actual extrema between constraint nodes, then exchange their u."""
    limits=np.array([3.8,3.8,3.49]);high=KF*1000**2;low=KF*150**2
    all_min=[];witness=[]
    for piece in range(2):
        duration=plan.durations[piece];begin=plan.durations[0] if piece else 0.
        def values(us):
            us=np.atleast_1d(us);ts=begin+us*duration
            vv=[plan.sample_many(ts,d).T for d in (1,2,3,4)]
            _,r,m=flat_actuators.map(len(us))(*vv)
            r,m=np.asarray(r),np.asarray(m)
            pos=plan.sample_many(ts)
            return np.vstack([1-(r/limits[:,None])**2,(m-low)/high,(high-m)/high,
                ((pos-np.array(cfg.workspace_lower))/30).T,
                ((np.array(cfg.workspace_upper)-pos)/30).T]).T
        us=np.linspace(0.,1.,1001);gg=values(us)
        from scipy.optimize import minimize_scalar
        for col in range(gg.shape[1]):
            ids=np.where((gg[1:-1,col]<=gg[:-2,col])&(gg[1:-1,col]<=gg[2:,col])&
                ((gg[1:-1,col]<gg[:-2,col]-1e-12)|(gg[1:-1,col]<gg[2:,col]-1e-12)))[0]+1
            best=(float(gg[:,col].min()),float(us[np.argmin(gg[:,col])]))
            for k in ids:
                r=minimize_scalar(lambda u:float(values([u])[0,col]),
                    bounds=(us[k-1],us[k+1]),method='bounded',options={'xatol':1e-11})
                if r.fun<best[0]:best=(float(r.fun),float(r.x))
            all_min.append(best[0])
            if best[0]<2e-6:witness.append((piece,best[1],best[0],col))
    return min(all_min),witness


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--outdir',type=Path,required=True)
    ap.add_argument('--max-starts',type=int,default=36)
    ap.add_argument('--budget-s',type=float,default=240.)
    ap.add_argument('--spin-hz',type=float,default=2.)
    ap.add_argument('--restore',action='store_true')
    ap.add_argument('--time-count',type=int,default=13)
    args=ap.parse_args();out=args.outdir.resolve();out.mkdir(parents=True,exist_ok=False)
    cfg=Config(compile_objective=False,max_solve_ms=1e9,constrained_iterations=300)
    track=build_case(args.spin_hz)
    began=time.perf_counter();model=PhysicalModel(track,cfg);warmup()
    build_s=time.perf_counter()-began
    save_json(out/'manifest.json',dict(config=asdict(cfg),spin_hz=args.spin_hz,
        frequency_hz=3.,amplitude_m=2.,build_s=build_s,flight_executed=False,
        objective='total duration',trajectory='original two-piece PV-MINCO',
        parameterization='physical polygon coordinates, equivalent interior points',
        safety_geometry='same original finite edges and margins',
        dynamics='same rotor/rate/workspace constraints, plus exchanged peak witnesses',
        optimizer='SLSQP analytic first derivatives, multi-start; not global proof',
        exact_continuous_dynamics_certificate=False))
    seeds=[]
    # Cover early through old crossing phases, both geometric components.
    for room,q in [('left',[-1.,0.]),('right',[1.6,0.])]:
        for t in np.linspace(.75,1.85,args.time_count):
            seeds.append((room,np.r_[np.log([t,max(1.,2.9-t)]),q,[0.,8.,0.]]))
    # Historical best is an incumbent, never disguised as an independent cold start.
    previous=Path(__file__).parents[1]/'results/spin2_ipopt_adaptive_v1_20261014/01_ipopt/plan.npz'
    if previous.exists():
        from nonconvex_timevarying_window.sc_dynatogt.sc_mapping import SCDiskMap
        old=np.load(previous);x=old['x'].copy()
        mp=SCDiskMap.load(Path(__file__).parents[1]/'results/spin2_jit_v1_20261013/sc_map.npz')
        x[2:4]=mp.map_unconstrained(x[2:4]);seeds.insert(0,('historical_ipopt',x))
    vlo,vhi=bound_knot_velocity(cfg)
    bounds=list(zip(np.r_[np.log([.3,.3]),[-1.6,-1.6],vlo],
                    np.r_[np.log([15.,15.]),[3.2,1.6],vhi]))
    rows=[];best=None;search_began=time.perf_counter()
    for index,(room,seed) in enumerate(seeds[:args.max_starts]):
        if time.perf_counter()-search_began>args.budget_s:break
        dynamic=model.initial_dynamics.copy();geom=model.initial_geometry.copy()
        dused=[17,17];gused=[len(cfg.passage_seed_offsets)]*2
        x=seed.copy();rounds=[]
        qb=((-1.6,-.4),(-.6,.6)) if room=='left' else ((0.,3.2),(-1.6,1.6))
        bb=bounds.copy();bb[2:4]=qb
        accepted=False
        for outer in range(12):
            if time.perf_counter()-search_began>args.budget_s:break
            par=model.parameters(dynamic,geom)
            origin=x.copy();scale=np.array([.04,1.,1.,1.,8.,8.,8.])
            cached=None;raw=None
            def evaluate(y):
                nonlocal cached,raw
                xx=origin+scale*y
                if cached is None or not np.array_equal(xx,cached):
                    raw=model.function(xx,par);cached=xx.copy()
                return raw
            y0=np.zeros(7)
            if args.restore and outer==0:
                low=(np.array([b[0] for b in bb])-origin)/scale
                high=(np.array([b[1] for b in bb])-origin)/scale
                restored=least_squares(lambda y:np.minimum(np.asarray(evaluate(y)[2]).ravel()-1e-5,0.),
                    y0,jac=lambda y:np.asarray(evaluate(y)[3])*scale*
                        (np.asarray(evaluate(y)[2]).ravel()<1e-5)[:,None],
                    bounds=(low,high),max_nfev=100,ftol=1e-9,xtol=1e-9,gtol=1e-9)
                y0=restored.x
            result=minimize(lambda y:(float(evaluate(y)[0]),np.asarray(evaluate(y)[1]).ravel()*scale),
                y0,method='SLSQP',jac=True,
                bounds=[((l-o)/s,(h-o)/s) for (l,h),o,s in zip(bb,origin,scale)],
                constraints={'type':'ineq',
                    'fun':lambda y:np.asarray(evaluate(y)[2]).ravel()-1e-5,
                    'jac':lambda y:np.asarray(evaluate(y)[3])*scale},
                options={'maxiter':300,'ftol':2e-10})
            x=origin+scale*result.x;rr=model.function(x,par)
            row=dict(round=outer,status=str(result.message),success=bool(result.success),
                finite_gmin=float(np.asarray(rr[2]).min()),duration_s=float(np.exp(x[:2]).sum()),
                x=x.tolist(),iterations=int(result.nit))
            rounds.append(row)
            if row['finite_gmin']<-2e-6:break
            plan=model.plan(x)
            parts=[passage(plan,track.windows[0],cfg.radius+cfg.frame_thickness+cfg.passage_margin,
                           piece,time.perf_counter()+60) for piece in (0,1)]
            minimum,witness=dynamics_peaks(plan,cfg)
            row.update(geometry=parts,dense_peak_dynamics_gmin=minimum)
            if all(p['passed'] for p in parts) and minimum>=-2e-7:
                accepted=True
                if best is None or row['duration_s']<best['nominal_time_s']:
                    best=dict(nominal_time_s=row['duration_s'],x=x.tolist(),q=x[2:4].tolist(),
                        crossing_time_s=float(np.exp(x[0])),origin_seed=room,seed_index=index,
                        finite_gmin=row['finite_gmin'],peak_refined_dynamics_gmin=minimum,
                        geometry=parts,posthoc_audit=audit(plan,track.windows[0],cfg),
                        continuous_dynamics_certificate=False,global_optimality_proven=False)
                    np.savez(out/'best_plan.npz',coefficients=plan.coefficients,
                             durations=plan.durations,x=x,physical_point=x[2:4])
                    save_json(out/'best.json',best)
                    print(json.dumps(dict(new_best=best['nominal_time_s'],q=best['q'],
                        seed=index,elapsed_s=time.perf_counter()-search_began)),flush=True)
                break
            added=False
            for piece,u,value,col in sorted(witness,key=lambda a:a[2]):
                if dused[piece]>=model.dynamics_slots:continue
                if np.min(abs(dynamic[piece,:dused[piece]]-u))<1e-6:continue
                dynamic[piece,dused[piece]]=u;dused[piece]+=1;added=True
            for piece,p in enumerate(parts):
                if p['passed'] or p['witness_u'] is None:continue
                delta=(1-p['witness_u'] if piece==0 else p['witness_u'])*plan.durations[piece]
                if gused[piece]>=model.geometry_slots:continue
                if np.min(abs(geom[piece,:gused[piece]]-delta))<1e-6:continue
                geom[piece,gused[piece]]=delta;gused[piece]+=1;added=True
            if not added:break
        rows.append(dict(seed_index=index,room=room,seed=seed.tolist(),accepted=accepted,rounds=rounds))
        save_json(out/'attempts.json',rows)
        save_json(out/'status.json',dict(finished=False,starts_completed=len(rows),
            elapsed_s=time.perf_counter()-search_began,best=best))
        print(json.dumps(dict(seed_done=index,room=room,accepted=accepted,
            candidate_s=rounds[-1]['duration_s'] if rounds else None)),flush=True)
    save_json(out/'status.json',dict(finished=True,starts_completed=len(rows),
        search_s=time.perf_counter()-search_began,build_s=build_s,best=best))


if __name__=='__main__':main()
