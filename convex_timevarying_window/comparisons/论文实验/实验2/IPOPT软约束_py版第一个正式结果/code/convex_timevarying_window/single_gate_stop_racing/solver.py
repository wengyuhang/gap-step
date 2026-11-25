"""Deadline includes local verification; failed iterates are seeds, never plans.

No change to the time objective, actuator limits, geometric margins or MPC.
The TOGT map removes aperture-membership inequalities, not whole-body safety
or actuator inequalities, so the remaining problem still uses SLSQP.
"""
import time
import numpy as np
from scipy.optimize import minimize,least_squares
from convex_timevarying_window.dynamic_passage_minco.planner import BudgetExpired
from . import point_mapping
from .geometry import passage


def add_witness(offsets,used,piece,offset):
    """A repeated violating witness signals optimizer failure, not a new row."""
    if np.any(abs(offsets[piece,:used[piece]]-offset)<1e-5):return False
    if used[piece]>=offsets.shape[1]:return False
    offsets[piece,used[piece]]=offset;used[piece]+=1
    return True


def rebase_seed(saved,now):
    x=saved['x'].copy();remaining=saved['now']+np.exp(x[0])-now
    if remaining<.3 or remaining>15.:return None
    x[0]=np.log(remaining)
    return x


def solve(self,state,now,first):
    if first!=0:raise ValueError('Only the nearest unpassed gate can be planned')
    began=time.perf_counter();deadline=began+self.cfg.max_solve_ms/1000.
    cfg=self.cfg;state=np.asarray(state)
    if len(state)==9:state=np.r_[state,np.zeros(3)]
    w=self.windows[0]
    if self.signs[0]*((state[:3]-w.center0)@self.rotation[:,2])<=0:
        raise RuntimeError('head_beyond_unpassed_window_no_reverse_reentry')
    d1=np.linalg.norm(w.center0-state[:3]);d2=np.linalg.norm(self.track.goal-w.center0)
    initial1=max(.8,d1/cfg.arrival_seed_speed+.5)
    initial2=max(.8,np.sqrt(6*d2/cfg.acceleration),(60*d2/cfg.jerk)**(1/3))
    period=w.motion.translation_period;omega=2*np.pi/period
    nearest=round((omega*(now+initial1)+w.motion.phase-np.pi/2)/np.pi)
    point0=point_mapping.initial(w.aperture) if self.point_dim else np.empty(0)
    velocity0=-.8*cfg.speed*self.signs[0]*self.rotation[:,2]
    base=np.asarray(cfg.passage_seed_offsets)
    offsets0=np.tile(np.r_[base,np.zeros(cfg.witness_slots-len(base))],(2,1))
    cold=[np.r_[np.log([max(.3,(np.pi/2+(nearest+k)*np.pi-w.motion.phase)/omega-now),
                        initial2*cfg.initial_tail_scale]),point0,velocity0] for k in (0,-1,-2,1)]
    par0=np.r_[now,state,offsets0.ravel()]
    def score(x):
        out=self.function(np.clip(x,self.lower,self.upper),par0)
        return float(np.max(np.maximum(-np.asarray(out[2]),0.))),float(out[0])
    cold.sort(key=score)
    seeds=[(x,offsets0.copy(),[len(base)]*2,'cold') for x in cold]
    saved=self.failed_seed
    if saved is not None:
        x=rebase_seed(saved,now)
        if x is not None:seeds.insert(0,(x,saved['offsets'].copy(),saved['used'].copy(),'failed_warm'))
    if self.admitted is not None:
        x=rebase_seed(dict(x=self.admitted.x,now=self.admitted.made_at),now)
        if x is not None:seeds.insert(0,(x,offsets0.copy(),[len(base)]*2,'admitted_warm'))
    attempts=[];iterations=0;best=None;failed_best=None;check_ms=0.
    def remember(x,offsets,used,violation):
        nonlocal failed_best
        if not np.all(np.isfinite(x)):return
        key=(-sum(used),violation,float(np.exp(x[:2]).sum()))
        if failed_best is None or key<failed_best[0]:
            failed_best=(key,dict(x=x.copy(),now=now,offsets=offsets.copy(),used=used.copy()))
    for seed,offsets,used,kind in seeds:
        if time.perf_counter()>=deadline-.003:break
        x=np.clip(seed,self.lower,self.upper)
        low,high=self.lower.copy(),self.upper.copy()
        # Motion extrema initialize the solve, but must not restrict arrival
        # to that phase interval. Retain only the original broad duration box.
        x=np.clip(x,low,high);records=[]
        for outer in range(cfg.exchange_rounds):
            if time.perf_counter()>=deadline-.003:break
            par=np.r_[now,state,offsets.ravel()]
            origin=x.copy();last=x.copy()
            scale=np.r_[1/max(1.,omega*np.exp(x[0])),1.,np.ones(self.point_dim),np.full(3,cfg.speed)]
            cached_x=None;cached=None;restore_best=np.inf;restore_x=np.zeros(self.nvar)
            def evaluate(y):
                nonlocal cached_x,cached
                if time.perf_counter()>=deadline-.003:raise BudgetExpired()
                z=origin+scale*y
                if cached_x is None or not np.array_equal(z,cached_x):
                    raw=self.function(z,par);cached_x=z.copy()
                    cached=(float(raw[0]),np.asarray(raw[1]).ravel(),np.asarray(raw[2]).ravel(),
                            np.asarray(raw[3]),raw)
                return cached
            def objective(y):
                a=evaluate(y)
                if cfg.log_cost_scaling:
                    cost=1000*a[0];return np.log1p(cost),a[1]*scale*(1000/(1+cost))
                return a[0],a[1]*scale
            def callback(y):
                nonlocal last
                last=origin+scale*y
                if time.perf_counter()>=deadline-.003:raise BudgetExpired()
            in_restore=True;restoration_evaluations=0
            try:
                def residual(y):
                    nonlocal restore_best,restore_x
                    g=evaluate(y)[2];r=np.minimum(g,0.);value=float(r@r)
                    if value<restore_best:restore_best=value;restore_x=y.copy()
                    return r
                def residual_jac(y):
                    a=evaluate(y)
                    return a[3]*scale*(a[2]<0.)[:,None]
                initial=np.zeros(self.nvar)
                if cfg.feasibility_restore and np.min(evaluate(initial)[2]) < -1e-5:
                    restored=least_squares(residual,initial,jac=residual_jac,
                        bounds=((low-origin)/scale,(high-origin)/scale),method='trf',
                        max_nfev=40,ftol=1e-8,xtol=1e-8,gtol=1e-8)
                    initial=restored.x;last=origin+scale*initial
                    restoration_evaluations=int(restored.nfev)
                in_restore=False
                if time.perf_counter()>=deadline-.003:raise BudgetExpired()
                result=minimize(objective,initial,method='SLSQP',jac=True,
                    bounds=list(zip((low-origin)/scale,(high-origin)/scale)),
                    constraints={'type':'ineq','fun':lambda y:evaluate(y)[2],
                                 'jac':lambda y:evaluate(y)[3]*scale},callback=callback,
                    options={'maxiter':cfg.constrained_iterations,'ftol':1e-7,'disp':False})
                x=origin+scale*result.x;iterations+=int(result.nit)
                status=str(result.message);converged=bool(result.success)
            except BudgetExpired:
                if in_restore and restore_best<np.inf:last=origin+scale*restore_x
                x=last;status='wall_budget';converged=False
            # One bounded final evaluation also preserves useful progress when
            # an iteration budget expired. It is not permission to execute it.
            if time.perf_counter()>=deadline:break
            out=self.function(x,par);g=np.asarray(out[2]).ravel();violation=max(0.,float(-g.min()))
            remember(x,offsets,used,violation)
            record=dict(round=outer,status=status,variables=x.tolist(),objective=1000*float(out[0]),
                        minimum_witness_residual=float(g.min()),restoration_evaluations=restoration_evaluations)
            records.append(record)
            if not converged or violation>1e-5:break
            plan=self._plan(x,now,out);check_start=time.perf_counter()
            parts=[passage(plan,w,cfg.radius+cfg.frame_thickness+cfg.passage_margin,piece,deadline)
                   for piece in (0,1)]
            check_ms+=1000*(time.perf_counter()-check_start)
            verified=dict(passed=all(p['passed'] for p in parts),parts=parts,
                outcome='numerically_bounded' if all(p['passed'] for p in parts) else 'violated_or_unresolved',
                minimum_sample_distance_m=min(p['minimum_sample_distance_m'] for p in parts),
                boxes=sum(p['boxes'] for p in parts),
                overlap_intervals=[v for p in parts for v in p['overlap_intervals']],
                evidence='floating-point Bernstein/sine enclosures, not a formal certificate')
            record['passage']=verified
            if verified['passed'] and time.perf_counter()<deadline:
                best=(plan,verified);break
            added=False
            for piece,p in enumerate(parts):
                if p['passed'] or p['witness_u'] is None:continue
                offset=((1-p['witness_u']) if piece==0 else p['witness_u'])*plan.durations[piece]
                added=add_witness(offsets,used,piece,offset) or added
            # Keep discovered witnesses for the next request too. They are
            # relative to the crossing knot and re-evaluated on the NEW curve.
            remember(x,offsets,used,-1e-12)
            if not added:break
        attempts.append(dict(seed=seed.tolist(),kind=kind,rounds=records))
        if best is not None:break
    if failed_best is not None:self.failed_seed=failed_best[1]
    elapsed=1000*(time.perf_counter()-began)
    stats=dict(iter_count=iterations,optimization_ms=elapsed,local_check_ms=check_ms,attempts=attempts,
        return_status='safe_passage_found' if best else 'no_safe_passage_under_budget',
        converged=best is not None,formal_certificate=False,candidate_policy='first_converged_locally_safe',
        interior_velocity_free=True,interior_acceleration_free=False,
        point_parameterization='TOGT_2024_disk_or_squared_barycentric',
        failed_candidate_is_executable=False)
    self.solvers[0][0].value=stats
    if best is None:raise RuntimeError('no_safe_passage_under_budget: '+str(elapsed))
    plan,verified=best;stats.update(final_objective=float(plan.durations.sum()),passage=verified)
    plan.optimizer_stats=stats;plan.solve_ms=elapsed
    plan.event_valid=True;plan.event_reason='local_passage_numerically_bounded';plan.terminal_gate=None
    self.failed_seed=None
    return plan
