"""Deadline includes local verification; failed iterates are seeds, never plans.

No change to the time objective, actuator limits, geometric margins or MPC.
The TOGT map removes aperture-membership inequalities, not whole-body safety
or actuator inequalities, so the remaining problem still uses SLSQP.
"""
import time
import numpy as np
from scipy.optimize import minimize,least_squares
from convex_timevarying_window.dynamic_passage_minco.planner import BudgetExpired
from .. import point_mapping
from .box_geometry import passage


def shape_inverse(aperture,point):
    """Interior inverse of the existing TOGT map; numerical seeds only."""
    p=np.asarray(point)
    if aperture.kind=='circle':
        r=aperture.radius;length=np.linalg.norm(p)
        if length>=r:p=p*(.99*r/length)
        return p/(r+np.sqrt(max(0.,r*r-p@p)))
    r=aperture.physical_vertices-p;length=np.linalg.norm(r,axis=1)
    next_r=np.roll(r,-1,axis=0)
    cross=r[:,0]*next_r[:,1]-r[:,1]*next_r[:,0]
    tan=abs(cross)/np.maximum(length*np.roll(length,-1)+np.sum(r*next_r,axis=1),1e-12)
    weights=(tan+np.roll(tan,1))/np.maximum(length,1e-12);weights/=weights.sum()
    return np.sqrt(np.r_[weights[1:],weights[:1]])


def crossing_point_seed(self,arrival,velocity):
    w=self.windows[0];cfg=self.cfg
    half=np.asarray(cfg.native_half)+cfg.passage_margin
    # The normal support is estimated before optimization. Final constraints
    # and exact local checking always recompute it using the candidate attitude.
    tau=(max(half[:2])+cfg.frame_thickness)/max(1.,np.linalg.norm(velocity))
    center,R=w.state_at(arrival)[:2]
    early=w.state_at(arrival-tau)[0];late=w.state_at(arrival+tau)[0]
    shift=.5*((early+late)/2-center)@R[:,:2]
    bound=(np.full(2,w.aperture.radius) if w.aperture.kind=='circle'
           else np.max(abs(w.aperture.physical_vertices),axis=0))
    room=np.maximum(.01,bound-max(half[:2])-(cfg.frame_thickness-cfg.frame_outward_offset))
    shift=np.clip(shift,-room,room)
    if w.aperture.kind=='circle' and np.linalg.norm(shift)>min(room):
        shift*=min(room)/np.linalg.norm(shift)
    return shape_inverse(w.aperture,shift)


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
    initial1=max(.8,np.sqrt(6*d1/cfg.acceleration),d1/(.65*cfg.speed))
    initial2=max(.8,np.sqrt(6*d2/cfg.acceleration),(60*d2/cfg.jerk)**(1/3))
    period=w.motion.translation_period;omega=2*np.pi/period
    nearest=round((omega*(now+initial1)+w.motion.phase-np.pi/2)/np.pi)
    point0=point_mapping.initial(w.aperture) if self.point_dim else np.empty(0)
    velocity0=-cfg.speed*self.signs[0]*self.rotation[:,2]
    acceleration0=np.zeros(3) if cfg.interior_acceleration_free else np.empty(0)
    base=np.asarray(cfg.passage_seed_offsets)
    offsets0=np.tile(np.r_[base,np.zeros(cfg.witness_slots-len(base))],(2,1))
    cold=[]
    for prediction in (initial1,1.5*initial1,2*initial1):
        nearest=round((omega*(now+prediction)+w.motion.phase-np.pi/2)/np.pi)
        for k in (0,-1):
            arrival=max(.3,(np.pi/2+(nearest+k)*np.pi-w.motion.phase)/omega-now)
            if arrival<=15.:
                cold.append(np.r_[np.log([arrival,initial2*cfg.initial_tail_scale]),point0,velocity0,acceleration0])
    if self.point_dim:
        for seed in cold:
            seed[2:2+self.point_dim]=crossing_point_seed(self,now+np.exp(seed[0]),velocity0)
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
        arrival=now+np.exp(seed[0])
        turn=(np.pi/2+round((omega*arrival+w.motion.phase-np.pi/2)/np.pi)*np.pi-w.motion.phase)/omega
        low[0]=np.log(max(.3,turn-period/4-now))
        high[0]=np.log(min(15.,max(.3,turn+period/4-now)))
        x=np.clip(x,low,high);records=[]
        for outer in range(cfg.exchange_rounds):
            if time.perf_counter()>=deadline-.003:break
            par=np.r_[now,state,offsets.ravel()]
            origin=x.copy();last=x.copy()
            scale=np.r_[1/max(1.,omega*np.exp(x[0])),1.,np.ones(self.point_dim),np.full(3,cfg.speed),
                np.full(3,cfg.acceleration) if cfg.interior_acceleration_free else np.empty(0)]
            cached_x=None;cached=None;restore_best=np.inf;restore_x=np.zeros(self.nvar)
            best_violation=np.inf;best_evaluated=x.copy()
            def evaluate(y):
                nonlocal cached_x,cached,best_violation,best_evaluated
                if time.perf_counter()>=deadline-.003:raise BudgetExpired()
                z=origin+scale*y
                if cached_x is None or not np.array_equal(z,cached_x):
                    raw=self.function(z,par);cached_x=z.copy()
                    cached=(float(raw[0]),np.asarray(raw[1]).ravel(),np.asarray(raw[2]).ravel(),
                            np.asarray(raw[3]),raw)
                    violation=max(0.,float(-cached[2].min()))
                    if np.isfinite(violation) and violation<best_violation:
                        best_violation=violation;best_evaluated=z.copy()
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
                if cfg.feasibility_restore and cfg.native_solver!='al_lbfgs' and np.min(evaluate(initial)[2]) < -1e-5:
                    restored=least_squares(residual,initial,jac=residual_jac,
                        bounds=((low-origin)/scale,(high-origin)/scale),method='trf',
                        max_nfev=40,ftol=1e-8,xtol=1e-8,gtol=1e-8)
                    initial=restored.x;last=origin+scale*initial
                    restoration_evaluations=int(restored.nfev)
                in_restore=False
                if time.perf_counter()>=deadline-.003:raise BudgetExpired()
                bounds=list(zip((low-origin)/scale,(high-origin)/scale))
                if cfg.native_solver=='al_lbfgs':
                    multipliers=np.zeros_like(evaluate(initial)[2]);mu=100.;previous_violation=np.inf
                    converged=False;iterate=initial;feasible=np.min(evaluate(iterate)[2])>=-1e-5
                    for al_round in range(10):
                        if feasible:break
                        def augmented(y):
                            data=evaluate(y)
                            residual=np.minimum(data[2]-1e-4-multipliers/mu,0.)
                            return .5*mu*(residual@residual),mu*(data[3]*scale).T@residual
                        result=minimize(augmented,iterate,method='L-BFGS-B',jac=True,
                            bounds=bounds,callback=callback,options=dict(maxiter=100,maxls=25,
                                ftol=1e-12,gtol=1e-7,maxcor=12))
                        iterate=result.x;iterations+=int(result.nit)
                        g=evaluate(iterate)[2];violation=max(0.,float(-g.min()))
                        feasible=violation<1e-5
                        if feasible:break
                        multipliers=np.maximum(0.,multipliers-mu*(g-1e-4))
                        if violation>.5*previous_violation:mu*=5.
                        previous_violation=violation
                    if feasible:
                        result=minimize(objective,iterate,method='SLSQP',jac=True,
                            bounds=bounds,constraints={'type':'ineq','fun':lambda y:evaluate(y)[2],
                            'jac':lambda y:evaluate(y)[3]*scale},callback=callback,
                            options=dict(maxiter=cfg.constrained_iterations,ftol=1e-7,disp=False))
                        iterations+=int(result.nit);iterate=result.x
                        converged=bool(result.success)
                        status='AL restoration -> SLSQP: '+str(result.message)
                    else:status='AL restoration unresolved: '+str(result.message)
                    x=origin+scale*iterate
                else:
                    result=minimize(objective,initial,method='SLSQP',jac=True,
                        bounds=bounds,constraints={'type':'ineq','fun':lambda y:evaluate(y)[2],
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
            # A failed SLSQP line search may discard useful restoration work.
            # Retain the best *evaluated* iterate only as the next seed; it
            # still needs convergence and independent passage checking.
            if not converged and best_violation<violation:
                x=best_evaluated;out=self.function(x,par)
                g=np.asarray(out[2]).ravel();violation=max(0.,float(-g.min()))
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
        solver=cfg.native_solver,
        return_status='safe_passage_found' if best else 'no_safe_passage_under_budget',
        converged=best is not None,formal_certificate=False,candidate_policy='first_converged_locally_safe',
        interior_velocity_free=True,interior_acceleration_free=cfg.interior_acceleration_free,
        point_parameterization='TOGT_2024_disk_or_squared_barycentric',
        failed_candidate_is_executable=False)
    self.solvers[0][0].value=stats
    if best is None:raise RuntimeError('no_safe_passage_under_budget: '+str(elapsed))
    plan,verified=best;stats.update(final_objective=float(plan.durations.sum()),passage=verified)
    plan.optimizer_stats=stats;plan.solve_ms=elapsed
    plan.event_valid=True;plan.event_reason='local_passage_numerically_bounded';plan.terminal_gate=None
    self.failed_seed=None
    return plan
