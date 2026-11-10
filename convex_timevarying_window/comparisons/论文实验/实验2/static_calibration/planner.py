"""Reuse the historical joint dynamic TOGT/x500 objective and safe maps."""
import argparse
import json
import time
from dataclasses import asdict, replace
from pathlib import Path
import numpy as np
from convex_dynamic_seven_window_gazebo.x500_togt.experiment import X500Objective,build_track
from nonconvex_timevarying_window.sc_dynatogt.optimizer import _minimize_togt_lbfgs
from nonconvex_timevarying_window.sc_dynatogt.time_mapping import durations_from_k,k_from_durations
from .scenario import Case,build_case,manifest


class Objective(X500Objective):
    def __init__(self,track,center=False,dynamic_limit_fraction=1.):
        if not np.isfinite(dynamic_limit_fraction) or not (0.0225 < dynamic_limit_fraction <= 1.):
            raise ValueError('Dynamic limit fraction must be in (0.0225, 1]')
        _,cfg=build_track()
        limits=cfg.dynamic_limits
        cfg=replace(cfg,samples_per_segment=32,dynamic_limits=replace(limits,
            max_collective_thrust=limits.max_collective_thrust*dynamic_limit_fraction,
            max_body_rate_xy=limits.max_body_rate_xy*dynamic_limit_fraction,
            max_body_rate_z=limits.max_body_rate_z*dynamic_limit_fraction,
            max_rotor_thrust=limits.max_rotor_thrust*dynamic_limit_fraction))
        super().__init__(track,cfg);self.center=center
        self.dynamic_limit_fraction=float(dynamic_limit_fraction)
        if dynamic_limit_fraction != 1.:
            from ..soft_dynamics.minco_penalty import ConfiguredMINCOCore
            self.core=ConfiguredMINCOCore(dynamic_limit_fraction)
        if center:self.dimension=self.temporal_dimension
    def split(self,x):
        if not self.center:return super().split(x)
        x=np.asarray(x,float)
        if x.shape!=(8,) or not np.isfinite(x).all():raise ValueError('center mode has eight duration variables')
        return x,tuple(w.aperture.initial_d() for w in self.track.windows)
    def value_and_gradient(self,x):
        cost,gradient=super().value_and_gradient(x)
        return cost,gradient[:8] if self.center else gradient
    def initial_guess(self):
        x=super().initial_guess()
        return x[:8] if self.center else x


def nominal_audit(forward,track):
    curve=forward.trajectory;tt=np.linspace(0,curve.total_time,int(np.ceil(curve.total_time/.001))+1);P=curve.evaluate(tt,0)
    rows=[]
    for w in track.windows:
        q=(P-w.center0)@w.R
        # This is a geometric proxy only, not the physical Gazebo verdict.
        distance=np.hypot(w.aperture.boundary_distance(q[:,:2]),q[:,2])-.3859447197013662
        rows.append(dict(name=w.name,min_zero_thickness_sphere_clearance_m=float(distance.min()),at_s=float(tt[distance.argmin()])))
    return dict(evidence='1ms nominal sphere proxy against historic aperture boundaries; not a physical collision verdict or certificate',per_window=rows)


def solve(case,method,out,*,dynamic_limit_fraction=1.,seconds_per_seed=180.):
    out=Path(out);out.mkdir(parents=True,exist_ok=False);track,_=build_case(case)
    began=time.perf_counter();o=Objective(track,method.endswith('_center'),dynamic_limit_fraction);build=time.perf_counter()-began
    initial=o.initial_guess();initial_T=durations_from_k(initial[:8]);candidates=[];best=None;began=time.perf_counter()
    for i,scale in enumerate((1.,.8,1.2,1.5)):
        x=initial.copy();x[:8]=k_from_durations(initial_T*scale);started=time.perf_counter();deadline=started+seconds_per_seed
        def measured(value):
            if time.perf_counter()>deadline:raise TimeoutError('per-seed wall budget')
            return o.scipy_value_and_gradient(value)
        try:
            opt=_minimize_togt_lbfgs(measured,x,o.config);cost,grad=o.value_and_gradient(opt.x)
            row=dict(seed=i,scale=scale,initial_variables=x.tolist(),variables=opt.x.tolist(),success=bool(opt.success),
                     status=int(opt.status),message=str(opt.message),objective=float(cost),iterations=int(opt.nit),evaluations=int(opt.nfev),
                     gradient_inf_norm=float(abs(grad).max()),elapsed_s=time.perf_counter()-started)
            if opt.success and np.isfinite(cost):
                forward=o.forward(opt.x)
                row['nominal_time_s']=float(forward.trajectory.total_time)
                if best is None or cost<best[0]:best=cost,i,opt.x.copy(),forward
        except TimeoutError as e:row=dict(seed=i,scale=scale,initial_variables=x.tolist(),success=False,message=str(e),elapsed_s=time.perf_counter()-started)
        candidates.append(row);print('SEED',case.layout,method,json.dumps({k:v for k,v in row.items() if k not in ('variables','initial_variables')}),flush=True)
    result=dict(found=best is not None,case=dict(layout=case.layout,level=case.level),method=method,build_s=build,solve_s=time.perf_counter()-began,
                candidates=candidates,selected_seed=best[1] if best else None,all_seven_joint=True,replanning=False,intermediate_stop_points=False,
                planner='historic X500Objective + exact shape-specific safe maps',optimizer='historic SciPy L-BFGS-B with TOGT past-32 stopping',
                settings=dict(memory=256,max_line_search=64,past=32,cost_tolerance=1e-5,gradient_tolerance=0),
                selection='lowest original objective among converged four duration starts',safety_objective_added=False,safety_filter_added=False)
    result.update(dynamic_limit_fraction=o.dynamic_limit_fraction,
        dynamic_limits=asdict(o.config.dynamic_limits),fixed_quadrature_intervals=32,
        tilt_penalty_weight=0.,all_four_attempted=len(candidates)==4,
        seconds_per_seed=None if np.isinf(seconds_per_seed) else seconds_per_seed)
    if best:
        _,_,x,f=best;C=f.trajectory.coefficients*f.durations[:,None,None]**np.arange(8)[None,:,None]
        result.update(variables=x.tolist(),durations=f.durations.tolist(),coefficients=C.tolist(),nominal_time_s=float(f.trajectory.total_time),
                      crossing_points=f.waypoints.tolist(),crossing_times_s=f.traversal_times.tolist(),made_at=0.,
                      safe_point_membership=[w.aperture.contains(q,safe=True) for w,q in zip(track.windows,f.local_points)],nominal_audit=nominal_audit(f,track))
        np.savez_compressed(out/'trajectory.npz',durations=f.durations,coefficients=C,variables=x)
    (out/'protocol.json').write_text(json.dumps(manifest(case),ensure_ascii=False,indent=2)+'\n')
    (out/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print('PLAN_RESULT',case.layout,method,result['found'],result.get('nominal_time_s'),flush=True);return result


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--layout',required=True);p.add_argument('--method',required=True,choices=('dynamic_togt','dynamic_togt_center'));p.add_argument('--outdir',type=Path,required=True);a=p.parse_args()
    solve(Case(a.layout,'static'),a.method,a.outdir)
