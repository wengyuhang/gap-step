"""Standalone validation of the minimal repair; never edits historical results."""
import argparse,os,time
from dataclasses import asdict
from pathlib import Path
import numpy as np
from convex_timevarying_window.tilt_heading_minco.planner import Planner
from convex_timevarying_window.tilt_heading_minco import dynamics
from ..basic_group.whole_course_difficulty import scenario,run as original
from ..basic_group.trajectory_only.collision import evaluate
from convex_timevarying_window.togt_minco_mpc.trajectory import basis

HERE=Path(__file__).resolve().parent
save=original.driver.save;read=original.driver.read;digest=original.driver.digest


def sources():
    result=original.sources()
    for folder in (HERE,scenario.ROOT/'convex_timevarying_window/tilt_heading_minco'):
        for p in folder.rglob('*'):
            if p.is_file() and p.suffix in ('.py','.md'):result[str(p.relative_to(scenario.ROOT))]=digest(p)
    return result


def attitude_audit(plan):
    margins=[];forces=[];errors=[];max_motor=-np.inf;min_motor=np.inf;max_rates=np.zeros(3);count=0;joins=[]
    for k,T in enumerate(plan.durations):
        u=np.linspace(0,1,max(2,int(np.ceil(T/.005))+1));n=len(u)
        jet=[(plan.coefficients[k].T@np.column_stack([np.asarray(basis(float(at),d)).ravel() for at in u]))/T**d for d in (1,2,3,4)]
        R,F,m=dynamics.orientation.map(n)(*jet[:2]);R=np.array(R)
        F=np.array(F).ravel();m=np.array(m).ravel();_,w,r=dynamics.flat_actuators.map(n)(*jet)
        w=np.array(w);r=np.array(r)
        errors.extend(float(np.max(abs(R[:,3*i:3*i+3].T@R[:,3*i:3*i+3]-np.eye(3)))) for i in range(n))
        margins.extend(m);forces.extend(F);count+=n
        max_motor=max(max_motor,float(r.max()));min_motor=min(min_motor,float(r.min()));max_rates=np.maximum(max_rates,np.max(abs(w),axis=1))
        if k:
            va=[plan.coefficients[k-1].T@np.asarray(basis(1.,d)).ravel()/plan.durations[k-1]**d for d in (1,2,3,4)]
            Ra=np.array(dynamics.orientation(*va[:2])[0]);wa=np.array(dynamics.flat_actuators(*va)[1]).ravel()
            joins.append(dict(knot=k,rotation_error=float(np.max(abs(Ra-R[:,:3]))),body_rate_error=float(np.max(abs(wa-w[:,0])))))
    domain=bool(np.all(np.isfinite(margins)) and min(margins)>dynamics.CHART_MIN and min(forces)>dynamics.FORCE_MIN and max(errors)<1e-7)
    return dict(passed=domain and all(j['rotation_error']<1e-6 and j['body_rate_error']<1e-5 for j in joins),
        sample_count=count,max_dt_s=.005,north_chart_min_margin=float(min(margins)),minimum_force_n=float(min(forces)),
        maximum_orthogonality_error=float(max(errors)),knot_continuity=joins,minimum_motor_n=min_motor,
        maximum_motor_n=max_motor,maximum_body_rates_radps=max_rates.tolist(),
        scope='5ms numerical attitude domain and continuity audit; dense actuator extrema diagnostic, not a continuous dynamics certificate')


def main():
    p=argparse.ArgumentParser();p.add_argument('--level',choices=scenario.LEVELS,required=True)
    p.add_argument('--method',choices=('ours','center'),required=True);p.add_argument('--outdir',type=Path,required=True)
    p.add_argument('--seconds-per-seed',type=float,default=180.);p.add_argument('--max-iterations',type=int,default=2000)
    a=p.parse_args();out=a.outdir.resolve();out.mkdir(parents=True,exist_ok=False)
    hashes=sources();case=scenario.Case(level=a.level);track,_=scenario.build_case(case)
    save(out/'protocol.json',dict(scene=scenario.manifest(case),method=a.method,source_sha256=hashes,
        algorithm='tilt_heading_minco minimal numerical repair',protocol=(HERE/'PROTOCOL.md').read_text(),
        seconds_per_seed=a.seconds_per_seed,max_iterations=a.max_iterations,gazebo_run=False))
    save(out/'status.json',dict(state='building',pid=os.getpid(),level=a.level,method=a.method))
    began=time.perf_counter();planner=Planner(track,a.method=='ours',pose_function=scenario.pose,max_iterations=a.max_iterations)
    build=time.perf_counter()-began
    save(out/'build.json',dict(build_s=build,variables=planner.nvar,pieces=planner.n,config=asdict(planner.cfg)))
    save(out/'status.json',dict(state='solving',pid=os.getpid(),level=a.level,method=a.method))
    with (out/'progress.jsonl').open('w') as log:
        def progress(row):
            import json
            line=json.dumps(row,ensure_ascii=False);log.write(line+'\n');log.flush();print(line,flush=True)
        plan,result=planner.solve_all(a.seconds_per_seed,progress)
    result.update(case=asdict(case),method=a.method,build_s=build,config=asdict(planner.cfg))
    if plan is not None:
        result.update(coefficients=plan.coefficients.tolist(),durations=plan.durations.tolist(),variables=plan.x.tolist(),
            nominal_time_s=float(plan.durations.sum()),crossing_times_s=plan.crossing_times.tolist(),made_at=0.)
        np.savez_compressed(out/'trajectory.npz',coefficients=plan.coefficients,durations=plan.durations,variables=plan.x)
        assessment=evaluate(plan.coefficients,plan.durations,track,scenario.ASSETS/'meshes')
        audit=attitude_audit(plan);save(out/'assessment.json',assessment);save(out/'attitude_audit.json',audit)
        result['independent_assessment']=dict(verdict=assessment['verdict'],passed=assessment['passed'],attitude_domain_audit_passed=audit['passed'])
    assert sources()==hashes,'source changed during validation'
    save(out/'result.json',result)
    save(out/'status.json',dict(state='complete',pid=os.getpid(),found=result['found'],
        method=a.method,level=a.level,nominal_time_s=result.get('nominal_time_s'),assessment=result.get('independent_assessment'),sources_unchanged=True))
    print('COMPLETE',read(out/'status.json'),flush=True)


if __name__=='__main__':main()
