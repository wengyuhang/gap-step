"""Standalone entry for canonical-static amplitude-continuation repair."""
import argparse,json,os
from dataclasses import asdict
from pathlib import Path
import numpy as np
from convex_timevarying_window.tilt_heading_minco.static_continuation import ContinuationPlanner
from .run import sources,save,read,attitude_audit,scenario,evaluate


def main():
    p=argparse.ArgumentParser();p.add_argument('--level',choices=scenario.LEVELS,required=True)
    p.add_argument('--method',choices=('ours','center'),required=True);p.add_argument('--outdir',type=Path,required=True)
    p.add_argument('--total-seconds',type=float,default=720.);p.add_argument('--stage-seconds',type=float,default=180.)
    args=p.parse_args();out=args.outdir.resolve();out.mkdir(parents=True,exist_ok=False)
    hashes=sources();case=scenario.Case(level=args.level);track,_=scenario.build_case(case)
    save(out/'protocol.json',dict(scene=scenario.manifest(case),method=args.method,source_sha256=hashes,
        algorithm='tilt_heading_minco canonical-static amplitude continuation',
        total_budget_s=args.total_seconds,per_stage_budget_s=args.stage_seconds,
        static_frequency_zero=True,static_effective_period_infinite=True,initial_step=.25,minimum_step=1/64,
        final_target_must_equal_lambda1=True,strict_optimizer_and_original_safety_acceptance=True,
        point_map_objective_minimizers_physical_constraints_unchanged=True,gazebo_run=False,
        differs_from_original_four_start_solver=True))
    save(out/'status.json',dict(state='solving',pid=os.getpid(),level=args.level,method=args.method))
    accepted=0.
    with (out/'progress.jsonl').open('w') as log:
        def progress(row):
            nonlocal accepted
            line=json.dumps(row,ensure_ascii=False);log.write(line+'\n');log.flush();print(line,flush=True)
            if row.get('stage')=='continuation_stage_finished' and row['found']:accepted=row['lambda_target']
            save(out/'status.json',dict(state='solving',pid=os.getpid(),level=args.level,method=args.method,
                lambda_target=row.get('lambda_target'),lambda_accepted=accepted))
        planner=ContinuationPlanner(track,args.method=='ours',pose_function=scenario.pose)
        plan,result=planner.solve(args.total_seconds,args.stage_seconds,progress)
    result.update(case=asdict(case),method=args.method,all_seven_joint=True,replanning=False,intermediate_stops=False)
    if plan is not None:
        result.update(coefficients=plan.coefficients.tolist(),durations=plan.durations.tolist(),variables=plan.x.tolist(),
            nominal_time_s=float(plan.durations.sum()),crossing_times_s=plan.crossing_times.tolist(),made_at=0.)
        np.savez_compressed(out/'trajectory.npz',coefficients=plan.coefficients,durations=plan.durations,variables=plan.x)
        assessment=evaluate(plan.coefficients,plan.durations,track,scenario.ASSETS/'meshes');audit=attitude_audit(plan)
        save(out/'assessment.json',assessment);save(out/'attitude_audit.json',audit)
        result['independent_assessment']=dict(verdict=assessment['verdict'],passed=assessment['passed'],attitude_domain_audit_passed=audit['passed'])
    assert sources()==hashes,'source changed during validation'
    save(out/'result.json',result)
    save(out/'status.json',dict(state='complete',pid=os.getpid(),found=result['found'],lambda_reached=result['lambda_reached'],
        nominal_time_s=result.get('nominal_time_s'),assessment=result.get('independent_assessment'),sources_unchanged=True))
    print('COMPLETE',read(out/'status.json'),flush=True)


if __name__=='__main__':main()
