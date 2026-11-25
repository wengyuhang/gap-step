"""Run the frozen ten-condition matrix, then audit the five-method tables."""
import argparse
from collections import deque
import fcntl
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time
import traceback

import numpy as np

from ..soft_dynamics import run as common
from ..soft_dynamics import best_center_release
from ..independent_repair.run import attitude_audit, frozen_plan
from ..radius_ablation.execution import audit
from . import report

HERE = Path(__file__).resolve().parent
MODULE = 'convex_timevarying_window.comparisons.论文实验.实验2.matched_full_matrix'
METHODS = ('ours', 'center', 'dynamic_togt', 'dynamic_togt_center')
LEVELS = tuple(common.scenario.LEVELS)
read, save, digest = common.read, common.save, common.previous.digest


def sources():
    result = {name:sha for name,sha in common.sources().items() if not name.endswith('.md')}
    for path in HERE.rglob('*.py'):
        result[str(path.relative_to(common.ROOT))] = digest(path)
    return result


def protect_previous():
    previous = common.RESULTS/'independent_best_center_a25_f6_v1_20261109'
    protected = dict(read(previous/'protocol.json')['prior_sha256'])
    for name in ('independent_best_center_a25_f6_v1_20261109',
            'independent_best_center_a25_f6_gazebo_v1_20261109',
            'latest_soft_margin90_full_matrix_v1_20261108'):
        protected.update({str(path):digest(path) for path in (common.RESULTS/name).rglob('*.json')
            if 'source_snapshot' not in path.parts})
    old = common.RESULTS/'planning_actual_separated_audit_20261107'
    for name in ('planning.json','actual.json'):
        protected[str(old/name)] = digest(old/name)
    for row in read(old/'actual.json'):
        if row['variant']=='wang2023':
            for path in Path(row['directory']).iterdir():
                if path.is_file() and path.suffix in ('.json','.npz','.gz','.ulg'):
                    protected[str(path)] = digest(path)
    return protected


def reuse_wang(out):
    old = common.RESULTS/'planning_actual_separated_audit_20261107'
    for stage, name in (('planning','planning.json'),('flight','actual.json')):
        rows = []
        for original in read(old/name):
            if original['variant']!='wang2023':
                continue
            success = original['outcome'] in ('frame_clear','old_rule_complete')
            collision = original['outcome'] in ('frame_collision','collision')
            row = dict(level=original['level'], variant='wang2023',passed=success,
                outcome='clear' if success else 'collision' if collision else original['outcome'],
                time_s=original.get('planned_full_duration_s') if stage=='planning' else original.get('old_rule_finish_time_s'),
                data_origin='original_frozen_wang',source_file=str(old/name),source_sha256=digest(old/name),
                measurement_definition='full_seven_gate_return_reference' if stage=='planning' else 'legacy_terminal_threshold')
            for key in ('actual_gate_count','actual_recorded_distance_m','truth_tracking_rms_m',
                    'truth_tracking_max_m','ground_contact_excluded','ground_violation_excluded'):
                if key in original:
                    row[key] = original[key]
            if stage=='flight':
                row.update(original_directory=original['directory'],
                    full_reference_execution_time_s=original['new_no_threshold_finish_time_s'])
            rows.append(row)
        assert len(rows)==10 and {row['level'] for row in rows}==set(LEVELS)
        save(out/f'wang_{stage}.json',rows)


def initialize(out, concurrency):
    out.mkdir(parents=True,exist_ok=False)
    (out/'logs').mkdir()
    frozen = sources()
    common.snapshot(out,frozen)
    reuse_wang(out)
    protocol = dict(source_sha256=frozen, prior_sha256=protect_previous(),levels=LEVELS,methods=METHODS,
        new_method_condition_records=40,initial_starts_per_method=8,maximum_candidate_solves=330,
        our_free_independent_starts=8,our_free_additional_best_center_starts=1,
        best_center_release_after_both_independent_branches=True,
        cross_condition_warm_start=False,initial_points='queried window centers',sampling_seed=0,
        initial_pvaj='analytic MINCO',global_initial_time_scaling=False,
        dynamics_model='same gravity-only x500 TOGT, zero tilt-yaw',dynamic_limit_fraction=.95,
        soft_rotor_bounds_n=[0.,8.54858*.95],soft_rate_xy_degps=209.,soft_rate_z_degps=190.,
        weights=[1000.,1000.],smoothing=.01,tilt_weight=0.,fixed_quadrature_intervals=32,
        passage_margin_m=.15,finite_point_inset_radius_m=.6259447197013662,
        our_final_oracle_radius_m=.6209447197013662,
        point_inset='identical frame-centerline erosion; exact original TOGT shape map',
        our_solver='normal IPOPT limited-memory, tiny push overrides removed',
        our_finite_max_iterations=5000,our_safety_exchange_rounds=24,
        togt_solver='original L-BFGS-B past 32 with analytic MINCO adjoint',
        togt_continuous_safety_constraints_added=False,wall_budget_per_seed=None,
        concurrency=concurrency,blas_threads=1,flight_repeats=1,activated_reference_retry=False,
        startup_retry_before_reference_activation=1,terminal_rule='clock and measurement reach full reference T',
        ground_contact_excluded_from_frame_collision=True,wang_reused=True,tests_passed=8,
        cases=[common.scenario.manifest(common.scenario.Case(level=level)) for level in LEVELS],
        container='convex_seven_togt_experiment',
        image_id=subprocess.check_output(['docker','image','inspect','px4io/px4-sitl-gazebo:latest',
            '--format','{{.Id}}'],text=True).strip())
    save(out/'protocol.json',protocol)
    for name in ('candidate_summary','planning_summary','flight_summary'):
        save(out/f'{name}.json',[])
    for level in LEVELS:
        folder = out/'candidates'/level
        (folder/'jobs').mkdir(parents=True)
        for seed in range(8):
            for method in METHODS:
                job = dict(id=f'{method}_{seed}',method=method,seed=seed,level=level)
                save(folder/'jobs'/f"{job['id']}.json",job)
    report.export(out)
    return protocol


def collect_candidate(out, job, wall, validators):
    folder = out/'candidates'/job['level']
    source = folder/'plans'/job['id']/'result.json'
    raw = read(source)
    assert raw['case']['level']==job['level'] and raw['dynamic_limit_fraction']==.95
    if job['method'] in ('ours','center','release'):
        key = job['level'],job['method']=='center'
        if key not in validators:
            track,_ = common.scenario.build_case(common.scenario.Case(level=job['level']))
            validators[key] = best_center_release.make_planner(track,not key[1])
        planner = validators[key]
        row = best_center_release.assess(folder,job,0,wall,planner,planner.track)
    else:
        row = dict(job=job['id'],method=job['method'],seed=job['seed'],found=raw['found'],
            passed=raw['found'],outcome='优化收敛' if raw['found'] else '未找到合格规划',
            source=str(source),objective=raw.get('objective'),nominal_time_s=raw.get('nominal_time_s'),
            solve_s=raw['solve_s'],build_s=raw['build_s'],observed_process_wall_s=wall)
        if raw['found']:
            assert all(raw['safe_point_membership'])
    row.update(level=job['level'],source_sha256=digest(source))
    return row


def publish(out,level,method,rows):
    eligible = [row for row in rows if row['passed']]
    selected = min(eligible,key=lambda row:row['objective'],default=None)
    folder = out/'plans'/f'{level}_{method}'
    folder.mkdir(parents=True,exist_ok=False)
    source = selected if selected is not None else rows[0]
    raw = read(Path(source['source']))
    result = dict(raw,found=selected is not None,case=dict(layout='togt_whole',level=level),method=method,
        selected_candidate_job=None if selected is None else selected['job'],
        selected_branch=None if selected is None else selected['method'],
        candidate_records=rows,number_initial_starts=len(rows),all_starts_attempted=True,
        selection='lowest original objective among admissible optimizer results',
        build_s=sum(row['build_s'] for row in rows),solve_s=sum(row['solve_s'] for row in rows))
    if method=='ours':
        center_rows = [row for row in read(out/'candidate_summary.json') if row['level']==level and row['method']=='center']
        result.update(shared_center_stage_s=sum(row['build_s']+row['solve_s'] for row in center_rows),
            center_release_attempted=any(row['method']=='release' for row in rows))
    save(folder/'result.json',result)
    if selected is not None:
        shutil.copyfile(Path(selected['source']).parent/'attitude_audit.json',folder/'attitude_audit.json')
    row = common.assess(out,level,method)
    row['qualified_starts'] = len(eligible)
    row['candidate_count'] = len(rows)
    if method=='ours':
        row['shared_center_stage_s'] = result['shared_center_stage_s']
        row['planning_total_s'] += result['shared_center_stage_s']
    save(folder/'validation.json',dict(passed=True,all_candidate_records_preserved=True,
        selected_source=None if selected is None else selected['source'],selection_recomputed=True,
        geometry_outcome=row['outcome'],dynamic_limit_fraction=.95,point_inset_aligned=True))
    print('PLAN_RESULT',level,method,row['outcome'],row['full_trajectory_time_s'],flush=True)
    return row


def validate_initials(out, level):
    for index in range(8):
        pair = {}
        for method in METHODS:
            pair[method] = read(out/'candidates'/level/'plans'/f'{method}_{index}'/'initial.json')
            assert pair[method]['point_variables']==(0 if method in ('center','dynamic_togt_center') else 25)
            assert not pair[method]['optimized_initial_solution_used']
        first = pair['ours']
        expected = np.array(first['coefficients']).reshape(8,3,8).transpose(0,2,1)
        for method in METHODS:
            np.testing.assert_allclose(pair[method]['durations_s'],first['durations_s'],rtol=0.,atol=1e-12)
            np.testing.assert_allclose(pair[method]['points'],first['points'],rtol=0.,atol=1e-10)
            actual = np.asarray(pair[method]['coefficients'])
            if method in ('ours','center'):
                actual = actual.reshape(8,3,8).transpose(0,2,1)
            np.testing.assert_allclose(actual,expected,rtol=0.,atol=1e-8)
    save(out/'candidates'/level/'initial_validation.json',dict(passed=True,
        all_eight_four_way_initial_curves_identical=True,center_points_and_analytic_minco=True))


def start_flight(out,level,method,attempt):
    if shutil.disk_usage(common.ROOT).free<3*2**30:
        raise RuntimeError('insufficient disk for native flight evidence')
    folder = out/'flights'/f'{level}_{method}'/f'attempt_{attempt}'
    folder.parent.mkdir(parents=True,exist_ok=True)
    if folder.exists():
        raise RuntimeError(f'existing flight attempt requires inspection {folder}')
    log = (out/'logs'/f'{level}_{method}_flight_{attempt}.log').open('w')
    process = subprocess.Popen([sys.executable,'-u','-m',
        'convex_timevarying_window.comparisons.论文实验.实验2.no_tilt_comparison.flight',
        '--source',str(out/'plans'/f'{level}_{method}'/'result.json'),
        '--method',method,'--outdir',str(folder)],cwd=common.ROOT,
        stdout=log,stderr=subprocess.STDOUT,env=os.environ.copy())
    print('FLY_START',level,method,attempt,process.pid,flush=True)
    return dict(process=process,log=log,level=level,method=method,attempt=attempt,
        directory=folder,began=time.perf_counter())


def collect_flight(out,flight):
    folder = flight['directory']
    row = read(folder/'comparison_audit.json')
    common.archive.pack(folder)
    with common.archive.restored(folder):
        again = audit(folder)
    for key in ('passed','verdict','frame_collision','ground_collision','actual_gate_count','full_reference_execution_time_s'):
        assert row[key]==again[key]
    reference = np.load(folder/'reference_trajectory.npz')
    source = out/'plans'/f"{flight['level']}_{flight['method']}"/'result.json'
    planned = read(source)
    for key in ('durations','coefficients'):
        np.testing.assert_array_equal(reference[key],planned[key])
    row.update(level=flight['level'],variant=flight['method'],directory=str(folder),
        attempt=flight['attempt'],reference_source_sha256=digest(source))
    print('FLY_RESULT',row['level'],row['variant'],row['verdict'],row['full_reference_execution_time_s'],flush=True)
    return row


def check_sources(protocol):
    assert sources()==protocol['source_sha256'],'production sources changed'


def finish(out,protocol,plans,flights,candidates):
    assert len(plans)==len(flights)==40
    check_sources(protocol)
    assert all(digest(Path(path))==sha for path,sha in protocol['prior_sha256'].items())
    for row in candidates:
        assert digest(Path(row['source']))==row['source_sha256']
    for level in LEVELS:
        validate_initials(out,level)
    records = []
    for row in plans:
        assert digest(Path(row['directory'])/'result.json')==row['result_sha256']
        assert read(Path(row['directory'])/'validation.json')['passed']
    for row in flights:
        if row.get('directory'):
            records.append((row,read(Path(row['directory'])/'result.json')))
    if records:
        assert all(raw['parameters']==records[0][1]['parameters'] and raw['mpc_config']==records[0][1]['mpc_config'] for _,raw in records)
    for level in LEVELS:
        assert len({digest(Path(row['directory'])/'world.sdf') for row,_ in records if row['level']==level})<=1
    report.export(out)
    assert len(read(out/'PLANNING.json'))==len(read(out/'FLIGHT.json'))==50
    save(out/'final_validation.json',dict(passed=True,forty_fresh_plans=True,
        physical_executions=len(records),all_native_records_restored_and_reaudited=True,
        candidate_solves=len(candidates),all_four_way_initials_matched=True,
        all_sources_and_prior_sha256_preserved=True,uniform_world_mpc_px4=True,
        reused_wang_plans=10,reused_wang_flights=10,separate_fifty_row_tables=True))


def main(out,concurrency,resume):
    report.self_check()
    with open('/tmp/convex_matched_full_matrix.lock','a') as lock, open('/tmp/convex_exp2_gazebo_matrix.lock','a') as physical:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        fcntl.flock(physical,fcntl.LOCK_EX|fcntl.LOCK_NB)
        protocol = read(out/'protocol.json') if resume else initialize(out,concurrency)
        assert protocol['concurrency']==concurrency
        check_sources(protocol)
        candidates,plans,flights = [read(out/f'{name}.json') for name in ('candidate_summary','planning_summary','flight_summary')]
        completed = {(row['level'],row['job']):row for row in candidates}
        planned = {(row['level'],row['variant']):row for row in plans}
        flown = {(row['level'],row['variant']):row for row in flights}
        queue = deque()
        for level in LEVELS:
            for index in range(8):
                for method in METHODS:
                    job = read(out/'candidates'/level/'jobs'/f'{method}_{index}.json')
                    if (level,job['id']) not in completed:
                        queue.append(job)
            release = out/'candidates'/level/'jobs/release_best.json'
            if release.exists() and (level,'release_best') not in completed:
                queue.appendleft(read(release))
        active,validators,flying = {},{},None
        began,last_check = time.perf_counter(),0.
        schedule = [(level,method) for i,level in enumerate(LEVELS)
            for method in (METHODS if i%2==0 else tuple(reversed(METHODS)))]
        try:
            while len(plans)<40 or len(flights)<40 or active or queue or flying:
                if time.perf_counter()-last_check>60.:
                    check_sources(protocol)
                    last_check=time.perf_counter()
                while queue and len(active)<concurrency:
                    job=queue.popleft();level=job['level'];key=(level,job['id'])
                    folder=out/'candidates'/level
                    source=folder/'plans'/job['id']/'result.json'
                    if source.exists():
                        row=collect_candidate(out,job,0.,validators)
                        candidates.append(row);completed[key]=row
                        save(out/'candidate_summary.json',candidates)
                        continue
                    if source.parent.exists():
                        raise RuntimeError(f'incomplete candidate retained {source.parent}')
                    log=(out/'logs'/f"{level}_{job['id']}_planning.log").open('w')
                    process=subprocess.Popen([sys.executable,'-u','-m',MODULE+'.worker',
                        '--outdir',str(folder),'--job',job['id']],cwd=common.ROOT,
                        stdout=log,stderr=subprocess.STDOUT,env=os.environ.copy())
                    active[key]=(process,log,job,time.perf_counter())
                    print('CANDIDATE_START',level,job['id'],process.pid,flush=True)
                for key,(process,log,job,started) in list(active.items()):
                    code=process.poll()
                    if code is None:
                        continue
                    log.close();del active[key]
                    if code:
                        raise RuntimeError(f'candidate {key} exited {code}')
                    row=collect_candidate(out,job,time.perf_counter()-started,validators)
                    candidates.append(row);completed[key]=row
                    save(out/'candidate_summary.json',candidates)
                    print('CANDIDATE_RESULT',*key,row['outcome'],row.get('nominal_time_s'),flush=True)
                changed=False
                for level in LEVELS:
                    by={method:[row for row in candidates if row['level']==level and row['method']==method]
                        for method in (*METHODS,'release')}
                    for method in METHODS[1:]:
                        if len(by[method])==8 and (level,method) not in planned:
                            row=publish(out,level,method,by[method]);plans.append(row);planned[(level,method)]=row;changed=True
                            save(out/'planning_summary.json',plans)
                    if len(by['ours'])!=8 or len(by['center'])!=8 or (level,'ours') in planned:
                        continue
                    center=min((row for row in by['center'] if row['passed']),key=lambda row:row['objective'],default=None)
                    release_file=out/'candidates'/level/'jobs/release_best.json'
                    if center is not None and not release_file.exists():
                        save(out/'candidates'/level/'best_center_result.json',read(Path(center['source'])))
                        job=dict(id='release_best',method='release',seed=center['seed'],level=level,
                            center_source=center['source'],center_objective=center['objective'])
                        save(release_file,job);queue.appendleft(job)
                    if center is not None and not by['release']:
                        continue
                    row=publish(out,level,'ours',by['ours']+by['release']);plans.append(row);planned[(level,'ours')]=row;changed=True
                    save(out/'planning_summary.json',plans)
                if flying is not None:
                    code=flying['process'].poll()
                    if code is None and time.perf_counter()-flying['began']>900.:
                        raise RuntimeError('flight worker exceeded wall timeout')
                    if code is not None:
                        flying['log'].close();folder=flying['directory']
                        if code or not (folder/'comparison_audit.json').exists():
                            marker=folder/'reference_activation.json'
                            if flying['attempt']==1 and marker.exists() and not read(marker)['activated']:
                                save(folder/'startup_incident.json',dict(exit_code=code,reference_activated=False))
                                common.archive.pack(folder)
                                flying=start_flight(out,flying['level'],flying['method'],2)
                            else:
                                raise RuntimeError(f'flight evidence failure {folder}, exit {code}')
                        else:
                            row=collect_flight(out,flying);flights.append(row);flown[(row['level'],row['variant'])]=row;flying=None;changed=True
                            save(out/'flight_summary.json',flights)
                if flying is None:
                    for pair in schedule:
                        if pair in flown or pair not in planned:
                            continue
                        if not planned[pair]['found']:
                            row=dict(level=pair[0],variant=pair[1],passed=False,verdict='planning_not_found',
                                frame_collision=False,full_reference_execution_time_s=None,actual_gate_count=None)
                            flights.append(row);flown[pair]=row;changed=True
                            save(out/'flight_summary.json',flights)
                            continue
                        existing=out/'flights'/f'{pair[0]}_{pair[1]}'
                        if existing.exists():
                            raise RuntimeError(f'existing flight evidence needs inspection {existing}')
                        flying=start_flight(out,*pair,1)
                        break
                if changed:
                    report.export(out)
                save(out/'status.json',dict(state='running',pid=os.getpid(),candidate_completed=len(candidates),
                    pending_candidates=len(queue),active_candidates=[dict(level=key[0],job=key[1],pid=value[0].pid) for key,value in active.items()],
                    planning_completed=len(plans),planning_total=40,flight_records=len(flights),flight_total=40,
                    flying=None if flying is None else dict(level=flying['level'],method=flying['method'],pid=flying['process'].pid,attempt=flying['attempt']),
                    elapsed_s=time.perf_counter()-began))
                if len(plans)<40 or len(flights)<40 or active or queue or flying:
                    time.sleep(5.)
            finish(out,protocol,plans,flights,candidates)
            save(out/'status.json',dict(state='complete',pid=os.getpid(),candidate_completed=len(candidates),
                planning_completed=40,flight_records=40,actual_flight_executions=sum(bool(row.get('directory')) for row in flights),elapsed_s=time.perf_counter()-began))
            print('COMPLETE',len(candidates),'candidates',40,'plans',40,'flight records',flush=True)
        except BaseException as error:
            for process,log,_,_ in active.values():
                if process.poll() is None:
                    process.terminate();process.wait(timeout=30)
                log.close()
            if flying is not None and flying['process'].poll() is None:
                flying['process'].send_signal(signal.SIGINT);flying['process'].wait(timeout=30);flying['log'].close()
            save(out/'error.json',dict(error=repr(error),traceback=traceback.format_exc()))
            save(out/'status.json',dict(state='stopped',pid=os.getpid(),planning_completed=len(plans),flight_records=len(flights),failure=repr(error)))
            report.export(out)
            raise


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--outdir',type=Path,required=True)
    parser.add_argument('--concurrency',type=int,default=16,choices=range(1,21))
    parser.add_argument('--resume',action='store_true')
    args=parser.parse_args()
    main(args.outdir.resolve(),args.concurrency,args.resume)
