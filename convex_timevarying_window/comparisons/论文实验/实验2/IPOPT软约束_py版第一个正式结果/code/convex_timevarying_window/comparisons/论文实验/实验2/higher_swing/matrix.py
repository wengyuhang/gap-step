"""Ten independent higher-swing conditions, five planners, one frozen flight each."""
import argparse
from collections import deque
from dataclasses import asdict
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

from . import scenario, flight, matrix_report as report
from . import run as higher
from .matrix_worker import METHODS, MATCHED_METHODS
from ..matched_full_matrix import run as engine
from ..soft_dynamics import run as common, best_center_release
from ..wang2023 import rollout

HERE = Path(__file__).resolve().parent
MODULE = 'convex_timevarying_window.comparisons.论文实验.实验2.higher_swing'
LEVELS = tuple(scenario.LEVELS)
TOTAL = len(LEVELS)*len(METHODS)
STARTS = dict.fromkeys(MATCHED_METHODS, 8) | {'wang2023': 1}
read, save, digest = common.read, common.save, common.previous.digest


def configure():
    common.scenario = scenario
    best_center_release.scenario = scenario
    rollout.scenario = scenario
    engine.audit = flight.audit


def sources():
    result = engine.sources()
    for path in HERE.rglob('*.py'):
        result[str(path.relative_to(common.ROOT))] = digest(path)
    return result


def protect_previous():
    protected = engine.protect_previous()
    for name in ('matched95_inset8_full_matrix_v1_20261109',
            'higher_swing_a75_f6_v1_20261109', 'higher_swing_a75_f6_view_v1_20261109',
            'higher_swing_a75_f6_view_v2_20261109', 'higher_swing_a75_f6_view_v3_20261109'):
        for path in (common.RESULTS/name).rglob('*'):
            if path.is_file() and path.suffix in ('.json', '.npz', '.gz') and 'source_snapshot' not in path.parts:
                protected[str(path)] = digest(path)
    return protected


def initialize(out, concurrency):
    out.mkdir(parents=True, exist_ok=False)
    (out/'logs').mkdir()
    frozen = sources()
    common.snapshot(out, frozen)
    protocol = dict(source_sha256=frozen, prior_sha256=protect_previous(),
        levels=LEVELS, methods=METHODS, matched_methods=MATCHED_METHODS,
        scenario_variant=scenario.VARIANT, swing_length_m=scenario.SWING_LENGTH,
        new_method_condition_records=TOTAL, starts=STARTS,
        maximum_global_candidates=330, maximum_wang_rollouts=10,
        our_free_independent_starts=8, our_free_additional_best_center_starts=1,
        best_center_release_after_both_independent_branches=True,
        cross_condition_warm_start=False, initial_points='queried window centers',
        sampling_seed=0, initial_pvaj='analytic MINCO', global_initial_time_scaling=False,
        matched_dynamic_limit_fraction=.95, matched_soft_rotor_bounds_n=[0.,8.54858*.95],
        matched_soft_rate_xy_degps=209., matched_soft_rate_z_degps=190.,
        matched_weights=[1000.,1000.], smoothing=.01, tilt_weight=0.,
        fixed_quadrature_intervals=32, passage_margin_m=.15,
        finite_point_inset_radius_m=.6259447197013662,
        our_final_oracle_radius_m=.6209447197013662,
        our_finite_max_iterations=5000, our_safety_exchange_rounds=24,
        togt_solver='original L-BFGS-B past 32 and analytic MINCO adjoint',
        togt_continuous_safety_constraints_added=False, wall_budget_per_global_seed=None,
        wang_reused=False, wang_settings=asdict(rollout.Settings()),
        wang_solver='unchanged verified N2/M4/s4, original four local time starts',
        wang_model='original x500 total-thrust 90% soft bound, norm body-rate 3.8 rad/s',
        wang_reference='own ideal execution with complete-body exit replans, then exact frozen prefix replay through the common MPC',
        wang_frame_safety_terms_added=False, concurrency=concurrency, blas_threads=1,
        flight_repeats=1, activated_reference_retry=False,
        startup_retry_before_reference_activation=1,
        terminal_rule='clock and measurement reach full seven-gate return reference T',
        gate_success='ordered finite aperture membership, forward once, complete body exit',
        frame_contact_is_collision=True, ground_contact_reported_separately=True,
        scene_cases=[scenario.manifest(scenario.Case(level=level)) for level in LEVELS],
        container='convex_seven_togt_experiment',
        image_id=subprocess.check_output(['docker','image','inspect','px4io/px4-sitl-gazebo:latest',
            '--format','{{.Id}}'],text=True).strip())
    save(out/'protocol.json', protocol)
    shutil.copyfile(HERE/'MATRIX_PROTOCOL.md', out/'PROTOCOL.md')
    for name in ('candidate_summary', 'planning_summary', 'flight_summary'):
        save(out/f'{name}.json', [])
    for level in LEVELS:
        folder = out/'candidates'/level
        (folder/'jobs').mkdir(parents=True)
        for method in METHODS:
            for seed in range(STARTS[method]):
                job = dict(id=f'{method}_{seed}', method=method, seed=seed, level=level)
                save(folder/'jobs'/f"{job['id']}.json", job)
    report.export(out)
    return protocol


def collect_candidate(out, job, wall, validators):
    if job['method'] != 'wang2023':
        return engine.collect_candidate(out, job, wall, validators)
    source = out/'candidates'/job['level']/'plans'/job['id']/'result.json'
    raw = read(source)
    assert raw['scenario_variant'] == scenario.VARIANT
    assert raw['case']['level'] == job['level']
    row = dict(job=job['id'], method='wang2023', seed=0, level=job['level'],
        found=raw['found'], passed=raw['found'],
        outcome='参考任务完成' if raw['found'] else '未找到合格规划',
        source=str(source), source_sha256=digest(source),
        nominal_time_s=raw['nominal_time_s'] if raw['found'] else None,
        objective=None, solve_s=raw['solve_s'], build_s=raw['build_s'],
        observed_process_wall_s=wall, request_count=len(raw['requests']),
        local_candidate_count=sum(len(request['candidates']) for request in raw['requests']))
    return row


def publish(out, level, method, rows):
    if method != 'wang2023':
        return engine.publish(out, level, method, rows)
    assert len(rows) == 1
    source = Path(rows[0]['source'])
    raw = read(source)
    folder = out/'plans'/f'{level}_{method}'
    folder.mkdir(parents=True, exist_ok=False)
    save(folder/'result.json', dict(raw, candidate_records=rows,
        selected_candidate_job=rows[0]['job'] if raw['found'] else None,
        selected_branch='wang2023' if raw['found'] else None))
    if raw['found']:
        shutil.copyfile(source.parent/'attitude_audit.json', folder/'attitude_audit.json')
    row = common.assess(out, level, method)
    row.update(request_count=len(raw['requests']),
        local_candidate_count=sum(len(request['candidates']) for request in raw['requests']),
        reference_task_completed=raw['found'], failure=raw['failure'])
    save(folder/'validation.json', dict(passed=True, all_local_candidates_preserved=True,
        original_wang_solver_and_settings=True, scenario_variant=scenario.VARIANT,
        geometry_outcome=row['outcome']))
    print('PLAN_RESULT', level, method, row['outcome'], row['full_trajectory_time_s'], flush=True)
    return row


def start_flight(out, level, method, attempt):
    if shutil.disk_usage(common.ROOT).free < 3*2**30:
        raise RuntimeError('insufficient disk for native flight evidence')
    folder = out/'flights'/f'{level}_{method}'/f'attempt_{attempt}'
    folder.parent.mkdir(parents=True, exist_ok=True)
    if folder.exists():
        raise RuntimeError(f'existing flight attempt requires inspection {folder}')
    log = (out/'logs'/f'{level}_{method}_flight_{attempt}.log').open('w')
    process = subprocess.Popen([sys.executable,'-u','-m',MODULE+'.matrix_flight',
        '--source',str(out/'plans'/f'{level}_{method}'/'result.json'),
        '--method',method,'--outdir',str(folder)],cwd=common.ROOT,
        stdout=log,stderr=subprocess.STDOUT,env=os.environ.copy())
    print('FLY_START',level,method,attempt,process.pid,flush=True)
    return dict(process=process,log=log,level=level,method=method,attempt=attempt,
        directory=folder,began=time.perf_counter())


collect_flight = engine.collect_flight


def check_sources(protocol):
    assert sources() == protocol['source_sha256'], 'production sources changed'


def finish(out, protocol, plans, flights, candidates):
    assert len(plans) == len(flights) == TOTAL
    check_sources(protocol)
    assert all(digest(Path(path))==sha for path,sha in protocol['prior_sha256'].items())
    for row in candidates:
        assert digest(Path(row['source'])) == row['source_sha256']
    for level in LEVELS:
        engine.validate_initials(out, level)
    records = []
    for row in plans:
        assert digest(Path(row['directory'])/'result.json') == row['result_sha256']
        assert read(Path(row['directory'])/'validation.json')['passed']
        assert read(Path(row['directory'])/'result.json')['scenario_variant'] == scenario.VARIANT
    for row in flights:
        if row.get('directory'):
            records.append((row,read(Path(row['directory'])/'result.json')))
    if records:
        assert all(raw['parameters']==records[0][1]['parameters'] and raw['mpc_config']==records[0][1]['mpc_config'] for _,raw in records)
    for level in LEVELS:
        assert len({digest(Path(row['directory'])/'world.sdf') for row,_ in records if row['level']==level}) <= 1
        assert all(read(Path(row['directory'])/'scene.json') == scenario.manifest(scenario.Case(level=level))
            for row,_ in records if row['level'] == level)
    report.export(out)
    assert len(read(out/'PLANNING.json')) == len(read(out/'FLIGHT.json')) == TOTAL
    save(out/'final_validation.json', dict(passed=True, new_planning_records=TOTAL,
        physical_executions=len(records), all_native_records_restored_and_reaudited=True,
        candidate_jobs=len(candidates), matched_four_way_initial_curves=True,
        all_sources_and_prior_sha256_preserved=True, uniform_world_mpc_px4=True,
        wang_recomputed_conditions=10, wang_history_reused=False,
        separate_fifty_row_tables=True))


def main(out,concurrency,resume):
    configure()
    report.self_check()
    with open('/tmp/convex_higher_swing_matrix.lock','a') as lock, open('/tmp/convex_exp2_gazebo_matrix.lock','a') as physical:
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
            for method in METHODS:
                for index in range(STARTS[method]):
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
            while len(plans)<TOTAL or len(flights)<TOTAL or active or queue or flying:
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
                    process=subprocess.Popen([sys.executable,'-u','-m',MODULE+'.matrix_worker',
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
                        if len(by[method])==STARTS[method] and (level,method) not in planned:
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
                    planning_completed=len(plans),planning_total=TOTAL,flight_records=len(flights),flight_total=TOTAL,
                    flying=None if flying is None else dict(level=flying['level'],method=flying['method'],pid=flying['process'].pid,attempt=flying['attempt']),
                    elapsed_s=time.perf_counter()-began))
                if len(plans)<TOTAL or len(flights)<TOTAL or active or queue or flying:
                    time.sleep(5.)
            finish(out,protocol,plans,flights,candidates)
            save(out/'status.json',dict(state='complete',pid=os.getpid(),candidate_completed=len(candidates),
                planning_completed=TOTAL,flight_records=TOTAL,actual_flight_executions=sum(bool(row.get('directory')) for row in flights),elapsed_s=time.perf_counter()-began))
            print('COMPLETE',len(candidates),'candidate jobs',TOTAL,'plans',TOTAL,'flight records',flush=True)
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
