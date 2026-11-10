"""Independent source, initialization, frozen-reference and actual-event checks."""
import argparse
from pathlib import Path
import shutil

import numpy as np
from . import planning, run, scenario
from ..实验2.basic_group.trajectory_only.collision import evaluate


def validate(out):
    protocol=planning.read(out/'protocol.json')
    flights_protocol=planning.read(out/'flight_protocol.json')
    sources=dict(protocol['source_sha256'],**flights_protocol['source_sha256'])
    for file,value in sources.items():
        assert run.digest(file)==value,file
    selected=planning.read(out/'selected.json')
    rows=planning.read(out/'flight_summary.json')
    paired=[]
    for level in selected:
        for seed in range(8):
            initials={method:planning.read(out/'plans'/f'l{int(level):02d}_{method}_{seed}'/'initial.json')
                for method in planning.LABELS}
            reference=initials['center']
            for method,row in initials.items():
                np.testing.assert_allclose(row['coefficients'],reference['coefficients'],rtol=0.,atol=1e-8)
                np.testing.assert_array_equal(row['durations_s'],reference['durations_s'])
                np.testing.assert_allclose(row['points'],reference['points'],rtol=0.,atol=1e-12)
                assert row['point_variables']==(4 if method in ('ours','instant') else 0)
                assert not row['optimized_initial_solution_used']
            paired.append(dict(level=int(level),seed=seed,matched_initial_curve=True))
    plan_checks=[]
    for level,methods in selected.items():
        track,_=scenario.build_case(scenario.Case(int(level)))
        for method,row in methods.items():
            if row is None:
                continue
            raw=planning.read(row['source'])
            result=evaluate(raw['coefficients'],raw['durations'],track,scenario.ASSETS/'meshes')
            assert result['verdict']==raw['geometry_verdict']
            if method in ('ours','center'):
                assert result['passed']
            if method in ('center','togt_center'):
                for point,at,window in zip(raw['crossing_points'],raw['crossing_times_s'],track.windows):
                    np.testing.assert_allclose(point,window.state_at(at)[0],rtol=0.,atol=1e-8)
                assert raw['point_variables']==0
            plan_checks.append(dict(level=int(level),method=method,
                independent_mesh_verdict=result['verdict'],strict_center_checked=method in ('center','togt_center')))
    actual_checks=[]
    parameter_reference=None
    mpc_reference=None
    for row in rows:
        folder=Path(row['folder'])
        raw=planning.read(folder/'result.json')
        reference=np.load(folder/'reference_trajectory.npz')
        planned=planning.read(selected[str(row['level'])][row['method']]['source'])
        np.testing.assert_array_equal(reference['durations'],planned['durations'])
        np.testing.assert_array_equal(reference['coefficients'],planned['coefficients'])
        if parameter_reference is None:
            parameter_reference=raw['parameters']
            mpc_reference=raw['mpc_config']
        assert raw['parameters']==parameter_reference
        assert raw['mpc_config']==mpc_reference
        truth=np.load(folder/'truth_trace.npz')
        t,p=truth['times'],truth['positions']
        track,_=scenario.build_case(scenario.Case(row['level']))
        window=track.windows[0]
        events=[]
        x=p[:,0]-window.center0[0]
        for index in np.flatnonzero(x[:-1]*x[1:]<0.):
            fraction=-x[index]/(x[index+1]-x[index])
            at=float(t[index]+fraction*(t[index+1]-t[index]))
            point=p[index]+fraction*(p[index+1]-p[index])
            q,_=window.world_to_local(point,at)
            if window.aperture.contains(q):
                events.append(dict(time_s=at,forward=bool(x[index+1]>x[index]),local=q.tolist()))
        if row['passed']:
            assert len(events)==1 and events[0]['forward']
            assert np.any((t>events[0]['time_s'])&(x>.3859447197013662+.085))
            assert raw['flight_time_s']>=raw['nominal_time_s']
            assert raw['finish_measurement_time_s']>=raw['nominal_time_s']
            assert not row['frame_collision']
        assert planning.read(folder/'validation.json')['passed']
        actual_checks.append(dict(level=row['level'],method=row['method'],actual_events=events,
            complete=row['passed'],frame_collision=row['frame_collision'],ground_collision=row['ground_collision'],
            reference_arrays_equal=True,parameters_equal=True,mpc_equal=True))
    for file,value in sources.items():
        target=out/'source_snapshot'/Path(file).relative_to(scenario.ROOT)
        target.parent.mkdir(parents=True,exist_ok=True)
        shutil.copyfile(file,target)
        assert run.digest(target)==value
    planning.save(out/'posthoc_validation.json',dict(passed=True,source_sha256_passed=True,
        paired_initials=paired,selected_plans=plan_checks,actual_flights=actual_checks,
        all_reference_arrays_equal=True,all_mpc_and_px4_parameters_equal=True,
        no_endpoint_threshold=True,source_snapshot=True))
    print('INDEPENDENT_VALIDATION_PASSED',len(paired),len(plan_checks),len(actual_checks),flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--outdir',type=Path,required=True)
    args=parser.parse_args()
    validate(args.outdir.resolve())
