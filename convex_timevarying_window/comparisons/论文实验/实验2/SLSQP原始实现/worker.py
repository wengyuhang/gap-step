from pathlib import Path
import argparse, importlib, json, sys, time, traceback
from dataclasses import asdict
import numpy as np

PACKAGE=Path(__file__).resolve().parent
sys.path.insert(0,str(PACKAGE/'code'))
base=importlib.import_module('convex_timevarying_window.comparisons.论文实验.实验2')
base.__path__.append(str(PACKAGE/'adapters'))
from convex_timevarying_window.comparisons.论文实验.实验2.higher_swing import scenario
from convex_timevarying_window.comparisons.论文实验.实验2.independent_free_solve.planner import Planner
from convex_timevarying_window.single_gate_stop_racing import point_mapping
from convex_timevarying_window.togt_minco_mpc.trajectory import basis
from convex_timevarying_window.comparisons.论文实验.实验2.basic_group.trajectory_only.collision import evaluate


def save(path,value):
    def convert(x):
        if hasattr(x,'tolist'):return x.tolist()
        if isinstance(x,Path):return str(x)
        raise TypeError(type(x).__name__)
    temporary=path.with_suffix(path.suffix+'.writing')
    temporary.write_text(json.dumps(value,ensure_ascii=False,indent=2,default=convert)+'\n')
    temporary.replace(path)


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--cases',type=Path,required=True)
    parser.add_argument('--level',required=True)
    parser.add_argument('--method',choices=('ours','center'),required=True)
    parser.add_argument('--seed',type=int,required=True)
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--validate-only',action='store_true')
    args=parser.parse_args();args.out.mkdir(parents=True,exist_ok=False)
    began=time.perf_counter()
    meta=json.loads((args.cases/'inputs'/f'{args.level}_{args.method}'/'manifest.json').read_text())
    original=np.asarray(meta['initials'][args.seed]['original'],dtype=float)
    track,_=scenario.build_case(scenario.Case(level=args.level))
    planner=Planner(track,args.method=='ours',pose_function=scenario.pose)
    vs=planner.velocity_start;n=planner.n;m=planner.m
    point_parameters=np.concatenate([point_mapping.initial(w.aperture) for w in track.windows]) if args.method=='ours' else np.empty(0)
    initial=np.r_[original[:n],point_parameters,original[meta['velocity_start']:]]
    assert initial.shape==(planner.nvar,)
    np.testing.assert_array_equal(initial[:n],original[:n])
    np.testing.assert_array_equal(initial[vs:],original[meta['velocity_start']:])
    times=np.exp(initial[:n]);arrivals=np.cumsum(times)
    expected=[]
    for w,t in zip(meta['windows'],arrivals):
        phase=2*np.pi*w['frequency']*t+w['phase']
        if w['swing']:
            angle=np.arcsin(w['amplitude']/w['length'])*np.sin(phase)
            local=np.array([w['length']*(1-np.cos(angle)),-w['length']*np.sin(angle),0.])
        else:local=np.array([0.,-w['amplitude']*np.sin(phase),0.])
        expected.append(np.asarray(w['center'])+np.asarray(w['rotation'])@local)
    for window,reference in zip(track.windows,meta['windows']):
        np.testing.assert_allclose(window.R,reference['rotation'],atol=1e-10,rtol=0)
        np.testing.assert_allclose(window.center0,reference['center'],atol=1e-10,rtol=0)
        assert (window.aperture.kind=='circle')==reference['circle']
        if reference['circle']:
            np.testing.assert_allclose(window.aperture.radius,reference['opening_radius'],atol=1e-10,rtol=0)
        else:
            np.testing.assert_allclose(window.aperture.physical_vertices,reference['vertices'],atol=1e-10,rtol=0)
    points=np.asarray(planner.point_function(initial)).T
    point_error=float(np.max(abs(points-np.asarray(expected))))
    raw=planner.curve(initial)[0]
    coefficients=np.asarray(raw).ravel().reshape(n,3,8).transpose(0,2,1)
    derivative_error=0.
    for order in (1,2,3):
        for gate in range(m):
            actual=coefficients[gate].T@np.asarray(basis(1.,order)).ravel()/times[gate]**order
            target=initial[vs+3*((order-1)*m+gate):vs+3*((order-1)*m+gate+1)]
            derivative_error=max(derivative_error,float(np.max(abs(actual-target))))
    if point_error>1e-8 or derivative_error>1e-6:raise RuntimeError('Physical initial trajectory mismatch')
    initial_audit=dict(passed=True,seed=args.seed,level=args.level,method=args.method,
        original_source='independent_full_matrix_v2_20261107',initial_variables=initial,
        times_identical=True,VAJ_identical=True,point_error_m=point_error,
        knot_derivative_error=derivative_error,nvar=planner.nvar,
        nodes_per_piece=planner.cfg.nodes_per_piece+1,thrust_fraction=planner.cfg.physical_thrust_fraction)
    save(args.out/'INITIAL_VALIDATION.json',initial_audit)
    if args.validate_only:return
    build_s=time.perf_counter()-began
    planner.seeds=lambda:[initial.copy()]
    with (args.out/'progress.jsonl').open('w') as log:
        def progress(row):log.write(json.dumps(row,ensure_ascii=False,default=lambda x:x.tolist())+'\n');log.flush()
        plan,statistics=planner.solve_all(seconds_per_seed=300.,progress=progress)
    statistics['historical_initialization_label']=statistics.get('initialization')
    statistics['initialization']='current frozen physical initial seed '+str(args.seed)
    statistics['all_four_attempted']=False
    statistics['worker_initials_attempted']=1
    geometry=None
    if plan is not None:
        geometry=evaluate(plan.coefficients,plan.durations,track,scenario.ASSETS/'meshes')
        save(args.out/'assessment.json',geometry)
    window_passed=bool(geometry is not None and all(c['outcome']=='clear' for c in geometry['checks'])
                       and geometry['ordered_exactly_once'] and geometry['endpoints_passed'])
    result=dict(found=plan is not None,passed=window_passed,level=args.level,method=args.method,
        seed=args.seed,build_s=build_s,process_wall_s=time.perf_counter()-began,
        statistics=statistics,initial_validation=initial_audit,config=asdict(planner.cfg),
        nominal_time_s=float(plan.durations.sum()) if plan is not None else None,
        historical_solver_class=Planner.__module__+'.Planner',seconds_per_seed=300.,
        center_release=False,number_current_initials=8)
    if plan is not None:result.update(durations=plan.durations,coefficients=plan.coefficients,variables=plan.x,
        ground=geometry.get('ground',[]))
    save(args.out/'result.json',result)
    print('COMPLETE',args.level,args.method,args.seed,result['passed'],result['nominal_time_s'],flush=True)

if __name__=='__main__':
    try:main()
    except Exception:
        traceback.print_exc();sys.exit(1)
