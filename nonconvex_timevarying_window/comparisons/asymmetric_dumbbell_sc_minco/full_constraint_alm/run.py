"""Frozen same-NLP single-left-start comparison, including failed outcomes."""
import argparse
from dataclasses import asdict
from datetime import datetime
import hashlib
from pathlib import Path
import time

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from shapely.geometry import Polygon

from convex_timevarying_window.single_gate_stop_racing.planner import Config
from nonconvex_timevarying_window.sc_dynatogt.sc_mapping import SCDiskMap
from ..scenario import VERTICES, LEFT_INITIAL_POINT, Case, build_case, room
from ..run import save_json
from ..planner import recenter_at_left
from ..analyze import audit_plan
from ..single_seed_poll.solver import Planner as LocalPlanner
from .solver import Planner as ALMPlanner


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--outdir', type=Path, required=True)
    parser.add_argument('--budget-ms', type=float, default=30000.)
    parser.add_argument('--rho', type=float, default=1.)
    parser.add_argument('--outer', type=int, default=20)
    parser.add_argument('--inner', type=int, default=100)
    parser.add_argument('--normalization', choices=['original','left'], default='original')
    args = parser.parse_args()
    args.outdir.mkdir(parents=True, exist_ok=False)
    root = Path(__file__).resolve().parents[4]
    parent = Path(__file__).resolve().parents[1]
    paths = list(Path(__file__).parent.glob('*.py'))+list(parent.glob('*.py'))
    paths += [parent/'single_seed_poll/solver.py']
    paths += [root/p for p in ['nonconvex_timevarying_window/sc_dynatogt/sc_mapping.py',
            'convex_timevarying_window/single_gate_stop_racing/planner.py',
            'convex_timevarying_window/single_gate_stop_racing/geometry.py',
            'convex_timevarying_window/single_gate_stop_racing/local_bounds.py',
            'convex_timevarying_window/dynamic_passage_velocity_minco/hard_dynamics.py',
            'convex_timevarying_window/dynamic_passage_velocity_minco/hard_planner.py',
            'convex_timevarying_window/dynamic_passage_velocity_minco/trajectory.py']]
    hashes = {str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
    cfg, case = Config(compile_objective=False, max_solve_ms=args.budget_ms), Case(3., 2.)
    began = time.perf_counter()
    mapping = SCDiskMap.fit(VERTICES)
    if args.normalization == 'left':
        mapping = recenter_at_left(mapping)
    fit_ms = 1000*(time.perf_counter()-began)
    mapping.save(args.outdir/'sc_map.npz')
    save_json(args.outdir/'manifest.json', dict(created_at=datetime.now().isoformat(), config=asdict(cfg),
        case=asdict(case), source_sha256=hashes, offline_fit_ms=fit_ms, arguments=vars(args)|{'outdir':str(args.outdir)},
        protocol='Identical single left state, same original full dynamic NLP. Only solver changes.',
        methods=['local','alm','nonlocal_poll'], flight_executed=False,
        dynamics_scope='Original 17 nodes/piece, not continuous dynamics certification'))
    rows=[]
    initial_reference=None
    for name in ['local', 'alm', 'nonlocal_poll']:
        track=build_case(case)
        folder=args.outdir/name
        folder.mkdir()
        cls=ALMPlanner if name=='alm' else LocalPlanner
        started=time.perf_counter()
        planner=cls(track,cfg,mapping)
        build_ms=1000*(time.perf_counter()-started)
        print({'stage':'built','method':name,'build_ms':build_ms},flush=True)
        row=dict(method=name,accepted=False,offline_build_ms=build_ms,flight_executed=False,
                 explicit_nonlocal_search=name=='nonlocal_poll')
        try:
            state=np.r_[track.start,np.zeros(9)]
            if name=='alm':
                plan=planner.solve(state,rho0=args.rho,outer_iterations=args.outer,inner_iterations=args.inner)
            else:
                plan=planner.solve(state,escape=name=='nonlocal_poll')
            row.update(accepted=True,final=dict(q=mapping.map_unconstrained(plan.x[2:4]),
                room=room(mapping.map_unconstrained(plan.x[2:4])),
                nominal_total_time_s=float(plan.durations.sum()),crossing_time_s=float(plan.crossing_times[0])))
            np.savez(folder/'plan.npz',coefficients=plan.coefficients,durations=plan.durations,x=plan.x)
            row['posthoc_audit']=audit_plan(folder,case,cfg.radius+cfg.frame_thickness+cfg.passage_margin)
        except RuntimeError as exc:
            row['failure']=str(exc)
        row['stats']=planner.solvers[0][0].stats()
        row['evaluations']=planner.evaluations
        row['points_outside_physical_polygon']=sum(not track.windows[0].aperture.contains(v['q'],tolerance=1e-7)
                                                  for v in planner.evaluations)
        init=np.asarray(row['stats']['initial_variables'])
        if initial_reference is None:
            initial_reference=init.copy()
        np.testing.assert_array_equal(init,initial_reference)
        np.testing.assert_allclose(mapping.map_unconstrained(init[2:4]),LEFT_INITIAL_POINT,atol=1e-7)
        # Same raw objective, all constraints and analytic Jacobian at the common initial state.
        offsets=np.tile(np.r_[cfg.passage_seed_offsets,np.zeros(cfg.witness_slots-len(cfg.passage_seed_offsets))],(2,1))
        raw=planner.function(init,np.r_[0.,track.start,np.zeros(9),offsets.ravel()])
        signature=np.r_[np.asarray(raw[0]).ravel(),np.asarray(raw[1]).ravel(),np.asarray(raw[2]).ravel(),np.asarray(raw[3]).ravel()]
        row['constraint_count']=int(np.asarray(raw[2]).size)
        row['initial_model_sha256']=hashlib.sha256(signature.tobytes()).hexdigest()
        if rows:
            assert row['initial_model_sha256']==rows[0]['initial_model_sha256']
        save_json(folder/'result.json',row)
        rows.append(row)
        save_json(args.outdir/'summary.json',rows)
        print({'method':name,'accepted':row['accepted'],'final':row.get('final'),
               'status':row['stats']['return_status'],'solve_ms':row['stats']['optimization_ms'],
               'last_round':row['stats']['rounds'][-1] if row['stats']['rounds'] else None},flush=True)
    fig,axes=plt.subplots(1,3,figsize=(15,4.8),sharex=True,sharey=True)
    outline=np.vstack([VERTICES,VERTICES[0]])
    eroded=Polygon(VERTICES).buffer(-(cfg.radius+cfg.frame_thickness+cfg.passage_margin))
    for ax,row in zip(axes,rows):
        ax.fill(*outline.T,color='#eeeeee'); ax.plot(*outline.T,'k-',lw=1.4)
        for part in eroded.geoms:
            ax.fill(*np.asarray(part.exterior.coords).T,color='#b5d9bd')
        q=np.array([a['q'] for a in row['stats']['trace']])
        # Scatter only: consecutive physical iterate chords need not lie in the polygon.
        ax.scatter(*q.T,s=8,c=np.arange(len(q)),cmap='viridis',alpha=.7)
        ax.scatter(*LEFT_INITIAL_POINT,marker='*',s=150,c='#2166ac',zorder=5)
        if row['accepted']:
            ax.scatter(*row['final']['q'],marker='o',s=70,c='#b2182b',zorder=6)
        ax.set(title=f"{row['method']}: {'accepted by original checks' if row['accepted'] else 'not accepted'}",
               xlabel='Gate-local horizontal (m)',ylabel='Gate-local vertical (m)',aspect='equal')
        ax.grid(alpha=.2)
    fig.suptitle('Original full dynamic NLP, identical single left initial guess\n'
                 'Dots are solver iterates, NOT flight; no continuous dynamics certificate',fontsize=11)
    fig.tight_layout();fig.savefig(args.outdir/'comparison.png',dpi=170);plt.close(fig)
    assert all(hashlib.sha256((root/p).read_bytes()).hexdigest()==h for p,h in hashes.items())
    print('FINISHED '+str(args.outdir),flush=True)


if __name__=='__main__':
    main()
