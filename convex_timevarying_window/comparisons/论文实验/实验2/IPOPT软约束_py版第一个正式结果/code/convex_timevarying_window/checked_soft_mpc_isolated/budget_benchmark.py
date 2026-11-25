"""Identical saved PVA/time/seed under three CPU-accounting arrangements."""
import argparse
from dataclasses import replace
import json
from pathlib import Path
import threading
import time

import numpy as np

from convex_dynamic_seven_window_gazebo.x500_togt.experiment import build_track
from convex_timevarying_window.receding_gate_racing.scenario import vary_track
from convex_timevarying_window.checked_soft_racing.planner import Planner, Config
from convex_timevarying_window.receding_gate_racing.planner import Plan
from convex_timevarying_window.checked_soft_mpc_racing.mpc import MPC, hover_state
from .worker import Worker, pack_plan


class ControlLoad:
    def __init__(self):
        self.controller = MPC()
        self.state = hover_state()
        self.reference = lambda ts: np.tile(np.r_[self.state[:3],np.zeros(6)],(len(ts),1))
        self.controller.command(self.state,0.,self.reference)
        self.error = None
        self.stop = threading.Event()
        self.ready = threading.Event()
        self.thread = threading.Thread(target=self.run)

    def run(self):
        k = 1
        try:
            while not self.stop.is_set():
                began = time.perf_counter()
                self.controller.command(self.state,k*.02,self.reference)
                k += 1
                self.ready.set()
                self.stop.wait(max(0.,.02-(time.perf_counter()-began)))
        except Exception as exc:
            self.error = repr(exc)
            self.ready.set()

    def start(self):
        self.thread.start()
        if not self.ready.wait(2.) or self.error:
            raise RuntimeError(f'MPC benchmark load did not start: {self.error}')

    def close(self):
        self.stop.set(); self.thread.join(2.)
        if self.thread.is_alive() or self.error:
            raise RuntimeError(f'MPC benchmark load failed: {self.error}')


def fixtures(path):
    result = json.loads(path.read_text())
    accepted = [s for s in result['solves'] if s['accepted']]
    selected = []
    for gate in (2,3):
        solves = [s for s in result['solves'] if s['gate']==gate and 'made_at' in s]
        for index in (0,5,10):
            s = solves[index]
            seed = [p for p in accepted if p['made_at']<=s['made_at']-.15+1e-8][-1]
            plan = Plan(seed['made_at'],seed['gate'],np.array(seed['durations']),np.array(seed['coefficients']),
                        np.array(seed['variables']),seed['cycle_ms'],seed['violation'],seed['status'])
            state = np.concatenate([plan.sample(s['made_at'],d) for d in range(3)])
            selected.append(dict(gate=gate,at=s['made_at'],state=state,seed=plan))
    return selected


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--outdir',type=Path,required=True)
    p.add_argument('--source',type=Path,default=Path('convex_timevarying_window/comparisons/checked_soft_mpc_racing/results/frozen_20261006/r1_t2_a1_p1_attempt1/result.json'))
    a = p.parse_args(); a.outdir.mkdir(parents=True,exist_ok=False)
    track,_ = build_track(); vary_track(track,2*np.pi/3,0.,1.,2.,1.)
    cfg = replace(Config(),horizon=2,workspace_lower=(-28.,-30.,.65),workspace_upper=(28.,22.,12.))
    local = Planner(track,cfg); remote = Worker(track,cfg)
    problems = fixtures(a.source); rows = []
    try:
        orders = [('alone','same_process','isolated'),('isolated','same_process','alone'),('same_process','alone','isolated')]
        for repeat,modes in enumerate(orders):
            for mode in modes:
                load = ControlLoad() if mode!='alone' else None
                if load: load.start()
                try:
                    for index,item in enumerate(problems):
                        state,at,gate,seed = (item[k] for k in ('state','at','gate','seed'))
                        if mode=='isolated':
                            remote.commit(seed); remote.submit(state,at,gate,reset=True)
                            response = None
                            deadline = time.monotonic()+2.
                            while response is None and time.monotonic()<deadline:
                                response=remote.poll()
                                if response is None: time.sleep(.0005)
                            if response is None: raise RuntimeError('Isolated benchmark timeout')
                            plan,clearance,elapsed,error = response
                            metrics = remote.metrics[-1]
                        else:
                            local.search=None; local.previous=None; local.dual_cache.clear(); local.collocation_cache={}
                            local.commit(seed)
                            began,cpu,thread = time.perf_counter(),time.process_time(),time.thread_time()
                            plan=local.solve(state,at,gate); clearance=local.prefix_clearance(plan,at)
                            elapsed=1000*(time.perf_counter()-began); error=None
                            metrics=dict(worker_wall_ms=elapsed,worker_process_cpu_ms=1000*(time.process_time()-cpu),
                                         worker_thread_cpu_ms=1000*(time.thread_time()-thread),
                                         iterations=local.solvers[gate][1].stats().get('iter_count'))
                        row=dict(mode=mode,repeat=repeat,fixture=index,gate=gate,at=at,elapsed_ms=elapsed,error=error,
                                 status=plan.status if plan else None,violation=plan.violation if plan else None,
                                 clearance=clearance,accepted=bool(plan and plan.violation<.003 and clearance>=.35 and elapsed<100),**metrics)
                        rows.append(row)
                finally:
                    if load: load.close()
                print(mode,repeat,'done',flush=True)
    finally: remote.close()
    summary = {}
    for mode in ('alone','same_process','isolated'):
        sub=[r for r in rows if r['mode']==mode]
        summary[mode]=dict(count=len(sub),accepted=sum(r['accepted'] for r in sub),
            **{k:dict(mean=float(np.mean([r[k] for r in sub])),max=float(np.max([r[k] for r in sub]))) for k in
               ('elapsed_ms','worker_thread_cpu_ms','worker_process_cpu_ms','iterations')})
    import hashlib
    result=dict(source=str(a.source.resolve()),source_sha256=hashlib.sha256(a.source.read_bytes()).hexdigest(),
                fixtures=[dict(gate=q['gate'],at=q['at'],state=q['state'].tolist(),seed=pack_plan(q['seed'])) for q in problems],
                summary=summary,rows=rows,evidence='Matched inputs with reset warm-start caches; hover MPC load, not PX4 flight.')
    (a.outdir/'result.json').write_text(json.dumps(result,indent=2,default=lambda v:v.tolist())+'\n')
    print(json.dumps(summary,indent=2),flush=True)


if __name__=='__main__': main()
