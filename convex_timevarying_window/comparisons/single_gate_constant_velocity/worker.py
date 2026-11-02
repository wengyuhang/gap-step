import multiprocessing as mp
import time
import os
from dataclasses import replace
import numpy as np
from types import SimpleNamespace
from convex_timevarying_window.wang_free_state_mpc.worker import Worker as Parent
from .planner import Planner


def serve(track,cfg,requests,results,ready):
    try:
        began=time.perf_counter(); planner=Planner(track,cfg)
        build_s=time.perf_counter()-began
        # Equal nominal head/time, separate offline diagnostic, never an online seed.
        head=np.r_[track.start,np.zeros(9)];at=.15
        diagnostic_budget = None if cfg.max_solve_ms is None else 5000.
        planner.cfg=replace(cfg,max_solve_ms=diagnostic_budget)
        started=time.perf_counter();diagnostic=dict(budget_ms=diagnostic_budget,made_at=at,pvaj=head.tolist(),
            offline_only=True,used_as_online_seed=False)
        try:
            plan=planner.solve(head,at,0)
            diagnostic.update(found=True,durations=plan.durations.tolist(),variables=plan.x.tolist(),
                coefficients=plan.coefficients.tolist())
        except Exception as exc:diagnostic.update(found=False,error=repr(exc))
        diagnostic.update(elapsed_ms=1000*(time.perf_counter()-started),stats=planner.solvers[0][0].stats())
        planner.cfg=cfg;planner.admitted=None
        ready.put(dict(pid=os.getpid(),build_s=build_s,signs=planner.signs,offline_diagnostic=diagnostic))
    except Exception as exc: ready.put(dict(failure=repr(exc))); return
    while True:
        request=requests.get()
        if request is None:return
        state,at,gate,admitted,submitted=request
        planner.commit(admitted); began=time.perf_counter(); cpu=time.process_time()
        plan=None;error=None
        try:plan=planner.solve(state,at,gate)
        except Exception as exc:error=repr(exc)
        metrics=dict(worker_wall_ms=1000*(time.perf_counter()-began),
            worker_process_cpu_ms=1000*(time.process_time()-cpu),queue_ms=1000*(began-submitted),
            worker_pid=os.getpid(),optimizer_stats=planner.solvers[gate][0].stats())
        results.put((plan,None,error,submitted,metrics))


class Worker(Parent):
    def __init__(self,track,cfg):
        context=mp.get_context('spawn')
        self.requests,self.results,self.ready=(context.Queue(1) for _ in range(3))
        self.process=context.Process(target=serve,args=(track,cfg,self.requests,self.results,self.ready),daemon=True)
        self.admitted=None;self.busy=False;self.metrics=[];self.process.start()
        try:
            self.startup=self.ready.get(timeout=600.)
            if 'failure' in self.startup:raise RuntimeError(self.startup['failure'])
        except Exception:self.close();raise
        self.geometry=SimpleNamespace(cfg=cfg,windows=list(track.windows),signs=self.startup['signs'])
