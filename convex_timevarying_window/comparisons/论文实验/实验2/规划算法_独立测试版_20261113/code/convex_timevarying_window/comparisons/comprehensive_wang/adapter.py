"""Only choose the predeclared ablation planner inside the isolated worker."""
import multiprocessing as mp
import os
from types import SimpleNamespace
from convex_timevarying_window.comparisons.original_fixed_plane_deadline.adapter import install
from convex_timevarying_window.wang_free_state_mpc.worker import Worker as BaseWorker


def serve(track,cfg,requests,results,ready,variant):
    install()
    from convex_timevarying_window.wang_free_state_mpc import worker as base
    if variant=='no_dense':
        from convex_timevarying_window.comparisons.original_fixed_plane_deadline_no_postcheck.planner import Planner
    else:
        from convex_timevarying_window.crossing_resolved_mpc.planner import Planner
    base.Planner=Planner;base._serve(track,cfg,requests,results,ready)


class Worker(BaseWorker):
    def __init__(self,track,cfg):
        ctx=mp.get_context('spawn');self.requests,self.results,self.ready=(ctx.Queue(1) for _ in range(3))
        variant=os.environ['EXPERIMENT_VARIANT']
        self.process=ctx.Process(target=serve,args=(track,cfg,self.requests,self.results,self.ready,variant),daemon=True)
        self.admitted=None;self.busy=False;self.metrics=[];self.process.start()
        try:
            self.startup=self.ready.get(timeout=600.)
            if 'failure' in self.startup:raise RuntimeError(self.startup['failure'])
        except Exception:self.close();raise
        self.geometry=SimpleNamespace(cfg=cfg,windows=[track.windows[i] for i in track.order],signs=self.startup['signs'])
