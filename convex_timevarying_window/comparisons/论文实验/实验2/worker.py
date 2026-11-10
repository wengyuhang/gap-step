"""Only external Wang baseline uses its original local optimization process."""
import multiprocessing as mp
from types import SimpleNamespace
from convex_timevarying_window.wang_free_state_mpc.worker import Worker as Parent
from convex_timevarying_window.wang_free_state_mpc import worker as base

def serve(track,cfg,requests,results,ready):
    from .baseline import Planner
    base.Planner=Planner;base._serve(track,cfg,requests,results,ready)

class Worker(Parent):
    def __init__(self,track,cfg):
        ctx=mp.get_context('spawn');self.requests,self.results,self.ready=(ctx.Queue(1) for _ in range(3))
        self.process=ctx.Process(target=serve,args=(track,cfg,self.requests,self.results,self.ready),daemon=True)
        self.admitted=None;self.busy=False;self.metrics=[];self.process.start()
        try:
            self.startup=self.ready.get(timeout=1800.)
            if 'failure' in self.startup:raise RuntimeError(self.startup['failure'])
        except Exception:self.close();raise
        self.geometry=SimpleNamespace(cfg=cfg,windows=list(track.windows),signs=self.startup['signs'])
