import multiprocessing as mp
from types import SimpleNamespace
from convex_timevarying_window.comparisons.single_gate_constant_velocity.worker import Worker as Parent
from .planner import Planner

def serve(*args):
    from convex_timevarying_window.comparisons.single_gate_constant_velocity import worker
    worker.Planner=Planner;worker.serve(*args)

class Worker(Parent):
    def __init__(self,track,cfg):
        context=mp.get_context('spawn');self.requests,self.results,self.ready=(context.Queue(1) for _ in range(3))
        self.process=context.Process(target=serve,args=(track,cfg,self.requests,self.results,self.ready),daemon=True)
        self.admitted=None;self.busy=False;self.metrics=[];self.process.start()
        try:
            self.startup=self.ready.get(timeout=600)
            if 'failure' in self.startup:raise RuntimeError(self.startup['failure'])
        except Exception:self.close();raise
        self.geometry=SimpleNamespace(cfg=cfg,windows=list(track.windows),signs=self.startup['signs'])
