"""Persistent isolated worker for the center-initialized, no-local-stop planner."""
import multiprocessing as mp
from types import SimpleNamespace

from convex_timevarying_window.wang_free_state_mpc.worker import Worker as BaseWorker


def serve(track,cfg,requests,results,ready):
    from convex_timevarying_window.wang_free_state_mpc import worker as base
    from .planner import Planner
    base.Planner=Planner
    base._serve(track,cfg,requests,results,ready)


class Worker(BaseWorker):
    def __init__(self,track,cfg):
        context=mp.get_context('spawn')
        self.requests=context.Queue(1);self.results=context.Queue(1);self.ready=context.Queue(1)
        self.process=context.Process(target=serve,args=(track,cfg,self.requests,self.results,self.ready),daemon=True)
        self.admitted=None;self.busy=False;self.metrics=[];self.process.start()
        try:
            self.startup=self.ready.get(timeout=600.)
            if 'failure' in self.startup: raise RuntimeError(self.startup['failure'])
        except Exception:
            self.close();raise
        self.geometry=SimpleNamespace(cfg=cfg,windows=[track.windows[i] for i in track.order],signs=self.startup['signs'])
