from convex_timevarying_window.comparisons.single_gate_swing.worker import Worker as Parent


class Worker(Parent):
    """Use the current unlimited worker with the existing affine planner."""
    def __init__(self, track, cfg):
        import multiprocessing as mp
        from types import SimpleNamespace
        context=mp.get_context('spawn')
        self.requests,self.results,self.ready=(context.Queue(1) for _ in range(3))
        self.process=context.Process(target=serve,args=(track,cfg,self.requests,self.results,self.ready),daemon=True)
        self.admitted=None;self.busy=False;self.metrics=[];self.process.start()
        try:
            self.startup=self.ready.get(timeout=None)
            if 'failure' in self.startup:raise RuntimeError(self.startup['failure'])
        except Exception:self.close();raise
        self.geometry=SimpleNamespace(cfg=cfg,windows=list(track.windows),signs=self.startup['signs'])


def serve(*args):
    from convex_timevarying_window.comparisons.single_gate_constant_velocity import worker
    from .planner import Planner
    worker.Planner=Planner
    worker.serve(*args)
