"""Isolated free-terminal planner with the frozen original-layout scene queries."""
import multiprocessing as mp
from types import SimpleNamespace
from convex_timevarying_window.wang_free_state_mpc.worker import Worker as BaseWorker
from convex_timevarying_window.comparisons.original_fixed_plane_ours import adapter


def install_scene_queries():
    from . import planner
    adapter.install_scene_queries()
    planner.symbolic_pose = adapter.symbolic_pose
    planner.symbolic_clearance_sq = adapter.symbolic_clearance_sq


def serve(track, cfg, requests, results, ready):
    install_scene_queries()
    from convex_timevarying_window.wang_free_state_mpc import worker as base
    from .planner import Planner
    base.Planner = Planner
    base._serve(track, cfg, requests, results, ready)


class Worker(BaseWorker):
    def __init__(self, track, cfg):
        ctx = mp.get_context('spawn')
        self.requests, self.results, self.ready = (ctx.Queue(1) for _ in range(3))
        self.process = ctx.Process(target=serve, args=(track,cfg,self.requests,self.results,self.ready), daemon=True)
        self.admitted = None; self.busy = False; self.metrics = []; self.process.start()
        try:
            self.startup = self.ready.get(timeout=600.)
            if 'failure' in self.startup: raise RuntimeError(self.startup['failure'])
        except Exception:
            self.close(); raise
        self.geometry = SimpleNamespace(cfg=cfg, windows=[track.windows[i] for i in track.order], signs=self.startup['signs'])
