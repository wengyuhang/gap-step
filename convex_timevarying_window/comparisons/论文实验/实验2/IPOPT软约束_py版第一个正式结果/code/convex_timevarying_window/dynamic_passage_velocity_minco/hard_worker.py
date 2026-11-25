"""Reuse worker transport without changing the repaired or historical workers."""
from . import repaired_worker
from .hard_planner import Planner


def serve(track,cfg,requests,results,ready):
    from convex_timevarying_window.wang_free_state_mpc import worker
    worker.Planner=Planner
    worker._serve(track,cfg,requests,results,ready)


class Worker(repaired_worker.Worker):
    def __init__(self,track,cfg):
        original=repaired_worker.serve
        repaired_worker.serve=serve
        try:super().__init__(track,cfg)
        finally:repaired_worker.serve=original
