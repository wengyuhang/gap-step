"""Persistent isolated MINCO planner; typed six-coefficient IPC, no fitting."""
import multiprocessing as mp
import os
import queue
import time
from types import SimpleNamespace
import numpy as np
from .planner import Planner


def _serve(track,cfg,requests,results,ready):
    try:
        began=time.perf_counter();planner=Planner(track,cfg)
        ready.put(dict(pid=os.getpid(),build_s=time.perf_counter()-began,signs=planner.signs))
    except Exception as exc:ready.put(dict(failure=repr(exc)));return
    while True:
        request=requests.get()
        if request is None:return
        state,at,gate,admitted,submitted=request
        planner.commit(admitted);began=time.perf_counter();cpu=time.process_time()
        plan,clearance,error=None,None,None
        try:
            plan=planner.solve(state,at,gate);clearance=planner.prefix_clearance(plan,at)
        except Exception as exc:error=repr(exc)
        metrics=dict(worker_wall_ms=1000*(time.perf_counter()-began),worker_process_cpu_ms=1000*(time.process_time()-cpu),
                     queue_ms=1000*(began-submitted),worker_pid=os.getpid())
        if plan is not None:metrics['iterations']=planner.solvers[gate][0].stats().get('iter_count')
        results.put((plan,clearance,error,submitted,metrics))


class Worker:
    def __init__(self,track,cfg):
        context=mp.get_context('spawn')
        self.requests=context.Queue(1);self.results=context.Queue(1);self.ready=context.Queue(1)
        self.process=context.Process(target=_serve,args=(track,cfg,self.requests,self.results,self.ready),daemon=True)
        self.admitted=None;self.busy=False;self.metrics=[];self.process.start()
        try:
            self.startup=self.ready.get(timeout=600.)  # Offline C code generation, before flight.
            if 'failure' in self.startup:raise RuntimeError(self.startup['failure'])
        except Exception:self.close();raise
        self.geometry=SimpleNamespace(cfg=cfg,windows=[track.windows[i] for i in track.order],signs=self.startup['signs'])

    def commit(self,plan):self.admitted=plan
    def submit(self,state,at,gate):
        if self.busy:raise RuntimeError('Planner request already in flight')
        self.requests.put_nowait((np.asarray(state).copy(),at,gate,self.admitted,time.perf_counter()));self.busy=True
    def poll(self):
        try:plan,clearance,error,submitted,metrics=self.results.get_nowait()
        except queue.Empty:
            if not self.process.is_alive():raise RuntimeError(f'Planner process exited {self.process.exitcode}')
            return None
        elapsed=1000*(time.perf_counter()-submitted);self.busy=False
        metrics['end_to_end_ms']=elapsed;self.metrics.append(metrics)
        return plan,clearance,elapsed,error
    def close(self):
        if self.process.is_alive():
            try:self.requests.put_nowait(None)
            except queue.Full:pass
            self.process.join(2.)
            if self.process.is_alive():self.process.terminate();self.process.join(2.)
        for channel in (self.requests,self.results,self.ready):channel.close();channel.cancel_join_thread()
