"""Three-gate TOGT with crossing-point geometry consistent with its executor.

Only the local point parameterization is inset. The physical apertures and
their motion are unchanged; no trajectory safety test or certificate is added.
"""
from dataclasses import replace
import threading
import numpy as np
from convex_timevarying_window.geometry import ConvexAperture
from convex_timevarying_window.dynamic_gate_mppi_v3.config import Config
from convex_timevarying_window.three_phase_mppi.reference import Planner as BasePlanner,AsyncPlanner as BaseAsyncPlanner
from convex_timevarying_window.three_phase_mppi.px4 import ReferenceWorker as BaseWorker


def crossing_point_track(track):
    cfg=Config()
    # The physical frame's centreline is offset outward by 65 mm and its
    # sleeve radius is 85 mm: the original aperture boundary needs +20 mm.
    inset=cfg.radius+cfg.error_margin+.02+.10
    windows=[]
    for w in track.windows:
        a=w.aperture
        if a.kind=='circle':
            aperture=replace(a,margin=2*inset)
        else:
            v=a.physical_vertices;edges=np.roll(v,-1,axis=0)-v
            orientation=np.sign(np.sum(v[:,0]*np.roll(v[:,1],-1)-v[:,1]*np.roll(v[:,0],-1)))
            normals=orientation*np.column_stack((-edges[:,1],edges[:,0]))/np.linalg.norm(edges,axis=1)[:,None]
            bounds=np.einsum('ij,ij->i',normals,v)+inset
            safe=np.array([np.linalg.solve(normals[[j-1,j]],bounds[[j-1,j]]) for j in range(len(v))])
            if np.any(safe@normals.T<bounds[None,:]-1e-9):
                raise ValueError('crossing-point inset is empty; no fallback to an unsafe point set')
            aperture=ConvexAperture('polygon',margin=2*inset,physical_vertices=v.copy(),safe_vertices=safe)
        windows.append(replace(w,aperture=aperture))
    return replace(track,windows=tuple(windows))


class Planner(BasePlanner):
    def __init__(self,track,config,racing_guidance=False):
        super().__init__(crossing_point_track(track) if racing_guidance else track,config)


def worker(connection,track,config):
    # AsyncPlanner has already copied/inset the planning-only track if opted in.
    planner=BasePlanner(track,config);connection.send(('ready',None))
    while True:
        request=connection.recv()
        if request is None:break
        identifier,head,now,first=request
        try:
            result=planner.solve(head,now,first)
            if result is None:result=dict(error='reference_not_returned',details=planner.last_stats)
        except Exception as error:result=dict(error=repr(error))
        connection.send((identifier,result))


class AsyncPlanner(BaseAsyncPlanner):
    def _start_worker(self):
        self.connection,child=self.context.Pipe()
        self.process=self.context.Process(target=worker,args=(child,self.track,self.config),daemon=True)
        self.process.start();self.ready=False;self.running=None;self.started=None


class ReferenceWorker(BaseWorker):
    def __init__(self,track,opt,racing_guidance=False):
        self.worker=AsyncPlanner(crossing_point_track(track) if racing_guidance else track,opt);self.lock=threading.Lock();self.returned=[];self.stop=False
        self.thread=threading.Thread(target=self.loop,daemon=True);self.thread.start()
