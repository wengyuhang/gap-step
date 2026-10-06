"""Private dynamics-aware three-gate reference, not a safety-certified plan."""
import ctypes
import hashlib
from pathlib import Path
import subprocess
import threading
import numpy as np
from convex_dynamic_seven_window_gazebo.x500_togt.native_backend import TOGT
from convex_timevarying_window.togt.native_backend import AnalyticTOGTCore
from convex_timevarying_window.three_phase_mppi.reference import Planner as BasePlanner,AsyncPlanner as BaseAsyncPlanner
from convex_timevarying_window.three_phase_mppi.px4 import ReferenceWorker as BaseWorker
from .reference import crossing_point_track
from convex_timevarying_window.dynamic_gate_mppi_v3.reference import PolynomialReference
from nonconvex_timevarying_window.sc_dynatogt.minco import BoundaryState,MincoSnap
import time

HERE=Path(__file__).resolve().parent

class Core:
    def __init__(self,bridge=False):
        self.bridge=bridge
        source=HERE/'native/racing_objective.cpp'
        dependency=HERE.parents[2]/'convex_dynamic_seven_window_gazebo/x500_togt/native/togt_x500.cpp'
        key=hashlib.sha256(source.read_bytes()+dependency.read_bytes()).hexdigest()[:16]
        library=HERE/'build'/f'racing_reference_{key}.so';library.parent.mkdir(exist_ok=True)
        names=['system/quadrotor_manifold.cpp','system/quadrotor_params.cpp','planner/traj_params.cpp',
               'base/parameter_base.cpp','type/quad_state.cpp','type/command.cpp','rotation/rotation_utils.cpp','math/math_utils.cpp']
        if not library.exists():
            subprocess.run(['g++','-std=c++17','-O3','-DNDEBUG','-fPIC','-shared',
                            f'-I{TOGT/"include"}',f'-I{TOGT.parent/"deps/eigen-3.4.0"}',
                            f'-I{TOGT/"build/_deps/rapidjson-src/include"}',str(source),
                            *(str(TOGT/'src'/n) for n in names),'-o',str(library)],check=True)
        self.library=ctypes.CDLL(str(library));self.function=self.library.racing_objective
        self.function.argtypes=[ctypes.c_int]+[ctypes.POINTER(ctypes.c_double)]*8+[ctypes.c_char_p,ctypes.c_int,ctypes.c_int,ctypes.POINTER(ctypes.c_double),ctypes.c_int]
        self.function.restype=ctypes.c_int;self.context=None
    def tail_function(self,*args):
        planes=[]
        if self.context is not None:
            windows,now,signs=self.context
            durations=np.ctypeslib.as_array(args[4],shape=(args[0],));times=now+np.cumsum(durations)[int(self.bridge):]
            for w,at,sign in zip(windows,times,signs):
                c,r,dc,dr=w.state_at(at);_,_,plus_dc,plus_dr=w.state_at(at+1e-4);_,_,minus_dc,minus_dr=w.state_at(at-1e-4)
                planes.append(np.r_[r[:,2],dr[:,2],(plus_dr[:,2]-minus_dr[:,2])/2e-4,c,dc,(plus_dc-minus_dc)/2e-4,sign])
        data=np.ascontiguousarray(planes,dtype=float)
        return self.function(*args,len(planes),data.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),int(self.bridge))
    @staticmethod
    def augmented(head,points,durations):
        h=min(.15,.25*durations[0]);alpha=.25 if durations[0]<.6 else 0.
        point=head[:,0]+h*head[:,1]+.5*h*h*head[:,2]
        return np.vstack((point,points)),np.r_[h,durations[0]-h,durations[1:]],h,alpha
    def value_and_gradient_with_tail(self,head,tail,points,durations):
        if not self.bridge:return AnalyticTOGTCore.value_and_gradient_with_tail(self,head,tail,points,durations)
        pp,tt,h,alpha=self.augmented(head,points,durations)
        cost,gp,gt,gh=AnalyticTOGTCore.value_and_gradient_with_tail(self,head,tail,pp,tt)
        gtime=gt[1:].copy()
        gtime[0]=alpha*gt[0]+(1-alpha)*gt[1]+alpha*np.dot(gp[0],head[:,1]+h*head[:,2])
        return cost,gp[1:],gtime,gh
    def value_and_gradient(self,*args):return self.value_and_gradient_with_tail(*args)[:3]

class Planner(BasePlanner):
    def __init__(self,track,config):
        self.track=crossing_point_track(track);self.optimization_config=config;self.core=Core()
        self.signs=[];previous=track.start
        for index in track.order:
            c,r,*_=track.windows[index].state_at(0.)
            self.signs.append(-np.sign(r[:,2]@(previous-c)));previous=c
    def solve(self,head,now,first,budget=.1):
        started=time.perf_counter()
        # A short extra piece is only useful for a large transient. At rest it
        # needlessly creates a very uneven MINCO mesh under a fixed solve budget.
        self.core.bridge=bool(np.linalg.norm(head.jerk)*.15>7.5)
        self.core.context=([self.track.windows[i] for i in self.track.order[first:first+3]],now,self.signs[first:first+3])
        result=super().solve(head,now,first,budget)
        if result is None:return None
        if self.core.bridge:
            original=result.trajectory
            tail=BoundaryState(*[original.evaluate(original.total_time,d) for d in range(4)])
            points=original.evaluate(np.cumsum(original.durations)[:-1],0)
            points,durations,_,_=self.core.augmented(head.matrix.T,points,original.durations)
            result.trajectory=PolynomialReference(MincoSnap(head,tail,points,durations))
        result.solve_ms=1000*(time.perf_counter()-started)
        return result if result.solve_ms<1000*budget else None

def worker(connection,track,config):
    planner=Planner(track,config);connection.send(('ready',None))
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
    def __init__(self,track,opt):
        self.worker=AsyncPlanner(track,opt);self.lock=threading.Lock();self.returned=[];self.stop=False
        self.thread=threading.Thread(target=self.loop,daemon=True);self.thread.start()
