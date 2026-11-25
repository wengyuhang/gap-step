"""Three-gate TOGT; zero-gate return uses the SAME objective, not a time guess."""
import time
import numpy as np
from convex_timevarying_window.dynamic_gate_mppi_v3.reference import (
    Planner as GatePlanner,AsyncPlanner as GateAsyncPlanner,Reference,PolynomialReference,_StopOptimization)
from nonconvex_timevarying_window.sc_dynatogt.minco import BoundaryState,MincoSnap
from nonconvex_timevarying_window.sc_dynatogt.optimizer import _minimize_togt_lbfgs
from nonconvex_timevarying_window.sc_dynatogt.time_mapping import durations_from_k,k_from_durations,backpropagate_to_k

class Planner(GatePlanner):
    def solve(self,head,now,first,budget=.1):
        if first<len(self.track.order):return super().solve(head,now,first,budget)
        started=time.perf_counter();goal=self.track.goal;tail=BoundaryState(goal)
        distance=np.linalg.norm(goal-head.position)
        total=max(1.5,distance/5.,np.linalg.norm(head.velocity)/4.)
        durations=np.array([.5,max(.5,(total-.5)/2),max(.5,(total-.5)/2)])
        points=np.array([head.position+.5*head.velocity,(head.position+goal)/2])
        initial=np.r_[k_from_durations(durations),points.ravel()];best=[np.inf,None];evaluations=0
        def evaluate(z):
            nonlocal evaluations
            if time.perf_counter()-started>.75*budget:raise _StopOptimization()
            try:
                durations=durations_from_k(z[:3]);points=z[3:].reshape(2,3)
                cost,gp,gt=self.core.value_and_gradient(head.matrix.T.copy(),tail.matrix.T.copy(),points,durations)
            except (RuntimeError,ValueError,np.linalg.LinAlgError,FloatingPointError,OverflowError):
                # L-BFGS line-search trials may map to numerically invalid
                # durations. Reject the trial, not the whole experiment.
                bounded=np.clip(z,-1e6,1e6);scale=self.optimization_config.invalid_trial_cost
                return scale*(1+1e-12*(bounded@bounded)),2e-12*scale*bounded
            evaluations+=1
            if np.isfinite(cost) and cost<best[0]:best[:]=[float(cost),z.copy()]
            return cost,np.r_[backpropagate_to_k(z[:3],gt),gp.ravel()]
        converged=False
        try:converged=bool(_minimize_togt_lbfgs(evaluate,initial,self.optimization_config).success)
        except _StopOptimization:pass
        self.last_stats=dict(evaluations=evaluations,elapsed_ms=1000*(time.perf_counter()-started),best_objective=best[0])
        if best[1] is None or best[0]>=self.optimization_config.invalid_trial_cost:return None
        z=best[1];trajectory=PolynomialReference(MincoSnap(head,tail,z[3:].reshape(2,3),durations_from_k(z[:3])))
        ms=1000*(time.perf_counter()-started)
        if ms>=1000*budget:return None
        return Reference(trajectory,now,first,np.empty(0),np.zeros((len(self.track.order),2)),ms,converged,evaluations)

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

class AsyncPlanner(GateAsyncPlanner):
    def _start_worker(self):
        self.connection,child=self.context.Pipe()
        self.process=self.context.Process(target=worker,args=(child,self.track,self.config),daemon=True)
        self.process.start();self.ready=False;self.running=None;self.started=None
