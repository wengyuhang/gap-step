"""Budgeted nested feasible-set refinement, identical for every course.

The centre-constrained problem supplies an incumbent. Free gate coordinates
are then released; a longer or infeasible result cannot displace that incumbent.
Both solves together share the online deadline (25 + 35 ms CPU budgets).
"""
from dataclasses import replace
import math
import time

from .planner import Planner


class _CostRecorder:
    def __init__(self,solver):
        self.solver=solver; self.cost=math.inf
    def __call__(self,**kwargs):
        result=self.solver(**kwargs); self.cost=float(result['f']); return result
    def stats(self): return self.solver.stats()


class NestedPlanner:
    def __init__(self,track,cfg):
        self.cfg=cfg
        self.center=Planner(track,replace(cfg,free_points=False,solver_seconds=.025))
        self.free=Planner(track,replace(cfg,free_points=True,solver_seconds=.035))
        for planner in (self.center,self.free):
            planner.solvers=[(row[0],_CostRecorder(row[1]),*row[2:]) for row in planner.solvers]
        self.track=track; self.windows=self.free.windows; self.frames=self.free.frames; self.signs=self.free.signs
        self.selection_counts={'center':0,'free':0,'infeasible':0}

    def solve(self,state,now,first):
        started=time.perf_counter()
        center=self.center.solve(state,now,first)
        if center.violation < .003:
            self.free.previous=center
            self.free.search=None
        free=self.free.solve(state,now,first)
        candidates=[(p,self.center.solvers[first][1].cost if p is center else self.free.solvers[first][1].cost)
                    for p in (center,free) if p.violation < .003]
        if candidates:
            selected=min(candidates,key=lambda item:(float(item[0].durations.sum()),item[1]))[0]
            self.free.previous=selected
            self.selection_counts['center' if selected is center else 'free']+=1
        else:
            selected=min((center,free),key=lambda plan:plan.violation)
            self.selection_counts['infeasible']+=1
        selected.solve_ms=1000*(time.perf_counter()-started)
        return selected

    def prefix_clearance(self,*args,**kwargs):
        return self.free.prefix_clearance(*args,**kwargs)
