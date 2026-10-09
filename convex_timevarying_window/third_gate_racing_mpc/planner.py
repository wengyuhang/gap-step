"""Same zero-VA third-center problem, with a racing-scale rolling initializer."""
from dataclasses import dataclass
from convex_timevarying_window.third_gate_stop_mpc.planner import Planner as Parent,Config as ParentConfig
from .initialization import seed
from .solver import BufferedSolver


@dataclass(frozen=True)
class Config(ParentConfig):
    # No fast-math: preserve floating-point semantics and solver tolerances.
    compiler_optimization: str = '-O1'
    exclude_postrace_time: bool = True


class Planner(Parent):
    def __init__(self,track,cfg=Config(),*,build_indices=None):
        super().__init__(track,cfg,build_indices=build_indices)

    def _build(self,first):
        solver,*rest=super()._build(first)
        return (BufferedSolver(solver.fg,self.cfg),*rest)

    def initial(self,state,now,first):
        # Durations are relative to `now`: blindly copying a rejected old x
        # shifts every moving-window event. Rebuild from the admitted curve.
        # Keep search geometry for refinement, but not its stale decision vector.
        if self.admitted is None:
            return seed(self,state,now,first,self.cfg.speed/2,.75*self.cfg.speed)
        if self.admitted.first_gate==first:
            searched=self.search.pop(first,None)
            try:return super().initial(state,now,first)
            finally:
                if searched is not None:self.search[first]=searched
        return seed(self,state,now,first,self.cfg.speed,.45*self.cfg.speed)
