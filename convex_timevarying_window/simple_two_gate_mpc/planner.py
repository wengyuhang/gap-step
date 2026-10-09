"""Two crossings + moving third center. Time, dynamics and safety penalties only."""
from dataclasses import dataclass
from convex_timevarying_window.third_gate_stop_mpc.planner import Planner as Geometry, Config as GeometryConfig
from convex_timevarying_window.third_gate_racing_mpc.initialization import seed
from convex_timevarying_window.third_gate_racing_mpc.solver import BufferedSolver


@dataclass(frozen=True)
class Config(GeometryConfig):
    jerk_weight: float = 0.
    exclude_postrace_time: bool = False
    compiler_optimization: str = '-O1'

    def __post_init__(self):
        super().__post_init__()
        if self.jerk_weight != 0 or self.exclude_postrace_time or self.horizon != 2:
            raise ValueError('Two gates, no smoothness objective, full return time required')


class Planner(Geometry):
    def __init__(self,track,cfg=Config(),*,build_indices=None):
        super().__init__(track,cfg,build_indices=build_indices)

    def _build(self,first):
        solver,*rest=super()._build(first)
        return (BufferedSolver(solver.fg,self.cfg),*rest)

    def initial(self,state,now,first):
        # One rule for both variants and every horizon: shared absolute events
        # warm-start from the admitted curve; new crossing offsets start at zero.
        # Rejected relative-time vectors are not reused at a different epoch.
        return seed(self,state,now,first,.5*self.cfg.speed,.45*self.cfg.speed)
