"""Original component-initialized MINCO; only moving geometry is replaced."""
from ..component_initialization.planner import Planner as ComponentPlanner
from .scenario import symbolic_pose
from .bounds import passage, warmup


class Planner(ComponentPlanner):
    symbolic_pose = staticmethod(symbolic_pose)

    def __init__(self, track, cfg, mapping):
        super().__init__(track, cfg, mapping)
        warmup()

    def check_passage(self, plan, window, radius, piece, deadline):
        return passage(plan, window, radius, piece, deadline)
