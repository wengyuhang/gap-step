"""Same inset domains, direct two-dimensional crossing coordinates."""
import casadi as ca
from bootstrap import load
load()
from repair import Planner as Base
from convex_timevarying_window.single_gate_stop_racing import point_mapping
# Process-local adapter. The explicit inset inequalities are in finite_xy.py.
point_mapping.dimension=lambda ap:2
point_mapping.symbolic=lambda ap,d:d
class Planner(Base):
    def constrained_minimize(self,fun,initial,**kwargs):
        import finite_xy
        return finite_xy.minimize(self,fun,initial,**kwargs)
