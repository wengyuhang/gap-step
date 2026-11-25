"""Time minimization with nominal per-rotor/rate bounds and local gate safety."""
from dataclasses import dataclass
import casadi as ca
import numpy as np
from .repaired_planner import Planner as Parent, Config as ParentConfig
from .hard_dynamics import constraints
from convex_timevarying_window.dynamic_passage_minco.safety import power_to_bernstein


@dataclass(frozen=True)
class Config(ParentConfig):
    forward_tail: bool = False
    strict_evaluation_deadline: bool = True
    witness_slots: int = 24
    passage_seed_offsets: tuple = tuple(np.arange(17)*.005)+(.10,.14,.20)

    def __post_init__(self):
        # The legacy entry explicitly passes forward_tail=True. The new method
        # permits braking beyond the goal, but never a return through the gate.
        object.__setattr__(self,'forward_tail',False)


class Planner(Parent):
    def objective_cost(self,times,dynamic):return ca.sum1(times)
    def extra_dynamic_residuals(self,state,times):return []
    def order_seeds(self,seeds,state,now):
        offsets=np.tile(np.r_[self.cfg.passage_seed_offsets,
                            np.zeros(self.cfg.witness_slots-len(self.cfg.passage_seed_offsets))],2)
        par=np.r_[now,state,offsets]
        def score(x):
            out=self.function(np.clip(x,self.lower,self.upper),par)
            g=np.array(out[2]).ravel()
            # Physical feasibility first, time breaks ties. No candidate is
            # executed merely because its seed is good.
            return np.max(np.maximum(-g,0.)),float(out[0])
        return sorted(seeds,key=score)
    def thrust_constraints(self,state,times):
        return constraints(state,times,self.cfg.nodes_per_piece,
                           self.cfg.workspace_lower,self.cfg.workspace_upper)
    def extra_constraints(self,state,times,C):
        result=self.thrust_constraints(state,times)
        normal=-self.signs[0]*self.rotation[:,2]
        power=C[8:,:]@ca.DM(normal);power[0]-=self.windows[0].center0@normal
        control=ca.DM(power_to_bernstein(np.eye(8)))@power
        result.extend(ca.vertsplit(control[1:]/10.))
        return result
