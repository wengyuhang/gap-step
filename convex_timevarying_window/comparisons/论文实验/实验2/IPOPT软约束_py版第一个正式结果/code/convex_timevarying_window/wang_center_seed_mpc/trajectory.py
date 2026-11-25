"""Direct Wang middle-position coordinates, no incoming-jerk reparameterization."""
from dataclasses import dataclass
import casadi as ca
from 复现.polynomial_dynamic_racing_2023.minco import normalized_inverse
from convex_timevarying_window.wang_free_state_mpc.trajectory import Plan as BasePlan

INVERSE = ca.DM(normalized_inverse(2))


def coefficients(head, tail, middle, duration):
    h = duration/2
    rhs = ca.horzcat(head[:, 0], h*head[:, 1], h*h*head[:, 2], middle, middle,
                    *[ca.DM.zeros(3) for _ in range(4)],
                    tail[:, 0], h*tail[:, 1], h*h*tail[:, 2]).T
    return INVERSE@rhs


@dataclass
class Plan(BasePlan):
    terminal_goal: bool = False
