"""Same planner and x500 CTBR MPC, only one original-size gate and a forward goal."""
from convex_timevarying_window.simple_two_gate_soft_only_mpc import flight as current
from . import scenario


def main():
    current.build_case=scenario.build_case
    current.launch=scenario.launch
    current.manifest=scenario.manifest
    current.MAX_FLIGHT_TIME=40
    current.main()


if __name__=='__main__':main()
